
package frc.robot.subsystems.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.CanIds;

import frc.robot.util.MechanismDiagnostics;
import frc.robot.util.ParallelModel;

// #endregion


/**
 * Telescópio 9460: controle comum à simulação e ao replay; a camada IO
 * fornece os sensores.
 *
 * O PID roda no roboRIO. Cada ciclo separa o que é SEGURANÇA (o que impede ou
 * limita o motor) do que é CONTROLE (o que calcula a tensão):
 * 1. sensores, 2. modelo em paralelo, 3. segurança: travamento,
 * 4. segurança: o motor pode andar?, 5. controle: PID + feedforward,
 * 6. segurança: limites de tensão e batentes, 7. acionamento, 8. telemetria.
 */
public class Telescopio extends SubsystemBase implements AutoCloseable {

    // #region ATRIBUTOS E COMPONENTES

    private final TelescopioIO io;

    private final TelescopioIOInputsAutoLogged inputs =
            new TelescopioIOInputsAutoLogged();

    private final TelescopioVisualizer visualizer =
            new TelescopioVisualizer();

    // #endregion


    // #region 🧪 MODELO EM PARALELO E DIAGNOSTICO (PADRAO DO TELESCOPIO 9458)

    // Física recebendo a tensão medida no motor: real x simulado.
    private final ParallelModel model = new ParallelModel("Telescopio", TelescopioIOSim::new,
            ParallelModel.Unit.of("Centimetros", 100.0),
            ParallelModel.Unit.of("CentimetrosPorSeg", 100.0),
            MODEL_TOLERANCE_CENTIMETERS,
            MODEL_REST_CENTIMETERS_PER_SEC);

    // Desligado em CanIds, o robô real só avisa (o motor nem é criado). Antes bastava
    // ter ID: aparecia MOTOR_SEM_RESPOSTA para um motor que não está no robô.
    private final MechanismDiagnostics diagnostics = CanIds.TELESCOPIO_9460_ENABLED
            ? new MechanismDiagnostics("Telescopio", "Telescópio 9460",
                    "TALONFX_ID_" + CanIds.TELESCOPIO_9460)
            : MechanismDiagnostics.disabledInCanIds("Telescopio", "Telescópio 9460");

    // #endregion


    // #region 🎯🚀 CONTROLE - PID E PERFIL DE MOVIMENTO

    // Perfil trapezoidal (velocidade e aceleração máximas) + PID de posição.
    // O feedforward (kG, kV, kA) é somado no passo 5.
    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    POSITION_KP,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            MAX_VELOCITY_METERS_PER_SEC,
                            MAX_ACCELERATION_METERS_PER_SEC_SQ),
                    LOOP_PERIOD_SECONDS
            );

    // #endregion


    // #region 🛑 SEGURANCA - TRAVAMENTO E TOPO

    private final Debouncer jamDebouncer =
            new Debouncer(
                    JAM_TIME_SECONDS,
                    Debouncer.DebounceType.kRising
            );

    private boolean jammed;
    private boolean topLimitActive;

    // #endregion


    // #region ESTADO DO TELESCOPIO

    private boolean positionControlActive;

    private double goalMeters;
    private double requestedVolts;

    private String status = "PARADO";

    // #endregion


    // #region CONSTRUTOR

    public Telescopio(TelescopioIO io) {

        this.io = Objects.requireNonNull(io);

        controller.setTolerance(
                POSITION_TOLERANCE_METERS,
                VELOCITY_TOLERANCE_METERS_PER_SEC
        );
    }

    // #endregion


    // #region FABRICA DE IO - REAL / SIM / REPLAY

    /**
     * Replay precisa selecionar IO vazio antes
     * de verificar se está no desktop.
     */
    public static TelescopioIO createIO() {

        if (Logger.hasReplaySource()) {
            return new TelescopioIO() {};
        }

        // Robô real: só cria o TalonFX quando ele estiver ligado no mapa.
        // Desligado, o IO vazio informa sensores indisponíveis e nada se move.
        if (RobotBase.isReal()) {
            return CanIds.TELESCOPIO_9460_ENABLED
                ? new TelescopioIOTalonFX(CanIds.TELESCOPIO_9460)
                : new TelescopioIO() {};
        }

        return new TelescopioIOSim();
    }

    // #endregion


    // #region 🔁 PERIODIC - ORDEM DE CADA CICLO (20 ms)

    @Override
    public void periodic() {
        readSensors();
        stepParallelModel();
        boolean jamDetected = detectJam();
        runSafetyAndControl(jamDetected);
        logTelemetry();
    }

    // #endregion


    // #region 🔁 1. SENSORES

    private void readSensors() {

        io.updateInputs(inputs);

        Logger.processInputs(
                "Telescopio/Inputs",
                inputs
        );
    }

    // #endregion


    // #region 🧪 2. MODELO EM PARALELO - PASSO DA FISICA

    private void stepParallelModel() {

        model.update(
                hasValidPosition(),
                inputs.extensionMeters,
                inputs.velocityMetersPerSec,
                inputs.appliedVolts,
                inputs.motorCurrentAmps
        );
    }

    // #endregion


    // #region 🛑 3. SEGURANCA - DETECCAO DE TRAVAMENTO

    /**
     * Travamento: corrente elevada e velocidade baixa por JAM_TIME_SECONDS.
     * Calculado em todo ciclo (o Debouncer conta o tempo), mesmo parado.
     */
    private boolean detectJam() {

        return jamDebouncer.calculate(
                positionControlActive
                && Math.abs(inputs.motorCurrentAmps) >= JAM_CURRENT_AMPS
                && Math.abs(inputs.velocityMetersPerSec) <= JAM_VELOCITY_METERS_PER_SEC
        );
    }

    // #endregion


    // #region 🛑 4. SEGURANCA - O MOTOR PODE ANDAR?

    /**
     * A segurança decide primeiro; o controle só calcula quando o motor pode
     * andar. A ordem das verificações é a ordem de prioridade.
     */
    private void runSafetyAndControl(boolean jamDetected) {

        topLimitActive = false;

        if (DriverStation.isDisabled()) {

            stop();
            status = "DESABILITADO";
            return;
        }

        if (!hasValidPosition()) {

            stop();
            status = "SEM_REFERENCIA_OU_COMUNICACAO";
            return;
        }

        if (!positionControlActive) {

            io.stop();
            return;
        }

        holdWhereJammed(jamDetected);

        double controlVolts = calculateControlVolts();

        requestedVolts = applyVoltageLimits(controlVolts);

        driveMotor();
    }

    /**
     * Travou: para de empurrar, mas continua sustentando o peso onde
     * travou. Só um comando novo (clearJam) tenta de novo.
     */
    private void holdWhereJammed(boolean jamDetected) {

        if (jamDetected && !jammed) {

            jammed = true;

            goalMeters = MathUtil.clamp(
                    inputs.extensionMeters,
                    MIN_EXTENSION_METERS,
                    MAX_EXTENSION_METERS
            );

            controller.reset(goalMeters, 0);
        }
    }

    // #endregion


    // #region 🎯 5. CONTROLE - PID + FEEDFORWARD

    /**
     * Tensão pedida pelo controle, ainda SEM os limites de segurança:
     * PID sobre o perfil + kG (peso) + kV (velocidade) + kA (aceleração).
     */
    private double calculateControlVolts() {

        // PID: avança o perfil e corrige a diferença até a referência.
        double previousVelocity =
                controller.getSetpoint().velocity;

        double feedback = controller.calculate(
                inputs.extensionMeters,
                goalMeters
        );

        var setpoint = controller.getSetpoint();

        // Feedforward: o que o motor precisa sem erro nenhum.
        double acceleration =
                (setpoint.velocity - previousVelocity)
                / LOOP_PERIOD_SECONDS;

        double feedforward =
                SIM_KG
                + SIM_KV * setpoint.velocity
                + SIM_KA * acceleration;

        return feedback + feedforward;
    }

    // #endregion


    // #region 🛑⚡ 6. SEGURANCA - LIMITES DE TENSAO E BATENTES

    /** Tensão máxima do controle e, depois, os batentes. */
    private double applyVoltageLimits(double controlVolts) {

        double clampedVolts = MathUtil.clamp(
                controlVolts,
                -MAX_CONTROL_VOLTS,
                MAX_CONTROL_VOLTS
        );

        double limitedVolts = limitAtEnds(
                clampedVolts,
                inputs.extensionMeters,
                goalMeters,
                controller.getSetpoint().velocity
        );

        topLimitActive =
                inputs.extensionMeters >= MAX_EXTENSION_METERS
                && limitedVolts < clampedVolts;

        return limitedVolts;
    }

    /**
     * Batentes:
     *
     * Embaixo não empurra para baixo.
     * No topo só sustenta o peso (kG).
     */
    static double limitAtEnds(
            double volts,
            double extensionMeters,
            double goalMeters,
            double setpointVelocity) {

        // Recolhido, apoia no limite inferior.
        if ((goalMeters == MIN_EXTENSION_METERS
                && extensionMeters <= 0.002
                && setpointVelocity <= 0)
                || (extensionMeters <= MIN_EXTENSION_METERS
                && volts < 0)) {

            return 0;
        }

        // No topo, não empurra o batente
        // e não deixa cair.
        if (extensionMeters >= MAX_EXTENSION_METERS
                && volts > SIM_KG) {

            return SIM_KG;
        }

        return volts;
    }

    // #endregion


    // #region 7. ACIONAMENTO DO MOTOR

    private void driveMotor() {

        io.setVoltage(requestedVolts);

        status = jammed
                ? "TRAVADO"
                : atGoal()
                    ? "MANTENDO_POSICAO"
                    : "MOVENDO";
    }

    // #endregion


    // #region 📊 8. TELEMETRIA (NOMES USADOS PELOS LAYOUTS)

    private void logTelemetry() {

        // Sem motor no robô real não há o que mostrar, e estes campos custariam
        // tempo a cada ciclo. Simulação e replay continuam completos.
        if (RobotBase.isReal() && !CanIds.TELESCOPIO_9460_ENABLED) {
            diagnostics.update(false, MechanismDiagnostics.MOTOR_SEM_RESPOSTA);
            return;
        }

        visualizer.update(
                inputs.extensionMeters,
                hasValidPosition()
        );

        logControl();
        logSafety();
        logExecutionMode();
        logModelConstants();

        model.log();

        diagnostics.update(
                inputs.motorConnected,
                MechanismDiagnostics.result(
                        inputs.motorConnected,
                        hasValidPosition(),
                        model.isDiverging())
        );
    }

    /** Posição, meta, perfil, tensão pedida, estado e comando. */
    private void logControl() {

        Logger.recordOutput("Telescopio/MetaMetros", goalMeters);
        Logger.recordOutput("Telescopio/ReferenciaPerfilMetros", controller.getSetpoint().position);
        Logger.recordOutput("Telescopio/ErroMetros", goalMeters - inputs.extensionMeters);
        Logger.recordOutput("Telescopio/TensaoSolicitadaVolts", requestedVolts);
        Logger.recordOutput("Telescopio/ControleAtivo", positionControlActive);
        Logger.recordOutput("Telescopio/NaMeta", atGoal());
        Logger.recordOutput("Telescopio/Estado", status);
        Logger.recordOutput("Telescopio/Comando", MechanismDiagnostics.commandName(this));
    }

    /** Travamento, limite do topo e limite de corrente configurado. */
    private void logSafety() {

        Logger.recordOutput("Telescopio/Protecao/Travado", jammed);
        Logger.recordOutput("Telescopio/Protecao/LimiteTopoAtivo", topLimitActive);
        Logger.recordOutput("Telescopio/Protecao/LimiteCorrenteEstatorAmps", STATOR_CURRENT_LIMIT_AMPS);
    }

    private void logExecutionMode() {

        Logger.recordOutput(
                "Telescopio/Modo",
                Logger.hasReplaySource()
                        ? "REPLAY"
                        : RobotBase.isReal()
                            ? CanIds.TELESCOPIO_9460_ENABLED
                                ? "ROBO_REAL_TALONFX_ID_" + CanIds.TELESCOPIO_9460
                                : "HARDWARE_DESLIGADO"
                            : "SIMULACAO_FISICA_APROXIMADA"
        );
    }

    /** Hipóteses da física publicadas para consulta nos gráficos. */
    private void logModelConstants() {

        Logger.recordOutput("Telescopio/Modelo/MassaEquivalenteKg", SIM_INITIAL_MOVING_MASS_KG);
        Logger.recordOutput("Telescopio/Modelo/RelacaoCaboPonta", SIM_CABLE_TO_TIP_RATIO);
        Logger.recordOutput("Telescopio/Modelo/Movimento", "SIMULTANEO_PROPORCIONAL_AO_CURSO");
    }

    // #endregion


    // #region 🎯 API - META (CONTROLE)

    public void setGoalMeters(double meters) {

        // Segurança: meta inválida, robô desabilitado ou sem referência.
        if (!Double.isFinite(meters)
                || DriverStation.isDisabled()
                || !hasValidPosition()) {

            stop();
            status = "META_RECUSADA";
            return;
        }

        // Mantém a posição do travamento
        // até um novo comando.
        if (jammed) {
            return;
        }

        if (!positionControlActive) {

            controller.reset(
                    inputs.extensionMeters,
                    inputs.velocityMetersPerSec
            );
        }

        goalMeters = MathUtil.clamp(
                meters,
                MIN_EXTENSION_METERS,
                MAX_EXTENSION_METERS
        );

        positionControlActive = true;
    }

    /**
     * Meta onde o perfil para com a
     * desaceleração máxima, sem voltar para trás.
     * Usado quando o piloto solta o botão de abrir/fechar.
     */
    public void holdAtStoppingPoint() {

        if (!positionControlActive) {
            return;
        }

        var setpoint = controller.getSetpoint();

        setGoalMeters(
                setpoint.position
                        + setpoint.velocity
                        * Math.abs(setpoint.velocity)
                        / (2 * MAX_ACCELERATION_METERS_PER_SEC_SQ)
        );
    }

    // #endregion


    // #region 🛑 API - PARADA E DESTRAVAMENTO (SEGURANCA)

    /**
     * Tensão zero.
     *
     * Manter uma altura exige controle
     * de posição ativo.
     */
    public void stop() {

        positionControlActive = false;
        jammed = false;
        requestedVolts = 0;

        io.stop();

        status = "PARADO";
    }

    /**
     * Cada comando novo é uma nova tentativa
     * depois de um travamento.
     */
    public void clearJam() {
        jammed = false;
    }

    // #endregion


    // #region CONSULTAS - POSICAO VALIDA E META

    public boolean hasValidPosition() {

        return inputs.motorConnected
                && inputs.positionValid
                && Double.isFinite(inputs.extensionMeters)
                && Double.isFinite(inputs.velocityMetersPerSec)
                && inputs.extensionMeters >=
                        MIN_EXTENSION_METERS - POSITION_TOLERANCE_METERS
                && inputs.extensionMeters <=
                        MAX_EXTENSION_METERS + POSITION_TOLERANCE_METERS;
    }

    public boolean atGoal() {

        return positionControlActive
                && !jammed
                && hasValidPosition()
                && Math.abs(goalMeters - inputs.extensionMeters)
                        <= POSITION_TOLERANCE_METERS
                && Math.abs(inputs.velocityMetersPerSec)
                        <= VELOCITY_TOLERANCE_METERS_PER_SEC
                && Math.abs(controller.getSetpoint().position - goalMeters)
                        <= POSITION_TOLERANCE_METERS;
    }

    // #endregion


    // #region 📊 GETTERS - TELEMETRIA E ESTADO

    public double getExtensionMeters() {
        return inputs.extensionMeters;
    }

    public double getVelocityMetersPerSecond() {
        return inputs.velocityMetersPerSec;
    }

    public double getGoalMeters() {
        return goalMeters;
    }

    public double getRequestedVolts() {
        return requestedVolts;
    }

    public boolean isPositionControlActive() {
        return positionControlActive;
    }

    public boolean isJammed() {
        return jammed;
    }

    // #endregion


    // #region COMANDOS (EM frc.robot.commands.telescopio)

    // Abrir, fechar, ir para uma altura, seguir a meta do painel e o ciclo
    // automático ficam em commands/telescopio, como os outros mecanismos.
    // O subsistema oferece o controle: setGoalMeters, stop, atGoal,
    // clearJam e holdAtStoppingPoint.

    // #endregion


    // #region ENCERRAMENTO E LIBERACAO DE RECURSOS

    @Override
    public void close() {

        stop();

        visualizer.close();

        CommandScheduler
                .getInstance()
                .unregisterSubsystem(this);
    }

    // #endregion
}
