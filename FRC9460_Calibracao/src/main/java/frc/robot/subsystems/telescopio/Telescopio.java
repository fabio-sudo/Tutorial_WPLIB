
package frc.robot.subsystems.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

// #endregion


/**
 * Controle comum à simulação e ao replay;
 * a camada IO fornece os sensores.
 */
public class Telescopio extends SubsystemBase implements AutoCloseable {

    // #region ATRIBUTOS E COMPONENTES

    private final TelescopioIO io;

    private final TelescopioIOInputsAutoLogged inputs =
            new TelescopioIOInputsAutoLogged();

    private final TelescopioVisualizer visualizer =
            new TelescopioVisualizer();

    // #endregion


    // #region CONTROLADOR PID E PERFIL DE MOVIMENTO

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


    // #region PROTECAO CONTRA TRAVAMENTO

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

        if (RobotBase.isReal()) {
            return new TelescopioIOTalonFX();
        }

        return new TelescopioIOSim();
    }

    // #endregion


    // #region PERIODIC - CONTROLE PRINCIPAL

    @Override
    public void periodic() {

        // #region ATUALIZACAO DOS SENSORES

        io.updateInputs(inputs);

        Logger.processInputs(
                "Telescopio/Inputs",
                inputs
        );

        // #endregion


        // #region DETECCAO DE TRAVAMENTO

        // Travamento: corrente elevada e velocidade baixa.
        boolean jamDetected = jamDebouncer.calculate(
                positionControlActive
                && Math.abs(inputs.motorCurrentAmps) >= JAM_CURRENT_AMPS
                && Math.abs(inputs.velocityMetersPerSec) <= JAM_VELOCITY_METERS_PER_SEC
        );

        topLimitActive = false;

        // #endregion


        // #region VALIDACAO DE SEGURANCA

        if (DriverStation.isDisabled()) {

            stop();
            status = "DESABILITADO";

        } else if (!hasValidPosition()) {

            stop();
            status = "SEM_REFERENCIA_OU_COMUNICACAO";

        } else if (positionControlActive) {

            // #region TRATAMENTO DO TRAVAMENTO

            if (jamDetected && !jammed) {

                // Para de empurrar, mas continua sustentando
                // o peso onde travou.

                jammed = true;

                goalMeters = MathUtil.clamp(
                        inputs.extensionMeters,
                        MIN_EXTENSION_METERS,
                        MAX_EXTENSION_METERS
                );

                controller.reset(goalMeters, 0);
            }

            // #endregion


            // #region CALCULO PID

            double previousVelocity =
                    controller.getSetpoint().velocity;

            double feedback = controller.calculate(
                    inputs.extensionMeters,
                    goalMeters
            );

            var setpoint = controller.getSetpoint();

            // #endregion


            // #region CALCULO FEEDFORWARD

            double acceleration =
                    (setpoint.velocity - previousVelocity)
                    / LOOP_PERIOD_SECONDS;

            double feedforward =
                    SIM_KG
                    + SIM_KV * setpoint.velocity
                    + SIM_KA * acceleration;

            // #endregion


            // #region LIMITACAO DE TENSAO

            requestedVolts = MathUtil.clamp(
                    feedback + feedforward,
                    -MAX_CONTROL_VOLTS,
                    MAX_CONTROL_VOLTS
            );

            double limitedVolts = limitAtEnds(
                    requestedVolts,
                    inputs.extensionMeters,
                    goalMeters,
                    setpoint.velocity
            );

            topLimitActive =
                    inputs.extensionMeters >= MAX_EXTENSION_METERS
                    && limitedVolts < requestedVolts;

            requestedVolts = limitedVolts;

            // #endregion


            // #region ACIONAMENTO DO MOTOR

            io.setVoltage(requestedVolts);

            status = jammed
                    ? "TRAVADO"
                    : atGoal()
                        ? "MANTENDO_POSICAO"
                        : "MOVENDO";

            // #endregion

        } else {

            io.stop();
        }

        // #endregion


        // #region ATUALIZACAO DA VISUALIZACAO

        visualizer.update(
                inputs.extensionMeters,
                hasValidPosition()
        );

        // #endregion


        // #region LOGGING - CONTROLE E POSICAO

        Logger.recordOutput(
                "Telescopio/MetaMetros",
                goalMeters
        );

        Logger.recordOutput(
                "Telescopio/ReferenciaPerfilMetros",
                controller.getSetpoint().position
        );

        Logger.recordOutput(
                "Telescopio/ErroMetros",
                goalMeters - inputs.extensionMeters
        );

        Logger.recordOutput(
                "Telescopio/TensaoSolicitadaVolts",
                requestedVolts
        );

        Logger.recordOutput(
                "Telescopio/ControleAtivo",
                positionControlActive
        );

        Logger.recordOutput(
                "Telescopio/NaMeta",
                atGoal()
        );

        Logger.recordOutput(
                "Telescopio/Estado",
                status
        );

        // #endregion


        // #region LOGGING - COMANDOS

        Logger.recordOutput(
                "Telescopio/Comando",
                getCurrentCommand() == null
                        ? "NENHUM"
                        : getCurrentCommand().getName()
        );

        // #endregion


        // #region LOGGING - PROTECOES

        Logger.recordOutput(
                "Telescopio/Protecao/Travado",
                jammed
        );

        Logger.recordOutput(
                "Telescopio/Protecao/LimiteTopoAtivo",
                topLimitActive
        );

        Logger.recordOutput(
                "Telescopio/Protecao/LimiteCorrenteEstatorAmps",
                STATOR_CURRENT_LIMIT_AMPS
        );

        // #endregion


        // #region LOGGING - MODO DE EXECUCAO

        Logger.recordOutput(
                "Telescopio/Modo",
                Logger.hasReplaySource()
                        ? "REPLAY"
                        : RobotBase.isReal()
                            ? "HARDWARE_NAO_CONFIGURADO"
                            : "SIMULACAO_FISICA_APROXIMADA"
        );

        // #endregion


        // #region LOGGING - MODELO FISICO

        Logger.recordOutput(
                "Telescopio/Modelo/MassaEquivalenteKg",
                SIM_INITIAL_MOVING_MASS_KG
        );

        Logger.recordOutput(
                "Telescopio/Modelo/RelacaoCaboPonta",
                SIM_CABLE_TO_TIP_RATIO
        );

        Logger.recordOutput(
                "Telescopio/Modelo/Movimento",
                "SIMULTANEO_PROPORCIONAL_AO_CURSO"
        );

        // #endregion
    }

    // #endregion


    // #region PROTECAO DOS LIMITES MECANICOS

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


    // #region DEFINICAO DE META DE POSICAO

    public void setGoalMeters(double meters) {

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

    // #endregion


    // #region PARADA DO TELESCOPIO

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

    // #endregion


    // #region VALIDACAO DOS SENSORES E POSICAO

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

    // #endregion


    // #region VERIFICACAO DA META

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


    // #region GETTERS - TELEMETRIA E ESTADO

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


    // #region REINICIALIZACAO DE TRAVAMENTO

    /**
     * Cada comando novo é uma nova tentativa
     * depois de um travamento.
     */
    private void clearJam() {
        jammed = false;
    }

    // #endregion


    // #region COMANDO - ACOMPANHAR META DINAMICA

    public Command followGoalCommand(DoubleSupplier goal) {

        return startRun(
                this::clearJam,
                () -> setGoalMeters(goal.getAsDouble())
        )
                .finallyDo(this::stop)
                .withName("Telescopio - acompanhar meta");
    }

    // #endregion


    // #region COMANDO - MOVER PARA POSICAO

    /**
     * Ao concluir normalmente, mantém a posição.
     * Interrupção corta a tensão.
     */
    public Command moveToCommand(double meters) {

        return startRun(
                this::clearJam,
                () -> setGoalMeters(meters)
        )
                .until(this::atGoal)
                .finallyDo(interrupted -> {

                    if (interrupted) {
                        stop();
                    }

                })
                .withName(
                        "Telescopio - mover para " + meters + " m"
                );
    }

    // #endregion


    // #region COMANDOS - ABRIR E FECHAR MANUALMENTE

    /**
     * Segurar abre até o fim de curso.
     * Ao soltar, freia pelo perfil e mantém a altura.
     */
    public Command extendWhileHeldCommand() {

        return moveWhileHeldCommand(
                MAX_EXTENSION_METERS,
                "abrir"
        );
    }

    /**
     * Segurar recolhe até o batente.
     * Ao soltar, freia pelo perfil e mantém a altura.
     */
    public Command retractWhileHeldCommand() {

        return moveWhileHeldCommand(
                MIN_EXTENSION_METERS,
                "fechar"
        );
    }

    // #endregion


    // #region CONTROLE DE MOVIMENTO ENQUANTO SEGURA O BOTAO

    private Command moveWhileHeldCommand(
            double limitMeters,
            String name) {

        return startRun(
                this::clearJam,
                () -> setGoalMeters(limitMeters)
        )
                .finallyDo(this::holdAtStoppingPoint)
                .withName(
                        "Telescopio - " + name + " (segurar)"
                );
    }

    // #endregion


    // #region DESACELERACAO E MANUTENCAO DE POSICAO

    /**
     * Meta onde o perfil para com a
     * desaceleração máxima, sem voltar para trás.
     */
    private void holdAtStoppingPoint() {

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


    // #region COMANDO - DEMONSTRACAO AUTOMATICA

    public Command demoCommand() {

        return Commands.sequence(

                moveToCommand(MAX_EXTENSION_METERS / 2),

                Commands.waitSeconds(DEMO_DWELL_SECONDS),

                moveToCommand(MAX_EXTENSION_METERS),

                Commands.waitSeconds(DEMO_DWELL_SECONDS),

                moveToCommand(MIN_EXTENSION_METERS),

                Commands.waitSeconds(DEMO_DWELL_SECONDS)

        )
                .repeatedly()
                .finallyDo(interrupted -> stop())
                .withName("Telescopio - ciclo automatico");
    }

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
