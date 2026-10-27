package frc.robot.subsystems.telescopio_9458;

// #region IMPORTS

import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.telescopio_9458.TelescopioMotionProfile_9458.Limits;

// #endregion


/**
 * Telescópio 9458 (cascata): controle comum à simulação e ao replay;
 * a camada IO fornece os sensores.
 *
 * Mapa da classe:
 * 1. periodic(): sensores → modelo em paralelo → travamento → controle → telemetria.
 * 2. API pública: meta, parada, zero recolhido e ganhos (somente desabilitado).
 * 3. Comandos: segurar RT/LT, mover para posição, acompanhar meta e ciclo automático.
 * 4. SysId: runCharacterization(), usado por frc.robot.calibration.Telescopio9458SysId.
 *
 * Executor (USE_TALON_MOTION_MAGIC): com Motion Magic, o Talon gera a curva em S e
 * fecha a malha a 1 kHz; aqui ficam a meta, o trecho lento perto dos batentes, a
 * parada ao soltar e as proteções. Sem ele, perfil trapezoidal + PID no roboRIO.
 */
public class Telescopio_9458 extends SubsystemBase implements AutoCloseable {

    // #region IO, SENSORES E VISUALIZACAO

    private final TelescopioIO_9458 io;

    private final TelescopioIOInputs_9458AutoLogged inputs =
            new TelescopioIOInputs_9458AutoLogged();

    private final TelescopioVisualizer_9458 visualizer =
            new TelescopioVisualizer_9458();

    // #endregion


    // #region GANHOS (GRAVADOS NO LOG PARA O REPLAY)

    private final TelescopioGainsInputsAutoLogged gainsInputs = new TelescopioGainsInputsAutoLogged();
    private TelescopioGains_9458 gains;

    // #endregion


    // #region CONTROLADOR PID E PERFIL DE MOVIMENTO

    // Usado quando o executor é o roboRIO. O kP inicial e as restrições são trocados
    // no construtor pelos ganhos e pelo perfil ativos.
    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    POSITION_KP, // kP
                    0,           // kI
                    0,           // kD
                    new TrapezoidProfile.Constraints(
                            MAX_VELOCITY_METERS_PER_SEC,
                            MAX_ACCELERATION_METERS_PER_SEC_SQ),
                    LOOP_PERIOD_SECONDS
            );

    // #endregion


    // #region PERFIL DE MOVIMENTO (GRAVADO NO LOG PARA O REPLAY)

    // true: Motion Magic (Talon real ou emulado na simulação); false: perfil no roboRIO.
    // Segue o que o IO informa nas entradas: no replay, o que foi gravado no log.
    private boolean motionMagic;

    private final TelescopioProfileInputsAutoLogged profileInputs = new TelescopioProfileInputsAutoLogged();
    private Limits profileLimits;
    private String profileStatus = "PADRAO";

    // Motion Magic: trecho lento já escolhido para a meta atual e aceleração da referência.
    private boolean slowApproach;
    private double previousReferenceVelocity;
    private double referenceAcceleration;

    // #endregion


    // #region ESTADO DO CONTROLE

    private boolean positionControlActive;

    private double goalMeters;
    private double requestedVolts;
    private double feedbackVolts;
    private double feedforwardVolts;

    // SysId: tensão em malha aberta, sem PID (calibration/Telescopio9458SysId).
    private boolean characterizationActive;
    private double characterizationVolts;

    private String status = "PARADO";

    // #endregion


    // #region PROTECOES: TRAVAMENTO E TOPO

    private final Debouncer jamDebouncer =
            new Debouncer(
                    JAM_TIME_SECONDS,
                    Debouncer.DebounceType.kRising
            );

    private boolean jammed;
    private boolean topLimitActive;

    private final Debouncer characterizationStallDebouncer =
            new Debouncer(
                    SYSID_STALL_TIME_SECONDS,
                    Debouncer.DebounceType.kRising
            );

    // Travou no SysId: recusa tensão até o próximo aperto do botão.
    private boolean characterizationStalled;

    // #endregion


    // #region DIAGNOSTICO E ALERTAS

    private String adjustmentStatus = "SEM_AJUSTE";
    private String referenceStatus = "ZERO_NA_PARTIDA";
    private final Alert positionAlert = new Alert("Telescopio 9458: referencia ou leitura invalida. Consulte Diagnostico.", AlertType.kError);
    private final Alert controlAlert = new Alert("Telescopio 9458: falha ao enviar comando ao TalonFX.", AlertType.kError);

    // #endregion


    // #region MODELO EM PARALELO (REAL X SIMULADO)

    // Física da simulação recebendo a mesma tensão do motor. No robô real,
    // mostra ao vivo o que o modelo prevê ao lado do que o telescópio faz.
    // No replay não roda: as saídas do log original já trazem o modelo.
    private final TelescopioIOSim_9458 model =
            Logger.hasReplaySource() ? null : new TelescopioIOSim_9458();

    private final TelescopioIO_9458.TelescopioIOInputs_9458 modelInputs =
            new TelescopioIO_9458.TelescopioIOInputs_9458();
    private boolean modelReady;
    private double modelAppliedVolts;

    private static final Pose3d MODEL_BASE_POSE =
            new Pose3d(0, MODEL_VIEW_OFFSET_Y_METERS, 0, new Rotation3d());

    // #endregion


    // #region CONSTRUTOR

    public Telescopio_9458(TelescopioIO_9458 io) {

        this.io = Objects.requireNonNull(io);
        gains = io.isRealHardware() ? REAL_GAINS : SIM_GAINS;
        gainsInputs.set(gains);
        controller.setPID(gains.kP(), gains.kI(), gains.kD());

        motionMagic = USE_TALON_MOTION_MAGIC && io.supportsMotionMagic();
        profileLimits = motionMagic ? MOTION_MAGIC_LIMITS : ROBORIO_LIMITS;
        profileInputs.set(profileLimits);
        controller.setConstraints(normalConstraints());
        controller.setIntegratorRange(-INTEGRAL_MAX_VOLTS, INTEGRAL_MAX_VOLTS);

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
    public static TelescopioIO_9458 createIO() {

        if (Logger.hasReplaySource()) {
            return new TelescopioIO_9458() {};
        }

        if (RobotBase.isReal()) {
            return new TelescopioIOTalonFX_9458();
        }

        return new TelescopioIOSim_9458();
    }

    // #endregion


    // #region PERIODIC - ORDEM DE CADA CICLO (20 ms)

    @Override
    public void periodic() {
        readSensorsAndGains();
        stepParallelModel();
        boolean jamDetected = detectJam();
        boolean characterizationStallDetected = detectCharacterizationStall();
        runControl(jamDetected, characterizationStallDetected);
        feedParallelModel();
        logTelemetry();
    }

    // #endregion


    // #region 1. SENSORES E GANHOS

    private void readSensorsAndGains() {

        io.updateInputs(inputs);
        Logger.processInputs("Telescopio9458/Inputs", inputs);
        motionMagic = USE_TALON_MOTION_MAGIC && inputs.motionMagicAvailable;

        // No replay, os ganhos vêm do log, inclusive os ajustados pelo painel.
        Logger.processInputs("Telescopio9458/Ganhos", gainsInputs);
        TelescopioGains_9458 recordedGains = gainsInputs.get();
        if (!recordedGains.equals(gains)) {
            gains = recordedGains;
            controller.setPID(gains.kP(), gains.kI(), gains.kD());
        }

        // Idem para o perfil. Logs antigos não têm o campo (zero): mantém o padrão.
        Logger.processInputs("Telescopio9458/Perfil", profileInputs);
        if (profileInputs.velocity > 0 && profileInputs.acceleration > 0 && profileInputs.jerk > 0) {
            Limits recordedProfile = profileInputs.get();
            if (!recordedProfile.equals(profileLimits)) {
                profileLimits = recordedProfile;
                controller.setConstraints(normalConstraints());
            }
        }
    }

    // #endregion


    // #region 2. MODELO EM PARALELO - PASSO DA FISICA

    private void stepParallelModel() {

        if (model == null) {
            return;
        }

        modelAppliedVolts = 0;

        if (!hasValidPosition() || !Double.isFinite(inputs.appliedVolts)) {

            // Leitura inválida não pode contaminar o modelo.
            modelReady = false;

        } else if (DriverStation.isDisabled() || !modelReady) {

            // Desabilitado (ou recém-recuperado): alinha o modelo ao real.
            model.setState(inputs.extensionMeters, inputs.velocityMetersPerSec);
            modelInputs.extensionMeters = inputs.extensionMeters;
            modelInputs.velocityMetersPerSec = inputs.velocityMetersPerSec;
            modelInputs.motorCurrentAmps = 0;
            modelReady = true;

        } else {

            // Tensão medida corresponde ao intervalo anterior, como a leitura de posição.
            // No real, não aplicar novamente o limite de corrente simulado.
            // Com Motion Magic a tensão muda a cada 1 ms: a média medida no intervalo é a referência.
            if (io.isRealHardware() || motionMagic) model.setMeasuredVoltage(inputs.appliedVolts);
            model.updateInputs(modelInputs);
            modelAppliedVolts = modelInputs.appliedVolts;
        }
    }

    // #endregion


    // #region 3. DETECCAO DE TRAVAMENTO

    /** Travamento: corrente elevada e velocidade baixa durante JAM_TIME_SECONDS. */
    private boolean detectJam() {

        return jamDebouncer.calculate(
                positionControlActive
                && Math.abs(inputs.motorCurrentAmps) >= JAM_CURRENT_AMPS
                && Math.abs(inputs.velocityMetersPerSec) <= JAM_VELOCITY_METERS_PER_SEC
        );
    }

    /** SysId: mesmo critério, por SYSID_STALL_TIME_SECONDS (a rampa começa parada). */
    private boolean detectCharacterizationStall() {

        return characterizationStallDebouncer.calculate(
                characterizationActive
                && Math.abs(inputs.motorCurrentAmps) >= JAM_CURRENT_AMPS
                && Math.abs(inputs.velocityMetersPerSec) <= JAM_VELOCITY_METERS_PER_SEC
        );
    }

    // #endregion


    // #region 4. SEGURANCA E CONTROLE DO MOTOR

    private void runControl(boolean jamDetected, boolean characterizationStallDetected) {

        topLimitActive = false;

        // Segurança primeiro: qualquer uma destas condições corta a tensão.
        if (DriverStation.isDisabled()) {

            stop();
            status = "DESABILITADO";

        } else if (!hasValidPosition()) {

            stop();
            status = "SEM_REFERENCIA_OU_COMUNICACAO";

        } else if (!inputs.controlRequestOK) {

            stop();
            status = "FALHA_COMANDO_MOTOR";

        } else if (characterizationActive) {

            if (characterizationStallDetected) {
                // Corrente alta sem movimento: para e não empurra mais neste aperto.
                stop();
                characterizationStalled = true;
                status = "SYSID_TRAVADO";
            } else {
                driveCharacterization();
            }

        } else if (positionControlActive) {

            if (jamDetected && !jammed) {
                holdWhereJammed();
            }
            if (motionMagic) {
                driveWithMotionMagic();
            } else {
                driveToGoal();
            }

        } else {

            stayStopped();
        }
    }

    /** Para de empurrar, mas continua sustentando o peso onde travou. */
    private void holdWhereJammed() {

        jammed = true;

        goalMeters = MathUtil.clamp(
                inputs.extensionMeters,
                MIN_EXTENSION_METERS,
                MAX_OPERATING_EXTENSION_METERS
        );

        controller.reset(goalMeters, 0);
    }

    /** PID sobre o perfil trapezoidal + feedforward, limitado nas pontas do curso. */
    private void driveToGoal() {

        // PID: erro entre a medida e a referência do perfil.
        double previousVelocity = controller.getSetpoint().velocity;
        double feedback = controller.calculate(inputs.extensionMeters, profileGoal());
        var setpoint = controller.getSetpoint();

        // Feedforward: kS, kG, kV e kA. A aceleração vem do perfil neste passo.
        double acceleration = (setpoint.velocity - previousVelocity) / LOOP_PERIOD_SECONDS;
        double feedforward = gains.feedforward(setpoint.velocity, acceleration);
        feedbackVolts = feedback;
        feedforwardVolts = feedforward;

        // Limite de tensão e dos batentes.
        double clampedVolts = MathUtil.clamp(
                feedback + feedforward,
                -MAX_CONTROL_VOLTS,
                MAX_CONTROL_VOLTS
        );
        requestedVolts = limitAtEnds(
                clampedVolts,
                inputs.extensionMeters,
                goalMeters,
                setpoint.velocity,
                gains.kG()
        );
        topLimitActive =
                inputs.extensionMeters >= MAX_OPERATING_EXTENSION_METERS
                && requestedVolts < clampedVolts;

        io.setVoltage(requestedVolts);

        status = jammed
                ? "TRAVADO"
                : atGoal()
                    ? "MANTENDO_POSICAO"
                    : "MOVENDO";
    }

    /**
     * Motion Magic: o Talon gera a curva em S e fecha a malha a 1 kHz. Aqui só se escolhe
     * a meta e a velocidade de cruzeiro: normal, ou lenta nos últimos centímetros antes
     * de um batente. A troca para o trecho lento é decidida pela referência do próprio
     * Talon, com antecedência para frear (com jerk) até a velocidade lenta.
     */
    private void driveWithMotionMagic() {

        double referenceVelocity = referenceVelocity();
        referenceAcceleration = MathUtil.clamp(
                (referenceVelocity - previousReferenceVelocity) / LOOP_PERIOD_SECONDS,
                -profileLimits.acceleration(), profileLimits.acceleration());
        previousReferenceVelocity = referenceVelocity;

        slowApproach |= shouldStartSlowApproach();
        io.setMotionMagicPosition(goalMeters,
                slowApproach ? END_APPROACH_VELOCITY_METERS_PER_SEC : profileLimits.velocity());

        // Telemetria: tensão medida e parcelas que o Talon calculou.
        requestedVolts = inputs.appliedVolts;
        feedforwardVolts = inputs.motionMagicActive ? inputs.closedLoopFeedforwardVolts : 0;
        feedbackVolts = inputs.motionMagicActive
                ? inputs.closedLoopOutputVolts - inputs.closedLoopFeedforwardVolts : 0;

        status = jammed
                ? "TRAVADO"
                : atGoal()
                    ? "MANTENDO_POSICAO"
                    : "MOVENDO";
    }

    /** Indo para um batente: a referência precisa começar a frear para entrar devagar? */
    private boolean shouldStartSlowApproach() {

        if (!isEndGoal()) {
            return false;
        }
        double toGoal = goalMeters - referencePosition();
        double remaining = Math.abs(toGoal);
        if (remaining <= END_APPROACH_DISTANCE_METERS) {
            return true;
        }
        double direction = Math.signum(toGoal);
        double speed = direction * referenceVelocity();
        if (speed <= END_APPROACH_VELOCITY_METERS_PER_SEC) {
            return false;
        }
        double braking = TelescopioMotionProfile_9458.brakingDistance(speed,
                direction * referenceAcceleration, END_APPROACH_VELOCITY_METERS_PER_SEC, profileLimits);
        return remaining <= END_APPROACH_DISTANCE_METERS + braking
                + speed * END_APPROACH_LOOKAHEAD_SECONDS;
    }

    /** Fundo ou topo operacional. */
    private boolean isEndGoal() {
        return goalMeters <= MIN_EXTENSION_METERS + GOAL_EPSILON_METERS
                || goalMeters >= MAX_OPERATING_EXTENSION_METERS - GOAL_EPSILON_METERS;
    }

    /** Referência do perfil: a do Talon (Motion Magic) ou a do perfil no roboRIO. */
    private double referencePosition() {
        if (!motionMagic) {
            return controller.getSetpoint().position;
        }
        return inputs.motionMagicActive ? inputs.closedLoopReferenceMeters : inputs.extensionMeters;
    }

    private double referenceVelocity() {
        if (!motionMagic) {
            return controller.getSetpoint().velocity;
        }
        return inputs.motionMagicActive
                ? inputs.closedLoopReferenceVelocityMetersPerSec : inputs.velocityMetersPerSec;
    }

    private TrapezoidProfile.Constraints normalConstraints() {
        return new TrapezoidProfile.Constraints(profileLimits.velocity(), profileLimits.acceleration());
    }

    private TrapezoidProfile.Constraints endApproachConstraints() {
        return new TrapezoidProfile.Constraints(END_APPROACH_VELOCITY_METERS_PER_SEC, profileLimits.acceleration());
    }

    /**
     * Meta do perfil. Indo para um fim de curso (fundo ou topo operacional), o perfil
     * chega ao início da zona final a END_APPROACH_VELOCITY e percorre os últimos
     * END_APPROACH_DISTANCE devagar, para o telescópio não bater no batente.
     */
    private TrapezoidProfile.State profileGoal() {

        boolean toEnd = isEndGoal();
        var setpoint = controller.getSetpoint();
        double direction = Math.signum(goalMeters - setpoint.position);
        double zoneStart = goalMeters - direction * END_APPROACH_DISTANCE_METERS;

        if (toEnd && direction * (zoneStart - setpoint.position) > 0) {
            // Antes da zona final: velocidade normal, entrando nela a 6 cm/s.
            controller.setConstraints(normalConstraints());
            return new TrapezoidProfile.State(
                    zoneStart, direction * END_APPROACH_VELOCITY_METERS_PER_SEC);
        }

        // Na zona final, devagar. Só troca o limite com a referência já lenta:
        // reduzir o limite com o perfil rápido faria a velocidade cair de uma vez.
        boolean slow = toEnd
                && Math.abs(setpoint.velocity) <= END_APPROACH_VELOCITY_METERS_PER_SEC + 1e-6;
        controller.setConstraints(slow ? endApproachConstraints() : normalConstraints());
        return new TrapezoidProfile.State(goalMeters, 0);
    }

    /** SysId: aplica a tensão pedida pela rotina, sem PID nem feedforward. */
    private void driveCharacterization() {

        requestedVolts = limitCharacterization(characterizationVolts, inputs.extensionMeters);
        feedbackVolts = 0;
        feedforwardVolts = 0;

        io.setVoltage(requestedVolts);

        status = "SYSID";
    }

    /** Sem controle de posição ativo: tensão zero. */
    private void stayStopped() {

        requestedVolts = 0;
        feedbackVolts = 0;
        feedforwardVolts = 0;
        io.stop();

        // Limpa um estado de segurança que já passou (ex.: DESABILITADO depois de
        // habilitar). A recusa de meta continua visível enquanto o comando insiste nela.
        if (!status.equals("META_RECUSADA")) {
            status = "PARADO";
        }
    }

    // #endregion


    // #region 5. MODELO EM PARALELO - TENSAO DO PROXIMO PASSO

    /**
     * Simulação: o modelo recebe a mesma tensão solicitada ao IO simulado.
     * Hardware real: o modelo já usou a tensão medida em stepParallelModel().
     */
    private void feedParallelModel() {

        if (model != null && !io.isRealHardware() && !motionMagic) {
            model.setVoltage(requestedVolts);
        }
    }

    // #endregion


    // #region 6. TELEMETRIA (NOMES USADOS PELOS LAYOUTS DO ADVANTAGESCOPE)

    private void logTelemetry() {
        visualizer.update(inputs.extensionMeters, hasValidPosition());
        logControl();
        logDiagnostics();
        logTurnsAndLimits();
        logParallelModel();
        logFixedParameters();
    }

    private void logControl() {

        // Posição e controle
        Logger.recordOutput("Telescopio9458/MetaMetros", goalMeters);
        Logger.recordOutput("Telescopio9458/ReferenciaPerfilMetros", referencePosition());
        Logger.recordOutput("Telescopio9458/ErroMetros", goalMeters - inputs.extensionMeters);
        Logger.recordOutput("Telescopio9458/TensaoSolicitadaVolts", requestedVolts);
        Logger.recordOutput("Telescopio9458/ControleAtivo", positionControlActive);
        Logger.recordOutput("Telescopio9458/SysId/Ativo", characterizationActive);
        Logger.recordOutput("Telescopio9458/SysId/Travou", characterizationStalled);
        Logger.recordOutput("Telescopio9458/NaMeta", atGoal());
        Logger.recordOutput("Telescopio9458/Estado", status);

        Command command = getCurrentCommand();
        Logger.recordOutput("Telescopio9458/Comando", command == null ? "NENHUM" : command.getName());

        // Parcelas do controlador
        Logger.recordOutput("Telescopio9458/PID/FeedbackVolts", feedbackVolts);
        Logger.recordOutput("Telescopio9458/PID/FeedforwardVolts", feedforwardVolts);
        Logger.recordOutput("Telescopio9458/PID/ErroPerfilMetros",
                referencePosition() - inputs.extensionMeters);
        Logger.recordOutput("Telescopio9458/PID/ReferenciaVelocidade", referenceVelocity());
        Logger.recordOutput("Telescopio9458/Controle/TrechoLento", slowApproach);

        // Proteções
        Logger.recordOutput("Telescopio9458/Protecao/Travado", jammed);
        Logger.recordOutput("Telescopio9458/Protecao/LimiteTopoAtivo", topLimitActive);
    }

    private void logDiagnostics() {

        positionAlert.set(!hasValidPosition());
        controlAlert.set(!inputs.controlRequestOK);

        Logger.recordOutput("Telescopio9458/Diagnostico", getDiagnostic());
        Logger.recordOutput("Telescopio9458/Ajuste/Resultado", adjustmentStatus);
        Logger.recordOutput("Telescopio9458/Perfil/Resultado", profileStatus);
        Logger.recordOutput("Telescopio9458/Referencia/Resultado", referenceStatus);
        Logger.recordOutput(
                "Telescopio9458/Modo",
                Logger.hasReplaySource()
                        ? "REPLAY"
                        : RobotBase.isReal()
                            ? "ROBO_REAL_TALONFX_ID_" + MOTOR_CAN_ID
                            : "SIMULACAO_FISICA_APROXIMADA"
        );
    }

    /** Leituras sem o clamp do desenho 3D, inclusive para revelar ultrapassagens. */
    private void logTurnsAndLimits() {

        // Posição em voltas desde o zero.
        Logger.recordOutput("Telescopio9458/Voltas/Motor", inputs.rotorPositionRotations);
        Logger.recordOutput("Telescopio9458/Voltas/SaidaRedutor",
                inputs.rotorPositionRotations / GEAR_REDUCTION);

        // Extensão da ponta desde o zero, calculada pelo encoder e pela transmissão.
        Logger.recordOutput("Telescopio9458/Comparacao/FisicoMetros", inputs.extensionMeters);
        Logger.recordOutput("Telescopio9458/Comparacao/FisicoCentimetros", inputs.extensionMeters * 100);
        Logger.recordOutput("Telescopio9458/Comparacao/Valida", hasValidPosition() && modelReady);

        Logger.recordOutput("Telescopio9458/Limites/DistanciaBatenteCentimetros",
                (MAX_EXTENSION_METERS - inputs.extensionMeters) * 100);
    }

    private void logParallelModel() {

        Logger.recordOutput("Telescopio9458/Modelo/Valido", model != null && modelReady);
        Logger.recordOutput("Telescopio9458/Modelo/TensaoAplicadaVolts", modelAppliedVolts);
        Logger.recordOutput("Telescopio9458/Modelo/DiferencaVelocidade",
                inputs.velocityMetersPerSec - modelInputs.velocityMetersPerSec);
        Logger.recordOutput("Telescopio9458/Modelo/DiferencaCorrenteAmps",
                inputs.motorCurrentAmps - modelInputs.motorCurrentAmps);
        Logger.recordOutput("Telescopio9458/Modelo/ComparacaoAtiva",
                model != null && modelReady && !DriverStation.isDisabled());

        if (model == null) {
            return;
        }

        Logger.recordOutput("Telescopio9458/Modelo/FonteTensao",
                io.isRealHardware() ? "TENSAO_MEDIDA" : "TENSAO_SOLICITADA");

        // Voltas e extensão previstas pelo modelo
        double modelMotorRotations = modelInputs.extensionMeters * MOTOR_ROTATIONS_PER_METER;
        double differenceMeters = inputs.extensionMeters - modelInputs.extensionMeters;
        Logger.recordOutput("Telescopio9458/Modelo/VoltasMotor", modelMotorRotations);
        Logger.recordOutput("Telescopio9458/Modelo/VoltasSaidaRedutor",
                modelMotorRotations / GEAR_REDUCTION);
        Logger.recordOutput("Telescopio9458/Voltas/DiferencaSaidaRedutor",
                (inputs.rotorPositionRotations - modelMotorRotations) / GEAR_REDUCTION);
        Logger.recordOutput("Telescopio9458/Comparacao/SimuladoMetros", modelInputs.extensionMeters);
        Logger.recordOutput("Telescopio9458/Comparacao/SimuladoCentimetros", modelInputs.extensionMeters * 100);
        Logger.recordOutput("Telescopio9458/Comparacao/DiferencaMetros", differenceMeters);
        Logger.recordOutput("Telescopio9458/Comparacao/DiferencaCentimetros", differenceMeters * 100);

        // Telescópio virtual desenhado ao lado do real no 3D
        Logger.recordOutput("Telescopio9458/Modelo/Pose", MODEL_BASE_POSE);
        Logger.recordOutput("Telescopio9458/Modelo/Componentes",
                TelescopioVisualizer_9458.componentPoses(modelInputs.extensionMeters));
        Logger.recordOutput("Telescopio9458/Modelo/ExtensaoMetros", modelInputs.extensionMeters);
        Logger.recordOutput("Telescopio9458/Modelo/VelocidadeMetrosPorSeg", modelInputs.velocityMetersPerSec);
        Logger.recordOutput("Telescopio9458/Modelo/CorrenteAmps", modelInputs.motorCurrentAmps);
        Logger.recordOutput("Telescopio9458/Modelo/DiferencaMetros", differenceMeters);
    }

    /** Constantes publicadas para consulta nos gráficos do AdvantageScope. */
    private void logFixedParameters() {

        Logger.recordOutput("Telescopio9458/Controle/Versao", "MOTION_MAGIC_CURVA_S_1");
        Logger.recordOutput("Telescopio9458/Controle/Executor",
                motionMagic ? "TALON_MOTION_MAGIC" : "ROBORIO_PERFIL_TRAPEZOIDAL");
        Logger.recordOutput("Telescopio9458/Controle/VelocidadeMaximaMetrosPorSeg", profileLimits.velocity());
        Logger.recordOutput("Telescopio9458/Controle/AceleracaoMaximaMetrosPorSeg2", profileLimits.acceleration());
        Logger.recordOutput("Telescopio9458/Controle/JerkMetrosPorSeg3", motionMagic ? profileLimits.jerk() : 0);
        Logger.recordOutput("Telescopio9458/Controle/AproximacaoMetrosPorSeg", END_APPROACH_VELOCITY_METERS_PER_SEC);
        Logger.recordOutput("Telescopio9458/Controle/FaixaAproximacaoCentimetros", END_APPROACH_DISTANCE_METERS * 100);
        Logger.recordOutput("Telescopio9458/Voltas/Reducao", GEAR_REDUCTION);

        Logger.recordOutput("Telescopio9458/Limites/FisicoMetros", MAX_EXTENSION_METERS);
        Logger.recordOutput("Telescopio9458/Limites/OperacionalMetros", MAX_OPERATING_EXTENSION_METERS);
        Logger.recordOutput("Telescopio9458/Limites/TalonMetros", UPPER_SOFT_LIMIT_METERS);
        Logger.recordOutput("Telescopio9458/Limites/FisicoCentimetros", MAX_EXTENSION_METERS * 100);
        Logger.recordOutput("Telescopio9458/Limites/OperacionalCentimetros", MAX_OPERATING_EXTENSION_METERS * 100);
        Logger.recordOutput("Telescopio9458/Limites/TalonCentimetros", UPPER_SOFT_LIMIT_METERS * 100);
        Logger.recordOutput("Telescopio9458/Limites/MargemCentimetros", UPPER_SAFETY_MARGIN_METERS * 100);
        Logger.recordOutput("Telescopio9458/Limites/OperacionalVoltasMotor",
                MAX_OPERATING_EXTENSION_METERS * MOTOR_ROTATIONS_PER_METER);
        Logger.recordOutput("Telescopio9458/Limites/OperacionalVoltasSaidaRedutor",
                MAX_OPERATING_EXTENSION_METERS * MOTOR_ROTATIONS_PER_METER / GEAR_REDUCTION);

        Logger.recordOutput("Telescopio9458/Protecao/LimiteCorrenteEstatorAmps", STATOR_CURRENT_LIMIT_AMPS);

        Logger.recordOutput("Telescopio9458/Modelo/MassaEquivalenteKg", SIM_INITIAL_MOVING_MASS_KG);
        Logger.recordOutput("Telescopio9458/Modelo/RelacaoCaboPonta", SIM_CABLE_TO_TIP_RATIO);
        Logger.recordOutput("Telescopio9458/Modelo/Movimento", "CASCATA_ESTAGIOS_IGUAIS");
    }

    // #endregion


    // #region API - META E PARADA

    public void setGoalMeters(double meters) {

        if (!Double.isFinite(meters)
                || DriverStation.isDisabled()
                || !hasValidPosition()
                || !inputs.controlRequestOK) {

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
            previousReferenceVelocity = inputs.velocityMetersPerSec;
            referenceAcceleration = 0;
            slowApproach = false;
        }

        double newGoal = MathUtil.clamp(
                meters,
                MIN_EXTENSION_METERS,
                MAX_OPERATING_EXTENSION_METERS
        );
        // A frenagem ao soltar pode produzir 1,7e-18 m em vez de zero.
        // Canonizar evita religar a sustentação no batente por arredondamento.
        if (newGoal <= MIN_EXTENSION_METERS + GOAL_EPSILON_METERS) {
            newGoal = MIN_EXTENSION_METERS;
        }
        // O trecho lento vale só para a meta em que foi escolhido.
        if (Math.abs(newGoal - goalMeters) > GOAL_EPSILON_METERS) {
            slowApproach = false;
        }
        goalMeters = newGoal;

        characterizationActive = false;
        positionControlActive = true;
    }

    /**
     * Tensão zero.
     *
     * Manter uma altura exige controle
     * de posição ativo.
     */
    public void stop() {

        positionControlActive = false;
        slowApproach = false;
        characterizationActive = false;
        characterizationVolts = 0;
        jammed = false;
        requestedVolts = 0;
        feedbackVolts = 0;
        feedforwardVolts = 0;

        io.stop();

        status = "PARADO";
    }

    // #endregion


    // #region API - SYSID (TENSAO EM MALHA ABERTA)

    /**
     * Chamado a cada ciclo pela rotina de SysId. Desliga o PID e aplica a tensão
     * pura, para medir kS, kG, kV e kA. As proteções de segurança continuam
     * valendo, e a tensão é cortada nas paradas do SysId (perto das pontas).
     * stop() encerra o modo SysId.
     */
    public void runCharacterization(double volts) {

        if (!Double.isFinite(volts)
                || DriverStation.isDisabled()
                || !hasValidPosition()
                || !inputs.controlRequestOK) {

            stop();
            status = "SYSID_RECUSADO";
            return;
        }

        if (characterizationStalled) {
            stop();
            status = "SYSID_TRAVADO";
            return;
        }

        positionControlActive = false;
        jammed = false;
        characterizationActive = true;
        characterizationVolts = volts;
    }

    /** Novo aperto do botão de SysId: nova tentativa depois de um travamento. */
    public void clearCharacterizationStall() {
        characterizationStalled = false;
    }

    // #endregion


    // #region API - ZERO E GANHOS (SOMENTE DESABILITADO)

    /** Não é homing: o operador confirma que já recolheu fisicamente o mecanismo. */
    public boolean confirmRetractedZero() {
        if (!DriverStation.isDisabled()) {
            referenceStatus = "RECUSADO_ROBO_HABILITADO";
            return false;
        }
        stop();
        boolean accepted = io.confirmRetractedZero();
        referenceStatus = accepted ? "ZERO_CONFIRMADO" : "FALHA_AO_CONFIRMAR_ZERO";
        if (accepted) {
            controller.reset(0, 0);
            goalMeters = 0;
            modelReady = false;
        }
        return accepted;
    }

    /** Aplica o conjunto inteiro, somente desabilitado; nunca altera ganhos em movimento. */
    public boolean applyGains(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
        if (!DriverStation.isDisabled()) {
            adjustmentStatus = "RECUSADO_ROBO_HABILITADO";
            return false;
        }
        try {
            var updated = new TelescopioGains_9458(kP, kI, kD, kS, kG, kV, kA);
            if (kS > MAX_CONTROL_VOLTS || kG > MAX_CONTROL_VOLTS) {
                adjustmentStatus = "RECUSADO_KS_OU_KG_ACIMA_DO_LIMITE";
                return false;
            }
            // Com Motion Magic o PID roda no Talon: os ganhos precisam ser aceitos por ele
            // (o Talon não tem a proteção de integral do controlador WPILib: kI = 0).
            if (motionMagic && !io.configureGains(updated)) {
                adjustmentStatus = "RECUSADO_PELO_TALON";
                return false;
            }
            gains = updated;
            gainsInputs.set(gains);
            controller.setPID(kP, kI, kD);
            controller.reset(hasValidPosition() ? inputs.extensionMeters : 0, 0);
            adjustmentStatus = "GANHOS_APLICADOS";
            return true;
        } catch (IllegalArgumentException invalid) {
            adjustmentStatus = "RECUSADO_GANHOS_INVALIDOS";
            return false;
        }
    }

    /**
     * Velocidade (m/s), aceleração (m/s²) e jerk (m/s³) do perfil. Somente desabilitado,
     * dentro da faixa segura (PROFILE_*); no executor do roboRIO o jerk não é usado.
     */
    public boolean applyProfile(double velocity, double acceleration, double jerk) {
        if (!DriverStation.isDisabled()) {
            profileStatus = "RECUSADO_ROBO_HABILITADO";
            return false;
        }
        Limits limits;
        try {
            limits = new Limits(velocity, acceleration, jerk);
        } catch (IllegalArgumentException invalid) {
            profileStatus = "RECUSADO_VALORES_INVALIDOS";
            return false;
        }
        if (velocity > PROFILE_MAX_VELOCITY_METERS_PER_SEC
                || acceleration > PROFILE_MAX_ACCELERATION_METERS_PER_SEC_SQ
                || jerk < PROFILE_MIN_JERK_METERS_PER_SEC_CUBED
                || jerk > PROFILE_MAX_JERK_METERS_PER_SEC_CUBED) {
            profileStatus = "RECUSADO_FORA_DA_FAIXA";
            return false;
        }
        if (motionMagic && !io.configureMotionProfile(limits)) {
            profileStatus = "RECUSADO_PELO_TALON";
            return false;
        }
        profileLimits = limits;
        profileInputs.set(limits);
        controller.setConstraints(normalConstraints());
        slowApproach = false;
        profileStatus = "PERFIL_APLICADO";
        return true;
    }

    // #endregion


    // #region CONSULTAS - POSICAO VALIDA, META E DIAGNOSTICO

    /**
     * Uma pequena ultrapassagem dos batentes continua válida, para que o
     * comando de volta (LT/L2 no topo, RT/R2 embaixo) possa corrigir a posição.
     */
    public boolean hasValidPosition() {

        return inputs.motorConnected
                && inputs.positionValid
                && Double.isFinite(inputs.extensionMeters)
                && Double.isFinite(inputs.velocityMetersPerSec)
                && inputs.extensionMeters >=
                        MIN_EXTENSION_METERS - POSITION_VALID_MARGIN_METERS
                && inputs.extensionMeters <=
                        MAX_EXTENSION_METERS + POSITION_VALID_MARGIN_METERS;
    }

    public boolean atGoal() {

        return positionControlActive
                && !jammed
                && hasValidPosition()
                && Math.abs(goalMeters - inputs.extensionMeters)
                        <= POSITION_TOLERANCE_METERS
                && Math.abs(inputs.velocityMetersPerSec)
                        <= VELOCITY_TOLERANCE_METERS_PER_SEC
                && Math.abs(referencePosition() - goalMeters)
                        <= POSITION_TOLERANCE_METERS;
    }

    /** Primeiro motivo encontrado, na ordem em que deve ser investigado. */
    public String getDiagnostic() {
        if (!inputs.motorConnected) return "SEM_COMUNICACAO_CAN";
        if (inputs.referenceLostOnReset) return "TALON_REINICIOU_CONFIRMAR_ZERO_RECOLHIDO";
        if (!inputs.positionValid) return "CONFIGURACAO_OU_ZERO_PENDENTE";
        if (!Double.isFinite(inputs.extensionMeters) || !Double.isFinite(inputs.velocityMetersPerSec))
            return "LEITURA_NAO_FINITA";
        if (!hasValidPosition()) return "POSICAO_FORA_DA_FAIXA";
        if (!inputs.controlRequestOK) return "FALHA_COMANDO_MOTOR";
        if (inputs.extensionMeters < MIN_EXTENSION_METERS || inputs.extensionMeters > MAX_EXTENSION_METERS)
            return "POSICAO_FORA_DO_CURSO_VISUAL_LIMITADO";
        return "OK";
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

    /** Tensão medida no motor (no TalonFX, MotorVoltage). */
    public double getAppliedVolts() {
        return inputs.appliedVolts;
    }

    public boolean isPositionControlActive() {
        return positionControlActive;
    }

    public boolean isJammed() {
        return jammed;
    }

    public boolean isCharacterizationActive() {
        return characterizationActive;
    }

    public boolean isCharacterizationStalled() {
        return characterizationStalled;
    }

    /** Mesmo texto publicado em Telescopio9458/Estado. */
    public String getStatus() {
        return status;
    }

    public TelescopioGains_9458 getGains() {
        return gains;
    }

    public Limits getProfileLimits() {
        return profileLimits;
    }

    /** Motion Magic (Talon ou emulado) em vez do perfil no roboRIO. */
    public boolean usesMotionMagic() {
        return motionMagic;
    }

    /** Extensão prevista pelo modelo em paralelo; NaN no replay. */
    public double getModelExtensionMeters() {
        return model == null ? Double.NaN : modelInputs.extensionMeters;
    }

    // #endregion


    // #region COMANDOS - ABRIR E FECHAR SEGURANDO (RT / LT)

    /**
     * Segurar abre até a meta máxima, com margem antes do batente físico.
     * Ao soltar, freia pelo perfil e mantém a altura.
     */
    public Command extendWhileHeldCommand() {

        return moveWhileHeldCommand(
                MAX_OPERATING_EXTENSION_METERS,
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

    private Command moveWhileHeldCommand(
            double limitMeters,
            String name) {

        return startRun(
                this::clearJam,
                () -> setGoalMeters(limitMeters)
        )
                .finallyDo(() -> holdAtStoppingPoint(limitMeters))
                .withName(
                        "Telescopio 9458 - " + name + " (segurar)"
                );
    }

    // #endregion


    // #region COMANDOS - PASSOS DE 10 CM SEGURANDO (R1 / L1 NO MODO TEST)

    /**
     * Segurar sobe TEST_STEP_METERS a partir da altura atual e mantém.
     * Cada aperto é mais um passo; soltar antes freia pelo perfil e para ali.
     */
    public Command stepUpWhileHeldCommand() {

        return stepWhileHeldCommand(TEST_STEP_METERS, "subir 10 cm");
    }

    /** Igual ao passo para cima, descendo. */
    public Command stepDownWhileHeldCommand() {

        return stepWhileHeldCommand(-TEST_STEP_METERS, "descer 10 cm");
    }

    private Command stepWhileHeldCommand(
            double deltaMeters,
            String name) {

        // Alvo fixado ao apertar. Parte da meta mantida (passos exatos, sem acumular o
        // erro de posição) ou da posição medida, se o controle estava parado.
        double[] target = new double[1];

        return startRun(
                () -> {
                    clearJam();
                    double start = positionControlActive ? goalMeters : inputs.extensionMeters;
                    target[0] = MathUtil.clamp(
                            start + deltaMeters,
                            MIN_EXTENSION_METERS,
                            MAX_OPERATING_EXTENSION_METERS
                    );
                },
                () -> setGoalMeters(target[0])
        )
                .finallyDo(() -> holdAtStoppingPoint(target[0]))
                .withName(
                        "Telescopio 9458 - " + name + " (segurar)"
                );
    }

    // #endregion


    // #region COMANDOS - MOVER PARA POSICAO E ACOMPANHAR META

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
                        "Telescopio 9458 - mover para " + meters + " m"
                );
    }

    public Command followGoalCommand(DoubleSupplier goal) {

        return startRun(
                this::clearJam,
                () -> setGoalMeters(goal.getAsDouble())
        )
                .finallyDo(this::stop)
                .withName("Telescopio 9458 - acompanhar meta");
    }

    // #endregion


    // #region COMANDO - DEMONSTRACAO AUTOMATICA

    public Command demoCommand() {

        return Commands.sequence(

                moveToCommand(MAX_OPERATING_EXTENSION_METERS / 2),

                Commands.waitSeconds(DEMO_DWELL_SECONDS),

                moveToCommand(MAX_OPERATING_EXTENSION_METERS),

                Commands.waitSeconds(DEMO_DWELL_SECONDS),

                moveToCommand(MIN_EXTENSION_METERS),

                Commands.waitSeconds(DEMO_DWELL_SECONDS)

        )
                .repeatedly()
                .finallyDo(interrupted -> stop())
                .withName("Telescopio 9458 - ciclo automatico");
    }

    // #endregion


    // #region AUXILIARES - LIMITES, FRENAGEM E TRAVAMENTO

    /**
     * Limites de operação:
     *
     * Embaixo não empurra para baixo.
     * No limite superior com margem só sustenta o peso (kG).
     */
    static double limitAtEnds(
            double volts,
            double extensionMeters,
            double goalMeters,
            double setpointVelocity) {
        return limitAtEnds(volts, extensionMeters, goalMeters, setpointVelocity, SIM_KG);
    }

    static double limitAtEnds(double volts, double extensionMeters, double goalMeters,
            double setpointVelocity, double gravityVolts) {

        // Recolhido, apoia no limite inferior.
        if ((goalMeters <= MIN_EXTENSION_METERS + GOAL_EPSILON_METERS
                && extensionMeters <= RETRACTED_REST_TOLERANCE_METERS
                && setpointVelocity <= 0)
                || (extensionMeters <= MIN_EXTENSION_METERS
                && volts < 0)) {

            return 0;
        }

        // Na margem superior, limita a abertura à compensação de gravidade.
        // Continua permitindo corrigir a posição para baixo.
        if (extensionMeters >= MAX_OPERATING_EXTENSION_METERS
                && volts > gravityVolts) {

            return gravityVolts;
        }

        return volts;
    }

    /**
     * SysId: corta a subida a partir de SYSID_UPPER_STOP_METERS e a descida
     * a partir de SYSID_LOWER_STOP_METERS; a tensão no sentido contrário continua liberada.
     */
    static double limitCharacterization(double volts, double extensionMeters) {

        if ((volts > 0 && extensionMeters >= SYSID_UPPER_STOP_METERS)
                || (volts < 0 && extensionMeters <= SYSID_LOWER_STOP_METERS)) {

            return 0;
        }

        return MathUtil.clamp(volts, -MAX_CONTROL_VOLTS, MAX_CONTROL_VOLTS);
    }

    /**
     * Meta onde o perfil para com a
     * desaceleração máxima, sem voltar para trás.
     * Não passa da meta do comando que terminou: freando para ela, o perfil já para ali.
     */
    private void holdAtStoppingPoint(double commandGoalMeters) {

        if (!positionControlActive) {
            return;
        }

        double velocity = referenceVelocity();
        // Parada ou quase: o perfil está partindo rumo à meta anterior.
        double direction = Math.abs(velocity) > VELOCITY_DIRECTION_THRESHOLD_METERS_PER_SEC
                ? Math.signum(velocity)
                : Math.signum(goalMeters - referencePosition());
        double distance = motionMagic
                // Curva em S: rampas de jerk, aceleração do momento e o atraso até a meta
                // nova chegar ao Talon (até lá ele segue o cruzeiro em uso).
                ? TelescopioMotionProfile_9458.releaseDistance(
                        Math.abs(velocity), direction * referenceAcceleration,
                        slowApproach ? END_APPROACH_VELOCITY_METERS_PER_SEC : profileLimits.velocity(),
                        RELEASE_DELAY_SECONDS, profileLimits)
                : velocity * velocity / (2 * profileLimits.acceleration());

        double stop = referencePosition() + direction * distance;
        boolean commandGoalAhead = direction * (commandGoalMeters - referencePosition()) >= 0;
        if (commandGoalAhead && direction * (stop - commandGoalMeters) > 0) {
            stop = commandGoalMeters;
        }

        // Soltou no trecho lento: continua lento até parar, sem reacelerar perto do batente.
        boolean wasSlow = slowApproach;
        setGoalMeters(stop);
        slowApproach = wasSlow && positionControlActive;
    }

    /**
     * Cada comando novo é uma nova tentativa
     * depois de um travamento.
     */
    private void clearJam() {
        jammed = false;
    }

    // #endregion


    // #region ENCERRAMENTO E LIBERACAO DE RECURSOS

    @Override
    public void close() {

        stop();

        visualizer.close();
        positionAlert.close();
        controlAlert.close();
        io.close();

        CommandScheduler
                .getInstance()
                .unregisterSubsystem(this);
    }

    // #endregion
}
