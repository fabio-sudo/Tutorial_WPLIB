package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.shooter.ShotCalculator.Aim;
import frc.robot.subsystems.shooter.ShotCalculator.MovingTarget;

import frc.robot.util.MechanismDiagnostics;
import frc.robot.util.ParallelModel;

// #endregion

/**
 * Shooter: pivô (gira o shooter), top (rodas de disparo) e tower (alimentador).
 * A centrífuga, que empurra a bola até a tower, tem pasta própria
 * (subsystems/centrifuga); a sequência de tiro comanda as duas.
 *
 * DEPENDE DA POSE DO ROBÔ. A pose vem do Drive, que soma odometria das rodas,
 * Pigeon e correções da Limelight. Com a pose, o shooter sabe a distância e
 * a direção do hub; sem ela, não tem como mirar. Por isso:
 * - recebe a pose por Supplier (não acessa câmera nem motores do Drive);
 * - só atira depois que a visão corrigiu a pose ao menos uma vez.
 *
 * Fluxo de cada ciclo (periodic), no padrão do telescópio 9458:
 * 1. lê os três IOs e grava as entradas (replay usa essas gravações);
 * 2. avança o modelo em paralelo de cada motor (real x simulado);
 * 3. aplica a segurança (desabilitado, zero perdido);
 * 4. grava a telemetria e atualiza o desenho do AdvantageScope.
 * Os comandos (mirar, atirar) rodam depois e só pedem posições e RPMs.
 */
public class Shooter extends SubsystemBase implements AutoCloseable {

    // #region IO E ENTRADAS

    private final ShooterPivotIO pivotIO;
    private final ShooterWheelIO topIO;
    private final ShooterWheelIO towerIO;

    private final ShooterPivotIOInputsAutoLogged pivotInputs = new ShooterPivotIOInputsAutoLogged();
    private final ShooterWheelIOInputsAutoLogged topInputs = new ShooterWheelIOInputsAutoLogged();
    private final ShooterWheelIOInputsAutoLogged towerInputs = new ShooterWheelIOInputsAutoLogged();

    // #endregion

    // #region 🧪 MODELO EM PARALELO E DIAGNÓSTICO (PADRÃO DO TELESCÓPIO 9458)

    // A física de cada motor recebe a tensão medida: real x simulado.
    // O pivô do modelo não tem erro de zero manual; as rodas não têm bolas.
    private static final double PIVOT_DEGREES_PER_ROTATION = ShotCalculator.motorRotationsToPivotDegrees(1.0);

    private final ParallelModel pivotModel = new ParallelModel("Shooter/Pivo",
        () -> new ShooterPivotIOSim(0.0),
        ParallelModel.Unit.of("Graus", PIVOT_DEGREES_PER_ROTATION),
        ParallelModel.Unit.of("GrausPorSeg", PIVOT_DEGREES_PER_ROTATION),
        PIVOT_MODEL_TOLERANCE_DEGREES, PIVOT_MODEL_REST_DEGREES_PER_SEC);
    private final ParallelModel topModel = new ParallelModel("Shooter/Top",
        () -> new ShooterWheelIOSim(TOP_MOTOR_COUNT, TOP_SIM_MOTOR, TOP_SIM_MOI_KG_M2),
        null, ParallelModel.Unit.of("Rpm", 1.0), TOP_MODEL_TOLERANCE_RPM);
    private final ParallelModel towerModel = new ParallelModel("Shooter/Tower",
        () -> new ShooterWheelIOSim(1, TOWER_SIM_MOTOR, TOWER_SIM_MOI_KG_M2),
        null, ParallelModel.Unit.of("Rpm", 1.0), TOWER_MODEL_TOLERANCE_RPM);

    private final MechanismDiagnostics pivotDiagnostics = new MechanismDiagnostics(
        "Shooter/Pivo", "Pivô do shooter", "TALONFX_ID_" + PIVOT_MOTOR_ID);
    private final MechanismDiagnostics topDiagnostics = new MechanismDiagnostics(
        "Shooter/Top", "Top do shooter", "SPARKFLEX_ID_" + TOP_LEFT_MOTOR_ID + "_" + TOP_RIGHT_MOTOR_ID);
    private final MechanismDiagnostics towerDiagnostics = new MechanismDiagnostics(
        "Shooter/Tower", "Tower do shooter", "SPARKFLEX_ID_" + TOWER_MOTOR_ID);

    // #endregion

    // #region DEPENDÊNCIAS DO DRIVE

    // Pose fundida (odometria + Limelight), velocidade do robô no campo e
    // "a visão já corrigiu a pose desde o último reset".
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    private final BooleanSupplier visionCorrectedSupplier;

    // Relógio do AdvantageKit: no replay, repete os tempos gravados.
    private final DoubleSupplier clock;

    private final ShooterVisualizer visualizer = new ShooterVisualizer();

    // #endregion

    // #region ESTADO

    /** O que o pivô está fazendo agora. */
    public enum PivotMode { PARADO, MANUAL, MIRANDO }

    private PivotMode pivotMode = PivotMode.PARADO;
    private double pivotTargetRotations = 0.0;
    private Translation2d aimTarget = BLUE_HUB_CENTER;
    private double topTargetRpm;
    private double towerTargetRpm;
    private String zeroStatus = "ZERADO_AO_LIGAR";

    // #endregion

    // #region CONSTRUÇÃO

    public Shooter(
        ShooterPivotIO pivotIO,
        ShooterWheelIO topIO,
        ShooterWheelIO towerIO,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier,
        BooleanSupplier visionCorrectedSupplier
    ) {
        this(pivotIO, topIO, towerIO, poseSupplier, fieldSpeedsSupplier,
            visionCorrectedSupplier, Timer::getTimestamp);
    }

    /** Relógio injetável para os testes. */
    public Shooter(
        ShooterPivotIO pivotIO,
        ShooterWheelIO topIO,
        ShooterWheelIO towerIO,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier,
        BooleanSupplier visionCorrectedSupplier,
        DoubleSupplier clock
    ) {
        this.pivotIO = Objects.requireNonNull(pivotIO);
        this.topIO = Objects.requireNonNull(topIO);
        this.towerIO = Objects.requireNonNull(towerIO);
        this.poseSupplier = Objects.requireNonNull(poseSupplier);
        this.fieldSpeedsSupplier = Objects.requireNonNull(fieldSpeedsSupplier);
        this.visionCorrectedSupplier = Objects.requireNonNull(visionCorrectedSupplier);
        this.clock = Objects.requireNonNull(clock);
    }

    // #endregion

    // #region FÁBRICA DE IO - REAL / SIM / REPLAY

    // Replay: IO vazio, as entradas vêm do arquivo. Robô: hardware.
    // Computador: física. A ordem importa: o replay também roda no computador.

    public static ShooterPivotIO createPivotIO() {
        if (Logger.hasReplaySource()) return new ShooterPivotIO() {};
        if (RobotBase.isReal()) return new ShooterPivotIOTalonFX();
        return new ShooterPivotIOSim();
    }

    public static ShooterWheelIO createTopIO() {
        if (Logger.hasReplaySource()) return new ShooterWheelIO() {};
        if (RobotBase.isReal()) {
            return new ShooterWheelIOSparkFlex(
                new int[] { TOP_LEFT_MOTOR_ID, TOP_RIGHT_MOTOR_ID },
                new boolean[] { TOP_LEFT_INVERTED, TOP_RIGHT_INVERTED });
        }
        return new ShooterWheelIOSim(TOP_MOTOR_COUNT, TOP_SIM_MOTOR, TOP_SIM_MOI_KG_M2);
    }

    public static ShooterWheelIO createTowerIO() {
        if (Logger.hasReplaySource()) return new ShooterWheelIO() {};
        if (RobotBase.isReal()) {
            return new ShooterWheelIOSparkFlex(new int[] { TOWER_MOTOR_ID }, new boolean[] { TOWER_INVERTED });
        }
        return new ShooterWheelIOSim(1, TOWER_SIM_MOTOR, TOWER_SIM_MOI_KG_M2);
    }

    // #endregion

    // #region 🔁 PERIODIC - ORDEM DE CADA CICLO (20 ms)

    @Override
    public void periodic() {
        readSensors();
        stepParallelModels();
        applySafety();
        logTelemetry();
    }

    // #endregion

    // #region 🔁 1. SENSORES

    // processInputs grava no robô e lê do arquivo no replay.
    private void readSensors() {
        pivotIO.updateInputs(pivotInputs);
        topIO.updateInputs(topInputs);
        towerIO.updateInputs(towerInputs);
        Logger.processInputs("Shooter/Pivo/Inputs", pivotInputs);
        Logger.processInputs("Shooter/Top/Inputs", topInputs);
        Logger.processInputs("Shooter/Tower/Inputs", towerInputs);
    }

    // #endregion

    // #region 🧪 2. MODELO EM PARALELO - PASSO DA FÍSICA

    // Rodas com dois motores no mesmo eixo: média dos motores.
    private void stepParallelModels() {
        pivotModel.update(pivotInputs.connected && pivotInputs.zeroValid,
            pivotInputs.motorPositionRotations, pivotInputs.motorVelocityRotationsPerSec,
            pivotInputs.appliedVolts, pivotInputs.statorCurrentAmps);
        topModel.update(topInputs.connected, 0.0, mean(topInputs.velocityRpm),
            mean(topInputs.appliedVolts), mean(topInputs.currentAmps));
        towerModel.update(towerInputs.connected, 0.0, mean(towerInputs.velocityRpm),
            mean(towerInputs.appliedVolts), mean(towerInputs.currentAmps));
    }

    // #endregion

    // #region 🛑 3. SEGURANÇA - DESABILITADO E ZERO PERDIDO

    // Desabilitado: nada continua ao habilitar de novo.
    // Zero perdido: o pivô não sabe onde está o centro, então não move.
    private void applySafety() {
        if (DriverStation.isDisabled()) {
            stopAll();
        } else if (!pivotInputs.zeroValid && pivotMode != PivotMode.PARADO) {
            stopPivot();
        }
    }

    /** Antes de mover o pivô: sem zero válido, para e recusa. */
    private boolean pivotMayMove() {
        if (pivotInputs.zeroValid) {
            return true;
        }
        stopPivot();
        return false;
    }

    // #endregion

    // #region 🛑 SEGURANÇA - PARADAS

    public void stopPivot() {
        pivotMode = PivotMode.PARADO;
        pivotIO.stop();
    }

    /**
     * O top para solto (Coast). A tower FREIA (alvo 0 RPM): solta, ela girava
     * quase 2 s e continuava subindo bolas depois de soltar o botão (visto
     * nos logs de 27/10).
     */
    public void stopWheels() {
        topTargetRpm = 0.0;
        towerTargetRpm = 0.0;
        topIO.stop();
        towerIO.setVelocityRpm(0.0);
    }

    public void stopAll() {
        stopWheels();
        stopPivot();
    }

    // #endregion

    // #region 🎯🚀 CONTROLE - MIRA DO PIVÔ (MOTION MAGIC NO TALONFX)

    // O PID e o perfil rodam dentro do TalonFX (ganhos em ShooterConstants,
    // configuração em ShooterPivotIOTalonFX). Aqui só se escolhe o alvo.

    /**
     * Calcula a mira até o alvo com a pose atual e envia ao pivô.
     * Chamado a cada ciclo pelos comandos enquanto o botão está apertado.
     */
    public Aim aimAt(Translation2d target) {
        aimTarget = target;
        Aim aim = ShotCalculator.aim(poseSupplier.get(), target);
        if (!pivotMayMove()) {
            return aim;
        }
        double rotations = ShotCalculator.pivotDegreesToMotorRotations(aim.limitedPivotDegrees());
        boolean wasAiming = pivotMode == PivotMode.MIRANDO;
        pivotMode = PivotMode.MIRANDO;
        // Alvo quase igual ao anterior: o Talon continua no perfil que já tem.
        if (wasAiming && Math.abs(rotations - pivotTargetRotations) < PIVOT_RETARGET_MIN_MOTOR_ROTATIONS) {
            return aim;
        }
        pivotTargetRotations = rotations;
        pivotIO.setTargetRotations(rotations);
        return aim;
    }

    /** Movimento manual lento: -1 a 1 escalado pelo percentual nas constantes. */
    public void manualPivot(double input) {
        if (!pivotMayMove()) {
            return;
        }
        pivotMode = PivotMode.MANUAL;
        pivotIO.setPercentOutput(MathUtil.clamp(input, -1.0, 1.0) * PIVOT_MANUAL_PERCENT);
    }

    // #endregion

    // #region 🎯🚀 CONTROLE - RODAS (PID DE VELOCIDADE NO SPARK FLEX)

    // O PID roda dentro do Spark Flex (ganhos em ShooterConstants,
    // configuração em ShooterWheelIOSparkFlex). Aqui só se escolhe o RPM.

    // 🚀 Único ponto que manda RPM às rodas. 🛑 O top é limitado; a tower já vem limitada da curva.
    /** Top e tower juntos, como no código de competição. */
    public void setShotSpeeds(double topRpm, double towerRpm) {
        topTargetRpm = MathUtil.clamp(topRpm, 0.0, NEO_FREE_SPEED_RPM);
        towerTargetRpm = towerRpm;
        topIO.setVelocityRpm(topTargetRpm);
        towerIO.setVelocityRpm(towerTargetRpm);
    }

    // #endregion

    // #region 🧭 ZERO MANUAL

    /**
     * Define o centro atual como zero. Procedimento: robô DESABILITADO,
     * shooter centralizado à mão, apertar o botão. Habilitado é recusado:
     * zerar com o shooter em movimento criaria um centro errado.
     * true = pedido aceito; o resultado real aparece em getZeroResult().
     */
    public boolean zeroPivotHere() {
        if (!DriverStation.isDisabled()) {
            zeroStatus = "RECUSADO_ROBO_HABILITADO";
            return false;
        }
        pivotIO.zeroHere();
        pivotTargetRotations = 0.0;
        pivotMode = PivotMode.PARADO;
        zeroStatus = "ZERADO_PELA_EQUIPE";
        return true;
    }

    /** 🧭 Resultado do zero para o log: nunca diz "zerado" com o zero inválido. */
    public String getZeroResult() {
        return MechanismDiagnostics.zeroResult(pivotInputs.zeroValid, zeroStatus);
    }

    // #endregion

    // #region CONSULTAS - PIVÔ E RODAS NO ALVO

    /** Erro menor que a tolerância, medindo no encoder (0,75°). */
    public boolean isPivotAtTarget() {
        return pivotMode == PivotMode.MIRANDO && isPivotHealthy()
            && Math.abs(pivotTargetRotations - pivotInputs.motorPositionRotations)
                <= PIVOT_TOLERANCE_MOTOR_ROTATIONS;
    }

    public double getPivotDegrees() {
        return ShotCalculator.motorRotationsToPivotDegrees(pivotInputs.motorPositionRotations);
    }

    public double getModeledPivotDegrees() { return pivotModel.getModeledPositionConverted(); }

    // Cada motor da roda precisa estar dentro da tolerância (o top tem dois).
    public boolean isTopAtSpeed() {
        return atSpeed(topInputs, topTargetRpm);
    }

    public boolean isTowerAtSpeed() {
        return atSpeed(towerInputs, towerTargetRpm);
    }

    private static boolean atSpeed(ShooterWheelIOInputsAutoLogged inputs, double targetRpm) {
        if (!healthy(inputs) || !Double.isFinite(targetRpm)) {
            return false;
        }
        for (double rpm : inputs.velocityRpm) {
            if (Math.abs(targetRpm - rpm) > VELOCITY_TOLERANCE_RPM) {
                return false;
            }
        }
        return true;
    }

    private static boolean healthy(ShooterWheelIOInputsAutoLogged inputs) {
        if (!inputs.connected || inputs.velocityRpm.length == 0) return false;
        for (double rpm : inputs.velocityRpm) if (!Double.isFinite(rpm)) return false;
        return true;
    }

    public boolean areWheelsHealthy() {
        return healthy(topInputs) && healthy(towerInputs);
    }

    public boolean isPivotHealthy() {
        return pivotInputs.connected && pivotInputs.zeroValid
            && Double.isFinite(pivotInputs.motorPositionRotations);
    }

    // #endregion

    // #region CONSULTAS USADAS PELOS COMANDOS

    public Pose2d getPose() {
        return poseSupplier.get();
    }

    public ChassisSpeeds getFieldSpeeds() {
        return fieldSpeedsSupplier.get();
    }

    /** A pose já foi corrigida pela Limelight (ou a exigência foi desligada). */
    public boolean isPoseTrusted() {
        return !REQUIRE_VISION_CORRECTION || visionCorrectedSupplier.getAsBoolean();
    }

    /**
     * Com FMS, o hub da aliança; no treino, o hub de treino (PRACTICE_HUB_ALLIANCE).
     * A aliança e o FMS vêm da Driver Station e ficam gravados no log.
     */
    public Translation2d getHubTarget() {
        return ShotCalculator.hubFor(DriverStation.getAlliance(), DriverStation.isFMSAttached());
    }

    /** Do shooter ao hub, em metros (a distância que entra na curva de RPM). */
    public double getHubDistanceMeters() {
        return ShotCalculator.aim(poseSupplier.get(), getHubTarget()).distanceMeters();
    }

    /**
     * Direção do ALVO ATUAL (o hub, ou o alvo virtual de quem anda) menos a
     * direção REAL do shooter pelo encoder. 0 = apontando certo. É o erro de
     * mira que a bola leva no disparo (graus, de -180 a 180).
     */
    public double getAimErrorDegrees() {
        Pose2d pose = poseSupplier.get();
        Aim toTarget = ShotCalculator.aim(pose, aimTarget);
        double heading = ShotCalculator.shooterFieldHeadingDegrees(pose, getPivotDegrees());
        return MathUtil.inputModulus(toTarget.fieldAngleDegrees() - heading, -180.0, 180.0);
    }

    /** Qual hub e por quê, para o log e para o painel. */
    public String getHubDescription() {
        boolean fms = DriverStation.isFMSAttached();
        boolean blue = getHubTarget().equals(BLUE_HUB_CENTER);
        return (blue ? "AZUL" : "VERMELHO") + (fms ? "_ALIANCA_FMS" : "_TREINO_SEM_FMS");
    }

    public double now() {
        return clock.getAsDouble();
    }

    public PivotMode getPivotMode() {
        return pivotMode;
    }

    // RPM medido (média dos motores) e alvo: usados pela simulação das bolas
    // e pela detecção de disparo (queda de RPM do top quando a bola passa).
    public double getTopRpm() {
        return average(topInputs);
    }

    public double getTopTargetRpm() {
        return topTargetRpm;
    }

    public double getTowerTargetRpm() {
        return towerTargetRpm;
    }

    /** Ângulo que o pivô está tentando alcançar (o último alvo enviado). */
    public double getPivotTargetDegrees() {
        return ShotCalculator.motorRotationsToPivotDegrees(pivotTargetRotations);
    }

    public double getTowerRpm() {
        return average(towerInputs);
    }

    /**
     * Ângulo REAL do pivô para a simulação das bolas. Na simulação é o
     * ângulo físico (mostra o efeito de um zero manual errado); no robô,
     * o encoder, que é o que existe.
     */
    public double getPivotPhysicalDegrees() {
        return pivotIO instanceof ShooterPivotIOSim sim ? sim.physicalDegrees() : getPivotDegrees();
    }

    /** Simulação: a bola tira energia do top ao sair (o RPM cai um pouco). */
    public void simulateBallLeaving() {
        topIO.simulateBallPassing();
    }

    // #endregion

    // #region COMANDOS

    // Mirar e atirar usam mais de um subsistema ou têm lógica própria: ficam
    // em frc.robot.commands.shooter (ShooterAimAtHubCommand e
    // ShooterTrackAndShootCommand). Aqui só os comandos do próprio shooter.

    /** Segurar: gira devagar pelo eixo do controle (teste de sentido). */
    public Command manualPivotCommand(DoubleSupplier input) {
        return run(() -> manualPivot(input.getAsDouble()))
            .finallyDo(interrupted -> stopPivot())
            .withName("Shooter - pivô manual");
    }

    /** Desabilitado: o centro atual vira o zero. */
    public Command zeroPivotHereCommand() {
        return runOnce(this::zeroPivotHere)
            .ignoringDisable(true)
            .withName("Shooter - zerar pivô no centro");
    }

    // #endregion

    // #region 📊 4. TELEMETRIA (NOMES USADOS PELO LAYOUT FRC9460-Shooter)

    private void logTelemetry() {
        Pose2d pose = poseSupplier.get();
        Translation2d hub = getHubTarget();
        // A mira até o hub é calculada sempre, mesmo parado: o layout mostra
        // distância e ângulo necessários antes de apertar o botão.
        Aim hubAim = ShotCalculator.aim(pose, hub);
        MovingTarget moving = ShotCalculator.compensateForMotion(pose, fieldSpeedsSupplier.get(), hub);
        double pivotDegrees = getPivotDegrees();
        double shooterHeading = ShotCalculator.shooterFieldHeadingDegrees(pose, pivotDegrees);

        Logger.recordOutput("Shooter/Pivo/Modo", pivotMode.name());
        Logger.recordOutput("Shooter/Pivo/AnguloGraus", pivotDegrees);
        Logger.recordOutput("Shooter/Pivo/AlvoGraus",
            ShotCalculator.motorRotationsToPivotDegrees(pivotTargetRotations));
        Logger.recordOutput("Shooter/Pivo/ErroGraus",
            ShotCalculator.motorRotationsToPivotDegrees(pivotTargetRotations - pivotInputs.motorPositionRotations));
        Logger.recordOutput("Shooter/Pivo/AlvoVoltasMotor", pivotTargetRotations);
        Logger.recordOutput("Shooter/Pivo/NoAlvo", isPivotAtTarget());
        Logger.recordOutput("Shooter/Pivo/ZeroValido", pivotInputs.zeroValid);
        Logger.recordOutput("Shooter/Pivo/ZeroResultado", getZeroResult());
        // 🚧 ATENÇÃO (pente fino 31/10/2026): estes campos são constantes e custam tempo a cada ciclo. Dá para gravar
        // uma vez só (recordMetadata) depois de ajustar os layouts que ainda leem estas chaves.
        Logger.recordOutput("Shooter/Pivo/LimiteGraus", PIVOT_MAX_ANGLE_DEGREES);
        Logger.recordOutput("Shooter/Pivo/ReducaoTotal", PIVOT_GEAR_REDUCTION);
        Logger.recordOutput("Shooter/Pivo/GrausPorVoltaRotor", PIVOT_DEGREES_PER_MOTOR_ROTATION);
        Logger.recordOutput("Shooter/Pivo/LimiteMaisVoltasRotor", PIVOT_MAX_MOTOR_ROTATIONS);
        Logger.recordOutput("Shooter/Pivo/LimiteMenosVoltasRotor", -PIVOT_MAX_MOTOR_ROTATIONS);
        Logger.recordOutput("Shooter/Pivo/MargemAtePontoMedidoGraus",
            PIVOT_CALIBRATION_ANGLE_DEGREES - PIVOT_MAX_ANGLE_DEGREES);
        Logger.recordOutput("Shooter/Pivo/LimiteMenosGraus", -PIVOT_MAX_ANGLE_DEGREES);

        Logger.recordOutput("Shooter/Mira/Alianca", DriverStation.getAlliance().map(Enum::name).orElse("SEM_ALIANCA"));
        Logger.recordOutput("Shooter/Mira/HubEscolhido", getHubDescription());
        Logger.recordOutput("Shooter/Mira/PoseConfiavel", isPoseTrusted());
        Logger.recordOutput("Shooter/Mira/Hub", new Pose2d(hub, pose.getRotation()));
        Logger.recordOutput("Campo/Hub", new double[] {hub.getX(), hub.getY(), 0.0});
        Logger.recordOutput("Shooter/Mira/AlvoVirtual", new Pose2d(moving.virtualTarget(), pose.getRotation()));
        Logger.recordOutput("Shooter/Mira/AlvoAtual", new Pose2d(aimTarget, pose.getRotation()));
        Logger.recordOutput("Shooter/Mira/DistanciaMetros", hubAim.distanceMeters());
        Logger.recordOutput("Shooter/Mira/AnguloNecessarioGraus", hubAim.desiredPivotDegrees());
        Logger.recordOutput("Shooter/Mira/HubNoCurso", hubAim.reachable());
        Logger.recordOutput("Shooter/Mira/DirecaoDoHubGraus", hubAim.fieldAngleDegrees());
        Logger.recordOutput("Shooter/Mira/DirecaoDoShooterGraus", shooterHeading);
        Logger.recordOutput("Shooter/Mira/ErroDirecaoGraus",
            MathUtil.inputModulus(hubAim.fieldAngleDegrees() - shooterHeading, -180.0, 180.0));
        Logger.recordOutput("Shooter/Mira/ErroAteOAlvoAtualGraus", getAimErrorDegrees());
        Logger.recordOutput("Shooter/Mira/TempoDeVooSegundos", moving.flightTimeSeconds());

        // RPM de cada motor em chaves simples, para os gráficos do layout.
        Logger.recordOutput("Shooter/Top/AlvoRpm", topTargetRpm);
        Logger.recordOutput("Shooter/Top/RpmEsquerdo", rpm(topInputs, 0));
        Logger.recordOutput("Shooter/Top/RpmDireito", rpm(topInputs, 1));
        Logger.recordOutput("Shooter/Top/NoRpm", isTopAtSpeed());
        Logger.recordOutput("Shooter/Tower/AlvoRpm", towerTargetRpm);
        Logger.recordOutput("Shooter/Tower/Rpm", rpm(towerInputs, 0));
        Logger.recordOutput("Shooter/Tower/NoRpm", isTowerAtSpeed());

        visualizer.update(pose, pivotDegrees, hubAim, hub, moving.virtualTarget());
        logModelsAndDiagnostics();
    }

    private void logModelsAndDiagnostics() {
        Logger.recordOutput("Shooter/Comando", MechanismDiagnostics.commandName(this));
        pivotModel.log();
        topModel.log();
        towerModel.log();
        pivotDiagnostics.update(pivotInputs.connected, MechanismDiagnostics.result(
            pivotInputs.connected, pivotInputs.zeroValid, pivotModel.isDiverging()));
        topDiagnostics.update(topInputs.connected, MechanismDiagnostics.result(
            topInputs.connected, true, topModel.isDiverging()));
        towerDiagnostics.update(towerInputs.connected, MechanismDiagnostics.result(
            towerInputs.connected, true, towerModel.isDiverging()));
    }

    private static double rpm(ShooterWheelIOInputsAutoLogged inputs, int motor) {
        return motor < inputs.velocityRpm.length ? inputs.velocityRpm[motor] : 0.0;
    }

    private static double average(ShooterWheelIOInputsAutoLogged inputs) {
        return mean(inputs.velocityRpm);
    }

    private static double mean(double[] values) {
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return values.length == 0 ? 0.0 : sum / values.length;
    }

    // #endregion

    // #region ENCERRAMENTO (TESTES)

    @Override
    public void close() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
        visualizer.close();
    }

    // #endregion
}
