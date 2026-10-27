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

// #endregion

/**
 * Shooter completo: pivô (gira o shooter), top, tower e centrífuga.
 *
 * DEPENDE DA POSE DO ROBÔ. A pose vem do Drive, que soma odometria das rodas,
 * Pigeon e correções da Limelight. Com a pose, o shooter sabe a distância e
 * a direção do hub; sem ela, não tem como mirar. Por isso:
 * - recebe a pose por Supplier (não acessa câmera nem motores do Drive);
 * - só atira depois que a visão corrigiu a pose ao menos uma vez.
 *
 * Fluxo de cada ciclo (periodic):
 * 1. lê os quatro IOs e grava as entradas (replay usa essas gravações);
 * 2. aplica as proteções (desabilitado, zero perdido);
 * 3. grava a telemetria e atualiza o desenho do AdvantageScope.
 * Os comandos (mirar, atirar) rodam depois e só pedem posições e RPMs.
 */
public class Shooter extends SubsystemBase implements AutoCloseable {

    // #region IO E ENTRADAS

    private final ShooterPivotIO pivotIO;
    private final ShooterWheelIO topIO;
    private final ShooterWheelIO towerIO;
    private final ShooterWheelIO centrifugeIO;

    private final ShooterPivotIOInputsAutoLogged pivotInputs = new ShooterPivotIOInputsAutoLogged();
    private final ShooterWheelIOInputsAutoLogged topInputs = new ShooterWheelIOInputsAutoLogged();
    private final ShooterWheelIOInputsAutoLogged towerInputs = new ShooterWheelIOInputsAutoLogged();
    private final ShooterWheelIOInputsAutoLogged centrifugeInputs = new ShooterWheelIOInputsAutoLogged();

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
    private double centrifugeTargetRpm;
    private String zeroStatus = "ZERADO_AO_LIGAR";

    // #endregion

    // #region CONSTRUÇÃO

    public Shooter(
        ShooterPivotIO pivotIO,
        ShooterWheelIO topIO,
        ShooterWheelIO towerIO,
        ShooterWheelIO centrifugeIO,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier,
        BooleanSupplier visionCorrectedSupplier
    ) {
        this(pivotIO, topIO, towerIO, centrifugeIO, poseSupplier, fieldSpeedsSupplier,
            visionCorrectedSupplier, Timer::getTimestamp);
    }

    /** Relógio injetável para os testes. */
    public Shooter(
        ShooterPivotIO pivotIO,
        ShooterWheelIO topIO,
        ShooterWheelIO towerIO,
        ShooterWheelIO centrifugeIO,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier,
        BooleanSupplier visionCorrectedSupplier,
        DoubleSupplier clock
    ) {
        this.pivotIO = Objects.requireNonNull(pivotIO);
        this.topIO = Objects.requireNonNull(topIO);
        this.towerIO = Objects.requireNonNull(towerIO);
        this.centrifugeIO = Objects.requireNonNull(centrifugeIO);
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
        return new ShooterWheelIOSim(TOP_MOTOR_COUNT, TOP_SIM_MOTOR, TOP_SIM_MOI_KG_M2,
            ShooterWheelIOSim.Controller.SPARK_FLEX);
    }

    public static ShooterWheelIO createTowerIO() {
        if (Logger.hasReplaySource()) return new ShooterWheelIO() {};
        if (RobotBase.isReal()) {
            return new ShooterWheelIOSparkFlex(new int[] { TOWER_MOTOR_ID }, new boolean[] { TOWER_INVERTED });
        }
        return new ShooterWheelIOSim(1, TOWER_SIM_MOTOR, TOWER_SIM_MOI_KG_M2,
            ShooterWheelIOSim.Controller.SPARK_FLEX);
    }

    public static ShooterWheelIO createCentrifugeIO() {
        if (Logger.hasReplaySource()) return new ShooterWheelIO() {};
        if (RobotBase.isReal()) return new ShooterWheelIOTalonFX();
        return new ShooterWheelIOSim(1, CENTRIFUGE_SIM_MOTOR, CENTRIFUGE_SIM_MOI_KG_M2,
            ShooterWheelIOSim.Controller.TALON_FX);
    }

    // #endregion

    // #region PERIODIC

    @Override
    public void periodic() {
        // 1. Sensores. processInputs grava no robô e lê do arquivo no replay.
        pivotIO.updateInputs(pivotInputs);
        topIO.updateInputs(topInputs);
        towerIO.updateInputs(towerInputs);
        centrifugeIO.updateInputs(centrifugeInputs);
        Logger.processInputs("Shooter/Pivo/Inputs", pivotInputs);
        Logger.processInputs("Shooter/Top/Inputs", topInputs);
        Logger.processInputs("Shooter/Tower/Inputs", towerInputs);
        Logger.processInputs("Shooter/Centrifuga/Inputs", centrifugeInputs);

        // 2. Proteções. Desabilitado: nada continua ao habilitar de novo.
        // Zero perdido: o pivô não sabe onde está o centro, então não move.
        if (DriverStation.isDisabled()) {
            stopAll();
        } else if (!pivotInputs.zeroValid && pivotMode != PivotMode.PARADO) {
            stopPivot();
        }

        // 3. Telemetria e desenho.
        logTelemetry();
    }

    // #endregion

    // #region MIRA (PIVÔ)

    /**
     * Calcula a mira até o alvo com a pose atual e envia ao pivô.
     * Chamado a cada ciclo pelos comandos enquanto o botão está apertado.
     */
    public Aim aimAt(Translation2d target) {
        aimTarget = target;
        Aim aim = ShotCalculator.aim(poseSupplier.get(), target);
        if (!pivotInputs.zeroValid) {
            stopPivot();
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

    /** Movimento manual lento: -1 a 1 vira até ±10% da tensão. */
    public void manualPivot(double input) {
        if (!pivotInputs.zeroValid) {
            stopPivot();
            return;
        }
        pivotMode = PivotMode.MANUAL;
        pivotIO.setPercentOutput(MathUtil.clamp(input, -1.0, 1.0) * PIVOT_MANUAL_PERCENT);
    }

    public void stopPivot() {
        pivotMode = PivotMode.PARADO;
        pivotIO.stop();
    }

    /** Erro menor que a tolerância, medindo no encoder (0,75°). */
    public boolean isPivotAtTarget() {
        return pivotMode == PivotMode.MIRANDO && pivotInputs.zeroValid
            && Math.abs(pivotTargetRotations - pivotInputs.motorPositionRotations)
                <= PIVOT_TOLERANCE_MOTOR_ROTATIONS;
    }

    public double getPivotDegrees() {
        return ShotCalculator.motorRotationsToPivotDegrees(pivotInputs.motorPositionRotations);
    }

    // #endregion

    // #region ZERO MANUAL

    /**
     * Define o centro atual como zero. Procedimento: robô DESABILITADO,
     * shooter centralizado à mão, apertar o botão. Habilitado é recusado:
     * zerar com o shooter em movimento criaria um centro errado.
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

    // #endregion

    // #region RODAS

    /** Top e tower juntos, como no código de competição. */
    public void setShotSpeeds(double topRpm, double towerRpm) {
        topTargetRpm = MathUtil.clamp(topRpm, 0.0, NEO_FREE_SPEED_RPM);
        towerTargetRpm = towerRpm;
        topIO.setVelocityRpm(topTargetRpm);
        towerIO.setVelocityRpm(towerTargetRpm);
    }

    public void setCentrifugeRpm(double rpm) {
        centrifugeTargetRpm = MathUtil.clamp(rpm, 0.0, KRAKEN_FREE_SPEED_RPM);
        centrifugeIO.setVelocityRpm(centrifugeTargetRpm);
    }

    public void stopCentrifuge() {
        centrifugeTargetRpm = 0.0;
        centrifugeIO.stop();
    }

    public void stopWheels() {
        topTargetRpm = 0.0;
        towerTargetRpm = 0.0;
        topIO.stop();
        towerIO.stop();
        stopCentrifuge();
    }

    public void stopAll() {
        stopWheels();
        stopPivot();
    }

    // Cada motor da roda precisa estar dentro da tolerância (o top tem dois).
    public boolean isTopAtSpeed() {
        return atSpeed(topInputs, topTargetRpm);
    }

    public boolean isTowerAtSpeed() {
        return atSpeed(towerInputs, towerTargetRpm);
    }

    public boolean isCentrifugeAtSpeed() {
        return atSpeed(centrifugeInputs, centrifugeTargetRpm);
    }

    private static boolean atSpeed(ShooterWheelIOInputsAutoLogged inputs, double targetRpm) {
        if (!inputs.connected || inputs.velocityRpm.length == 0) {
            return false;
        }
        for (double rpm : inputs.velocityRpm) {
            if (Math.abs(targetRpm - rpm) > VELOCITY_TOLERANCE_RPM) {
                return false;
            }
        }
        return true;
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

    /** Hub da aliança informada pela Driver Station (gravada no log). */
    public Translation2d getHubTarget() {
        return ShotCalculator.hubFor(DriverStation.getAlliance());
    }

    public double now() {
        return clock.getAsDouble();
    }

    public PivotMode getPivotMode() {
        return pivotMode;
    }

    public double getCentrifugeTargetRpm() {
        return centrifugeTargetRpm;
    }

    // #endregion

    // #region COMANDOS

    /** Segurar: o pivô segue o hub. Rodas paradas. */
    public Command aimAtHubCommand() {
        return run(() -> {
            if (isPoseTrusted()) {
                aimAt(getHubTarget());
            } else {
                stopPivot();
            }
        }).finallyDo(interrupted -> stopPivot())
            .withName("Shooter - mirar no hub");
    }

    /** Segurar: mira, acelera e alimenta quando tudo estiver pronto. */
    public Command trackAndShootCommand() {
        return new ShooterTrackAndShootCommand(this);
    }

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

    // #region TELEMETRIA (NOMES USADOS PELO LAYOUT FRC9460-Shooter)

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
        Logger.recordOutput("Shooter/Pivo/ZeroResultado", zeroStatus);
        Logger.recordOutput("Shooter/Pivo/LimiteGraus", PIVOT_MAX_ANGLE_DEGREES);
        Logger.recordOutput("Shooter/Pivo/LimiteMenosGraus", -PIVOT_MAX_ANGLE_DEGREES);

        Logger.recordOutput("Shooter/Mira/Alianca", DriverStation.getAlliance().map(Enum::name).orElse("SEM_ALIANCA"));
        Logger.recordOutput("Shooter/Mira/PoseConfiavel", isPoseTrusted());
        Logger.recordOutput("Shooter/Mira/Hub", new Pose2d(hub, pose.getRotation()));
        Logger.recordOutput("Shooter/Mira/AlvoVirtual", new Pose2d(moving.virtualTarget(), pose.getRotation()));
        Logger.recordOutput("Shooter/Mira/AlvoAtual", new Pose2d(aimTarget, pose.getRotation()));
        Logger.recordOutput("Shooter/Mira/DistanciaMetros", hubAim.distanceMeters());
        Logger.recordOutput("Shooter/Mira/AnguloNecessarioGraus", hubAim.desiredPivotDegrees());
        Logger.recordOutput("Shooter/Mira/HubNoCurso", hubAim.reachable());
        Logger.recordOutput("Shooter/Mira/DirecaoDoHubGraus", hubAim.fieldAngleDegrees());
        Logger.recordOutput("Shooter/Mira/DirecaoDoShooterGraus", shooterHeading);
        Logger.recordOutput("Shooter/Mira/ErroDirecaoGraus",
            MathUtil.inputModulus(hubAim.fieldAngleDegrees() - shooterHeading, -180.0, 180.0));
        Logger.recordOutput("Shooter/Mira/TempoDeVooSegundos", moving.flightTimeSeconds());

        // RPM de cada motor em chaves simples, para os gráficos do layout.
        Logger.recordOutput("Shooter/Top/AlvoRpm", topTargetRpm);
        Logger.recordOutput("Shooter/Top/RpmEsquerdo", rpm(topInputs, 0));
        Logger.recordOutput("Shooter/Top/RpmDireito", rpm(topInputs, 1));
        Logger.recordOutput("Shooter/Top/NoRpm", isTopAtSpeed());
        Logger.recordOutput("Shooter/Tower/AlvoRpm", towerTargetRpm);
        Logger.recordOutput("Shooter/Tower/Rpm", rpm(towerInputs, 0));
        Logger.recordOutput("Shooter/Tower/NoRpm", isTowerAtSpeed());
        Logger.recordOutput("Shooter/Centrifuga/AlvoRpm", centrifugeTargetRpm);
        Logger.recordOutput("Shooter/Centrifuga/Rpm", rpm(centrifugeInputs, 0));
        Logger.recordOutput("Shooter/Centrifuga/NoRpm", isCentrifugeAtSpeed());

        visualizer.update(pose, pivotDegrees, hubAim, hub, moving.virtualTarget());
    }

    private static double rpm(ShooterWheelIOInputsAutoLogged inputs, int motor) {
        return motor < inputs.velocityRpm.length ? inputs.velocityRpm[motor] : 0.0;
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
