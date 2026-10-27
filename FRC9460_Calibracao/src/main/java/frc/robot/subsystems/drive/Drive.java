package frc.robot.subsystems.drive;

// #region IMPORTS
import static frc.robot.Constants.DriveConstants.*;
import org.littletonrobotics.junction.Logger;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

// #endregion
/** Um único estimador combina os módulos, o Pigeon e as câmeras. */
public class Drive extends SubsystemBase {

        // #region HISTÓRICO E CONTROLE TEMPORAL DA VISÃO
        // Timer.getTimestamp acompanha o relógio do AdvantageKit durante replay.
        private DoubleSupplier timestampSupplier = Timer::getTimestamp;
        private double firstOdometryTimestamp = Double.NaN;
        private double lastOdometryTimestamp = Double.NaN;
        private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;
        private final Map<String, Double> lastVisionByCamera = new HashMap<>();
        private final Map<String, Long> visionAcceptedByCamera = new HashMap<>();
        private long poseResetGeneration = 0;
        private String lastVisionResult = "NOT_REQUESTED";
        // A WPILib 2026 guarda 1,5 s. Usamos margem de 100 ms na consulta.
        private static final double SAFE_HISTORY_SECONDS = 1.4;
        // #endregion

        // #region VELOCIDADE DESEJADA DO CHASSI
        // Guarda a última ordem de movimento recebida pelo Drive
        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        private boolean xLockActive = false;
        // Referência do piloto separada do heading de campo usado pela visão.
        private Rotation2d driverHeadingOffset = new Rotation2d();
        private String fieldHeadingResetResult = "NOT_REQUESTED";
        private String driverHeadingResetResult = "NOT_REQUESTED";
        // #endregion

        // #region MODO CALIBRAÇÃO POR TENSÃO
        // Indica se estamos executando um teste direto de tensão.
        private boolean calibrationVoltageMode = false;
        // Tensão solicitada para os motores de Drive.
        private double calibrationDriveVolts = 0.0;

        // #region MODO CALIBRAÇÃO DIRETA DO STEER
        // ============================================================
        // MODO DE CALIBRAÇÃO DIRETA DO STEER
        // ============================================================
        //
        // Quando true, o periodic() deixa de utilizar temporariamente
        // os estados normais calculados pela cinemática e envia
        // diretamente o mesmo ângulo aos quatro módulos.
        //
        // Utilizado pelos testes determinísticos de:
        //
        // 0°
        // 45°
        // 90°
        // 180°
        //
        // sem depender da posição do joystick.
        //
        // ============================================================
        // Indica que o teste direto de ângulo está ativo.
        private boolean calibrationSteerMode = false;
        // Ângulo desejado para os quatro módulos durante o teste.
        private double calibrationSteerAngleRad = 0.0;
        // #endregion
        // ============================================================
        // ÂNGULO DO STEER DURANTE A CALIBRAÇÃO
        // ============================================================
        //
        // 0 rad = rodas apontadas para frente.
        //
        // IMPORTANTE:
        // Durante o kS não utilizaremos SwerveModuleState.optimize().
        //
        private static final double CALIBRATION_TURN_ANGLE_RAD = 0.0;

        // #region GIRO MANUAL ACUMULADO
        private double manualPreviousYawRad = 0.0;
        private double manualAccumulatedYawRad = 0.0;
        private boolean manualAccumulatedInitialized = false;
        // #endregion
        // ============================================================
        // TOLERÂNCIA DE ALINHAMENTO
        // ============================================================
        //
        // Só permitiremos tensão nos motores de Drive quando
        // TODOS os quatro módulos estiverem a no máximo
        // aproximadamente 3 graus da posição desejada.
        //
        private static final double CALIBRATION_TURN_TOLERANCE_RAD = Math.toRadians(3.0);
        // #endregion

        // #region REFERÊNCIA INDEPENDENTE DA SIMULAÇÃO
        // Referência cinemática dos sensores virtuais, sem correções de visão.
        // Não representa atrito, derrapagem ou colisões de um campo físico.
        private static final Pose2d SIM_INITIAL_POSE =
                        new Pose2d(2.0, 2.0, new Rotation2d());
        private boolean simulationReferenceEnabled = false;
        private final DriveStartupInputsAutoLogged startupInputs = new DriveStartupInputsAutoLogged();
        private boolean startupPoseApplied = false;
        private SwerveDriveOdometry simulationOdometry;
        private Pose2d simulationPose = SIM_INITIAL_POSE;
        // #endregion

        // #region POSE DO ROBÔ
        // Pose atual estimada pela odometria
        private Pose2d pose = new Pose2d(
                        0.0,
                        0.0,
                        new Rotation2d());
        // #endregion

        // #region POSIÇÃO DOS MÓDULOS NO CHASSI
        // FL - Front Left
        private final Translation2d frontLeftLocation = new Translation2d(
                        kWheelBaseMeters / 2.0,
                        kTrackWidthMeters / 2.0);
        // FR - Front Right
        private final Translation2d frontRightLocation = new Translation2d(
                        kWheelBaseMeters / 2.0,
                        -kTrackWidthMeters / 2.0);
        // BL - Back Left
        private final Translation2d backLeftLocation = new Translation2d(
                        -kWheelBaseMeters / 2.0,
                        kTrackWidthMeters / 2.0);
        // BR - Back Right
        private final Translation2d backRightLocation = new Translation2d(
                        -kWheelBaseMeters / 2.0,
                        -kTrackWidthMeters / 2.0);
        // #endregion

        // #region CINEMÁTICA DO SWERVE
        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        frontLeftLocation,
                        frontRightLocation,
                        backLeftLocation,
                        backRightLocation);
        // #endregion

        // #region POSE ESTIMATOR
        // ============================================================
        // ESTIMADOR DE POSIÇÃO DO SWERVE
        // ============================================================
        //
        // Combina Pigeon 2, posição dos módulos e medições MegaTag2.
        // Vision valida e organiza as câmeras; somente Drive altera o estimador.
        //
        // ============================================================
        private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                        // Geometria do Swerve
                        kinematics,
                        // Orientação inicial do robô
                        new Rotation2d(),
                        // Posições iniciais dos módulos
                        new SwerveModulePosition[] {
                                        new SwerveModulePosition(),
                                        new SwerveModulePosition(),
                                        new SwerveModulePosition(),
                                        new SwerveModulePosition()
                        },
                        // Pose inicial
                        new Pose2d(
                                        0.0,
                                        0.0,
                                        new Rotation2d()));
        // #endregion

        // #region MÓDULOS SWERVE
        // 0 = Front Left
        private final Module frontLeft;
        // 1 = Front Right
        private final Module frontRight;
        // 2 = Back Left
        private final Module backLeft;
        // 3 = Back Right
        private final Module backRight;
        // #endregion

        // #region GYRO
        // A implementação será escolhida
        // no construtor do Drive:
        //
        // SIM -> GyroIOSim
        // REAL -> GyroIOPigeon2
        private final GyroIO gyroIO;
        // Dados do gyro registrados
        // pelo AdvantageKit
        private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
        // #endregion

        // #region CONSTRUTOR
        public Drive() {
                // ============================================================
                // ROBÔ REAL
                // ============================================================
                // Replay não cria dispositivos CAN nem integra sensores simulados.
                // Robot.java precisa configurar a fonte de replay antes do container.
                if (Logger.hasReplaySource()) {
                        frontLeft = new Module(new ModuleIO() {
                        }, 0);
                        frontRight = new Module(new ModuleIO() {
                        }, 1);
                        backLeft = new Module(new ModuleIO() {
                        }, 2);
                        backRight = new Module(new ModuleIO() {
                        }, 3);
                        gyroIO = new GyroIO() {
                        };
                } else if (RobotBase.isReal()) {
                        // --------------------------------------------------------
                        // FRONT LEFT
                        // --------------------------------------------------------
                        frontLeft = new Module(
                                        new ModuleIOTalonFX(
                                                        13, // Drive Motor ID
                                                        11, // Steer Motor ID
                                                        12, // CANcoder ID
                                                        -0.067626953125, // Offset do CANcoder
                                                        false, // Drive invertido?
                                                        true, // Steer invertido?
                                                        false // CANcoder invertido?
                                        ),
                                        0);
                        // --------------------------------------------------------
                        // FRONT RIGHT
                        // --------------------------------------------------------
                        frontRight = new Module(
                                        new ModuleIOTalonFX(
                                                        19, // Drive Motor ID
                                                        20, // Steer Motor ID
                                                        18, // CANcoder ID
                                                        -0.204833984375, // Offset do CANcoder
                                                        true, // Drive invertido?
                                                        true, // Steer invertido?
                                                        false // CANcoder invertido?
                                        ),
                                        1);
                        // --------------------------------------------------------
                        // BACK LEFT
                        // --------------------------------------------------------
                        backLeft = new Module(
                                        new ModuleIOTalonFX(
                                                        17, // Drive Motor ID
                                                        16, // Steer Motor ID
                                                        21, // CANcoder ID
                                                        -0.291259765625, // Offset do CANcoder
                                                        false, // Drive invertido?
                                                        true, // Steer invertido?
                                                        false // CANcoder invertido?
                                        ),
                                        2);
                        // --------------------------------------------------------
                        // BACK RIGHT
                        // --------------------------------------------------------
                        backRight = new Module(
                                        new ModuleIOTalonFX(
                                                        14, // Drive Motor ID
                                                        10, // Steer Motor ID
                                                        15, // CANcoder ID
                                                        0.360595703125, // Offset do CANcoder
                                                        true, // Drive invertido?
                                                        true, // Steer invertido?
                                                        false // CANcoder invertido?
                                        ),
                                        3);
                        // ============================================================
                        // GYRO REAL
                        // ============================================================
                        // Cria o Pigeon 2 real do drivetrain
                        // CAN ID = 3
                        gyroIO = new GyroIOPigeon2(
                                        3);
                }
                // ============================================================
                // SIMULAÇÃO
                // ============================================================
                else {
                        // Front Left
                        frontLeft = new Module(
                                        new ModuleIOSim(),
                                        0);
                        // Front Right
                        frontRight = new Module(
                                        new ModuleIOSim(),
                                        1);
                        // Back Left
                        backLeft = new Module(
                                        new ModuleIOSim(),
                                        2);
                        // Back Right
                        backRight = new Module(
                                        new ModuleIOSim(),
                                        3);
                        // --------------------------------------------------------
                        // GYRO SIMULADO
                        // --------------------------------------------------------
                        gyroIO = new GyroIOSim();
                        // Somente o Drive desktop normal ativa esta referência.
                        // Robô real, replay e sensores injetados mantêm sua inicialização.
                        simulationReferenceEnabled = true;
                }
        }

        // #endregion
        // Permite validar a lógica com entradas controladas, sem hardware real.
        public Drive(GyroIO gyroIO, ModuleIO frontLeftIO, ModuleIO frontRightIO,
                        ModuleIO backLeftIO, ModuleIO backRightIO) {
                this(gyroIO, frontLeftIO, frontRightIO, backLeftIO, backRightIO,
                                Timer::getTimestamp);
        }

        /**
         * Permite testar a fusão sem hardware, usando sensores e relógio controlados.
         */
        public Drive(GyroIO gyroIO, ModuleIO frontLeftIO, ModuleIO frontRightIO,
                        ModuleIO backLeftIO, ModuleIO backRightIO, DoubleSupplier clock) {
                timestampSupplier = Objects.requireNonNull(clock);
                this.gyroIO = Objects.requireNonNull(gyroIO);
                frontLeft = new Module(frontLeftIO, 0);
                frontRight = new Module(frontRightIO, 1);
                backLeft = new Module(backLeftIO, 2);
                backRight = new Module(backRightIO, 3);
        }

        // #region PERIODIC
        @Override
        public void periodic() {
                // ==========================================================
                // 1 - ATUALIZAÇÃO DOS MÓDULOS
                // ==========================================================
                frontLeft.periodic();
                frontRight.periodic();
                backLeft.periodic();
                backRight.periodic();
                // ==========================================================
                // 2 - MOVIMENTO REAL MEDIDO PELOS MÓDULOS
                // ==========================================================
                // Converte o movimento REAL das quatro rodas
                // para o movimento do chassi
                ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(
                                getModuleStates());
                // ==========================================================
                // 3 - GYRO SIMULADO
                // ==========================================================
                // Usa a rotação REAL calculada pelos módulos
                gyroIO.setSimYawVelocity(
                                measuredChassisSpeeds.omegaRadiansPerSecond);
                // Atualiza o sensor
                gyroIO.updateInputs(
                                gyroInputs);
                // Registra no AdvantageKit
                Logger.processInputs(
                                "Drive/Gyro",
                                gyroInputs);
                // ==========================================================
                // GIRO MANUAL ACUMULADO
                // ==========================================================
                //
                // Funciona durante o controle normal do robô.
                // Soma a pequena variação do Pigeon a cada ciclo de 20 ms
                // e mostra no AdvantageScope os graus e voltas acumulados.
                //
                // ==========================================================
                double manualCurrentYawRad = gyroInputs.yawPositionRad;
                if (!isGyroHealthy()) {
                        manualAccumulatedInitialized = false;
                } else if (!manualAccumulatedInitialized) {
                        manualPreviousYawRad = manualCurrentYawRad;
                        manualAccumulatedInitialized = true;
                } else {
                        double manualDeltaYawRad = MathUtil.angleModulus(
                                        manualCurrentYawRad
                                                        - manualPreviousYawRad);
                        manualAccumulatedYawRad += manualDeltaYawRad;
                        manualPreviousYawRad = manualCurrentYawRad;
                }
                double manualAccumulatedDegrees = Math.toDegrees(
                                manualAccumulatedYawRad);
                double manualAccumulatedTurns = manualAccumulatedDegrees
                                / 360.0;
                Logger.recordOutput(
                                "Drive/GyroManual/AccumulatedDegrees",
                                manualAccumulatedDegrees);
                Logger.recordOutput(
                                "Drive/GyroManual/AccumulatedTurns",
                                manualAccumulatedTurns);
                Logger.recordOutput(
                                "Drive/GyroManual/AccumulatedDegreesAbsolute",
                                Math.abs(
                                                manualAccumulatedDegrees));
                Logger.recordOutput(
                                "Drive/GyroManual/AccumulatedTurnsAbsolute",
                                Math.abs(
                                                manualAccumulatedTurns));
                Logger.recordOutput(
                                "Drive/GyroManual/CurrentYawDegrees",
                                Math.toDegrees(
                                                gyroInputs.yawPositionRad));
                // ==========================================================
                // A escolha inicial também é um input: no replay ela vem do log.
                // Não usamos Robot/Pose gravada para alimentar o novo estimador.
                updateStartupPose();
                // Sensores já foram lidos. Atualiza a referência antes do estimador.
                updateSimulationReference();
                // 4 - ODOMETRIA
                // 4 - ATUALIZAÇÃO DO POSE ESTIMATOR
                // ==========================================================
                //
                // Atualiza a posição utilizando:
                //
                // - Pigeon 2
                // - Encoders dos quatro módulos
                //
                // A atualização usa rodas e gyro; Vision adiciona medições ao estimador.
                //
                // ==========================================================
                if (isGyroHealthy()) {
                        double now = timestampSupplier.getAsDouble();
                        // Mesmo relógio utilizado na fila da visão e no replay.
                        if (Double.isFinite(now)
                                        && (!Double.isFinite(lastOdometryTimestamp) || now >= lastOdometryTimestamp)) {
                                pose = poseEstimator.updateWithTime(now, getRawGyroRotation(), getModulePositions());
                                if (!Double.isFinite(firstOdometryTimestamp)) {
                                        firstOdometryTimestamp = now;
                                }
                                lastOdometryTimestamp = now;
                        }
                }
                // Também zera saídas na simulação. Disabled nunca mantém um teste ativo.
                if (DriverStation.isDisabled()) {
                        calibrationVoltageMode = false;
                        calibrationSteerMode = false;
                        calibrationDriveVolts = 0.0;
                        xLockActive = false;
                        stop();
                }
                // ==========================================================
                // 5 - CINEMÁTICA DESEJADA
                // ==========================================================
                // Converte a ordem de movimento do chassi
                // em quatro estados de módulos
                SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
                                chassisSpeeds);
                // ==========================================================
                // 6 - LIMITA VELOCIDADE DAS RODAS
                // ==========================================================
                SwerveDriveKinematics.desaturateWheelSpeeds(
                                moduleStates,
                                kMaxLinearSpeed);
                // ==========================================================
                // 7 - COMANDA OS QUATRO MÓDULOS
                // ==========================================================
                if (DriverStation.isDisabled()) {
                        frontLeft.setDriveVoltage(0.0);
                        frontRight.setDriveVoltage(0.0);
                        backLeft.setDriveVoltage(0.0);
                        backRight.setDriveVoltage(0.0);
                        frontLeft.setTurnVoltage(0.0);
                        frontRight.setTurnVoltage(0.0);
                        backLeft.setTurnVoltage(0.0);
                        backRight.setTurnVoltage(0.0);
                } else if (calibrationVoltageMode) {
                        // ==========================================================
                        // MODO CALIBRAÇÃO POR TENSÃO
                        // ==========================================================
                        //
                        // Durante este modo:
                        //
                        // 1. Os quatro Steers são enviados diretamente para 0°.
                        //
                        // 2. NÃO utilizamos SwerveModuleState.optimize().
                        //
                        // 3. Verificamos se TODOS chegaram perto de 0°.
                        //
                        // 4. Somente depois liberamos tensão para os Drives.
                        //
                        // ==========================================================
                        // ==========================================================
                        // 1 - ALINHA OS QUATRO STEERS
                        // ==========================================================
                        frontLeft.setCalibrationTurnPosition(
                                        CALIBRATION_TURN_ANGLE_RAD);
                        frontRight.setCalibrationTurnPosition(
                                        CALIBRATION_TURN_ANGLE_RAD);
                        backLeft.setCalibrationTurnPosition(
                                        CALIBRATION_TURN_ANGLE_RAD);
                        backRight.setCalibrationTurnPosition(
                                        CALIBRATION_TURN_ANGLE_RAD);
                        // ==========================================================
                        // 2 - VERIFICA O ALINHAMENTO DE CADA MÓDULO
                        // ==========================================================
                        boolean frontLeftAligned = frontLeft.isCalibrationTurnAligned(
                                        CALIBRATION_TURN_ANGLE_RAD,
                                        CALIBRATION_TURN_TOLERANCE_RAD);
                        boolean frontRightAligned = frontRight.isCalibrationTurnAligned(
                                        CALIBRATION_TURN_ANGLE_RAD,
                                        CALIBRATION_TURN_TOLERANCE_RAD);
                        boolean backLeftAligned = backLeft.isCalibrationTurnAligned(
                                        CALIBRATION_TURN_ANGLE_RAD,
                                        CALIBRATION_TURN_TOLERANCE_RAD);
                        boolean backRightAligned = backRight.isCalibrationTurnAligned(
                                        CALIBRATION_TURN_ANGLE_RAD,
                                        CALIBRATION_TURN_TOLERANCE_RAD);
                        // ==========================================================
                        // 3 - TODOS PRECISAM ESTAR ALINHADOS
                        // ==========================================================
                        boolean allModulesAligned = frontLeftAligned
                                        && frontRightAligned
                                        && backLeftAligned
                                        && backRightAligned;
                        // ==========================================================
                        // 4 - TRAVA DE SEGURANÇA
                        // ==========================================================
                        //
                        // Se QUALQUER módulo estiver fora do alinhamento:
                        //
                        // tensão aplicada = 0 V
                        //
                        // Somente depois que TODOS estiverem alinhados
                        // a tensão solicitada pelo teste será liberada.
                        //
                        // ==========================================================
                        double appliedCalibrationVolts = allModulesAligned
                                        ? calibrationDriveVolts
                                        : 0.0;
                        // ==========================================================
                        // 5 - APLICA TENSÃO AOS QUATRO DRIVES
                        // ==========================================================
                        frontLeft.setDriveVoltage(
                                        appliedCalibrationVolts);
                        frontRight.setDriveVoltage(
                                        appliedCalibrationVolts);
                        backLeft.setDriveVoltage(
                                        appliedCalibrationVolts);
                        backRight.setDriveVoltage(
                                        appliedCalibrationVolts);
                        // ==========================================================
                        // 6 - LOGS DE SEGURANÇA
                        // ==========================================================
                        // Tensão que o teste está PEDINDO
                        Logger.recordOutput(
                                        "Calibration/DriveVoltageRequested",
                                        calibrationDriveVolts);
                        // Tensão que realmente foi LIBERADA
                        Logger.recordOutput(
                                        "Calibration/DriveVoltageApplied",
                                        appliedCalibrationVolts);
                        // Estado geral do alinhamento
                        Logger.recordOutput(
                                        "Calibration/SteerAligned",
                                        allModulesAligned);
                        // Estado individual
                        Logger.recordOutput(
                                        "Calibration/FrontLeftAligned",
                                        frontLeftAligned);
                        Logger.recordOutput(
                                        "Calibration/FrontRightAligned",
                                        frontRightAligned);
                        Logger.recordOutput(
                                        "Calibration/BackLeftAligned",
                                        backLeftAligned);
                        Logger.recordOutput(
                                        "Calibration/BackRightAligned",
                                        backRightAligned);
                }
                // ==========================================================
                // MODO CALIBRAÇÃO DIRETA DO STEER
                // ==========================================================
                //
                // Neste modo os Drives ficam parados e somente os motores
                // de Steer recebem posição. Isso permite testar 0°, 45°,
                // 90° e 180° de forma repetível, sem joystick.
                //
                // ==========================================================
                else if (calibrationSteerMode) {
                        // ======================================================
                        // 1 - GARANTE QUE OS DRIVES FIQUEM PARADOS
                        // ======================================================
                        frontLeft.setDriveVoltage(
                                        0.0);
                        frontRight.setDriveVoltage(
                                        0.0);
                        backLeft.setDriveVoltage(
                                        0.0);
                        backRight.setDriveVoltage(
                                        0.0);
                        // ======================================================
                        // 2 - ENVIA O MESMO ÂNGULO PARA OS QUATRO STEERS
                        // ======================================================
                        frontLeft.setCalibrationTurnPosition(
                                        calibrationSteerAngleRad);
                        frontRight.setCalibrationTurnPosition(
                                        calibrationSteerAngleRad);
                        backLeft.setCalibrationTurnPosition(
                                        calibrationSteerAngleRad);
                        backRight.setCalibrationTurnPosition(
                                        calibrationSteerAngleRad);
                        // ======================================================
                        // 3 - VERIFICA O ALINHAMENTO DE CADA MÓDULO
                        // ======================================================
                        boolean frontLeftAligned = frontLeft.isCalibrationTurnAligned(
                                        calibrationSteerAngleRad,
                                        CALIBRATION_TURN_TOLERANCE_RAD);
                        boolean frontRightAligned = frontRight.isCalibrationTurnAligned(
                                        calibrationSteerAngleRad,
                                        CALIBRATION_TURN_TOLERANCE_RAD);
                        boolean backLeftAligned = backLeft.isCalibrationTurnAligned(
                                        calibrationSteerAngleRad,
                                        CALIBRATION_TURN_TOLERANCE_RAD);
                        boolean backRightAligned = backRight.isCalibrationTurnAligned(
                                        calibrationSteerAngleRad,
                                        CALIBRATION_TURN_TOLERANCE_RAD);
                        // ======================================================
                        // 4 - VERIFICA SE TODOS CHEGARAM AO ALVO
                        // ======================================================
                        boolean allModulesAligned = frontLeftAligned
                                        && frontRightAligned
                                        && backLeftAligned
                                        && backRightAligned;
                        // ======================================================
                        // 5 - LOGS PARA O ADVANTAGESCOPE
                        // ======================================================
                        Logger.recordOutput(
                                        "Calibration/Swerve/SteerMode",
                                        true);
                        Logger.recordOutput(
                                        "Calibration/Swerve/SteerTargetRad",
                                        calibrationSteerAngleRad);
                        Logger.recordOutput(
                                        "Calibration/Swerve/SteerTargetDegrees",
                                        Math.toDegrees(
                                                        calibrationSteerAngleRad));
                        Logger.recordOutput(
                                        "Calibration/Swerve/AllModulesAligned",
                                        allModulesAligned);
                } else {
                        // ======================================================
                        // MODO NORMAL DO SWERVE
                        // ======================================================
                        frontLeft.setDesiredState(
                                        moduleStates[0]);
                        frontRight.setDesiredState(
                                        moduleStates[1]);
                        backLeft.setDesiredState(
                                        moduleStates[2]);
                        backRight.setDesiredState(
                                        moduleStates[3]);
                }
                // ==========================================================
                // 8 - ADVANTAGEKIT
                // ==========================================================
                // Pose calculada pela odometria
                Logger.recordOutput("Drive/XLockActive", xLockActive);
                Logger.recordOutput("Drive/Heading/RawYawDegrees", getRawGyroRotation().getDegrees());
                Logger.recordOutput("Drive/Heading/FieldYawDegrees", getRotation().getDegrees());
                Logger.recordOutput("Drive/Heading/DriverYawDegrees", getDriverRotation().getDegrees());
                Logger.recordOutput("Drive/Heading/GyroHealthy", isGyroHealthy());
                Logger.recordOutput("Drive/Heading/FieldResetResult", fieldHeadingResetResult);
                Logger.recordOutput("Drive/Heading/DriverResetResult", driverHeadingResetResult);
                Logger.recordOutput(
                                "Robot/Pose",
                                pose);
                // Movimento que PEDIMOS ao robô
                Logger.recordOutput(
                                "Drive/ChassisSpeeds",
                                chassisSpeeds);
                // Estados que PEDIMOS aos módulos
                Logger.recordOutput(
                                "Drive/SwerveStates",
                                moduleStates);
                // Estados REAIS dos módulos simulados
                Logger.recordOutput(
                                "Drive/ActualSwerveStates",
                                getModuleStates());
                // Distância acumulada + ângulo
                // dos quatro módulos
                Logger.recordOutput(
                                "Drive/ModulePositions",
                                getModulePositions());
                // Movimento REAL calculado
                // pelos sensores dos módulos
                Logger.recordOutput(
                                "Drive/MeasuredChassisSpeeds",
                                measuredChassisSpeeds);
                // ==========================================================
                // DIAGNÓSTICO - CHASSIS SPEEDS DESEJADO
                // ==========================================================
                Logger.recordOutput(
                                "Drive/Diagnostic/DesiredVx",
                                chassisSpeeds.vxMetersPerSecond);
                Logger.recordOutput(
                                "Drive/Diagnostic/DesiredVy",
                                chassisSpeeds.vyMetersPerSecond);
                Logger.recordOutput(
                                "Drive/Diagnostic/DesiredOmega",
                                chassisSpeeds.omegaRadiansPerSecond);
                // ==========================================================
                // DIAGNÓSTICO - CHASSIS SPEEDS MEDIDO
                // ==========================================================
                Logger.recordOutput(
                                "Drive/Diagnostic/MeasuredVx",
                                measuredChassisSpeeds.vxMetersPerSecond);
                Logger.recordOutput(
                                "Drive/Diagnostic/MeasuredVy",
                                measuredChassisSpeeds.vyMetersPerSecond);
                Logger.recordOutput(
                                "Drive/Diagnostic/MeasuredOmega",
                                measuredChassisSpeeds.omegaRadiansPerSecond);
        }
        // #endregion

        // #region ESTADOS DOS MÓDULOS
        public SwerveModuleState[] getModuleStates() {
                return new SwerveModuleState[] {
                                frontLeft.getState(),
                                frontRight.getState(),
                                backLeft.getState(),
                                backRight.getState()
                };
        }
        // #endregion

        // #region POSIÇÕES DOS MÓDULOS
        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] {
                                frontLeft.getPosition(),
                                frontRight.getPosition(),
                                backLeft.getPosition(),
                                backRight.getPosition()
                };
        }
        // #endregion

        // #region ATUALIZAÇÃO DA REFERÊNCIA SIMULADA
        private void updateStartupPose() {
                if (!Logger.hasReplaySource()) {
                        startupInputs.initializeSimulationPose = simulationReferenceEnabled;
                        startupInputs.initialPose = simulationReferenceEnabled ? SIM_INITIAL_POSE : new Pose2d();
                }
                Logger.processInputs("Drive/Startup", startupInputs);
                if (!startupPoseApplied && startupInputs.initializeSimulationPose && isGyroHealthy()) {
                        // Executa uma vez, inclusive quando o gyro só fica saudável
                        // depois da primeira leitura. O robô real mantém sua inicialização.
                        poseEstimator.resetPosition(getRawGyroRotation(), getModulePositions(), startupInputs.initialPose);
                        pose = poseEstimator.getEstimatedPosition();
                        startupPoseApplied = true;
                        clearVisionHistoryAfterReset();
                }
        }

        private void updateSimulationReference() {
                if (!simulationReferenceEnabled || !RobotBase.isSimulation()
                                || Logger.hasReplaySource() || !isGyroHealthy()) {
                        return;
                }
                Rotation2d gyro = getRawGyroRotation();
                SwerveModulePosition[] positions = getModulePositions();
                if (simulationOdometry == null) {
                        simulationOdometry = new SwerveDriveOdometry(
                                        kinematics, gyro, positions, SIM_INITIAL_POSE);
                        // O alinhamento do estimador é feito uma única vez por
                        // updateStartupPose; esta odometria continua independente.
                }
                simulationPose = simulationOdometry.update(gyro, positions);
                Logger.recordOutput("Drive/Simulation/ReferencePose", simulationPose);
        }

        public boolean isSimulationReferenceReady() {
                return simulationOdometry != null && RobotBase.isSimulation()
                                && !Logger.hasReplaySource() && isGyroHealthy();
        }

        /** Referência dos sensores simulados, independente do PoseEstimator. */
        public Pose2d getSimulationPose() {
                return simulationPose;
        }
        // #endregion

        // #region POSE
        public Pose2d getPose() {
                return pose;
        }
        // #endregion

        // #region VISION MEASUREMENT
        // ============================================================
        // SINCRONIZAÇÃO INICIAL DA POSE COM A VISÃO
        // ============================================================
        //
        // Utilizado apenas em uma inicialização controlada.
        //
        // X e Y vêm do MegaTag2.
        // A orientação continua vindo do Pigeon 2.
        //
        // NÃO executar continuamente no periodic().
        //
        // ============================================================
        public void seedPoseFromVision(Pose2d visionPose) {
                // Proteção dos dados recebidos
                if (!isGyroHealthy() || visionPose == null
                                || !Double.isFinite(visionPose.getX())
                                || !Double.isFinite(visionPose.getY())) {
                        return;
                }
                // Mantém o heading de campo, inclusive o offset de inicialização.
                Rotation2d heading = getRotation();
                if (!Double.isFinite(heading.getRadians())) {
                        return;
                }
                // Combina coordenadas absolutas com o heading do gyro
                Pose2d initialFieldPose = new Pose2d(
                                visionPose.getTranslation(),
                                heading);
                // Sincroniza o PoseEstimator
                clearVisionHistoryAfterReset();
                poseEstimator.resetPosition(
                                getRawGyroRotation(),
                                getModulePositions(),
                                initialFieldPose);
                // Atualiza a pose utilizada pelo Drive
                pose = poseEstimator.getEstimatedPosition();
                // AdvantageKit
                Logger.recordOutput(
                                "Drive/Vision/SeedPose",
                                pose);
        }

        // #region FUSÃO DE VISÃO

        /** Compatibilidade com chamadas antigas. Vision usa a versão identificada. */
        public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
                addVisionMeasurement(visionPose, timestampSeconds, 0.70);
        }

        public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
                        double xyStdDevMeters) {
                tryAddVisionMeasurement("Legacy", visionPose, timestampSeconds, xyStdDevMeters);
        }

        /**
         * Aplica uma medição já validada pelo Vision.
         * Retorna true somente quando ela foi enviada a um histórico válido.
         * Timestamps iguais de câmeras diferentes são permitidos.
         * Repetir o mesmo timestamp da mesma câmera não é permitido.
         */
        public boolean tryAddVisionMeasurement(String cameraName, Pose2d visionPose,
                        double timestampSeconds, double xyStdDevMeters) {
                if (cameraName == null || cameraName.isBlank() || cameraName.contains("/")
                                || visionPose == null || !Double.isFinite(visionPose.getX())
                                || !Double.isFinite(visionPose.getY())
                                || !Double.isFinite(visionPose.getRotation().getRadians())
                                || !Double.isFinite(timestampSeconds) || timestampSeconds <= 0.0
                                || !Double.isFinite(xyStdDevMeters) || xyStdDevMeters <= 0.0) {
                        return rejectVision("INVALID_DATA");
                }
                if (!isGyroHealthy()) {
                        return rejectVision("GYRO_UNAVAILABLE");
                }
                if (timestampSeconds <= lastVisionByCamera.getOrDefault(
                                cameraName, Double.NEGATIVE_INFINITY)) {
                        return rejectVision("DUPLICATE_OR_OLDER_CAMERA_FRAME");
                }
                // A WPILib remove correções posteriores quando recebe visão antiga.
                // A fila do Vision reduz esse caso; esta proteção impede retroceder.
                if (timestampSeconds < lastVisionTimestamp) {
                        return rejectVision("OUT_OF_ORDER");
                }
                Optional<Pose2d> atCapture = samplePoseAt(timestampSeconds);
                if (atCapture.isEmpty()) {
                        return rejectVision("NO_ODOMETRY_AT_CAPTURE");
                }

                // Este projeto funde somente X/Y, tanto para MegaTag2 quanto para
                // PhotonLib. O heading permanece no histórico do giroscópio.
                // O yaw visual continua disponível nos logs para diagnóstico.
                Pose2d translationOnly = new Pose2d(
                                visionPose.getTranslation(), atCapture.get().getRotation());
                poseEstimator.addVisionMeasurement(translationOnly, timestampSeconds,
                                VecBuilder.fill(xyStdDevMeters, xyStdDevMeters, 9999999.0));
                pose = poseEstimator.getEstimatedPosition();
                lastVisionTimestamp = timestampSeconds;
                lastVisionByCamera.put(cameraName, timestampSeconds);
                visionAcceptedByCamera.merge(cameraName, 1L, Long::sum);
                lastVisionResult = "ACCEPTED";

                Logger.recordOutput("Robot/Pose", pose);
                Logger.recordOutput("Drive/Vision/LastResult", lastVisionResult);
                Logger.recordOutput("Drive/Vision/LastCamera", cameraName);
                Logger.recordOutput("Drive/Vision/LastTimestampSeconds", timestampSeconds);
                Logger.recordOutput("Drive/Vision/XYStdDevMeters", xyStdDevMeters);
                Logger.recordOutput("Drive/Vision/" + cameraName + "/XYStdDevMeters", xyStdDevMeters);
                Logger.recordOutput("Drive/Vision/" + cameraName + "/AcceptedCount",
                                getVisionAcceptedCount(cameraName));
                return true;
        }

        private boolean rejectVision(String reason) {
                lastVisionResult = reason;
                Logger.recordOutput("Drive/Vision/LastResult", reason);
                return false;
        }

        /**
         * Consulta a estimativa histórica somente quando há cobertura real.
         * sampleAt da WPILib sozinho limita o tempo às pontas do histórico;
         * por isso verificamos os limites antes de chamá-lo.
         */
        public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
                if (!Double.isFinite(timestampSeconds)
                                || !Double.isFinite(firstOdometryTimestamp)
                                || !Double.isFinite(lastOdometryTimestamp)
                                || timestampSeconds < firstOdometryTimestamp
                                || timestampSeconds > lastOdometryTimestamp
                                || lastOdometryTimestamp - timestampSeconds > SAFE_HISTORY_SECONDS) {
                        return Optional.empty();
                }
                return poseEstimator.sampleAt(timestampSeconds);
        }

        private void clearVisionHistoryAfterReset() {
                firstOdometryTimestamp = Double.NaN;
                lastOdometryTimestamp = Double.NaN;
                lastVisionTimestamp = Double.NEGATIVE_INFINITY;
                lastVisionByCamera.clear();
                poseResetGeneration++;
                Logger.recordOutput("Drive/Vision/PoseResetGeneration", poseResetGeneration);
        }

        public long getPoseResetGeneration() {
                return poseResetGeneration;
        }

        public String getLastVisionResult() {
                return lastVisionResult;
        }

        public long getVisionAcceptedCount(String cameraName) {
                return visionAcceptedByCamera.getOrDefault(cameraName, 0L);
        }

        /**
         * A Limelight corrigiu a pose ao menos uma vez desde o último reset.
         * O shooter exige isso antes de mirar pela pose de campo.
         */
        public boolean hasVisionCorrectionSinceReset() {
                return Double.isFinite(lastVisionTimestamp);
        }

        /** Velocidade medida pelas rodas, no referencial do campo. */
        public ChassisSpeeds getFieldRelativeSpeeds() {
                return ChassisSpeeds.fromRobotRelativeSpeeds(
                                kinematics.toChassisSpeeds(getModuleStates()), getRotation());
        }

        // #endregion

        // ============================================================
        // VERIFICA SE O ROBÔ ESTÁ PARADO
        // ============================================================
        public boolean isStationary() {
                // Verifica a velocidade angular do Pigeon 2
                if (!isGyroHealthy()
                                || Math.abs(gyroInputs.yawVelocityRadPerSec) > Math.toRadians(5.0)) {
                        return false;
                }
                // Verifica a velocidade real dos quatro módulos
                for (SwerveModuleState state : getModuleStates()) {
                        if (!Double.isFinite(state.speedMetersPerSecond)
                                        || Math.abs(state.speedMetersPerSecond) > 0.05) {
                                return false;
                        }
                }
                return true;
        }
        // #endregion

        // #region ROTAÇÃO DO ROBÔ
        public Rotation2d getRotation() {
                return pose.getRotation();
        }

        private Rotation2d getRawGyroRotation() {
                return Rotation2d.fromRadians(
                                gyroInputs.yawPositionRad);
        }

        public Rotation2d getDriverRotation() {
                return getRotation().minus(driverHeadingOffset);
        }

        public boolean isGyroHealthy() {
                return gyroInputs.connected
                                && Double.isFinite(gyroInputs.yawPositionRad)
                                && Double.isFinite(gyroInputs.yawVelocityRadPerSec);
        }
        // #endregion

        // #region VELOCIDADE ANGULAR DO ROBÔ
        public double getYawVelocityDegreesPerSecond() {
                if (!isGyroHealthy()) {
                        return Double.NaN;
                }
                return Math.toDegrees(
                                gyroInputs.yawVelocityRadPerSec);
        }
        // #endregion

        // #region RESET GIRO MANUAL ACUMULADO
        public void resetManualAccumulatedGyro() {
                manualAccumulatedYawRad = 0.0;
                manualPreviousYawRad = gyroInputs.yawPositionRad;
                manualAccumulatedInitialized = true;
                Logger.recordOutput(
                                "Drive/GyroManual/AccumulatedDegrees",
                                0.0);
                Logger.recordOutput(
                                "Drive/GyroManual/AccumulatedTurns",
                                0.0);
                Logger.recordOutput(
                                "Drive/GyroManual/AccumulatedDegreesAbsolute",
                                0.0);
                Logger.recordOutput(
                                "Drive/GyroManual/AccumulatedTurnsAbsolute",
                                0.0);
        }
        // #endregion

        // #region ZERO HEADING
        /**
         * Define 0 graus de campo: executar desabilitado, parado e alinhado ao +X azul.
         */
        public void zeroHeading() {
                if (!DriverStation.isDisabled()) {
                        fieldHeadingResetResult = "REJECTED_ENABLED";
                        return;
                }
                if (!isGyroHealthy()) {
                        fieldHeadingResetResult = "REJECTED_GYRO";
                        return;
                }
                if (!isStationary()) {
                        fieldHeadingResetResult = "REJECTED_MOVING";
                        return;
                }
                // A WPILib calcula o offset entre o sensor e o campo. Não fabricamos
                // uma leitura zero e não dependemos de uma escrita CAN no Pigeon.
                clearVisionHistoryAfterReset();
                poseEstimator.resetPosition(
                                getRawGyroRotation(),
                                getModulePositions(),
                                new Pose2d(pose.getTranslation(), new Rotation2d()));
                pose = poseEstimator.getEstimatedPosition();
                driverHeadingOffset = new Rotation2d();
                resetManualAccumulatedGyro();
                fieldHeadingResetResult = "ACCEPTED";
        }

        /** Zera apenas a referência do piloto e o contador do teste de voltas. */
        public void zeroDriverHeading() {
                if (!isGyroHealthy()) {
                        driverHeadingResetResult = "REJECTED_GYRO";
                        return;
                }
                if (!isStationary()) {
                        driverHeadingResetResult = "REJECTED_MOVING";
                        return;
                }
                driverHeadingOffset = getRotation();
                resetManualAccumulatedGyro();
                driverHeadingResetResult = "ACCEPTED";
        }
        // #endregion

        // #region DRIVE
        public void drive(
                        double vxMetersPerSecond,
                        double vyMetersPerSecond,
                        double omegaRadiansPerSecond) {
                chassisSpeeds = new ChassisSpeeds(
                                vxMetersPerSecond,
                                vyMetersPerSecond,
                                omegaRadiansPerSecond);
        }
        // #endregion

        // #region DRIVE FIELD RELATIVE
        public void driveFieldRelative(
                        double vxMetersPerSecond,
                        double vyMetersPerSecond,
                        double omegaRadiansPerSecond) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                vxMetersPerSecond,
                                vyMetersPerSecond,
                                omegaRadiansPerSecond,
                                // Orientação atual do robô
                                getRotation());
        }

        // #endregion
        /** Movimento relativo à referência do piloto, sem mudar o campo da visão. */
        public void driveDriverRelative(double vxMetersPerSecond,
                        double vyMetersPerSecond, double omegaRadiansPerSecond) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                vxMetersPerSecond, vyMetersPerSecond,
                                omegaRadiansPerSecond, getDriverRotation());
        }

        // #region CALIBRAÇÃO - VOLTAGEM DO DRIVE
        // ============================================================
        // ATIVAR CALIBRAÇÃO POR TENSÃO
        // ============================================================
        //
        // Usado somente para testes de calibração.
        //
        // Exemplo:
        //
        // setCalibrationDriveVoltage(0.3);
        //
        // Isso significa:
        //
        // aplicar aproximadamente 0.3 V
        // aos quatro motores de Drive.
        //
        // ============================================================
        public void setCalibrationDriveVoltage(
                        double volts) {
                // --------------------------------------------------------
                // ENTRA NO MODO DE CALIBRAÇÃO
                // --------------------------------------------------------
                calibrationVoltageMode = true;
                // --------------------------------------------------------
                // DESATIVA O TESTE DIRETO DO STEER
                // --------------------------------------------------------
                //
                // Garante que apenas um modo especial de calibração
                // esteja ativo por vez.
                //
                calibrationSteerMode = false;
                // --------------------------------------------------------
                // LIMITE DE SEGURANÇA
                //
                // Para nosso teste manual de kS não queremos mandar
                // tensão alta acidentalmente.
                //
                // Limite:
                //
                // -2 V até +2 V
                //
                // --------------------------------------------------------
                calibrationDriveVolts = Math.max(
                                -2.0,
                                Math.min(
                                                2.0,
                                                volts));
                // --------------------------------------------------------
                // GARANTE QUE NÃO EXISTA COMANDO NORMAL DE CHASSI
                // --------------------------------------------------------
                chassisSpeeds = new ChassisSpeeds();
                // --------------------------------------------------------
                // LOG
                // --------------------------------------------------------
                Logger.recordOutput(
                                "Calibration/DriveVoltage",
                                calibrationDriveVolts);
        }

        // ============================================================
        // DESATIVAR CALIBRAÇÃO POR TENSÃO
        // ============================================================
        public void stopCalibrationDriveVoltage() {
                // --------------------------------------------------------
                // REMOVE A TENSÃO
                // --------------------------------------------------------
                calibrationDriveVolts = 0.0;
                frontLeft.setDriveVoltage(0.0);
                frontRight.setDriveVoltage(0.0);
                backLeft.setDriveVoltage(0.0);
                backRight.setDriveVoltage(0.0);
                // --------------------------------------------------------
                // VOLTA PARA O CONTROLE NORMAL
                // --------------------------------------------------------
                calibrationVoltageMode = false;
                // Também garante que nenhum teste direto de Steer
                // permaneça ativo ao encerrar a calibração por tensão.
                calibrationSteerMode = false;
                chassisSpeeds = new ChassisSpeeds();
                // ============================================================
                // LOGS
                // ============================================================
                Logger.recordOutput(
                                "Calibration/DriveVoltage",
                                0.0);
                Logger.recordOutput(
                                "Calibration/DriveVoltageRequested",
                                0.0);
                Logger.recordOutput(
                                "Calibration/DriveVoltageApplied",
                                0.0);
                Logger.recordOutput(
                                "Calibration/SteerAligned",
                                false);
        }

        // ============================================================
        // SYSID - TENSÃO REAL DOS MOTORES DE DRIVE
        // ============================================================
        //
        // Retorna a tensão realmente medida/aplicada
        // em cada um dos quatro módulos.
        //
        // Ordem:
        //
        // 0 = Front Left
        // 1 = Front Right
        // 2 = Back Left
        // 3 = Back Right
        //
        // ============================================================
        public double[] getDriveAppliedVolts() {
                return new double[] {
                                frontLeft.getDriveAppliedVolts(),
                                frontRight.getDriveAppliedVolts(),
                                backLeft.getDriveAppliedVolts(),
                                backRight.getDriveAppliedVolts()
                };
        }
        // #endregion

        // #region CALIBRAÇÃO - TESTES CONTROLADOS DO SWERVE
        // ============================================================
        // 1 - TESTE DIRETO DO ÂNGULO DO STEER
        // ============================================================
        //
        // Ativa o modo em que os quatro módulos recebem diretamente
        // o mesmo ângulo. O periodic() mantém os Drives em 0 V.
        //
        // Exemplos:
        //
        // 0° -> Math.toRadians(0.0)
        // 45° -> Math.toRadians(45.0)
        // 90° -> Math.toRadians(90.0)
        // 180° -> Math.toRadians(180.0)
        //
        // ============================================================
        public void setCalibrationSteerAngle(
                        double angleRadians) {
                // Não permite que o SysId por tensão permaneça ativo.
                calibrationVoltageMode = false;
                calibrationDriveVolts = 0.0;
                // Ativa o modo direto do Steer.
                calibrationSteerMode = true;
                // Guarda o ângulo solicitado.
                //
                // Rotation2d mantém uma representação angular adequada
                // para o controle contínuo do módulo.
                calibrationSteerAngleRad = Rotation2d
                                .fromRadians(
                                                angleRadians)
                                .getRadians();
                // Nenhum movimento de chassi durante este teste.
                chassisSpeeds = new ChassisSpeeds();
                // Logs.
                Logger.recordOutput(
                                "Calibration/Swerve/SteerMode",
                                true);
                Logger.recordOutput(
                                "Calibration/Swerve/SteerTargetRad",
                                calibrationSteerAngleRad);
                Logger.recordOutput(
                                "Calibration/Swerve/SteerTargetDegrees",
                                Math.toDegrees(
                                                calibrationSteerAngleRad));
        }

        // ============================================================
        // 2 - TESTE DE VELOCIDADE RETA
        // ============================================================
        //
        // Cria um comando de chassi Robot-Relative conhecido:
        //
        // vx = velocidade solicitada
        // vy = 0
        // omega = 0
        //
        // A cinemática normal converte esse comando nos quatro módulos.
        //
        // ============================================================
        public void setCalibrationDriveVelocity(
                        double speedMetersPerSecond) {
                // Sai dos modos diretos de calibração.
                calibrationVoltageMode = false;
                calibrationDriveVolts = 0.0;
                calibrationSteerMode = false;
                // Velocidade exata e repetível.
                chassisSpeeds = new ChassisSpeeds(
                                speedMetersPerSecond,
                                0.0,
                                0.0);
                // Logs.
                Logger.recordOutput(
                                "Calibration/Swerve/SteerMode",
                                false);
                Logger.recordOutput(
                                "Calibration/Swerve/RequestedSpeedMps",
                                speedMetersPerSecond);
        }

        // ============================================================
        // 3 - TESTE CONTROLADO DE CHASSIS SPEEDS
        // ============================================================
        //
        // Permite fornecer diretamente:
        //
        // vx
        // vy
        // omega
        //
        // em coordenadas Robot-Relative.
        //
        // Para rotação pura usaremos, por exemplo:
        //
        // vx = 0
        // vy = 0
        // omega = +0.50 rad/s
        //
        // ============================================================
        public void setCalibrationChassisSpeeds(
                        double vxMetersPerSecond,
                        double vyMetersPerSecond,
                        double omegaRadiansPerSecond) {
                // Desativa os modos diretos.
                calibrationVoltageMode = false;
                calibrationDriveVolts = 0.0;
                calibrationSteerMode = false;
                // Envia uma ordem de movimento exatamente conhecida.
                chassisSpeeds = new ChassisSpeeds(
                                vxMetersPerSecond,
                                vyMetersPerSecond,
                                omegaRadiansPerSecond);
                // Logs.
                Logger.recordOutput(
                                "Calibration/Swerve/SteerMode",
                                false);
                Logger.recordOutput(
                                "Calibration/Swerve/RequestedVx",
                                vxMetersPerSecond);
                Logger.recordOutput(
                                "Calibration/Swerve/RequestedVy",
                                vyMetersPerSecond);
                Logger.recordOutput(
                                "Calibration/Swerve/RequestedOmega",
                                omegaRadiansPerSecond);
        }

        // ============================================================
        // 4 - PARADA GERAL DOS TESTES DE CALIBRAÇÃO
        // ============================================================
        //
        // Usado pelo finallyDo() dos comandos para garantir que o robô
        // sempre termine o teste parado, mesmo se o comando for
        // interrompido.
        //
        // ============================================================
        public void stopCalibration() {
                // Desativa todos os modos especiais.
                calibrationVoltageMode = false;
                calibrationSteerMode = false;
                calibrationDriveVolts = 0.0;
                // Zera o movimento solicitado ao chassi.
                chassisSpeeds = new ChassisSpeeds();
                // Garante 0 V imediatamente nos quatro Drives.
                frontLeft.setDriveVoltage(
                                0.0);
                frontRight.setDriveVoltage(
                                0.0);
                backLeft.setDriveVoltage(
                                0.0);
                backRight.setDriveVoltage(
                                0.0);
                // Logs.
                Logger.recordOutput(
                                "Calibration/Swerve/Active",
                                false);
                Logger.recordOutput(
                                "Calibration/Swerve/SteerMode",
                                false);
                Logger.recordOutput(
                                "Calibration/Swerve/RequestedSpeedMps",
                                0.0);
                Logger.recordOutput(
                                "Calibration/Swerve/RequestedVx",
                                0.0);
                Logger.recordOutput(
                                "Calibration/Swerve/RequestedVy",
                                0.0);
                Logger.recordOutput(
                                "Calibration/Swerve/RequestedOmega",
                                0.0);
        }
        // #endregion

        // #region STOP
        public void stop() {
                chassisSpeeds = new ChassisSpeeds();
        }

        /** Para o chassi e orienta as rodas em X usando a geometria dos módulos. */
        public void stopWithX() {
                calibrationVoltageMode = false;
                calibrationSteerMode = false;
                calibrationDriveVolts = 0.0;
                kinematics.resetHeadings(
                                frontLeftLocation.getAngle(),
                                frontRightLocation.getAngle(),
                                backLeftLocation.getAngle(),
                                backRightLocation.getAngle());
                stop();
                xLockActive = true;
        }

        public void releaseXLock() {
                stopCalibration();
                xLockActive = false;
        }

        /** Ocupa o Drive até soltar o botão ou desabilitar o robô. */
        public Command xLockCommand() {
                return runEnd(this::stopWithX, this::releaseXLock)
                                .beforeStarting(() -> {
                                        stopCalibration();
                                        stopWithX();
                                })
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                                .withName("FreioEmX");
        }

        public boolean isXLockActive() {
                return xLockActive;
        }
        // #endregion
}
