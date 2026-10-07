package frc.robot.subsystems.drive;

// #region IMPORTS

import static frc.robot.Constants.DriveConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.RobotBase;

// #endregion

public class Drive extends SubsystemBase {

    // #region VELOCIDADE DESEJADA DO CHASSI

    // Guarda a última ordem de movimento recebida pelo Drive

    private ChassisSpeeds chassisSpeeds =

            new ChassisSpeeds();

    // #endregion

    // #region MODO CALIBRAÇÃO POR TENSÃO

    // Indica se estamos executando um teste direto de tensão.

    private boolean calibrationVoltageMode = false;

    // Tensão solicitada para os motores de Drive.

    private double calibrationDriveVolts = 0.0;

    // 🆕 #region MODO CALIBRAÇÃO DIRETA DO STEER

    // 🆕 ============================================================

    // 🆕 MODO DE CALIBRAÇÃO DIRETA DO STEER

    // 🆕 ============================================================

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

    // 🆕 Indica que o teste direto de ângulo está ativo.

    private boolean calibrationSteerMode =

            false;

    // 🆕 Ângulo desejado para os quatro módulos durante o teste.

    private double calibrationSteerAngleRad =

            0.0;

    // 🆕 #endregion

    // ============================================================

    // ÂNGULO DO STEER DURANTE A CALIBRAÇÃO

    // ============================================================

    //

    // 0 rad = rodas apontadas para frente.

    //

    // IMPORTANTE:

    // Durante o kS não utilizaremos SwerveModuleState.optimize().

    //

    private static final double CALIBRATION_TURN_ANGLE_RAD =

            0.0;

    // #region GIRO MANUAL ACUMULADO

    private double manualPreviousYawRad =

            0.0;

    private double manualAccumulatedYawRad =

            0.0;

    private boolean manualAccumulatedInitialized =

            false;

    // #endregion

    // ============================================================

    // TOLERÂNCIA DE ALINHAMENTO

    // ============================================================

    //

    // Só permitiremos tensão nos motores de Drive quando

    // TODOS os quatro módulos estiverem a no máximo

    // aproximadamente 3 graus da posição desejada.

    //

    private static final double CALIBRATION_TURN_TOLERANCE_RAD =

            Math.toRadians(3.0);

    // #endregion

    // #region POSE DO ROBÔ

    // Pose atual estimada pela odometria

    private Pose2d pose =

            new Pose2d(

                    0.0,

                    0.0,

                    new Rotation2d()

            );

    // #endregion

    // #region POSIÇÃO DOS MÓDULOS NO CHASSI

    // FL - Front Left

    private final Translation2d frontLeftLocation =

            new Translation2d(

                    kWheelBaseMeters / 2.0,

                    kTrackWidthMeters / 2.0

            );

    // FR - Front Right

    private final Translation2d frontRightLocation =

            new Translation2d(

                    kWheelBaseMeters / 2.0,

                    -kTrackWidthMeters / 2.0

            );

    // BL - Back Left

    private final Translation2d backLeftLocation =

            new Translation2d(

                    -kWheelBaseMeters / 2.0,

                    kTrackWidthMeters / 2.0

            );

    // BR - Back Right

    private final Translation2d backRightLocation =

            new Translation2d(

                    -kWheelBaseMeters / 2.0,

                    -kTrackWidthMeters / 2.0

            );

    // #endregion

    // #region CINEMÁTICA DO SWERVE

    private final SwerveDriveKinematics kinematics =

            new SwerveDriveKinematics(

                    frontLeftLocation,

                    frontRightLocation,

                    backLeftLocation,

                    backRightLocation

            );

    // #endregion

    // #region POSE ESTIMATOR

    // ============================================================

    // ESTIMADOR DE POSIÇÃO DO SWERVE

    // ============================================================

    //

    // Atualmente utiliza:

    // - Pigeon 2

    // - Posição dos quatro módulos

    //

    // FUTURAMENTE:

    // - Receberá também a pose do MegaTag2.

    //

    // Por enquanto NÃO utilizaremos dados da Limelight.

    //

    // ============================================================

    private final SwerveDrivePoseEstimator poseEstimator =

            new SwerveDrivePoseEstimator(

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

                            new Rotation2d()

                    )

            );

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

    private final GyroIOInputsAutoLogged gyroInputs =

            new GyroIOInputsAutoLogged();

    // #endregion

    // #region CONSTRUTOR

    public Drive() {

        // ============================================================

        // ROBÔ REAL

        // ============================================================

        if (RobotBase.isReal()) {

            // --------------------------------------------------------

            // FRONT LEFT

            // --------------------------------------------------------

            frontLeft =

                    new Module(

                            new ModuleIOTalonFX(

                                    13, // Drive Motor ID

                                    11, // Steer Motor ID

                                    12, // CANcoder ID

                                    -0.067626953125, // Offset do CANcoder

                                    false, // Drive invertido?

                                    true, // Steer invertido?

                                    false // CANcoder invertido?

                            ),

                            0

                    );

            // --------------------------------------------------------

            // FRONT RIGHT

            // --------------------------------------------------------

            frontRight =

                    new Module(

                            new ModuleIOTalonFX(

                                    19, // Drive Motor ID

                                    20, // Steer Motor ID

                                    18, // CANcoder ID

                                    -0.204833984375, // Offset do CANcoder

                                    true, // Drive invertido?

                                    true, // Steer invertido?

                                    false // CANcoder invertido?

                            ),

                            1

                    );

            // --------------------------------------------------------

            // BACK LEFT

            // --------------------------------------------------------

            backLeft =

                    new Module(

                            new ModuleIOTalonFX(

                                    17, // Drive Motor ID

                                    16, // Steer Motor ID

                                    21, // CANcoder ID

                                    -0.291259765625, // Offset do CANcoder

                                    false, // Drive invertido?

                                    true, // Steer invertido?

                                    false // CANcoder invertido?

                            ),

                            2

                    );

            // --------------------------------------------------------

            // BACK RIGHT

            // --------------------------------------------------------

            backRight =

                    new Module(

                            new ModuleIOTalonFX(

                                    14, // Drive Motor ID

                                    10, // Steer Motor ID

                                    15, // CANcoder ID

                                    0.360595703125, // Offset do CANcoder

                                    true, // Drive invertido?

                                    true, // Steer invertido?

                                    false // CANcoder invertido?

                            ),

                            3

                    );

            // ============================================================

            // GYRO REAL

            // ============================================================

            // Cria o Pigeon 2 real do drivetrain

            // CAN ID = 3

            gyroIO =

                    new GyroIOPigeon2(

                            3

                    );

        }

        // ============================================================

        // SIMULAÇÃO

        // ============================================================

        else {

            // Front Left

            frontLeft =

                    new Module(

                            new ModuleIOSim(),

                            0

                    );

            // Front Right

            frontRight =

                    new Module(

                            new ModuleIOSim(),

                            1

                    );

            // Back Left

            backLeft =

                    new Module(

                            new ModuleIOSim(),

                            2

                    );

            // Back Right

            backRight =

                    new Module(

                            new ModuleIOSim(),

                            3

                    );

            // --------------------------------------------------------

            // GYRO SIMULADO

            // --------------------------------------------------------

            gyroIO =

                    new GyroIOSim();

        }

    }

    // #endregion

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

        ChassisSpeeds measuredChassisSpeeds =

                kinematics.toChassisSpeeds(

                        getModuleStates()

                );

        // ==========================================================

        // 3 - GYRO SIMULADO

        // ==========================================================

        // Usa a rotação REAL calculada pelos módulos

        gyroIO.setSimYawVelocity(

                measuredChassisSpeeds.omegaRadiansPerSecond

        );

        // Atualiza o sensor

        gyroIO.updateInputs(

                gyroInputs

        );

        // Registra no AdvantageKit

        Logger.processInputs(

                "Drive/Gyro",

                gyroInputs

        );

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

        if (!manualAccumulatedInitialized) {

            manualPreviousYawRad = manualCurrentYawRad;

            manualAccumulatedInitialized = true;
        }

        else {

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

        // 4 - ODOMETRIA

        // 4 - ATUALIZAÇÃO DO POSE ESTIMATOR

        // ==========================================================

        //

        // Atualiza a posição utilizando:

        //

        // - Pigeon 2

        // - Encoders dos quatro módulos

        //

        // A Limelight ainda NÃO participa do cálculo.

        //

        // ==========================================================

        pose =

                poseEstimator.update(

                        Rotation2d.fromRadians(

                                gyroInputs.yawPositionRad

                        ),

                        getModulePositions()

                );

        // ==========================================================

        // 5 - CINEMÁTICA DESEJADA

        // ==========================================================

        // Converte a ordem de movimento do chassi

        // em quatro estados de módulos

        SwerveModuleState[] moduleStates =

                kinematics.toSwerveModuleStates(

                        chassisSpeeds

                );

        // ==========================================================

        // 6 - LIMITA VELOCIDADE DAS RODAS

        // ==========================================================

        SwerveDriveKinematics.desaturateWheelSpeeds(

                moduleStates,

                kMaxLinearSpeed

        );

        // ==========================================================

        // 7 - COMANDA OS QUATRO MÓDULOS

        // ==========================================================

        if (calibrationVoltageMode) {

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

                    CALIBRATION_TURN_ANGLE_RAD

            );

            frontRight.setCalibrationTurnPosition(

                    CALIBRATION_TURN_ANGLE_RAD

            );

            backLeft.setCalibrationTurnPosition(

                    CALIBRATION_TURN_ANGLE_RAD

            );

            backRight.setCalibrationTurnPosition(

                    CALIBRATION_TURN_ANGLE_RAD

            );

            // ==========================================================

            // 2 - VERIFICA O ALINHAMENTO DE CADA MÓDULO

            // ==========================================================

            boolean frontLeftAligned =

                    frontLeft.isCalibrationTurnAligned(

                            CALIBRATION_TURN_ANGLE_RAD,

                            CALIBRATION_TURN_TOLERANCE_RAD

                    );

            boolean frontRightAligned =

                    frontRight.isCalibrationTurnAligned(

                            CALIBRATION_TURN_ANGLE_RAD,

                            CALIBRATION_TURN_TOLERANCE_RAD

                    );

            boolean backLeftAligned =

                    backLeft.isCalibrationTurnAligned(

                            CALIBRATION_TURN_ANGLE_RAD,

                            CALIBRATION_TURN_TOLERANCE_RAD

                    );

            boolean backRightAligned =

                    backRight.isCalibrationTurnAligned(

                            CALIBRATION_TURN_ANGLE_RAD,

                            CALIBRATION_TURN_TOLERANCE_RAD

                    );

            // ==========================================================

            // 3 - TODOS PRECISAM ESTAR ALINHADOS

            // ==========================================================

            boolean allModulesAligned =

                    frontLeftAligned

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

            double appliedCalibrationVolts =

                    allModulesAligned

                            ? calibrationDriveVolts

                            : 0.0;

            // ==========================================================

            // 5 - APLICA TENSÃO AOS QUATRO DRIVES

            // ==========================================================

            frontLeft.setDriveVoltage(

                    appliedCalibrationVolts

            );

            frontRight.setDriveVoltage(

                    appliedCalibrationVolts

            );

            backLeft.setDriveVoltage(

                    appliedCalibrationVolts

            );

            backRight.setDriveVoltage(

                    appliedCalibrationVolts

            );

            // ==========================================================

            // 6 - LOGS DE SEGURANÇA

            // ==========================================================

            // Tensão que o teste está PEDINDO

            Logger.recordOutput(

                    "Calibration/DriveVoltageRequested",

                    calibrationDriveVolts

            );

            // Tensão que realmente foi LIBERADA

            Logger.recordOutput(

                    "Calibration/DriveVoltageApplied",

                    appliedCalibrationVolts

            );

            // Estado geral do alinhamento

            Logger.recordOutput(

                    "Calibration/SteerAligned",

                    allModulesAligned

            );

            // Estado individual

            Logger.recordOutput(

                    "Calibration/FrontLeftAligned",

                    frontLeftAligned

            );

            Logger.recordOutput(

                    "Calibration/FrontRightAligned",

                    frontRightAligned

            );

            Logger.recordOutput(

                    "Calibration/BackLeftAligned",

                    backLeftAligned

            );

            Logger.recordOutput(

                    "Calibration/BackRightAligned",

                    backRightAligned

            );

        }

        // 🆕 ==========================================================

        // 🆕 MODO CALIBRAÇÃO DIRETA DO STEER

        // 🆕 ==========================================================

        //

        // Neste modo os Drives ficam parados e somente os motores

        // de Steer recebem posição. Isso permite testar 0°, 45°,

        // 90° e 180° de forma repetível, sem joystick.

        //

        // ==========================================================

        else if (calibrationSteerMode) {

            // 🆕 ======================================================

            // 🆕 1 - GARANTE QUE OS DRIVES FIQUEM PARADOS

            // 🆕 ======================================================

            frontLeft.setDriveVoltage(

                    0.0

            );

            frontRight.setDriveVoltage(

                    0.0

            );

            backLeft.setDriveVoltage(

                    0.0

            );

            backRight.setDriveVoltage(

                    0.0

            );

            // 🆕 ======================================================

            // 🆕 2 - ENVIA O MESMO ÂNGULO PARA OS QUATRO STEERS

            // 🆕 ======================================================

            frontLeft.setCalibrationTurnPosition(

                    calibrationSteerAngleRad

            );

            frontRight.setCalibrationTurnPosition(

                    calibrationSteerAngleRad

            );

            backLeft.setCalibrationTurnPosition(

                    calibrationSteerAngleRad

            );

            backRight.setCalibrationTurnPosition(

                    calibrationSteerAngleRad

            );

            // 🆕 ======================================================

            // 🆕 3 - VERIFICA O ALINHAMENTO DE CADA MÓDULO

            // 🆕 ======================================================

            boolean frontLeftAligned =

                    frontLeft.isCalibrationTurnAligned(

                            calibrationSteerAngleRad,

                            CALIBRATION_TURN_TOLERANCE_RAD

                    );

            boolean frontRightAligned =

                    frontRight.isCalibrationTurnAligned(

                            calibrationSteerAngleRad,

                            CALIBRATION_TURN_TOLERANCE_RAD

                    );

            boolean backLeftAligned =

                    backLeft.isCalibrationTurnAligned(

                            calibrationSteerAngleRad,

                            CALIBRATION_TURN_TOLERANCE_RAD

                    );

            boolean backRightAligned =

                    backRight.isCalibrationTurnAligned(

                            calibrationSteerAngleRad,

                            CALIBRATION_TURN_TOLERANCE_RAD

                    );

            // 🆕 ======================================================

            // 🆕 4 - VERIFICA SE TODOS CHEGARAM AO ALVO

            // 🆕 ======================================================

            boolean allModulesAligned =

                    frontLeftAligned

                            && frontRightAligned

                            && backLeftAligned

                            && backRightAligned;

            // 🆕 ======================================================

            // 🆕 5 - LOGS PARA O ADVANTAGESCOPE

            // 🆕 ======================================================

            Logger.recordOutput(

                    "Calibration/Swerve/SteerMode",

                    true

            );

            Logger.recordOutput(

                    "Calibration/Swerve/SteerTargetRad",

                    calibrationSteerAngleRad

            );

            Logger.recordOutput(

                    "Calibration/Swerve/SteerTargetDegrees",

                    Math.toDegrees(

                            calibrationSteerAngleRad

                    )

            );

            Logger.recordOutput(

                    "Calibration/Swerve/AllModulesAligned",

                    allModulesAligned

            );

        }

        else {

            // ======================================================

            // MODO NORMAL DO SWERVE

            // ======================================================

            frontLeft.setDesiredState(

                    moduleStates[0]

            );

            frontRight.setDesiredState(

                    moduleStates[1]

            );

            backLeft.setDesiredState(

                    moduleStates[2]

            );

            backRight.setDesiredState(

                    moduleStates[3]

            );

        }

        // ==========================================================

        // 8 - ADVANTAGEKIT

        // ==========================================================

        // Pose calculada pela odometria

        Logger.recordOutput(

                "Robot/Pose",

                pose

        );

        // Movimento que PEDIMOS ao robô

        Logger.recordOutput(

                "Drive/ChassisSpeeds",

                chassisSpeeds

        );

        // Estados que PEDIMOS aos módulos

        Logger.recordOutput(

                "Drive/SwerveStates",

                moduleStates

        );

        // Estados REAIS dos módulos simulados

        Logger.recordOutput(

                "Drive/ActualSwerveStates",

                getModuleStates()

        );

        // Distância acumulada + ângulo

        // dos quatro módulos

        Logger.recordOutput(

                "Drive/ModulePositions",

                getModulePositions()

        );

        // Movimento REAL calculado

        // pelos sensores dos módulos

        Logger.recordOutput(

                "Drive/MeasuredChassisSpeeds",

                measuredChassisSpeeds

        );

        // ==========================================================

        // DIAGNÓSTICO - CHASSIS SPEEDS DESEJADO

        // ==========================================================

        Logger.recordOutput(

                "Drive/Diagnostic/DesiredVx",

                chassisSpeeds.vxMetersPerSecond

        );

        Logger.recordOutput(

                "Drive/Diagnostic/DesiredVy",

                chassisSpeeds.vyMetersPerSecond

        );

        Logger.recordOutput(

                "Drive/Diagnostic/DesiredOmega",

                chassisSpeeds.omegaRadiansPerSecond

        );

        // ==========================================================

        // DIAGNÓSTICO - CHASSIS SPEEDS MEDIDO

        // ==========================================================

        Logger.recordOutput(

                "Drive/Diagnostic/MeasuredVx",

                measuredChassisSpeeds.vxMetersPerSecond

        );

        Logger.recordOutput(

                "Drive/Diagnostic/MeasuredVy",

                measuredChassisSpeeds.vyMetersPerSecond

        );

        Logger.recordOutput(

                "Drive/Diagnostic/MeasuredOmega",

                measuredChassisSpeeds.omegaRadiansPerSecond

        );

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

        if (visionPose == null

                || !Double.isFinite(visionPose.getX())

                || !Double.isFinite(visionPose.getY())) {

            return;

        }

        // Mantém a orientação atual do Pigeon 2

        Rotation2d heading = getRotation();

        if (!Double.isFinite(heading.getRadians())) {

            return;

        }

        // Combina coordenadas absolutas com o heading do gyro

        Pose2d initialFieldPose = new Pose2d(

                visionPose.getTranslation(),

                heading

        );

        // Sincroniza o PoseEstimator

        poseEstimator.resetPosition(

                heading,

                getModulePositions(),

                initialFieldPose

        );

        // Atualiza a pose utilizada pelo Drive

        pose = poseEstimator.getEstimatedPosition();

        // AdvantageKit

        Logger.recordOutput(

                "Drive/Vision/SeedPose",

                pose

        );

    }

    // ============================================================

    // FUSÃO DE VISÃO - MEGATAG2

    // ============================================================

    //

    // Recebe uma medição de posição da Limelight.

    //

    // IMPORTANTE:

    //

    // A validação das AprilTags será realizada pelo Vision.java.

    //

    // O PoseEstimator combina:

    //

    // - Pigeon 2

    // - Encoders dos módulos

    // - Posição medida pelo MegaTag2

    //

    // O timestamp permite compensar o atraso da câmera.

    //

    // ============================================================

    public void addVisionMeasurement(

            Pose2d visionPose,

            double timestampSeconds

    ) {

        // ========================================================

        // 1 - PROTEÇÃO DOS DADOS RECEBIDOS

        // ========================================================

        if (visionPose == null

                || !Double.isFinite(visionPose.getX())

                || !Double.isFinite(visionPose.getY())

                || !Double.isFinite(visionPose.getRotation().getRadians())

                || !Double.isFinite(timestampSeconds)) {

            return;

        }

        // ========================================================

        // 2 - ENVIA A MEDIÇÃO AO POSE ESTIMATOR

        // ========================================================

        //

        // Desvio padrão inicial:

        //

        // X = 0.70 metro

        // Y = 0.70 metro

        // Theta = confiança praticamente nula na rotação

        //

        // A orientação continuará sendo fornecida principalmente

        // pelo Pigeon 2, como recomendado para MegaTag2.

        //

        // Esses valores poderão ser ajustados após os testes.

        //

        // ========================================================

        poseEstimator.addVisionMeasurement(

                visionPose,

                timestampSeconds,

                VecBuilder.fill(

                        0.70,

                        0.70,

                        9999999.0

                )

        );

        // ========================================================

        // 3 - SINCRONIZA A POSE ATUAL

        // ========================================================

        pose = poseEstimator.getEstimatedPosition();

    }

    // ============================================================

    // VERIFICA SE O ROBÔ ESTÁ PARADO

    // ============================================================

    public boolean isStationary() {

        // Verifica a velocidade angular do Pigeon 2

        if (!Double.isFinite(gyroInputs.yawVelocityRadPerSec)

                || Math.abs(gyroInputs.yawVelocityRadPerSec)

                        > Math.toRadians(5.0)) {

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

        return Rotation2d.fromRadians(

                gyroInputs.yawPositionRad

        );

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

    public void zeroHeading() {

        // ==========================================================

        // 1 - ZERA O GYRO

        // ==========================================================

        gyroIO.resetYaw();

        // Atualiza imediatamente os valores internos

        gyroInputs.yawPositionRad = 0.0;

        gyroInputs.yawVelocityRadPerSec = 0.0;

        // ==========================================================
        // 1.5 - ZERA O CONTADOR DE GIRO MANUAL
        // ==========================================================
        //
        // Ao executar Zero Heading antes do teste manual,
        // o contador no AdvantageScope também começa em 0 graus.
        //
        // ==========================================================

        resetManualAccumulatedGyro();

        // ==========================================================

        // 2 - MANTÉM X E Y DA POSE

        // MAS ZERA A ROTAÇÃO

        // ==========================================================

        Pose2d newPose =

                new Pose2d(

                        pose.getTranslation(),

                        new Rotation2d()

                );

        // ==========================================================

        // 3 - SINCRONIZA O POSE ESTIMATOR

        // ==========================================================

        poseEstimator.resetPosition(

                new Rotation2d(),

                getModulePositions(),

                newPose

        );

        pose = poseEstimator.getEstimatedPosition();

    }

    // #endregion

    // #region DRIVE

    public void drive(

            double vxMetersPerSecond,

            double vyMetersPerSecond,

            double omegaRadiansPerSecond

    ) {

        chassisSpeeds =

                new ChassisSpeeds(

                        vxMetersPerSecond,

                        vyMetersPerSecond,

                        omegaRadiansPerSecond

                );

    }

    // #endregion

    // #region DRIVE FIELD RELATIVE

    public void driveFieldRelative(

            double vxMetersPerSecond,

            double vyMetersPerSecond,

            double omegaRadiansPerSecond

    ) {

        chassisSpeeds =

                ChassisSpeeds.fromFieldRelativeSpeeds(

                        vxMetersPerSecond,

                        vyMetersPerSecond,

                        omegaRadiansPerSecond,

                        // Orientação atual do robô

                        getRotation()

                );

    }

    // #endregion

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

            double volts

    ) {

        // --------------------------------------------------------

        // ENTRA NO MODO DE CALIBRAÇÃO

        // --------------------------------------------------------

        calibrationVoltageMode = true;

        // 🆕 --------------------------------------------------------

        // 🆕 DESATIVA O TESTE DIRETO DO STEER

        // 🆕 --------------------------------------------------------

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

        calibrationDriveVolts =

                Math.max(

                        -2.0,

                        Math.min(

                                2.0,

                                volts

                        )

                );

        // --------------------------------------------------------

        // GARANTE QUE NÃO EXISTA COMANDO NORMAL DE CHASSI

        // --------------------------------------------------------

        chassisSpeeds =

                new ChassisSpeeds();

        // --------------------------------------------------------

        // LOG

        // --------------------------------------------------------

        Logger.recordOutput(

                "Calibration/DriveVoltage",

                calibrationDriveVolts

        );

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

        // 🆕 Também garante que nenhum teste direto de Steer

        // 🆕 permaneça ativo ao encerrar a calibração por tensão.

        calibrationSteerMode = false;

        chassisSpeeds =

                new ChassisSpeeds();

        // ============================================================

        // LOGS

        // ============================================================

        Logger.recordOutput(

                "Calibration/DriveVoltage",

                0.0

        );

        Logger.recordOutput(

                "Calibration/DriveVoltageRequested",

                0.0

        );

        Logger.recordOutput(

                "Calibration/DriveVoltageApplied",

                0.0

        );

        Logger.recordOutput(

                "Calibration/SteerAligned",

                false

        );

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

    // 🆕 #region CALIBRAÇÃO - TESTES CONTROLADOS DO SWERVE

    // 🆕 ============================================================

    // 🆕 1 - TESTE DIRETO DO ÂNGULO DO STEER

    // 🆕 ============================================================

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

            double angleRadians

    ) {

        // 🆕 Não permite que o SysId por tensão permaneça ativo.

        calibrationVoltageMode = false;

        calibrationDriveVolts = 0.0;

        // 🆕 Ativa o modo direto do Steer.

        calibrationSteerMode = true;

        // 🆕 Guarda o ângulo solicitado.

        //

        // Rotation2d mantém uma representação angular adequada

        // para o controle contínuo do módulo.

        calibrationSteerAngleRad =

                Rotation2d

                        .fromRadians(

                                angleRadians

                        )

                        .getRadians();

        // 🆕 Nenhum movimento de chassi durante este teste.

        chassisSpeeds =

                new ChassisSpeeds();

        // 🆕 Logs.

        Logger.recordOutput(

                "Calibration/Swerve/SteerMode",

                true

        );

        Logger.recordOutput(

                "Calibration/Swerve/SteerTargetRad",

                calibrationSteerAngleRad

        );

        Logger.recordOutput(

                "Calibration/Swerve/SteerTargetDegrees",

                Math.toDegrees(

                        calibrationSteerAngleRad

                )

        );

    }

    // 🆕 ============================================================

    // 🆕 2 - TESTE DE VELOCIDADE RETA

    // 🆕 ============================================================

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

            double speedMetersPerSecond

    ) {

        // 🆕 Sai dos modos diretos de calibração.

        calibrationVoltageMode = false;

        calibrationDriveVolts = 0.0;

        calibrationSteerMode = false;

        // 🆕 Velocidade exata e repetível.

        chassisSpeeds =

                new ChassisSpeeds(

                        speedMetersPerSecond,

                        0.0,

                        0.0

                );

        // 🆕 Logs.

        Logger.recordOutput(

                "Calibration/Swerve/SteerMode",

                false

        );

        Logger.recordOutput(

                "Calibration/Swerve/RequestedSpeedMps",

                speedMetersPerSecond

        );

    }

    // 🆕 ============================================================

    // 🆕 3 - TESTE CONTROLADO DE CHASSIS SPEEDS

    // 🆕 ============================================================

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

            double omegaRadiansPerSecond

    ) {

        // 🆕 Desativa os modos diretos.

        calibrationVoltageMode = false;

        calibrationDriveVolts = 0.0;

        calibrationSteerMode = false;

        // 🆕 Envia uma ordem de movimento exatamente conhecida.

        chassisSpeeds =

                new ChassisSpeeds(

                        vxMetersPerSecond,

                        vyMetersPerSecond,

                        omegaRadiansPerSecond

                );

        // 🆕 Logs.

        Logger.recordOutput(

                "Calibration/Swerve/SteerMode",

                false

        );

        Logger.recordOutput(

                "Calibration/Swerve/RequestedVx",

                vxMetersPerSecond

        );

        Logger.recordOutput(

                "Calibration/Swerve/RequestedVy",

                vyMetersPerSecond

        );

        Logger.recordOutput(

                "Calibration/Swerve/RequestedOmega",

                omegaRadiansPerSecond

        );

    }

    // 🆕 ============================================================

    // 🆕 4 - PARADA GERAL DOS TESTES DE CALIBRAÇÃO

    // 🆕 ============================================================

    //

    // Usado pelo finallyDo() dos comandos para garantir que o robô

    // sempre termine o teste parado, mesmo se o comando for

    // interrompido.

    //

    // ============================================================

    public void stopCalibration() {

        // 🆕 Desativa todos os modos especiais.

        calibrationVoltageMode = false;

        calibrationSteerMode = false;

        calibrationDriveVolts = 0.0;

        // 🆕 Zera o movimento solicitado ao chassi.

        chassisSpeeds =

                new ChassisSpeeds();

        // 🆕 Garante 0 V imediatamente nos quatro Drives.

        frontLeft.setDriveVoltage(

                0.0

        );

        frontRight.setDriveVoltage(

                0.0

        );

        backLeft.setDriveVoltage(

                0.0

        );

        backRight.setDriveVoltage(

                0.0

        );

        // 🆕 Logs.

        Logger.recordOutput(

                "Calibration/Swerve/Active",

                false

        );

        Logger.recordOutput(

                "Calibration/Swerve/SteerMode",

                false

        );

        Logger.recordOutput(

                "Calibration/Swerve/RequestedSpeedMps",

                0.0

        );

        Logger.recordOutput(

                "Calibration/Swerve/RequestedVx",

                0.0

        );

        Logger.recordOutput(

                "Calibration/Swerve/RequestedVy",

                0.0

        );

        Logger.recordOutput(

                "Calibration/Swerve/RequestedOmega",

                0.0

        );

    }

    // 🆕 #endregion

    // #region STOP

    public void stop() {

        chassisSpeeds =

                new ChassisSpeeds();

    }

    // #endregion

}
