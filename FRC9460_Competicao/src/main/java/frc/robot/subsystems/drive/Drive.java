package frc.robot.subsystems.drive;

// #region IMPORTS

import static frc.robot.Constants.DriveConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.RobotBase;
// #endregion


public class Drive extends SubsystemBase {


    // #region VELOCIDADE DESEJADA DO CHASSI

    // Guarda a última ordem de movimento recebida pelo Drive
    private ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds();

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


    // #region ODOMETRIA

    private final SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(

            // Geometria do Swerve
            kinematics,

            // Ângulo inicial do robô
            new Rotation2d(),

            // Posições iniciais dos módulos
            new SwerveModulePosition[] {

                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },

            // Pose inicial no campo
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
    // SIM  -> GyroIOSim
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

                    13,                 // Drive Motor ID
                    11,                 // Steer Motor ID
                    12,                 // CANcoder ID

                    -0.067626953125,    // Offset do CANcoder

                    false,              // Drive invertido?
                    true,               // Steer invertido?
                    false               // CANcoder invertido?
                ),

                0
            );



        // --------------------------------------------------------
        // FRONT RIGHT
        // --------------------------------------------------------

        frontRight =
            new Module(

                new ModuleIOTalonFX(

                    19,                 // Drive Motor ID
                    20,                 // Steer Motor ID
                    18,                 // CANcoder ID

                    -0.204833984375,    // Offset do CANcoder

                    true,               // Drive invertido?
                    true,               // Steer invertido?
                    false               // CANcoder invertido?
                ),

                1
            );



        // --------------------------------------------------------
        // BACK LEFT
        // --------------------------------------------------------

        backLeft =
            new Module(

                new ModuleIOTalonFX(

                    17,                 // Drive Motor ID
                    16,                 // Steer Motor ID
                    21,                 // CANcoder ID

                    -0.291259765625,    // Offset do CANcoder

                    false,              // Drive invertido?
                    true,               // Steer invertido?
                    false               // CANcoder invertido?
                ),

                2
            );



        // --------------------------------------------------------
        // BACK RIGHT
        // --------------------------------------------------------

        backRight =
            new Module(

                new ModuleIOTalonFX(

                    14,                 // Drive Motor ID
                    10,                 // Steer Motor ID
                    15,                 // CANcoder ID

                    0.360595703125,     // Offset do CANcoder

                    true,               // Drive invertido?
                    true,               // Steer invertido?
                    false               // CANcoder invertido?
                ),

                3
            );



        //  ============================================================
        //  GYRO REAL
        //  ============================================================

        //  Cria o Pigeon 2 real do drivetrain
        //  CAN ID = 3
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
            //  GYRO SIMULADO
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
        // 4 - ODOMETRIA
        // ==========================================================

        // Calcula X, Y e rotação usando:
        //
        // - Gyro
        // - Distância percorrida pelos quatro módulos
        pose =
            odometry.update(

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

   
    // #region ROTAÇÃO DO ROBÔ

    public Rotation2d getRotation() {

        return Rotation2d.fromRadians(
            gyroInputs.yawPositionRad
        );
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
        // 2 - MANTÉM X E Y DA POSE
        //     MAS ZERA A ROTAÇÃO
        // ==========================================================

        Pose2d newPose =
            new Pose2d(

                pose.getTranslation(),

                new Rotation2d()
            );

        // ==========================================================
        // 3 - SINCRONIZA A ODOMETRIA
        // ==========================================================

        odometry.resetPosition(

            new Rotation2d(),

            getModulePositions(),

            newPose
        );


        pose = newPose;
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

    
    // #region STOP

    public void stop() {

        chassisSpeeds =
            new ChassisSpeeds();
    }

    // #endregion

}



