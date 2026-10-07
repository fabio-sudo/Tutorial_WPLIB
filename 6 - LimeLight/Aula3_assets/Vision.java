package frc.robot.subsystems.vision;


// #region IMPORTS

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;


import frc.robot.subsystems.drive.Drive;

// #endregion



public class Vision extends SubsystemBase {


    // #region DRIVE

    private final Drive drive;

    // #endregion


    // #region CONSTRUTOR

    public Vision(Drive drive) {
        
        //===========================================DRIVE
        this.drive = drive;

        // ========================================================
        // POSIÇÃO FÍSICA DA LIMELIGHT NO ROBÔ
        // ========================================================
        //
        // Câmera utilizada:
        //
        // Limelight traseira
        //
        // Os valores estão armazenados no Constants.java.
        //
        // REAR_FORWARD
        // REAR_RIGHT
        // REAR_UP
        //
        // REAR_ROLL
        // REAR_PITCH
        // REAR_YAW
        //
        // ========================================================

        LimelightHelpers.setCameraPose_RobotSpace(

            LimelightConstants.REAR_NAME,

            LimelightConstants.REAR_FORWARD,

            LimelightConstants.REAR_RIGHT,

            LimelightConstants.REAR_UP,

            LimelightConstants.REAR_ROLL,

            LimelightConstants.REAR_PITCH,

            LimelightConstants.REAR_YAW
        );


        // ========================================================
        // FILTRO DE APRILTAGS
        // ========================================================
        //
        // Atualmente nossa arena de teste utiliza:
        //
        // Tag 25
        // Tag 26
        //
        // Qualquer outra AprilTag será ignorada pela Limelight
        // para localização.
        //
        // ========================================================

        LimelightHelpers.SetFiducialIDFiltersOverride(

            LimelightConstants.REAR_NAME,

            LimelightConstants.VALID_TAG_IDS
        );


    }

    // #endregion


    // #region PERIODIC

    @Override
    public void periodic() {


        // #region LEITURA DA LIMELIGHT

        // ========================================================
        // VERIFICA SE EXISTE ALVO
        // ========================================================

        boolean hasTarget =

            LimelightHelpers.getTV(
                LimelightConstants.REAR_NAME
            );


        // ========================================================
        // ÂNGULO HORIZONTAL
        // ========================================================
        //
        // Negativo = alvo para um lado
        // Positivo = alvo para o outro
        //
        // Vamos validar fisicamente o sentido depois.
        //
        // ========================================================

        double tx =

            LimelightHelpers.getTX(
                LimelightConstants.REAR_NAME
            );


        // ========================================================
        // ÂNGULO VERTICAL
        // ========================================================

        double ty =

            LimelightHelpers.getTY(
                LimelightConstants.REAR_NAME
            );


        // ========================================================
        // ÁREA DO ALVO NA IMAGEM
        // ========================================================

        double ta =

            LimelightHelpers.getTA(
                LimelightConstants.REAR_NAME
            );


        // ========================================================
        // ID DA APRILTAG
        // ========================================================

        double tagId =

            LimelightHelpers.getFiducialID(
                LimelightConstants.REAR_NAME
            );



        // ========================================================
        // ORIENTAÇÃO DO ROBÔ - PIGEON 2
        // ========================================================

        double robotYawDegrees =

            drive
                .getRotation()
                .getDegrees();

                // ========================================================
                // ENVIA A ORIENTAÇÃO DO ROBÔ PARA A LIMELIGHT
                // ========================================================

                LimelightHelpers.SetRobotOrientation(

                    LimelightConstants.REAR_NAME,

                    robotYawDegrees,

                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                );

                // ========================================================
                // POSE DO ROBÔ CALCULADA PELO MEGATAG2
                // ========================================================

                LimelightHelpers.PoseEstimate megaTag2Estimate =

                    LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(
                            LimelightConstants.REAR_NAME
                        );


    // #endregion
    
    
        // #region ADVANTAGEKIT


            Logger.recordOutput(
                "Vision/Rear/RobotYawDegrees",
                robotYawDegrees
            );


            Logger.recordOutput(
                "Vision/Rear/HasTarget",
                hasTarget
            );


            Logger.recordOutput(
                "Vision/Rear/TX",
                tx
            );


            Logger.recordOutput(
                "Vision/Rear/TY",
                ty
            );


            Logger.recordOutput(
                "Vision/Rear/TA",
                ta
            );


            Logger.recordOutput(
                "Vision/Rear/TagID",
                tagId
            );

        

            // ========================================================
            // MEGATAG2
            // ========================================================

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/Pose",
                megaTag2Estimate.pose
            );

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/X",
                megaTag2Estimate.pose.getX()
            );

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/Y",
                megaTag2Estimate.pose.getY()
            );

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/RotationDegrees",
                megaTag2Estimate.pose
                    .getRotation()
                    .getDegrees()
            );

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/TagCount",
                megaTag2Estimate.tagCount
            );

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/AvgTagDist",
                megaTag2Estimate.avgTagDist
            );
        // #endregion

    }

    // #endregion


    // #region GETTERS

    public boolean hasTarget() {

        return LimelightHelpers.getTV(
            LimelightConstants.REAR_NAME
        );
    }


    public double getTX() {

        return LimelightHelpers.getTX(
            LimelightConstants.REAR_NAME
        );
    }


    public double getTY() {

        return LimelightHelpers.getTY(
            LimelightConstants.REAR_NAME
        );
    }


    public double getTA() {

        return LimelightHelpers.getTA(
            LimelightConstants.REAR_NAME
        );
    }


    public int getTagID() {

        return (int) LimelightHelpers.getFiducialID(
            LimelightConstants.REAR_NAME
        );
    }

    // #endregion
}