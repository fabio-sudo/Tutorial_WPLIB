package frc.robot.subsystems.vision;

// #region NEW - IMPORTS

import frc.robot.Constants.LimelightConstants;

// #endregion

public class VisionIOLimelightRear extends VisionIOLimelight {

    // #region NEW - CONFIGURAÇÃO DA LIMELIGHT TRASEIRA

    public VisionIOLimelightRear() {
        super(
            LimelightConstants.REAR_NAME,

            LimelightConstants.REAR_FORWARD,
            LimelightConstants.REAR_RIGHT,
            LimelightConstants.REAR_UP,

            LimelightConstants.REAR_ROLL,
            LimelightConstants.REAR_PITCH,
            LimelightConstants.REAR_YAW,

            LimelightConstants.VALID_TAG_IDS
        );
    }

    // #endregion
}