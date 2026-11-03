package frc.robot.subsystems.vision;

// #region NEW - IMPORTS


// #endregion

public class VisionIOLimelightFront extends VisionIOLimelight {

    // #region 📷 NEW - CONFIGURAÇÃO DA LIMELIGHT FRONTAL

    public VisionIOLimelightFront() {
        super(
            LimelightConstants.FRONT_NAME,

            LimelightConstants.FRONT_FORWARD,
            LimelightConstants.FRONT_RIGHT,
            LimelightConstants.FRONT_UP,

            LimelightConstants.FRONT_ROLL,
            LimelightConstants.FRONT_PITCH,
            LimelightConstants.FRONT_YAW,

            LimelightConstants.FRONT_VALID_TAG_IDS
        );
    }

    // #endregion
}