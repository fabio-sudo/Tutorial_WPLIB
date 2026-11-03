package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

/** Ordem e eixos das peças móveis do modelo 3D (model_0 intake, model_1 shooter). */
class RobotComponentsVisualizerTest {

    @Test
    void posesFollowTheModelOrderAndAxes() {
        Pose3d[] closedCentered = RobotComponentsVisualizer.componentPoses(90.0, 0.0);
        assertEquals(2, closedCentered.length);
        assertEquals(IntakeConstants.PIVOT_IN_ROBOT, closedCentered[0].getTranslation());
        assertEquals(ShooterConstants.TURRET_PIVOT_IN_ROBOT, closedCentered[1].getTranslation());

        // Intake: Rotation3d(0, -subida, 0) levanta.
        assertEquals(-90.0, Math.toDegrees(closedCentered[0].getRotation().getY()), 1e-9);
        assertEquals(0.0, Math.toDegrees(RobotComponentsVisualizer.componentPoses(0.0, 0.0)[0]
            .getRotation().getY()), 1e-9);

        // Shooter: o CAD aponta para 175,5°; o zero do código é 180°.
        double cadOffset = ShooterConstants.PIVOT_ZERO_FACING_DEGREES - ShooterConstants.CAD_ZERO_FACING_DEGREES;
        assertEquals(cadOffset, Math.toDegrees(closedCentered[1].getRotation().getZ()), 1e-9);
        assertEquals(cadOffset + 30.0, Math.toDegrees(RobotComponentsVisualizer.componentPoses(90.0, 30.0)[1]
            .getRotation().getZ()), 1e-9);
    }
}
