package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Confere a conversão do perfil da lente para os parâmetros da PhotonLib. */
class VisionSimulationConfigTest {
    @Test
    void limelight4UsesHorizontalAndVerticalFovInsteadOfTreatingThemAsDiagonal() {
        var properties = VisionSimulationScenario.createCameraProperties(VisionSimulationConstants.REAR);
        assertEquals(1280, properties.getResWidth());
        assertEquals(800, properties.getResHeight());
        assertEquals(82.0, properties.getHorizFOV().getDegrees(), 1e-8);
        assertEquals(56.2, properties.getVertFOV().getDegrees(), 1e-8);
        assertEquals(60.0, properties.getFPS(), 1e-8);
        assertEquals(0.0, properties.getPixelYaw(640).getDegrees(), 1e-8);
        assertEquals(41.0, Math.abs(properties.getPixelYaw(0).getDegrees()), 1e-8);
    }

    @Test
    void sharedProfileDoesNotShareMutableCameraPropertiesBetweenCameras() {
        var rear = VisionSimulationScenario.createCameraProperties(VisionSimulationConstants.REAR);
        var front = VisionSimulationScenario.createCameraProperties(VisionSimulationConstants.FRONT);
        rear.setFPS(15);
        assertEquals(60.0, front.getFPS(), 1e-8);
        assertEquals(82.0, front.getHorizFOV().getDegrees(), 1e-8);
    }
}
