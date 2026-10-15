package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Arrays;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.Test;

/** Teste de integração com PhotonLib/OpenCV reais, sem câmeras físicas. */
class VisionSimulationScenarioTest {
    @Test void rotationChangesVisibleCameraAndDiagnosticFramesDoNotEnterFusion() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        try (var scenario = new VisionSimulationScenario()) {
            // Coloca o robô a dois metros da face da tag, olhando para ela.
            // Usa o mapa selecionado, sem inventar coordenadas da arena.
            Pose2d tag = VisionFieldConstants.FIELD_LAYOUT.getTagPose(25).orElseThrow().toPose2d();
            Translation2d position = tag.getTranslation().plus(
                    new Translation2d(2.0, 0).rotateBy(tag.getRotation()));
            Pose2d facingTag = new Pose2d(position,
                    tag.getRotation().plus(Rotation2d.fromDegrees(180)));
            runFrames(scenario, facingTag);
            assertTrue(contains(scenario.getDetectedTagIds(true), 25),
                    "Frontal deve observar a tag 25 à frente dela");
            assertFalse(contains(scenario.getDetectedTagIds(false), 25));
            assertTrue(scenario.getProcessedFrameCount(true) > 0);
            assertTrue(scenario.getProcessedFrameCount(false) > 0);

            Pose2d turned = new Pose2d(position, tag.getRotation());
            runFrames(scenario, turned);
            assertFalse(contains(scenario.getDetectedTagIds(true), 25));
            assertTrue(contains(scenario.getDetectedTagIds(false), 25),
                    "Depois de girar, a traseira deve observar a mesma tag");

            // Fora do alcance das tags, os IDs antigos devem desaparecer.
            runFrames(scenario, new Pose2d(100, 100, new Rotation2d()));
            assertEquals(0, scenario.getDetectedTagIds(true).length);
            assertEquals(0, scenario.getDetectedTagIds(false).length);

            // O cenário é diagnóstico: não pode continuar injetando a antiga pose fixa.
            for (VisionCamera camera : scenario.getCameras()) {
                camera.updateInputs(0, 0);
                assertFalse(camera.inputs.hasPoseEstimate);
                assertFalse(camera.inputs.hasPose);
                assertEquals(0, camera.inputs.tagCount);
            }
        } finally {
            SimHooks.resumeTiming();
        }
    }

    private static void runFrames(VisionSimulationScenario scenario, Pose2d reference) {
        for (int i = 0; i < 40; i++) {
            SimHooks.stepTiming(0.02);
            // Uma estimativa propositalmente errada não pode movimentar o mundo.
            scenario.update(reference, new Pose2d(100, 100, new Rotation2d()));
            NetworkTableInstance.getDefault().flushLocal();
        }
    }

    private static boolean contains(int[] ids, int id) {
        return Arrays.stream(ids).anyMatch(value -> value == id);
    }
}
