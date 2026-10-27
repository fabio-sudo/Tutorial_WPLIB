package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Arrays;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.junit.jupiter.api.Test;

/** Teste de integração com PhotonLib/OpenCV reais, sem câmeras físicas. */
class VisionSimulationScenarioTest {
    @Test void rotationChangesVisibleCameraAndOnlyPhotonEstimatesEnterInputs() {
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
            VisionCamera front = scenario.getCameras()[1];
            front.updateInputs(0, 0);
            assertTrue(front.inputs.usesObservationBatch);
            assertTrue(front.inputs.poseObservations.length > 0);
            for (var observation : front.inputs.poseObservations) {
                assertEquals(PoseObservationType.PHOTONVISION, observation.type());
                assertTrue(observation.pose().toPose2d().getTranslation()
                        .getDistance(facingTag.getTranslation()) < 0.15,
                        "A medição deve vir da geometria PhotonLib, mesmo com estimador em (100,100)");
            }

            Pose2d turned = new Pose2d(position, tag.getRotation());
            runFrames(scenario, turned);
            assertFalse(contains(scenario.getDetectedTagIds(true), 25));
            assertTrue(contains(scenario.getDetectedTagIds(false), 25),
                    "Depois de girar, a traseira deve observar a mesma tag");

            // Fora do alcance das tags, os IDs antigos devem desaparecer.
            runFrames(scenario, new Pose2d(100, 100, new Rotation2d()));
            assertEquals(0, scenario.getDetectedTagIds(true).length);
            assertEquals(0, scenario.getDetectedTagIds(false).length);

            // A perda das tags deve limpar os indicadores da última detecção.
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

    @Test void photonFusionCorrectsDriftAndRecoversWithEitherCameraAlone() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        try (var scenario = new VisionSimulationScenario()) {
            var gyro = new VisionMultiCameraTest.SensorGyro();
            var modules = new VisionMultiCameraTest.SensorModule[] {
                new VisionMultiCameraTest.SensorModule(), new VisionMultiCameraTest.SensorModule(),
                new VisionMultiCameraTest.SensorModule(), new VisionMultiCameraTest.SensorModule()
            };
            var drive = new Drive(gyro, modules[0], modules[1], modules[2], modules[3]);
            drive.periodic();
            Pose2d tag = VisionFieldConstants.FIELD_LAYOUT.getTagPose(25).orElseThrow().toPose2d();
            Translation2d position = tag.getTranslation().plus(
                    new Translation2d(2.0, 0).rotateBy(tag.getRotation()));
            Pose2d reference = new Pose2d(position,
                    tag.getRotation().plus(Rotation2d.fromDegrees(180)));
            drive.seedPoseFromVision(new Pose2d(position.plus(new Translation2d(0.6, 0.4)),
                    new Rotation2d()));
            long generation = drive.getPoseResetGeneration();
            var vision = new Vision(drive, scenario.getCameras());
            double initialError = drive.getPose().getTranslation().getDistance(position);
            double initialHeading = drive.getRotation().getRadians();

            // A frontal deve funcionar com a traseira desligada.
            scenario.setCameraConnected(false, false);
            runFusionFrames(scenario, drive, vision, reference, 180);
            assertTrue(drive.getVisionAcceptedCount("Front") > 20);
            assertEquals(0, drive.getVisionAcceptedCount("Rear"));
            double correctedError = drive.getPose().getTranslation().getDistance(position);
            assertTrue(correctedError < 0.10 && correctedError < initialError * 0.25,
                    "As estimativas PhotonLib devem corrigir o erro de X/Y do Drive");
            assertEquals(initialHeading, drive.getRotation().getRadians(), 1e-9);

            // Sem as duas câmeras, uma deriva deliberada nos encoders permanece.
            scenario.setCameraConnected(true, false);
            runFusionFrames(scenario, drive, vision, reference, 20);
            long frontCount = drive.getVisionAcceptedCount("Front");
            for (var module : modules) module.distanceMeters = 0.8;
            runFusionFrames(scenario, drive, vision, reference, 20);
            assertEquals(frontCount, drive.getVisionAcceptedCount("Front"));
            assertEquals(0, drive.getVisionAcceptedCount("Rear"));
            assertTrue(drive.getPose().getTranslation().getDistance(position) > 0.5);

            // Gira o robô no mundo e o gyro na mesma quantidade. Agora a traseira
            // vê a tag 25; deve recuperar o Drive com a frontal ainda desligada.
            gyro.yaw += Math.PI;
            Pose2d turned = new Pose2d(position,
                    reference.getRotation().plus(Rotation2d.fromDegrees(180)));
            scenario.setCameraConnected(false, true);
            runFusionFrames(scenario, drive, vision, turned, 180);
            assertTrue(drive.getVisionAcceptedCount("Rear") > 20);
            assertEquals(frontCount, drive.getVisionAcceptedCount("Front"));
            assertTrue(drive.getPose().getTranslation().getDistance(position) < 0.10);
            assertEquals(generation, drive.getPoseResetGeneration(),
                    "Fusão não deve disparar resets de pose");
        } finally {
            CommandScheduler.getInstance().unregisterAllSubsystems();
            SimHooks.resumeTiming();
        }
    }

    private static void runFusionFrames(VisionSimulationScenario scenario, Drive drive,
            Vision vision, Pose2d reference, int count) {
        for (int i = 0; i < count; i++) {
            SimHooks.stepTiming(0.02);
            drive.periodic();
            vision.periodic();
            scenario.update(reference, drive.getPose());
            NetworkTableInstance.getDefault().flushLocal();
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
