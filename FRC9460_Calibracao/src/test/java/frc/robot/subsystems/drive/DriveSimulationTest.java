package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.*;

/** Exercita os motores virtuais reais do projeto, sem controladora CAN. */
class DriveSimulationTest {
    private Drive drive;

    @BeforeAll static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }

    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        drive = new Drive();
        tick();
    }

    @AfterEach void cleanup() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        SimHooks.resumeTiming();
    }

    private void tick() {
        SimHooks.stepTiming(0.02);
        drive.periodic();
    }

    @Test void startsAtTwoTwoZeroAndStaysStillWhileDisabled() {
        assertTrue(drive.isSimulationReferenceReady());
        long generation = drive.getPoseResetGeneration();
        for (int i = 0; i < 30; i++) tick();
        assertEquals(new Pose2d(2, 2, new Rotation2d()), drive.getSimulationPose());
        assertEquals(drive.getSimulationPose(), drive.getPose());
        assertEquals(generation, drive.getPoseResetGeneration(), "Inicializa somente uma vez");
    }

    @Test void referenceFollowsTranslationAndRotationFromModules() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        drive.drive(1.0, 0.0, 0.0);
        for (int i = 0; i < 75; i++) tick();
        assertTrue(drive.getSimulationPose().getX() > 2.5);
        assertEquals(drive.getSimulationPose().getX(), drive.getPose().getX(), 1e-6);
        drive.drive(0.0, 0.0, 1.0);
        for (int i = 0; i < 75; i++) tick();
        assertTrue(Math.abs(drive.getSimulationPose().getRotation().getRadians()) > 0.2);
    }

    @Test void visionCorrectionAndSeedCannotMoveTheReference() {
        tick();
        double capture = Timer.getTimestamp();
        tick();
        Pose2d reference = drive.getSimulationPose();
        assertTrue(drive.tryAddVisionMeasurement("Test", new Pose2d(3, 2, new Rotation2d()),
                capture, 0.05));
        assertTrue(drive.getPose().getX() > reference.getX());
        assertEquals(reference, drive.getSimulationPose());
        drive.seedPoseFromVision(new Pose2d(8, 5, new Rotation2d()));
        tick();
        assertEquals(reference, drive.getSimulationPose());
        assertEquals(8.0, drive.getPose().getX(), 1e-6);
    }
}
