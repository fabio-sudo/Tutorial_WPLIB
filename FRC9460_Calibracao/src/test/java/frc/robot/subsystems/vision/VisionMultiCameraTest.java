package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.littletonrobotics.junction.LogTable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Cenários de comportamento: fusão real da WPILib com entradas controladas. */
class VisionMultiCameraTest {
    double now;
    Drive drive;
    Vision vision;
    VisionIOSim rear;
    VisionIOSim front;
    SensorGyro gyro;
    SensorModule[] modules;

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        now = 10.0;
        gyro = new SensorGyro();
        modules = new SensorModule[] {new SensorModule(), new SensorModule(),
                new SensorModule(), new SensorModule()};
        drive = new Drive(gyro, modules[0], modules[1], modules[2], modules[3], () -> now);
        rear = new VisionIOSim(() -> now);
        front = new VisionIOSim(() -> now);
        vision = new Vision(drive, () -> now,
                new VisionCamera("Rear", rear), new VisionCamera("Front", front));
        drive.periodic();
        tick(10.10);
    }

    @AfterEach
    void cleanup() {
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    void tick(double time) {
        now = time;
        drive.periodic();
        vision.periodic();
    }

    void frame(VisionIOSim io, double x, int count, int id, double timestamp) {
        io.publishFrame(new Pose2d(x, 0.0, new Rotation2d()), count, id, 1.8, timestamp, 20.0);
    }

    @Test
    void differentTagsAtExactlySameTimeBothContribute() {
        frame(rear, 1.0, 1, 9, 10.08);
        frame(front, 1.05, 2, 25, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
        assertTrue(drive.getPose().getX() > 0.1);
    }

    @Test
    void sameTagsAtSameTimeDoNotOverwriteOrResetTheEstimator() {
        frame(rear, 1.0, 2, 9, 10.08);
        frame(front, 1.02, 2, 9, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
        assertTrue(drive.getPose().getX() > 0.1 && drive.getPose().getX() < 1.0);
        assertEquals(0, drive.getPoseResetGeneration());
    }

    @Test
    void rearContinuesWhileFrontIsDisconnected() {
        front.setConnected(false);
        frame(rear, 1.0, 2, 9, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(0, drive.getVisionAcceptedCount("Front"));
    }

    @Test
    void frontContinuesWhileRearIsDisconnected() {
        rear.setConnected(false);
        frame(front, 1.0, 2, 25, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
    }

    @Test
    void oneCameraWithoutTagsDoesNotBlockTheOtherAndCanRecover() {
        frame(rear, 1.0, 2, 9, 10.08);
        frame(front, 50.0, 0, -1, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(0, drive.getVisionAcceptedCount("Front"));
        frame(front, 1.02, 2, 25, 10.18);
        vision.periodic();
        tick(10.31);
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
    }

    @Test
    void frozenFrameIsNeverFusedTwice() {
        frame(rear, 1.0, 2, 9, 10.08);
        vision.periodic();
        tick(10.20);
        Pose2d once = drive.getPose();
        tick(10.22);
        tick(10.30);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(once, drive.getPose());
    }

    @Test
    void reverseArrivalAcrossCyclesIsSortedByCaptureTime() {
        frame(rear, 1.0, 2, 9, 10.08);
        vision.periodic();
        tick(10.12);
        frame(front, 1.02, 2, 25, 10.06);
        vision.periodic();
        tick(10.22);
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
    }

    @Test
    void excessivelyLateFrameDoesNotEraseAnAppliedCorrection() {
        frame(rear, 1.0, 2, 9, 10.08);
        vision.periodic();
        tick(10.20);
        Pose2d corrected = drive.getPose();
        frame(front, 1.02, 2, 25, 10.06);
        vision.periodic();
        assertEquals("OUT_OF_ORDER", vision.getFusionResult("Front"));
        assertEquals(0, drive.getVisionAcceptedCount("Front"));
        assertEquals(corrected, drive.getPose());
    }

    @Test
    void contradictoryPairIsRejectedAndNextCoherentPairRecovers() {
        frame(rear, 1.0, 2, 9, 10.08);
        frame(front, 6.0, 2, 25, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals("CAMERAS_DISAGREE", vision.getFusionResult("Rear"));
        assertEquals("CAMERAS_DISAGREE", vision.getFusionResult("Front"));
        assertEquals(0.0, drive.getPose().getX(), 1e-9);
        frame(rear, 1.0, 2, 9, 10.18);
        frame(front, 1.02, 2, 25, 10.18);
        vision.periodic();
        tick(10.31);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
    }

    @Test
    void futureAndStaleFramesAreRejectedWithoutBlockingHealthyCamera() {
        frame(rear, 1.0, 2, 9, 10.50);
        frame(front, 1.0, 2, 25, 9.00);
        vision.periodic();
        tick(10.65);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
        assertEquals(0, drive.getVisionAcceptedCount("Front"));
        frame(front, 1.0, 2, 25, 10.63);
        vision.periodic();
        tick(10.76);
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
    }

    @Test
    void slightFutureWithinToleranceWaitsForOdometryInsteadOfApplyingEarly() {
        frame(rear, 1.0, 2, 9, 10.11);
        vision.periodic();
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
        tick(10.23);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
    }

    @Test
    void nonFiniteCameraDataCannotPoisonThePose() {
        frame(rear, Double.NaN, 2, 9, 10.08);
        frame(front, 1.0, 2, 25, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
        assertTrue(Double.isFinite(drive.getPose().getX()));
    }

    @Test
    void poseResetDiscardsPendingFramesAndDoesNotRequeueTheSameFrame() {
        frame(rear, 10.0, 2, 9, 10.08);
        vision.periodic();
        drive.seedPoseFromVision(new Pose2d(3.0, 4.0, new Rotation2d()));
        tick(10.20);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
        assertEquals(3.0, drive.getPose().getX(), 1e-9);
    }

    @Test
    void noVisionMeansNormalOdometry() {
        for (SensorModule module : modules) {
            module.distanceMeters = 1.0;
        }
        tick(10.20);
        assertEquals(1.0, drive.getPose().getX(), 1e-9);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
    }

    @Test
    void megaTagYawDoesNotChangeFieldOrDriverHeading() {
        gyro.yaw = 0.7;
        tick(10.12);
        drive.zeroDriverHeading();
        rear.publishFrame(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(170)),
                2, 9, 1.8, 10.12, 20);
        vision.periodic();
        tick(10.25);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(0.7, drive.getRotation().getRadians(), 1e-9);
        assertEquals(0.0, drive.getDriverRotation().getRadians(), 1e-9);
    }

    @Test
    void duplicateCameraNamesAreRejectedAtConstruction() {
        assertThrows(IllegalArgumentException.class, () -> new Vision(drive, () -> now,
                new VisionCamera("Rear", rear), new VisionCamera("Rear", front)));
    }

    @Test
    void frontOnlyConfigurationWorks() {
        vision = new Vision(drive, () -> now, new VisionCamera("Front", front));
        frame(front, 1.0, 2, 25, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
        assertTrue(vision.hasTarget("Front"));
    }

    @Test
    void manualSeedRequiresStationaryRecentMultiTagFrame() {
        frame(rear, 1.0, 2, 9, 10.08);
        vision.periodic();
        tick(10.20);
        modules[0].speed = 0.2;
        drive.periodic();
        vision.trySeedDrivePose();
        assertEquals(0, drive.getPoseResetGeneration());
        modules[0].speed = 0.0;
        drive.periodic();
        vision.trySeedDrivePose();
        assertEquals(1, drive.getPoseResetGeneration());
        assertEquals(1.0, drive.getPose().getX(), 1e-9);
    }

    @Test
    void gyroLossPreventsVisionFusion() {
        gyro.connected = false;
        drive.periodic();
        frame(rear, 1.0, 2, 9, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
    }

    @Test
    void megaTag2OutsideTheFieldNeverReachesTheEstimator() {
        double[][] outside = {{1.0, -0.5}, {1.0, VisionFieldConstants.FIELD_WIDTH_METERS + 0.5},
            {-0.5, 1.0}, {VisionFieldConstants.FIELD_LENGTH_METERS + 0.5, 1.0}};
        double capture = 10.08;
        for (double[] point : outside) {
            rear.publishFrame(new Pose2d(point[0], point[1], new Rotation2d()), 2, 9, 1.8, capture, 20);
            vision.periodic();
            tick(capture + 0.12);
            assertEquals(0, drive.getVisionAcceptedCount("Rear"), point[0] + ", " + point[1]);
            capture += 0.13;
        }
        assertTrue(vision.getDiagnosis("Rear").endsWith("OutsideField"), vision.getDiagnosis("Rear"));
    }

    @Test
    void healthyFramesFromSameCameraRecoverAfterRejectedFutureTimestamp() {
        frame(rear, 1.0, 2, 9, 1000.0);
        vision.periodic();
        tick(10.20);
        frame(rear, 1.0, 2, 9, 10.18);
        vision.periodic();
        tick(10.31);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"),
                "Rejeitar um timestamp futuro nao pode bloquear frames validos posteriores");
    }

    @Test
    void frozenRejectedFutureFrameDoesNotBecomeNewWhenTimeCatchesUp() {
        frame(rear, 1.0, 2, 9, 10.50);
        vision.periodic();
        tick(10.49);
        tick(10.62);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
    }

    private PoseObservation photon(double x, double timestamp, int tagCount, double ambiguity) {
        return new PoseObservation(timestamp,
                new Pose3d(x, 0.5, 0.0, new Rotation3d()), ambiguity,
                tagCount, 1.8, PoseObservationType.PHOTONVISION);
    }

    @Test
    void photonCamerasSeeingDifferentTagsAtSameTimeBothContribute() {
        rear.publishObservation(photon(1.0, 10.08, 2, 0), 25, 20);
        front.publishObservation(photon(1.02, 10.08, 2, 0), 9, 20);
        vision.periodic();
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
        assertTrue(drive.getPose().getX() > 0.1);
        assertEquals(0, drive.getPoseResetGeneration());
    }

    @Test
    void allPhotonFramesInOneBatchAreSortedAndDuplicatesAreNotAppliedTwice() {
        var latest = photon(1.0, 10.08, 2, 0.0);
        rear.publishObservation(latest, 25, 20.0);
        rear.publishObservation(photon(1.0, 10.04, 2, 0.0), 25, 60.0);
        rear.publishObservation(photon(1.0, 10.06, 2, 0.0), 25, 40.0);
        rear.publishObservation(latest, 25, 20.0);
        // A última observação é inválida. Não pode apagar as três anteriores.
        rear.publishObservation(photon(1.0, 10.50, 2, 0.0), 25, 0.0);
        vision.periodic();
        tick(10.25);
        assertEquals(3, drive.getVisionAcceptedCount("Rear"));
        Pose2d corrected = drive.getPose();
        tick(10.27);
        assertEquals(3, drive.getVisionAcceptedCount("Rear"));
        assertEquals(corrected, drive.getPose());
    }

    @Test
    void ambiguousSingleTagCannotBlockAnotherCameraAndCanRecover() {
        rear.publishObservation(photon(1.0, 10.08, 1, 0.8), 25, 20.0);
        front.publishObservation(photon(1.0, 10.08, 2, 0.8), 9, 20.0);
        vision.periodic();
        tick(10.20);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
        rear.publishObservation(photon(1.0, 10.18, 1, 0.05), 25, 20.0);
        vision.periodic();
        tick(10.31);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
    }

    @Test
    void photonGeometryAndNumericalOutliersCannotPoisonHealthyFrames() {
        Pose3d[] invalidPoses = {
            new Pose3d(1, 0.5, 1.0, new Rotation3d()),
            new Pose3d(1, 0.5, 0, new Rotation3d(Math.toRadians(45), 0, 0)),
            new Pose3d(-1, 0.5, 0, new Rotation3d()),
            new Pose3d(1, VisionFieldConstants.FIELD_WIDTH_METERS + 1, 0, new Rotation3d()),
            new Pose3d(Double.NaN, 0.5, 0, new Rotation3d())
        };
        for (int i = 0; i < invalidPoses.length; i++) {
            rear.publishObservation(new PoseObservation(10.01 + 0.01 * i,
                    invalidPoses[i], 0.0, 2, 1.8, PoseObservationType.PHOTONVISION), 25, 20);
        }
        rear.publishObservation(new PoseObservation(10.07,
                new Pose3d(1, 0.5, 0, new Rotation3d()), 0, 2, Double.NaN,
                PoseObservationType.PHOTONVISION), 25, 20);
        front.publishObservation(photon(1, 10.08, 2, 0), 9, 20);
        vision.periodic();
        tick(10.25);
        assertEquals(0, drive.getVisionAcceptedCount("Rear"));
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
        assertTrue(Double.isFinite(drive.getPose().getX()));
    }

    @Test
    void cameraDisconnectWhileReorderingDoesNotApplyPendingObservations() {
        rear.publishObservation(photon(1, 10.08, 2, 0), 25, 20);
        front.publishObservation(photon(1, 10.08, 2, 0), 9, 20);
        vision.periodic();
        front.setConnected(false);
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals(0, drive.getVisionAcceptedCount("Front"));
        front.setConnected(true);
        front.publishObservation(photon(1, 10.18, 2, 0), 9, 20);
        vision.periodic();
        tick(10.31);
        assertEquals(1, drive.getVisionAcceptedCount("Front"));
    }

    @Test
    void photonCorrectionUsesCaptureHistoryAndPreservesGyroHeading() {
        rear.publishObservation(new PoseObservation(10.08,
                new Pose3d(1, 0, 0, new Rotation3d(0, 0, Math.PI / 2)),
                0, 2, 1.8, PoseObservationType.PHOTONVISION), 25, 20);
        vision.periodic();
        // Um metro percorrido depois da captura deve permanecer na pose atual.
        for (var module : modules) module.distanceMeters = 1.0;
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertTrue(drive.getPose().getX() > 1.1);
        assertEquals(0.0, drive.getRotation().getRadians(), 1e-9);
    }

    @Test
    void individualLimelightInputsRemainCompatibleWithBatchConsumer() {
        VisionIO individual = new VisionIO() {
            public void updateInputs(VisionIOInputs inputs) {
                inputs.connected = true;
                inputs.hasPoseEstimate = true;
                inputs.hasPose = true;
                inputs.hasTarget = true;
                inputs.pose = new Pose2d(1, 0, new Rotation2d());
                inputs.tagCount = 2;
                inputs.avgTagDistanceMeters = 1.8;
                inputs.captureTimestampSeconds = 10.08;
                inputs.readTimestampSeconds = now;
            }
        };
        vision = new Vision(drive, () -> now, new VisionCamera("Individual", individual));
        vision.periodic();
        tick(10.20);
        tick(10.22);
        assertEquals(1, drive.getVisionAcceptedCount("Individual"));
        assertTrue(drive.getPose().getX() > 0.1);
    }

    @Test
    void emptyBatchNeverFallsBackToIndividualPoseFields() {
        VisionIO emptyBatch = new VisionIO() {
            public void updateInputs(VisionIOInputs inputs) {
                inputs.connected = true;
                inputs.usesObservationBatch = true;
                inputs.hasPoseEstimate = true;
                inputs.hasPose = true;
                inputs.pose = new Pose2d(10, 0, new Rotation2d());
                inputs.tagCount = 2;
                inputs.avgTagDistanceMeters = 1.8;
                inputs.captureTimestampSeconds = now - 0.02;
                inputs.readTimestampSeconds = now;
            }
        };
        vision = new Vision(drive, () -> now, new VisionCamera("Empty", emptyBatch));
        vision.periodic();
        tick(10.25);
        assertEquals(0, drive.getVisionAcceptedCount("Empty"));
        assertEquals(0.0, drive.getPose().getX(), 1e-9);
    }

    @Test
    void observationsRecoveredFromAdvantageKitLogReachDriveExactlyOnce() {
        var recorded = new VisionIOInputsAutoLogged();
        rear.publishObservation(photon(1, 10.08, 2, 0), 25, 20);
        rear.updateInputs(recorded);
        var table = new LogTable(10_100_000);
        recorded.toLog(table);
        VisionIO loggedInputs = new VisionIO() {
            public void updateInputs(VisionIOInputs inputs) {
                ((VisionIOInputsAutoLogged) inputs).fromLog(table);
            }
        };
        vision = new Vision(drive, () -> now, new VisionCamera("Logged", loggedInputs));
        vision.periodic();
        tick(10.20);
        tick(10.22);
        assertEquals(1, drive.getVisionAcceptedCount("Logged"));
        assertTrue(drive.getPose().getX() > 0.1);
    }

    // 29/10/2026: a Limelight mostrava tags, a pose não era corrigida e o
    // motivo ficava espalhado em vários Reject/*. Vision/Diagnostico diz qual é.
    @Test
    void diagnosisNamesWhatBlocksTheCorrection() {
        assertTrue(vision.getDiagnosis("Rear").startsWith("SEM_TAG_ACEITA"), vision.getDiagnosis("Rear"));

        frame(rear, 1.0, 2, 9, 10.08);
        vision.periodic();
        tick(10.20);
        assertEquals(1, drive.getVisionAcceptedCount("Rear"));
        assertEquals("OK_CORRIGINDO", vision.getDiagnosis("Rear"));
        assertTrue(vision.getDiagnosis().startsWith("Rear: OK_CORRIGINDO"), vision.getDiagnosis());

        rear.setConnected(false);
        tick(10.30);
        assertTrue(vision.getDiagnosis("Rear").startsWith("SEM_CONEXAO"), vision.getDiagnosis("Rear"));

        rear.setConnected(true);
        gyro.connected = false;
        frame(rear, 1.0, 2, 9, 10.38);
        tick(10.40);
        assertTrue(vision.getDiagnosis("Rear").startsWith("GIROSCOPIO_INDISPONIVEL"),
                vision.getDiagnosis("Rear"));
    }

    static class SensorGyro implements GyroIO {
        double yaw;
        boolean connected = true;
        public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = connected;
            inputs.configurationOk = true;
            inputs.yawPositionRad = yaw;
            inputs.yawVelocityRadPerSec = 0.0;
        }
    }

    static class SensorModule implements ModuleIO {
        double distanceMeters;
        double speed;
        public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.encoderConnected = true;
            inputs.drivePositionRad = distanceMeters / DriveConstants.kWheelRadiusMeters;
            inputs.driveVelocityRadPerSec = speed / DriveConstants.kWheelRadiusMeters;
            inputs.turnPositionRad = 0.0;
        }
    }
}
