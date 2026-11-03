package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.DriveConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Regressões de freio e referências angulares com sensores controlados. */
class DriveControlsTest {
    private static final double EPSILON = 1e-6;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private FakeGyro gyro;
    private FakeModule[] modules;
    private Drive drive;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        setEnabled(false);
        gyro = new FakeGyro();
        modules = new FakeModule[] {
            new FakeModule(), new FakeModule(), new FakeModule(), new FakeModule()
        };
        drive = new Drive(gyro, modules[0], modules[1], modules[2], modules[3]);
        drive.periodic();
    }

    @AfterEach
    void cleanup() {
        scheduler.cancelAll();
        scheduler.unregisterAllSubsystems();
        setEnabled(false);
    }

    private void setEnabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    @Test
    void xLockLeavesCalibrationAndRequestsZeroSpeedOnBothDiagonals() {
        setEnabled(true);
        drive.setCalibrationDriveVoltage(1.0);
        drive.periodic();
        assertEquals(1.0, modules[0].driveVoltage, EPSILON);

        drive.stopWithX();
        drive.periodic();
        assertTrue(drive.isXLockActive());
        double[] headings = {45.0, -45.0, 135.0, -135.0};
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, modules[i].velocitySetpoint, EPSILON);
            // Optimize pode escolher um ângulo equivalente a 180 graus.
            assertEquals(0.0,
                    Math.sin(modules[i].turnSetpoint - Math.toRadians(headings[i])), EPSILON);
        }

        drive.releaseXLock();
        drive.drive(0.25, 0.0, 0.0);
        drive.periodic();
        assertFalse(drive.isXLockActive());
        assertEquals(0.25, modules[0].velocitySetpoint, EPSILON);
    }

    @Test
    void brakeInterruptsTestAndRejectsNewDriveCommandsUntilReleased() {
        setEnabled(true);
        Command calibration = drive.runEnd(
                () -> drive.setCalibrationDriveVoltage(1.0), drive::stopCalibration);
        scheduler.schedule(calibration);
        scheduler.run();
        assertTrue(calibration.isScheduled());

        Command brake = drive.xLockCommand();
        scheduler.schedule(brake);
        scheduler.run();
        assertFalse(calibration.isScheduled());
        assertTrue(brake.isScheduled());

        Command otherTest = drive.run(() -> drive.setCalibrationDriveVoltage(2.0));
        scheduler.schedule(otherTest);
        assertFalse(otherTest.isScheduled());
        assertTrue(brake.isScheduled());

        brake.cancel();
        assertFalse(drive.isXLockActive());
        assertFalse(calibration.isScheduled());
        scheduler.schedule(otherTest);
        assertTrue(otherTest.isScheduled());
    }

    @Test
    void disableCancelsBrakeAndRemovesDriveAndSteerOutputs() {
        setEnabled(true);
        Command brake = drive.xLockCommand();
        scheduler.schedule(brake);
        scheduler.run();
        setEnabled(false);
        scheduler.run();
        assertFalse(brake.isScheduled());
        assertFalse(drive.isXLockActive());
        for (FakeModule module : modules) {
            assertEquals(0.0, module.driveVoltage, EPSILON);
            assertEquals(0.0, module.turnVoltage, EPSILON);
        }
    }

    @Test
    void fieldZeroPreservesTranslationAndSurvivesNextSensorRead() {
        gyro.yaw = Math.toRadians(73.0);
        drive.periodic();
        drive.seedPoseFromVision(new Pose2d(3.0, 4.0, new Rotation2d()));
        drive.zeroHeading();
        assertEquals(0.0, drive.getRotation().getDegrees(), EPSILON);
        assertEquals(Math.toRadians(73.0), gyro.yaw, EPSILON);
        for (int i = 0; i < 5; i++) {
            drive.periodic();
            assertEquals(0.0, drive.getRotation().getDegrees(), EPSILON);
            assertEquals(3.0, drive.getPose().getX(), EPSILON);
            assertEquals(4.0, drive.getPose().getY(), EPSILON);
        }
        gyro.yaw += Math.PI / 2.0;
        drive.periodic();
        assertEquals(90.0, drive.getRotation().getDegrees(), EPSILON);
    }

    @Test
    void fieldHeading180ForTheRobotFacingTheBlueWall() {
        // Start + A: robô de costas para o hub azul (Limelight traseira nas tags 25/26).
        gyro.yaw = Math.toRadians(73.0);
        drive.periodic();
        drive.seedPoseFromVision(new Pose2d(2.0, 4.0, new Rotation2d()));
        drive.setFieldHeading(Rotation2d.k180deg);
        assertEquals(180.0, Math.abs(drive.getRotation().getDegrees()), EPSILON);
        for (int i = 0; i < 5; i++) {
            drive.periodic();
            assertEquals(180.0, Math.abs(drive.getRotation().getDegrees()), EPSILON);
            assertEquals(2.0, drive.getPose().getX(), EPSILON);
        }
        // Girar 90° anti-horário a partir de 180° leva a -90° no campo.
        gyro.yaw += Math.PI / 2.0;
        drive.periodic();
        assertEquals(-90.0, drive.getRotation().getDegrees(), EPSILON);
        // Habilitado é recusado, como o Start + Y.
        setEnabled(true);
        drive.setFieldHeading(new Rotation2d());
        assertEquals(-90.0, drive.getRotation().getDegrees(), EPSILON);
    }

    @Test
    void driverZeroPreservesFieldHeadingAndChangesOnlyDrivingReference() {
        gyro.yaw = Math.PI / 2.0;
        drive.periodic();
        drive.seedPoseFromVision(new Pose2d(3.0, 4.0, new Rotation2d()));
        Pose2d fieldPose = drive.getPose();
        drive.zeroDriverHeading();
        assertEquals(fieldPose, drive.getPose());
        assertEquals(0.0, drive.getDriverRotation().getDegrees(), EPSILON);

        setEnabled(true);
        drive.driveDriverRelative(0.25, 0.0, 0.0);
        drive.periodic();
        assertEquals(0.25, modules[0].velocitySetpoint, EPSILON);
        assertEquals(0.0, modules[0].turnSetpoint, EPSILON);
        assertEquals(90.0, drive.getRotation().getDegrees(), EPSILON);

        gyro.yaw += Math.PI / 4.0;
        drive.periodic();
        assertEquals(135.0, drive.getRotation().getDegrees(), EPSILON);
        assertEquals(45.0, drive.getDriverRotation().getDegrees(), EPSILON);
    }

    @Test
    void visionSeedPreservesFieldOffsetAfterZeroHeading() {
        gyro.yaw = 1.0;
        drive.periodic();
        drive.zeroHeading();
        gyro.yaw += 0.2;
        drive.periodic();
        drive.seedPoseFromVision(new Pose2d(6.0, 2.0, Rotation2d.fromDegrees(170.0)));
        drive.periodic();
        assertEquals(0.2, drive.getRotation().getRadians(), EPSILON);
        assertEquals(6.0, drive.getPose().getX(), EPSILON);
        assertEquals(2.0, drive.getPose().getY(), EPSILON);
    }

    @Test
    void absoluteFieldDriveDoesNotUsePilotOffset() {
        gyro.yaw = Math.PI / 2.0;
        drive.periodic();
        drive.zeroDriverHeading();
        setEnabled(true);
        drive.driveFieldRelative(0.25, 0.0, 0.0);
        drive.periodic();
        // +X do campo está à direita do robô, mesmo após zerar o piloto.
        assertEquals(-Math.PI / 2.0, modules[0].turnSetpoint, EPSILON);
        assertEquals(90.0, drive.getRotation().getDegrees(), EPSILON);
    }

    @Test
    void fieldResetIsRejectedWhileEnabled() {
        gyro.yaw = 0.7;
        drive.periodic();
        setEnabled(true);
        drive.zeroHeading();
        drive.periodic();
        assertEquals(0.7, drive.getRotation().getRadians(), EPSILON);
    }

    @Test
    void bothResetsAreRejectedWhileMoving() {
        gyro.yaw = 0.7;
        modules[0].measuredSpeed = 0.2;
        drive.periodic();
        drive.zeroHeading();
        drive.zeroDriverHeading();
        assertEquals(0.7, drive.getRotation().getRadians(), EPSILON);
        assertEquals(0.7, drive.getDriverRotation().getRadians(), EPSILON);
    }

    @Test
    void disconnectedGyroRejectsResetsAndInvalidatesVisionYawRate() {
        gyro.yaw = 0.7;
        drive.periodic();
        gyro.connected = false;
        drive.periodic();
        assertFalse(drive.isGyroHealthy());
        assertFalse(drive.isStationary());
        assertTrue(Double.isNaN(drive.getYawVelocityDegreesPerSecond()));
        drive.zeroHeading();
        drive.zeroDriverHeading();
        assertEquals(0.7, drive.getRotation().getRadians(), EPSILON);
        assertEquals(0.7, drive.getDriverRotation().getRadians(), EPSILON);
    }

    @Test
    void invalidGyroSampleDoesNotPoisonEstimator() {
        gyro.yaw = 0.7;
        drive.periodic();
        gyro.yaw = Double.NaN;
        drive.periodic();
        assertFalse(drive.isGyroHealthy());
        assertEquals(0.7, drive.getRotation().getRadians(), EPSILON);
        gyro.yaw = 0.8;
        drive.periodic();
        assertTrue(drive.isGyroHealthy());
        assertEquals(0.8, drive.getRotation().getRadians(), EPSILON);
    }

    @Test
    void lostGyroKeepsTheHeadingGoingByTheWheelsAndTheGyroRulesWhenItReturns() {
        gyro.yaw = 0.5;
        // Rodas tangentes ao centro: o robô gira no lugar, sentido anti-horário.
        double[] tangentDegrees = {135.0, 45.0, -135.0, -45.0};
        for (int i = 0; i < 4; i++) {
            modules[i].turnPositionRad = Math.toRadians(tangentDegrees[i]);
        }
        drive.periodic();
        gyro.connected = false;
        drive.periodic();
        assertEquals(0.5, drive.getRotation().getRadians(), EPSILON);

        double turnRad = 0.3;
        double radius = Math.hypot(DriveConstants.kWheelBaseMeters, DriveConstants.kTrackWidthMeters) / 2.0;
        for (FakeModule module : modules) {
            module.drivePositionRad = turnRad * radius / DriveConstants.kWheelRadiusMeters;
        }
        drive.periodic();
        assertEquals(0.8, drive.getRotation().getRadians(), 1e-3);
        assertEquals(0.8, drive.getDriverRotation().getRadians(), 1e-3);

        gyro.connected = true;
        gyro.yaw = 0.75;
        drive.periodic();
        assertEquals(0.75, drive.getRotation().getRadians(), EPSILON);
    }

    private static class FakeGyro implements GyroIO {
        double yaw;
        boolean connected = true;

        @Override
        public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = connected;
            inputs.configurationOk = true;
            inputs.yawPositionRad = yaw;
            inputs.yawVelocityRadPerSec = 0.0;
        }
    }

    private static class FakeModule implements ModuleIO {
        double measuredSpeed;
        double drivePositionRad;
        double turnPositionRad;
        double velocitySetpoint;
        double turnSetpoint;
        double driveVoltage;
        double turnVoltage;

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.encoderConnected = true;
            inputs.driveVelocityRadPerSec = measuredSpeed / DriveConstants.kWheelRadiusMeters;
            inputs.drivePositionRad = drivePositionRad;
            inputs.turnPositionRad = turnPositionRad;
        }

        @Override
        public void setDriveVelocity(double speed) {
            velocitySetpoint = speed;
        }

        @Override
        public void setTurnPosition(double radians) {
            turnSetpoint = radians;
        }

        @Override
        public void setDriveVoltage(double volts) {
            driveVoltage = volts;
        }

        @Override
        public void setTurnVoltage(double volts) {
            turnVoltage = volts;
        }
    }
}
