package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Subsistema e sequência de tiro com os IOs simulados.
 * A pose é controlada pelo teste, no lugar do Drive.
 */
class ShooterTest {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private Shooter shooter;
    private Pose2d pose;
    private boolean visionCorrected;

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(false);
        setEnabled(false);
        visionCorrected = true;
        pose = robotWithShooterAt(7.5, BLUE_HUB_CENTER.getY(), 0.0);
        shooter = new Shooter(
            new ShooterPivotIOSim(0.0),
            Shooter.createTopIO(),
            Shooter.createTowerIO(),
            Shooter.createCentrifugeIO(),
            () -> pose,
            ChassisSpeeds::new,
            () -> visionCorrected);
        tick(1);
        setEnabled(true);
    }

    @AfterEach
    void cleanup() {
        scheduler.cancelAll();
        shooter.close();
        setEnabled(false);
        SimHooks.resumeTiming();
    }

    private static Pose2d robotWithShooterAt(double x, double y, double headingDegrees) {
        Rotation2d heading = Rotation2d.fromDegrees(headingDegrees);
        Translation2d offset = new Translation2d(SHOOTER_OFFSET_FORWARD_METERS, SHOOTER_OFFSET_LEFT_METERS)
            .rotateBy(heading);
        return new Pose2d(new Translation2d(x, y).minus(offset), heading);
    }

    private void setEnabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    private void tick(double seconds) {
        for (int i = 0; i < Math.max(1, Math.round(seconds / 0.02)); i++) {
            SimHooks.stepTiming(0.02);
            scheduler.run();
        }
    }

    /** Roda o tiro e diz se a centrífuga chegou a ser ligada. */
    private boolean shootFor(double seconds) {
        Command shoot = shooter.trackAndShootCommand();
        scheduler.schedule(shoot);
        boolean fed = false;
        for (int i = 0; i < Math.round(seconds / 0.02); i++) {
            tick(0.02);
            fed |= shooter.getCentrifugeTargetRpm() > 0.0;
        }
        return fed;
    }

    @Test
    void waitsForTheLimelightBeforeShooting() {
        visionCorrected = false;
        assertFalse(shootFor(2.0));
        assertEquals(Shooter.PivotMode.PARADO, shooter.getPivotMode());
    }

    @Test
    void shootsWhenTheHubIsBehindTheRobot() {
        assertTrue(shootFor(2.5));
        assertEquals(0.0, shooter.getPivotDegrees(), 1.0);
        // A tower (modelo NEO) chega ao RPM com o kV da competição. O top, com o
        // modelo NEO Vortex, fica acima da tolerância: o tiro sai pelo tempo.
        assertTrue(shooter.isTowerAtSpeed());
    }

    @Test
    void turnsThePivotToAHubAtTheSide() {
        // Hub a 45° do zero: shooter a 3 m na diagonal.
        double d = 3.0 / Math.sqrt(2.0);
        pose = robotWithShooterAt(BLUE_HUB_CENTER.getX() + d, BLUE_HUB_CENTER.getY() + d, 0.0);
        assertTrue(shootFor(3.0));
        assertEquals(45.0, shooter.getPivotDegrees(), 1.0);
    }

    @Test
    void neverFeedsWhenTheHubIsOutOfThePivotCourse() {
        // Robô de costas para o hub: o shooter teria de girar 180°.
        pose = robotWithShooterAt(2.0, BLUE_HUB_CENTER.getY(), 0.0);
        assertFalse(shootFor(3.0));
        assertEquals(PIVOT_MAX_ANGLE_DEGREES, Math.abs(shooter.getPivotDegrees()), 1.0);
    }

    @Test
    void releasingTheButtonStopsEverything() {
        Command shoot = shooter.trackAndShootCommand();
        scheduler.schedule(shoot);
        tick(2.0);
        scheduler.cancel(shoot);
        tick(0.04);
        assertEquals(0.0, shooter.getCentrifugeTargetRpm());
        assertEquals(Shooter.PivotMode.PARADO, shooter.getPivotMode());
    }

    @Test
    void zeroIsOnlyAcceptedWithTheRobotDisabled() {
        assertFalse(shooter.zeroPivotHere());
        setEnabled(false);
        tick(0.02);
        assertTrue(shooter.zeroPivotHere());
    }

    @Test
    void componentPosesFollowTheModelOrder() {
        assertTurretYaw(ShooterVisualizer.componentPoses(0.0), 180.0 - CAD_ZERO_FACING_DEGREES);
        assertTurretYaw(ShooterVisualizer.componentPoses(30.0), 210.0 - CAD_ZERO_FACING_DEGREES);
    }

    private static void assertTurretYaw(Pose3d[] poses, double yawDegrees) {
        assertEquals(2, poses.length, "model_0 = intake, model_1 = shooter");
        assertEquals(INTAKE_PIVOT_IN_ROBOT, poses[0].getTranslation());
        assertEquals(TURRET_PIVOT_IN_ROBOT, poses[1].getTranslation());
        assertEquals(yawDegrees, Math.toDegrees(poses[1].getRotation().getZ()), 1e-9);
    }
}
