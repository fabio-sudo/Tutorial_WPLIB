package frc.robot.subsystems.fuel;

import static frc.robot.subsystems.fuel.FuelConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;

/** Mundo das bolas com um robô de mentira: coleta, lançamento, voo, hub e chão. */
class FuelSimulationTest {

    /** Robô controlado pelo teste: cada campo é um sensor. */
    private static final class FakeRobot implements FuelRobotState {
        Pose2d pose = new Pose2d();
        boolean intakeOpen = true;
        boolean rollerOn = true;
        double rollerRpm = 2000.0;
        double top, tower, centrifuge;
        Translation2d shooter = new Translation2d();
        double heading;
        int collectedHooks, launchedHooks;

        @Override public Pose2d pose() { return pose; }
        @Override public ChassisSpeeds fieldSpeeds() { return new ChassisSpeeds(); }
        @Override public boolean intakeReadyToCollect() { return intakeOpen; }
        @Override public boolean rollerPulling() { return rollerOn; }
        @Override public double rollerRpm() { return rollerRpm; }
        @Override public double rollerCurrentAmps() { return 0.0; }
        @Override public double topRpm() { return top; }
        @Override public boolean topCommanded() { return top > 0.0; }
        @Override public double towerRpm() { return tower; }
        @Override public double centrifugeRpm() { return centrifuge; }
        @Override public boolean centrifugeRunning() { return centrifuge > 0.0; }
        @Override public Translation2d shooterFieldPosition() { return shooter; }
        @Override public double shooterHeadingDegrees() { return heading; }
        @Override public void onBallCollected() { collectedHooks++; }
        @Override public void onBallLaunched() { launchedHooks++; }
    }

    private FakeRobot robot;
    private double now;

    @BeforeEach
    void setup() {
        robot = new FakeRobot();
        now = 10.0;
    }

    private FuelSimulation world(List<Translation3d> balls) {
        FuelSimulation sim = new FuelSimulation(robot, () -> now, balls);
        sim.setHopperCount(0);
        return sim;
    }

    private void run(FuelSimulation sim, double seconds) {
        for (int i = 0; i < Math.round(seconds / 0.02); i++) {
            now += 0.02;
            sim.update();
        }
    }

    /** Shooter a d metros do hub azul, apontado para ele, rodas no RPM da curva. */
    private void aimFrom(double distanceMeters, double rpmFactor) {
        robot.shooter = ShooterConstants.BLUE_HUB_CENTER.plus(new Translation2d(distanceMeters, 0.0));
        robot.heading = 180.0;
        var shot = ShotCalculator.forDistance(distanceMeters);
        robot.top = shot.topRpm() * rpmFactor;
        robot.tower = shot.towerRpm() * rpmFactor;
        robot.centrifuge = shot.centrifugeRpm() * rpmFactor;
    }

    @Test
    void ballAtTheIntakeMouthIsCollected() {
        // Robô em (5, 5) olhando para +Y: a bola 0,6 m à frente fica em (5, 5,6).
        robot.pose = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(90.0));
        var sim = world(new ArrayList<>(List.of(new Translation3d(5.0, 5.6, BALL_RADIUS_METERS),
            new Translation3d(9.0, 1.0, BALL_RADIUS_METERS))));
        run(sim, 0.1);
        assertEquals(1, sim.getCollected());
        assertEquals(1, sim.getHopperCount());
        assertEquals(1, sim.getFloorCount(), "a bola longe continua no chão");
        assertEquals(1, robot.collectedHooks, "o rolo simulado é avisado");
    }

    @Test
    void closedIntakeOrStoppedRollerDoesNotCollect() {
        robot.pose = new Pose2d(5.0, 5.0, Rotation2d.kZero);
        var sim = world(new ArrayList<>(List.of(new Translation3d(5.6, 5.0, BALL_RADIUS_METERS))));
        robot.intakeOpen = false;
        run(sim, 0.5);
        robot.intakeOpen = true;
        robot.rollerOn = false;
        run(sim, 0.5);
        assertEquals(0, sim.getCollected());
        robot.rollerOn = true;
        run(sim, 0.1);
        assertEquals(1, sim.getCollected());
    }

    @Test
    void hopperStopsAtCapacity() {
        robot.pose = new Pose2d(5.0, 5.0, Rotation2d.kZero);
        var balls = new ArrayList<Translation3d>();
        for (int i = 0; i < HOPPER_CAPACITY + 5; i++) {
            balls.add(new Translation3d(5.6, 5.0, BALL_RADIUS_METERS));
        }
        var sim = world(balls);
        run(sim, (HOPPER_CAPACITY + 10) * COLLECT_PERIOD_SECONDS);
        assertEquals(HOPPER_CAPACITY, sim.getHopperCount());
        assertEquals(5, sim.getFloorCount());
    }

    @Test
    void shotWithTheCompetitionCurveScoresFromThreeMeters() {
        var sim = world(new ArrayList<>());
        sim.setHopperCount(1);
        aimFrom(3.0, 1.0);
        run(sim, 4.0);
        assertEquals(1, sim.getLaunched());
        assertEquals(1, robot.launchedHooks, "o top simulado perde RPM");
        assertEquals(1, sim.getScored());
        assertEquals(0, sim.getMissed());
        assertTrue(sim.getLastShotTrace().size() > 10, "trajetória gravada para o desenho");
    }

    @Test
    void slowWheelsMissShortAndTheBallStaysOnTheField() {
        var sim = world(new ArrayList<>());
        sim.setHopperCount(1);
        aimFrom(4.0, 0.6);
        run(sim, 4.0);
        assertEquals(1, sim.getMissed());
        assertEquals(0, sim.getScored());
        assertEquals(1, sim.getFloorCount(), "a bola caiu e pode ser coletada de novo");
    }

    @Test
    void noLaunchWithoutCentrifugeAndTower() {
        var sim = world(new ArrayList<>());
        sim.setHopperCount(3);
        aimFrom(3.0, 1.0);
        robot.centrifuge = 0.0;
        run(sim, 1.0);
        assertEquals(0, sim.getLaunched());
        assertEquals(3, sim.getHopperCount());
    }

    @Test
    void initialFieldHas456Balls() {
        assertEquals(456, FuelSimulation.loadInitialPositions().size());
    }
}
