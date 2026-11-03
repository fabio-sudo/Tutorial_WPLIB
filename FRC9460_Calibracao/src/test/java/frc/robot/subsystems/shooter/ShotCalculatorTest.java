package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Contas do tiro: curva de RPM igual à competição, mira e alvo por aliança. */
class ShotCalculatorTest {

    private static final double EPS = 1e-6;

    // Shooter fica em (-0,05; -0,14) do centro: este ponto coloca o robô
    // com o shooter exatamente em (x, y) quando o robô aponta para +X.
    private static Pose2d robotWithShooterAt(double x, double y, double headingDegrees) {
        Rotation2d heading = Rotation2d.fromDegrees(headingDegrees);
        Translation2d offset = new Translation2d(SHOOTER_OFFSET_FORWARD_METERS, SHOOTER_OFFSET_LEFT_METERS)
            .rotateBy(heading);
        return new Pose2d(new Translation2d(x, y).minus(offset), heading);
    }

    @Test
    void hubCentersComeFromTheFieldTags() {
        assertEquals(4.6255, BLUE_HUB_CENTER.getX(), 1e-3);
        assertEquals(4.0345, BLUE_HUB_CENTER.getY(), 1e-3);
        assertEquals(11.9155, RED_HUB_CENTER.getX(), 1e-3);
        assertEquals(4.0345, RED_HUB_CENTER.getY(), 1e-3);
        assertEquals(BLUE_HUB_CENTER, ShotCalculator.hubFor(Optional.empty()));
        assertEquals(BLUE_HUB_CENTER, ShotCalculator.hubFor(Optional.of(Alliance.Blue)));
        assertEquals(RED_HUB_CENTER, ShotCalculator.hubFor(Optional.of(Alliance.Red)));
    }

    @Test
    void withoutFmsThePracticeHubIsUsedEvenWithTheDefaultRedAlliance() {
        // Fora do FMS a Driver Station vem como Red 1: no treino vale o hub azul,
        // o mesmo alvo fixo do código de competição.
        assertEquals(BLUE_HUB_CENTER, ShotCalculator.hubFor(Optional.of(Alliance.Red), false));
        assertEquals(BLUE_HUB_CENTER, ShotCalculator.hubFor(Optional.empty(), false));
        // Com FMS, o hub da aliança.
        assertEquals(RED_HUB_CENTER, ShotCalculator.hubFor(Optional.of(Alliance.Red), true));
        assertEquals(BLUE_HUB_CENTER, ShotCalculator.hubFor(Optional.of(Alliance.Blue), true));
    }

    @Test
    void curveMatchesCompetitionValuesAtBothEnds() {
        var near = ShotCalculator.forDistance(SHOT_MIN_DISTANCE_METERS);
        assertEquals(2350.0 * 0.95, near.topRpm(), EPS);
        assertEquals(2350.0 * 0.95 + 180.0, near.towerRpm(), EPS);
        assertEquals((2350.0 * 0.95 + 180.0) * 1.15, near.centrifugeRpm(), EPS);

        var far = ShotCalculator.forDistance(SHOT_MAX_DISTANCE_METERS);
        assertEquals(3900.0 * 0.95, far.topRpm(), EPS);
        assertEquals(3900.0 * 0.95 + 180.0, far.towerRpm(), EPS);
        assertEquals(CENTRIFUGE_SHOT_MAX_RPM, far.centrifugeRpm(), EPS);

        // Fora da faixa calibrada o RPM fica no extremo, e mais longe nunca pede menos.
        assertEquals(far.topRpm(), ShotCalculator.forDistance(9.0).topRpm(), EPS);
        assertTrue(ShotCalculator.forDistance(3.0).topRpm() > near.topRpm());
        assertTrue(ShotCalculator.forDistance(4.0).topRpm() > ShotCalculator.forDistance(3.0).topRpm());
        assertFalse(ShotCalculator.forDistance(0.01).valid());
    }

    @Test
    void hubBehindTheRobotNeedsTheCenteredPivot() {
        // Zero do pivô aponta para trás (180°): hub atrás = pivô em 0°.
        var aim = ShotCalculator.aim(robotWithShooterAt(7.0, BLUE_HUB_CENTER.getY(), 0.0), BLUE_HUB_CENTER);
        assertEquals(0.0, aim.desiredPivotDegrees(), 1e-9);
        assertTrue(aim.reachable());
        assertEquals(7.0 - BLUE_HUB_CENTER.getX(), aim.distanceMeters(), 1e-9);
    }

    @Test
    void hubToTheSidesIsWithinTheCourseAndInFrontIsOutOfReach() {
        // Hub à direita do robô: o shooter gira +90° (anti-horário a partir de trás).
        var right = ShotCalculator.aim(robotWithShooterAt(BLUE_HUB_CENTER.getX(), 7.0, 0.0), BLUE_HUB_CENTER);
        assertEquals(90.0, right.desiredPivotDegrees(), 1e-9);
        assertTrue(right.reachable());
        assertEquals(90.0 / PIVOT_DEGREES_PER_MOTOR_ROTATION,
            ShotCalculator.pivotDegreesToMotorRotations(right.limitedPivotDegrees()), 1e-9);

        var left = ShotCalculator.aim(robotWithShooterAt(BLUE_HUB_CENTER.getX(), 1.0, 0.0), BLUE_HUB_CENTER);
        assertEquals(-90.0, left.desiredPivotDegrees(), 1e-9);

        // Hub na frente: o shooter teria de virar 180°, além da fiação.
        var front = ShotCalculator.aim(robotWithShooterAt(2.0, BLUE_HUB_CENTER.getY(), 0.0), BLUE_HUB_CENTER);
        assertFalse(front.reachable());
        assertEquals(PIVOT_MAX_ANGLE_DEGREES, Math.abs(front.limitedPivotDegrees()), 1e-9);
    }

    @Test
    void robotRotationIsDiscounted() {
        // Robô virado para +Y; hub a -Y fica atrás dele.
        var aim = ShotCalculator.aim(robotWithShooterAt(BLUE_HUB_CENTER.getX(), 7.0, 90.0), BLUE_HUB_CENTER);
        assertEquals(0.0, aim.desiredPivotDegrees(), 1e-9);
        assertEquals(-90.0, aim.fieldAngleDegrees(), 1e-9);
        assertEquals(-90.0, ShotCalculator.shooterFieldHeadingDegrees(robotWithShooterAt(0, 0, 90.0), 0.0), 1e-9);
    }

    @Test
    void motorConversionUsesMeasuredTravelAndStopsAt130Degrees() {
        // Valor independente: 8,979981 voltas medidas em 132°.
        double at45 = 8.979981 * 45.0 / 132.0;
        assertEquals(at45, ShotCalculator.pivotDegreesToMotorRotations(45.0), EPS);
        assertEquals(45.0, ShotCalculator.motorRotationsToPivotDegrees(at45), EPS);
        assertEquals(PIVOT_MAX_MOTOR_ROTATIONS, ShotCalculator.pivotDegreesToMotorRotations(170.0), EPS);
        assertEquals(24.490857272727276, PIVOT_GEAR_REDUCTION, EPS);
        assertEquals(8.843920681818181, PIVOT_MAX_MOTOR_ROTATIONS, EPS);
        assertEquals(132.0, ShotCalculator.motorRotationsToPivotDegrees(8.979981), EPS);
        assertEquals(-PIVOT_MAX_MOTOR_ROTATIONS, ShotCalculator.pivotDegreesToMotorRotations(-170.0), EPS);
        assertEquals(130.0, ShotCalculator.motorRotationsToPivotDegrees(PIVOT_MAX_MOTOR_ROTATIONS), EPS);
        assertTrue(PIVOT_MAX_MOTOR_ROTATIONS < PIVOT_CALIBRATION_TRAVEL_ROTATIONS);
    }

    @Test
    void targetAt130IsReachableButBeyondEitherLimitIsNot() {
        for (double degrees : new double[] {-131.0, -130.0, 130.0, 131.0}) {
            Pose2d robot = robotWithShooterAt(0, 0, 0);
            double radians = Math.toRadians(PIVOT_ZERO_FACING_DEGREES + degrees);
            var target = new Translation2d(3 * Math.cos(radians), 3 * Math.sin(radians));
            var result = ShotCalculator.aim(robot, target);
            assertEquals(Math.abs(degrees) <= 130, result.reachable());
            assertTrue(Math.abs(result.limitedPivotDegrees()) <= 130);
        }
    }

    @Test
    void movingRobotAimsAtAVirtualTargetAgainstItsVelocity() {
        Pose2d pose = robotWithShooterAt(7.5, BLUE_HUB_CENTER.getY(), 0.0);
        var stopped = ShotCalculator.compensateForMotion(pose, new ChassisSpeeds(), BLUE_HUB_CENTER);
        assertEquals(BLUE_HUB_CENTER, stopped.virtualTarget());

        // Andando em +Y a 1 m/s: a bola herda +Y, então o alvo virtual vai para -Y.
        var moving = ShotCalculator.compensateForMotion(pose, new ChassisSpeeds(0.0, 1.0, 0.0), BLUE_HUB_CENTER);
        assertTrue(moving.flightTimeSeconds() > 0.0);
        assertEquals(BLUE_HUB_CENTER.getX(), moving.virtualTarget().getX(), 1e-9);
        // Deslocamento ≈ velocidade × tempo de voo (o tempo é refinado em duas passadas).
        assertTrue(moving.virtualTarget().getY() < BLUE_HUB_CENTER.getY());
        assertEquals(BLUE_HUB_CENTER.getY() - moving.flightTimeSeconds(), moving.virtualTarget().getY(), 0.02);
    }
}
