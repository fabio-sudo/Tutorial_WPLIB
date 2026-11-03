package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Curso do pivô (±130° medidos no Tuner) e curva de RPM por distância.
 * Prova as contas; o ângulo físico real se confere no robô com transferidor.
 */
class PivotCourseAndShotCurveTest {

    // #region CURSO DO PIVÔ

    @Test void calibratedCourseIs130DegreesEachSide() {
        assertEquals(8.843920681818181, ShotCalculator.pivotDegreesToMotorRotations(130.0), 1e-9);
        assertEquals(-8.843920681818181, ShotCalculator.pivotDegreesToMotorRotations(-130.0), 1e-9);
        assertEquals(PIVOT_MAX_MOTOR_ROTATIONS, ShotCalculator.pivotDegreesToMotorRotations(130.0), 1e-12);
        assertEquals(45.0, ShotCalculator.motorRotationsToPivotDegrees(
            ShotCalculator.pivotDegreesToMotorRotations(45.0)), 1e-9);
    }

    @Test void conversionNeverLeavesTheCourse() {
        assertEquals(PIVOT_MAX_MOTOR_ROTATIONS, ShotCalculator.pivotDegreesToMotorRotations(200.0));
        assertEquals(-PIVOT_MAX_MOTOR_ROTATIONS, ShotCalculator.pivotDegreesToMotorRotations(-200.0));
        // A margem até o ponto medido de 132° continua positiva.
        assertTrue(PIVOT_MAX_ANGLE_DEGREES < PIVOT_CALIBRATION_ANGLE_DEGREES);
    }

    /** Robô em 0°, alvo a 3 m numa direção que exige este ângulo do pivô. */
    private static ShotCalculator.Aim aimNeeding(double pivotDegrees) {
        Pose2d pose = new Pose2d(8.0, 4.0, Rotation2d.kZero);
        Translation2d shooterAt = ShotCalculator.shooterFieldPosition(pose);
        Rotation2d direction = Rotation2d.fromDegrees(PIVOT_ZERO_FACING_DEGREES + pivotDegrees);
        return ShotCalculator.aim(pose, shooterAt.plus(new Translation2d(3.0, direction)));
    }

    @Test void hubAtTheEdgeOfTheCourseIsReachableJustBeyondIsNot() {
        ShotCalculator.Aim edge = aimNeeding(129.9);
        assertTrue(edge.reachable());
        assertEquals(129.9, edge.limitedPivotDegrees(), 1e-6);

        ShotCalculator.Aim beyond = aimNeeding(130.5);
        assertFalse(beyond.reachable(), "fora do curso: espera o robô girar");
        assertEquals(PIVOT_MAX_ANGLE_DEGREES, beyond.limitedPivotDegrees(), 1e-6);

        ShotCalculator.Aim otherSide = aimNeeding(-130.5);
        assertFalse(otherSide.reachable());
        assertEquals(-PIVOT_MAX_ANGLE_DEGREES, otherSide.limitedPivotDegrees(), 1e-6);
    }

    // #endregion

    // #region CURVA DE RPM

    @Test void fartherShotsNeverAskForLessSpeed() {
        ShotCalculator.Shot previous = ShotCalculator.forDistance(SHOT_MIN_DISTANCE_METERS);
        for (double d = SHOT_MIN_DISTANCE_METERS + 0.05; d <= SHOT_MAX_DISTANCE_METERS; d += 0.05) {
            ShotCalculator.Shot shot = ShotCalculator.forDistance(d);
            assertTrue(shot.topRpm() >= previous.topRpm() - 1e-9, "top caiu em " + d + " m");
            assertTrue(shot.towerRpm() >= previous.towerRpm() - 1e-9, "tower caiu em " + d + " m");
            previous = shot;
        }
        // Além do fim da curva, o RPM fica no máximo da curva (não extrapola).
        assertEquals(ShotCalculator.forDistance(SHOT_MAX_DISTANCE_METERS).topRpm(),
            ShotCalculator.forDistance(SHOT_MAX_DISTANCE_METERS + 3.0).topRpm(), 1e-9);
    }

    @Test void everyShotStaysInsideTheMotorAndFeedLimits() {
        for (double d = 0.0; d <= SHOT_MAX_DISTANCE_METERS + 1.0; d += 0.1) {
            ShotCalculator.Shot shot = ShotCalculator.forDistance(d);
            assertTrue(shot.towerRpm() <= NEO_FREE_SPEED_RPM, "tower acima do motor em " + d);
            assertTrue(shot.towerRpm() >= Math.min(shot.topRpm() + TOWER_LEAD_OVER_TOP_RPM, NEO_FREE_SPEED_RPM) - 1e-9,
                "tower precisa puxar mais que o top em " + d);
            assertTrue(shot.centrifugeRpm() >= CENTRIFUGE_SHOT_MIN_RPM
                && shot.centrifugeRpm() <= CENTRIFUGE_SHOT_MAX_RPM, "centrífuga fora da faixa em " + d);
        }
    }

    // #endregion
}
