package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.fuel.FuelConstants.HUB_OPENING_RADIUS_METERS;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.fuel.ShotComparison;

/**
 * Tiro andando: a mira do comando (alvo virtual, RPM e direção do pivô) e a
 * bola voando pela MESMA física da simulação (ShotComparison/BallisticModel),
 * com a velocidade do robô somada à da bola.
 *
 * Replay de 03/11/2026: parado acertava 98%; acima de 0,75 m/s, ~5%. A bola
 * caía em média 0,45 m à frente, no sentido do movimento.
 */
class ShotOnTheMoveTest {

    private static final double[] DISTANCES_METERS = {2.0, 3.0, 4.0, 4.5};
    // De que lado do hub o robô está (graus no campo, a partir do hub).
    private static final double[] BEARINGS_DEGREES = {180.0, 135.0, 225.0};
    // Sentido do movimento em relação à linha robô -> hub.
    private static final double[] MOVE_DEGREES = {0.0, 90.0, 180.0, 270.0};

    /** Robô de costas para o hub (pivô no centro), com o shooter a esta distância. */
    private static Pose2d robotFacingAway(double distance, double bearingDegrees) {
        Rotation2d bearing = Rotation2d.fromDegrees(bearingDegrees);
        Translation2d shooter = BLUE_HUB_CENTER.plus(new Translation2d(distance, bearing));
        Translation2d offset = new Translation2d(SHOOTER_OFFSET_FORWARD_METERS, SHOOTER_OFFSET_LEFT_METERS)
            .rotateBy(bearing);
        return new Pose2d(shooter.minus(offset), bearing);
    }

    /**
     * Distância (m) do centro do hub onde a bola desce pela altura da boca.
     * NaN: o comando não alimenta, porque o alvo virtual passou do alcance da
     * curva (a mesma trava do ShooterTrackAndShootCommand).
     */
    private static double missMeters(Pose2d pose, ChassisSpeeds fieldSpeeds) {
        var moving = ShotCalculator.compensateForMotion(pose, fieldSpeeds, BLUE_HUB_CENTER);
        var shot = ShotCalculator.forPose(pose, moving.virtualTarget());
        if (BLOCK_SHOTS_BEYOND_CURVE
                && shot.distanceMeters() > SHOT_MAX_DISTANCE_METERS + SHOT_DISTANCE_MARGIN_METERS) {
            return Double.NaN;
        }
        var aim = ShotCalculator.aim(pose, moving.virtualTarget());
        assertTrue(aim.reachable(), "o alvo virtual saiu do curso do pivô");
        var estimate = ShotComparison.estimate(ShotCalculator.shooterFieldPosition(pose),
            aim.fieldAngleDegrees(), shot.topRpm(), shot.towerRpm(), shot.centrifugeRpm(), fieldSpeeds);
        // -1: a bola nem chegou à altura da boca.
        return estimate.distanceFromHubCenterMeters() < 0.0
            ? Double.POSITIVE_INFINITY : estimate.distanceFromHubCenterMeters();
    }

    private static ChassisSpeeds moving(double speed, double bearingDegrees, double moveDegrees) {
        // 0° = indo para o hub (o hub fica no sentido bearing + 180°).
        Rotation2d direction = Rotation2d.fromDegrees(bearingDegrees + 180.0 + moveDegrees);
        return new ChassisSpeeds(speed * direction.getCos(), speed * direction.getSin(), 0.0);
    }

    /** Pior erro entre todas as posições e sentidos, nesta velocidade. */
    private static double worstMissMeters(double speed, List<String> misses, List<String> blocked) {
        double worst = 0.0;
        for (double distance : DISTANCES_METERS) {
            for (double bearing : BEARINGS_DEGREES) {
                for (double move : MOVE_DEGREES) {
                    double miss = missMeters(robotFacingAway(distance, bearing), moving(speed, bearing, move));
                    if (Double.isNaN(miss)) {
                        blocked.add(String.format("%.1f m/s, %.1f m, movimento %.0f°", speed, distance, move));
                        continue;
                    }
                    worst = Math.max(worst, miss);
                    if (miss > HUB_OPENING_RADIUS_METERS) {
                        misses.add(String.format("%.1f m/s, %.1f m, lado %.0f°, movimento %.0f°: %.2f m",
                            speed, distance, bearing, move, miss));
                    }
                }
            }
        }
        return worst;
    }

    @Test
    void stoppedShotsLandInTheHubAtEveryTestedDistance() {
        List<String> misses = new ArrayList<>();
        List<String> blocked = new ArrayList<>();
        double worst = worstMissMeters(0.0, misses, blocked);
        assertTrue(blocked.isEmpty(), "parado, nenhum tiro de 2 a 4,5 m pode ser travado: " + blocked);
        assertTrue(misses.isEmpty(), "parado, erros fora do hub: " + misses);
        assertTrue(worst <= HUB_OPENING_RADIUS_METERS);
    }

    @Test
    void movingShotsUpTo1_5MetersPerSecondStillLandInTheHub() {
        List<String> misses = new ArrayList<>();
        List<String> blocked = new ArrayList<>();
        StringBuilder worstBySpeed = new StringBuilder();
        int shotsPerSpeed = DISTANCES_METERS.length * BEARINGS_DEGREES.length * MOVE_DEGREES.length;
        for (double speed : new double[] {0.5, 1.0, 1.5}) {
            int missesBefore = misses.size();
            int blockedBefore = blocked.size();
            double worst = worstMissMeters(speed, misses, blocked);
            int fired = shotsPerSpeed - (blocked.size() - blockedBefore);
            worstBySpeed.append(String.format(" %.1f m/s -> %d/%d no hub (%d travados por distância), pior %.2f m;",
                speed, fired - (misses.size() - missesBefore), fired, shotsPerSpeed - fired, worst));
        }
        System.out.println("Tiro andando (raio do hub " + HUB_OPENING_RADIUS_METERS + " m):" + worstBySpeed);
        System.out.println("Travados: " + blocked);
        assertTrue(misses.isEmpty(), misses.size() + " tiros fora do hub (" + worstBySpeed + "): " + misses);
        // Só se afastando do hub a trava pode segurar o tiro: o alvo virtual fica mais longe.
        assertTrue(blocked.stream().allMatch(shot -> shot.endsWith("movimento 180°")), "travados: " + blocked);
    }

    @Test
    void compensationUsesTheBallFlightTimeOfTheBallisticModel() {
        // A 4 m o arco do BallisticModel leva ~1,2 s até descer pela boca (replay de 03/11/2026).
        var stopped = ShotCalculator.compensateForMotion(robotFacingAway(4.0, 180.0), new ChassisSpeeds(),
            BLUE_HUB_CENTER);
        assertEquals(1.2, stopped.flightTimeSeconds(), 0.15);
    }
}
