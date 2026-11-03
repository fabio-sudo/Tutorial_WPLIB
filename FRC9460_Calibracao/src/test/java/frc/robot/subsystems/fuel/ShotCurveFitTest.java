package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;

/**
 * A curva de RPM e o modelo da bola precisam concordar: com o robô parado e
 * a mira certa, o tiro planejado cai dentro da boca do hub.
 *
 * 29/10/2026: o TOP_FORCE_SCALE subiu de 0,88 para 0,95 depois do teste
 * físico e o modelo continuou calibrado para 0,88. Na simulação todo tiro
 * passava do hub; só acertava entre ~2,3 e ~3,7 m. Se a curva mudar de
 * novo, este teste avisa que BALL_SPEED_EFFICIENCY precisa ser reajustada.
 */
class ShotCurveFitTest {

    @Test
    void plannedShotLandsInsideTheHubAcrossTheShotCurve() {
        Translation2d hub = ShooterConstants.BLUE_HUB_CENTER;
        for (double distance = 1.5; distance <= 5.0 + 1e-9; distance += 0.25) {
            // Shooter atrás do hub (lado azul), mirando em +X.
            Translation2d shooter = hub.minus(new Translation2d(distance, 0.0));
            ShotCalculator.Shot shot = ShotCalculator.forDistance(distance);
            ShotComparison.Estimate estimate = ShotComparison.estimate(shooter, 0.0,
                shot.topRpm(), shot.towerRpm(), shot.centrifugeRpm(), new ChassisSpeeds());
            System.out.printf("CURVA %.2f m: cai a %.2f m do centro%n",
                distance, estimate.distanceFromHubCenterMeters());
            assertTrue(estimate.hits(), String.format(
                "a %.2f m a bola cai a %.2f m do centro (raio %.3f m)", distance,
                estimate.distanceFromHubCenterMeters(), FuelConstants.HUB_OPENING_RADIUS_METERS));
        }
    }

    @Test
    void arrivalPointIsOnTheHubHeightAndMatchesTheReportedDistance() {
        Translation2d hub = ShooterConstants.BLUE_HUB_CENTER;
        double distance = 3.0;
        ShotCalculator.Shot shot = ShotCalculator.forDistance(distance);
        ShotComparison.Estimate estimate = ShotComparison.estimate(
            hub.minus(new Translation2d(distance, 0.0)), 0.0,
            shot.topRpm(), shot.towerRpm(), shot.centrifugeRpm(), new ChassisSpeeds());
        var arrival = estimate.arrival();
        assertTrue(arrival.isPresent(), "a bola chega à altura do hub");
        assertEquals(ShooterConstants.TARGET_HEIGHT_METERS, arrival.get().getZ(), 1e-9);
        assertEquals(estimate.distanceFromHubCenterMeters(),
            arrival.get().toTranslation2d().getDistance(hub), 1e-9,
            "o ponto de chegada é exatamente o que gerou a distância do centro");
    }

    @Test
    void ballTooSlowToReachTheHubHasNoArrivalPoint() {
        Translation2d hub = ShooterConstants.BLUE_HUB_CENTER;
        ShotComparison.Estimate estimate = ShotComparison.estimate(
            hub.minus(new Translation2d(3.0, 0.0)), 0.0, 600.0, 700.0, 800.0, new ChassisSpeeds());
        assertTrue(estimate.arrival().isEmpty());
        assertEquals(-1.0, estimate.distanceFromHubCenterMeters());
        assertFalse(estimate.hits());
    }
}
