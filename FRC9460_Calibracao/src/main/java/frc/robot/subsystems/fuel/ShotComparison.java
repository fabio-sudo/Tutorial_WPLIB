package frc.robot.subsystems.fuel;

// #region IMPORTS

import static frc.robot.subsystems.fuel.FuelConstants.*;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;

// #endregion

/**
 * Para onde vai uma bola, dados a posição e a direção do shooter, os RPMs
 * das rodas e a velocidade do robô. A mesma conta serve para comparar:
 *
 * - PLANEJADO: com os valores que o código pediu (RPM alvo, ângulo alvo do
 *   pivô). É onde a bola DEVERIA ir;
 * - SENSORES: com os valores medidos no instante do disparo (RPM real,
 *   encoder do pivô). É onde ela foi, segundo os sensores;
 * - ONDE CHEGOU: na simulação, o voo físico (FuelSimulation); no robô real,
 *   a anotação da equipe (FuelDashboard).
 *
 * A diferença entre planejado e sensores mostra a culpa de cada parte: roda
 * fora do RPM, pivô atrasado ou robô andando.
 */
public final class ShotComparison {

    // #region RESULTADO

    /**
     * @param trajectory pontos do voo (um a cada 0,1 s) para desenhar
     * @param distanceFromHubCenterMeters onde a bola desce pela altura da boca
     *        do hub, medido do centro; -1 se nem chega nessa altura
     * @param arrival o ponto (X, Y, altura da boca) em que a bola desce pela
     *        altura do hub: ONDE CHEGOU. Vazio se nem chega nessa altura
     */
    public record Estimate(
        Translation3d[] trajectory,
        double exitSpeedMps,
        double headingDegrees,
        double distanceFromHubCenterMeters,
        boolean hits,
        Optional<Translation3d> arrival
    ) {}

    // #endregion

    // #region MARCADOR DE CHEGADA (3D)

    /** Pose de um ponto de chegada: cone apontando para baixo, com a ponta no ponto. */
    public static Pose3d arrivalMarker(Translation3d point) {
        return new Pose3d(point, new Rotation3d(0.0, Math.PI / 2.0, 0.0));
    }

    // #endregion

    // #region 🚀 VELOCIDADE DE SAÍDA DA BOLA

    /**
     * Velocidade da bola: a mesma soma ponderada de RPMs da competição, com a
     * eficiência da simulação (BALL_SPEED_EFFICIENCY), ajustada para a curva
     * de RPM da competição acertar o hub com o ângulo de saída estimado.
     */
    public static double ballExitSpeed(double topRpm, double towerRpm, double centrifugeRpm) {
        return ShotCalculator.effectiveRpm(topRpm, towerRpm, centrifugeRpm) * 2.0 * Math.PI / 60.0
            * ShooterConstants.FLYWHEEL_RADIUS_METERS * BALL_SPEED_EFFICIENCY;
    }

    // #endregion

    // #region ESTIMATIVA DE UM TIRO

    public static Estimate estimate(Translation2d shooterFieldPosition, double headingDegrees,
            double topRpm, double towerRpm, double centrifugeRpm, ChassisSpeeds robotFieldSpeeds) {
        double exitSpeed = ballExitSpeed(topRpm, towerRpm, centrifugeRpm);
        BallisticModel.State start = new BallisticModel.State(
            new Translation3d(shooterFieldPosition.getX(), shooterFieldPosition.getY(),
                ShooterConstants.RELEASE_HEIGHT_METERS),
            BallisticModel.launchVelocity(headingDegrees, exitSpeed, LAUNCH_ANGLE_DEGREES, robotFieldSpeeds));
        List<Translation3d> path = BallisticModel.predict(start, PREDICTION_MAX_SECONDS, PREDICTION_STEP_SECONDS);
        Optional<Translation2d> crossing =
            BallisticModel.descendingCrossing(path, ShooterConstants.TARGET_HEIGHT_METERS);
        double distance = crossing.map(ShotComparison::distanceToNearestHub).orElse(-1.0);
        return new Estimate(
            BallisticModel.sample(path, PREDICTION_STEP_SECONDS, PREDICTION_OUTPUT_STEP_SECONDS),
            exitSpeed, headingDegrees, distance,
            distance >= 0.0 && distance <= HUB_OPENING_RADIUS_METERS,
            crossing.map(point -> new Translation3d(point.getX(), point.getY(),
                ShooterConstants.TARGET_HEIGHT_METERS)));
    }

    /** Distância até o centro do hub mais próximo (azul ou vermelho). */
    public static double distanceToNearestHub(Translation2d point) {
        return Math.min(point.getDistance(ShooterConstants.BLUE_HUB_CENTER),
            point.getDistance(ShooterConstants.RED_HUB_CENTER));
    }

    // #endregion

    // #region SEM INSTÂNCIAS

    private ShotComparison() {}

    // #endregion
}
