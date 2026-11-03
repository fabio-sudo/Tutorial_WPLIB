package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// #endregion

/**
 * Contas do tiro, sem motores e sem estado: dão o mesmo resultado no robô,
 * na simulação, no replay e nos testes.
 *
 * Entrada principal: a pose do robô, que vem do Drive já corrigida pela
 * Limelight. Uma pose errada aqui vira mira e RPM errados; por isso o
 * shooter espera a primeira correção da visão antes de atirar.
 *
 * Lógica portada do ShooterSetpointCalculator e do ShooterPivotSubsystem do
 * código de competição. Diferenças: o alvo segue a aliança, e a mira informa
 * quando o hub está fora do curso de ±130° (o código antigo apenas limitava).
 */
public final class ShotCalculator {

    // #region RESULTADOS

    /**
     * RPM de cada mecanismo para uma distância.
     * valid = false quando o shooter está praticamente em cima do alvo.
     */
    public record Shot(
        double distanceMeters,
        double topRpm,
        double towerRpm,
        double centrifugeRpm,
        double exitVelocityMetersPerSec,
        boolean valid
    ) {}

    /**
     * Direção que o pivô precisa para apontar no alvo.
     *
     * desiredPivotDegrees: ângulo pedido, sem limite.
     * limitedPivotDegrees: o mesmo ângulo, preso ao curso de ±130° (PIVOT_MAX_ANGLE_DEGREES).
     * reachable: o hub cabe no curso sem girar o robô.
     */
    public record Aim(
        double fieldAngleDegrees,
        double robotFrameAngleDegrees,
        double desiredPivotDegrees,
        double limitedPivotDegrees,
        boolean reachable,
        double distanceMeters
    ) {}

    /** Alvo deslocado para compensar o movimento do robô durante o voo. */
    public record MovingTarget(Translation2d virtualTarget, double flightTimeSeconds) {}

    // #endregion

    // #region 🧭 ALVO DA ALIANÇA

    /**
     * Hub da aliança atual. O mapa usa sempre a origem azul, então o hub
     * vermelho é outro ponto do mesmo campo. Sem aliança informada: azul.
     */
    /**
     * Competição (FMS conectado): hub da aliança. Treino: sempre o
     * PRACTICE_HUB_ALLIANCE, porque fora do FMS a aliança da DS é só o padrão.
     */
    public static Translation2d hubFor(Optional<Alliance> alliance, boolean fmsAttached) {
        return hubFor(fmsAttached ? alliance : Optional.of(PRACTICE_HUB_ALLIANCE));
    }

    public static Translation2d hubFor(Optional<Alliance> alliance) {
        return alliance.orElse(Alliance.Blue) == Alliance.Red
            ? RED_HUB_CENTER
            : BLUE_HUB_CENTER;
    }

    // #endregion

    // #region 🧭 POSIÇÃO DO SHOOTER NO CAMPO

    /**
     * O shooter não fica no centro do robô. Gira o deslocamento junto com o
     * robô e soma à pose: é desse ponto que a bola sai.
     */
    public static Translation2d shooterFieldPosition(Pose2d robotPose) {
        Translation2d offsetInRobot = new Translation2d(
            SHOOTER_OFFSET_FORWARD_METERS,
            SHOOTER_OFFSET_LEFT_METERS);
        return robotPose.getTranslation().plus(offsetInRobot.rotateBy(robotPose.getRotation()));
    }

    // #endregion

    // #region 🧭 MIRA DO PIVÔ

    /**
     * Passos (os mesmos do código de competição):
     * 1. vetor do shooter até o alvo, no campo;
     * 2. esse vetor no referencial do robô (desconta a rotação do robô);
     * 3. ângulo em relação ao zero do pivô (que aponta para trás);
     * 4. limite angular calibrado em ShooterConstants.
     */
    public static Aim aim(Pose2d robotPose, Translation2d target) {
        Translation2d toTarget = target.minus(shooterFieldPosition(robotPose));
        double distance = toTarget.getNorm();

        // Em cima do alvo a direção não existe: mantém o pivô no centro.
        if (distance < 0.05) {
            return new Aim(0.0, PIVOT_ZERO_FACING_DEGREES, 0.0, 0.0, true, distance);
        }

        double fieldAngle = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
        Translation2d inRobotFrame = toTarget.rotateBy(robotPose.getRotation().unaryMinus());
        double robotFrameAngle = Math.toDegrees(Math.atan2(inRobotFrame.getY(), inRobotFrame.getX()));
        double desired = MathUtil.inputModulus(robotFrameAngle - PIVOT_ZERO_FACING_DEGREES, -180.0, 180.0);
        double limited = MathUtil.clamp(desired, -PIVOT_MAX_ANGLE_DEGREES, PIVOT_MAX_ANGLE_DEGREES);

        return new Aim(fieldAngle, robotFrameAngle, desired, limited,
            Math.abs(desired) <= PIVOT_MAX_ANGLE_DEGREES, distance);
    }

    /** Direção real do shooter no campo: robô + zero do pivô + ângulo do pivô. */
    public static double shooterFieldHeadingDegrees(Pose2d robotPose, double pivotDegrees) {
        return MathUtil.inputModulus(
            robotPose.getRotation().getDegrees() + PIVOT_ZERO_FACING_DEGREES + pivotDegrees,
            -180.0, 180.0);
    }

    // Conversão medida no conjunto, relativa ao centro zerado ao ligar.
    // Mesmo curso dos soft limits do TalonFX, sem aplicar a redução duas vezes.
    public static double pivotDegreesToMotorRotations(double pivotDegrees) {
        return MathUtil.clamp(
            pivotDegrees / PIVOT_DEGREES_PER_MOTOR_ROTATION * PIVOT_DIRECTION_SIGN,
            -PIVOT_MAX_MOTOR_ROTATIONS, PIVOT_MAX_MOTOR_ROTATIONS);
    }

    public static double motorRotationsToPivotDegrees(double motorRotations) {
        return motorRotations * PIVOT_DEGREES_PER_MOTOR_ROTATION * PIVOT_DIRECTION_SIGN;
    }

    // #endregion

    // #region 🚀 RPM POR DISTÂNCIA

    /**
     * Curva por distância, sem fórmula balística: evita a divisão que explode
     * quando o robô chega perto do alvo. Mais forte quanto mais longe.
     */
    public static Shot forDistance(double distanceMeters) {
        double clamped = MathUtil.clamp(distanceMeters, SHOT_MIN_DISTANCE_METERS, SHOT_MAX_DISTANCE_METERS);
        double normalized = (clamped - SHOT_MIN_DISTANCE_METERS)
            / Math.max(0.001, SHOT_MAX_DISTANCE_METERS - SHOT_MIN_DISTANCE_METERS);

        // 🚀 Curva progressiva: suave perto do alvo e mais forte longe dele.
        double shotDemand = 0.25 * normalized + 0.75 * normalized * normalized;
        double towerDemand = 0.40 * normalized + 0.60 * shotDemand;

        double top = lerp(TOP_SHOT_MIN_RPM, TOP_SHOT_MAX_RPM, shotDemand) * TOP_FORCE_SCALE;
        double towerBase = lerp(TOWER_SHOT_MIN_RPM, TOWER_SHOT_MAX_RPM, towerDemand) * TOWER_FORCE_SCALE;
        // A tower gira acima do top para entregar a bola sem freá-lo.
        double tower = MathUtil.clamp(Math.max(towerBase, top + TOWER_LEAD_OVER_TOP_RPM),
            TOWER_SHOT_MIN_RPM, NEO_FREE_SPEED_RPM);
        double centrifuge = MathUtil.clamp(tower * CENTRIFUGE_FORCE_SCALE,
            CENTRIFUGE_SHOT_MIN_RPM, CENTRIFUGE_SHOT_MAX_RPM);

        return new Shot(distanceMeters, top, tower, centrifuge,
            exitVelocityForRpm(top, tower, centrifuge), distanceMeters > 0.05);
    }

    /**
     * Velocidade com que a bola sai, estimada pelo código de competição:
     * soma ponderada dos três RPMs × raio da roda × eficiência.
     * Usada no tempo de voo do tiro andando (lógica da competição). A física
     * das bolas usa outra eficiência: veja fuel/ShotComparison.
     */
    public static double exitVelocityForRpm(double topRpm, double towerRpm, double centrifugeRpm) {
        return effectiveRpm(topRpm, towerRpm, centrifugeRpm) * 2.0 * Math.PI / 60.0
            * FLYWHEEL_RADIUS_METERS * VELOCITY_TRANSFER_EFFICIENCY;
    }

    /** RPM "equivalente" da saída: quanto cada roda contribui para empurrar a bola. */
    public static double effectiveRpm(double topRpm, double towerRpm, double centrifugeRpm) {
        return topRpm * TOP_EXIT_CONTRIBUTION
            + towerRpm * TOWER_EXIT_CONTRIBUTION
            + centrifugeRpm * CENTRIFUGE_EXIT_CONTRIBUTION;
    }

    public static Shot forPose(Pose2d robotPose, Translation2d target) {
        return forDistance(target.minus(shooterFieldPosition(robotPose)).getNorm());
    }

    // #endregion

    // #region TIRO ANDANDO

    /**
     * A bola herda a velocidade do robô. Mira-se num alvo virtual deslocado
     * no sentido contrário ao movimento, por (velocidade × tempo de voo).
     * Duas passadas estabilizam o tempo de voo. Parado: alvo virtual = hub.
     */
    public static MovingTarget compensateForMotion(
        Pose2d robotPose,
        ChassisSpeeds fieldRelativeSpeeds,
        Translation2d target
    ) {
        double flightTime = flightTimeSeconds(forPose(robotPose, target));
        Translation2d virtualTarget = target;
        for (int i = 0; i < 2; i++) {
            virtualTarget = target.minus(new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond * flightTime,
                fieldRelativeSpeeds.vyMetersPerSecond * flightTime));
            flightTime = flightTimeSeconds(forPose(robotPose, virtualTarget));
        }
        return new MovingTarget(virtualTarget, flightTime);
    }

    public static double flightTimeSeconds(Shot shot) {
        return shot.distanceMeters() / Math.max(0.10, shot.exitVelocityMetersPerSec());
    }

    // #endregion

    // #region AUXILIARES

    private static double lerp(double min, double max, double blend) {
        return min + (max - min) * MathUtil.clamp(blend, 0.0, 1.0);
    }

    private ShotCalculator() {}

    // #endregion
}
