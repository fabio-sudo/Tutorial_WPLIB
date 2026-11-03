package frc.robot.subsystems.fuel;

// #region IMPORTS

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// #endregion

/**
 * O que a simulação das bolas e o contador precisam saber do robô.
 * O RobotContainer liga cada método ao subsistema certo (Drive, Intake,
 * Roller, Shooter, Centrífuga); os testes usam valores próprios.
 */
public interface FuelRobotState {

    // #region 🧭 POSE E MOVIMENTO (DRIVE)

    /** Pose que o código conhece: odometria + Limelight (Drive.getPose). */
    Pose2d pose();

    ChassisSpeeds fieldSpeeds();

    /**
     * Onde o robô está DE VERDADE. Só a simulação sabe: é a referência sem o
     * erro da visão. No robô real não existe e fica igual a pose().
     * A simulação das bolas coleta e lança por ela; o código mira por pose().
     */
    default Pose2d physicalPose() { return pose(); }

    // #endregion

    // #region COLETA (INTAKE E ROLLER)

    /** Intake aberto o bastante para a bola entrar. */
    boolean intakeReadyToCollect();

    /** Rolete mandado puxar. */
    boolean rollerPulling();

    double rollerRpm();

    double rollerCurrentAmps();

    // #endregion

    // #region DISPARO (SHOOTER E CENTRÍFUGA)

    double topRpm();

    /** O top está sendo comandado (alvo > 0). Parado, ele só desacelera solto. */
    boolean topCommanded();

    double towerRpm();

    double centrifugeRpm();

    /** Centrífuga mandada girar: está empurrando bolas para a tower. */
    boolean centrifugeRunning();

    Translation2d shooterFieldPosition();

    /** Direção REAL do shooter no campo, em graus (na simulação, o ângulo físico). */
    double shooterHeadingDegrees();

    // #endregion

    // #region VALORES PLANEJADOS E MEDIDOS (COMPARAÇÃO DE CADA TIRO)

    // O que o código PEDIU. Padrão: igual ao medido (testes simples).
    default double topTargetRpm() { return topRpm(); }

    default double towerTargetRpm() { return towerRpm(); }

    default double centrifugeTargetRpm() { return centrifugeRpm(); }

    /** Direção que o código queria: robô + zero + ângulo ALVO do pivô. */
    default double plannedShooterHeadingDegrees() { return shooterHeadingDegrees(); }

    /** Direção pelo encoder do pivô (o que o robô real sabe medir). */
    default double encoderShooterHeadingDegrees() { return shooterHeadingDegrees(); }

    // #endregion

    // #region ALVO E MIRA NO MOMENTO DO TIRO (CONSTÂNCIA)
    /** O hub que o shooter está mirando (aliança, ou o hub de treino sem FMS). */
    default Translation2d hubTarget() { return frc.robot.subsystems.shooter.ShooterConstants.BLUE_HUB_CENTER; }
    /** Do shooter ao hub, em metros. NaN = desconhecida. */
    default double hubDistanceMeters() { return Double.NaN; }
    /** Direção do alvo atual menos direção do shooter, em graus. 0 = apontando certo. NaN = desconhecido. */
    default double aimErrorDegrees() { return Double.NaN; }
    // #endregion

    // #region 📷 VISÃO (LIMELIGHT) NO MOMENTO DO TIRO

    // Padrão dos testes: visão sempre em dia.

    /** A Limelight corrigiu a pose ao menos uma vez desde o último reset. */
    default boolean visionCorrectedSinceReset() { return true; }

    /** A última correção aceita tem no máximo 1 s (Drive.VISION_RECENT_CORRECTION_SECONDS). */
    default boolean visionCorrectionRecent() { return true; }

    /** Segundos desde a última correção aceita. NaN = nenhuma. */
    default double visionCorrectionAgeSeconds() { return 0.0; }

    /** Câmera da última correção aceita. */
    default String lastVisionCamera() { return "NENHUMA"; }

    // #endregion

    // #region 🧪 AVISOS DA SIMULAÇÃO

    /** Uma bola entrou: a simulação do rolo sente a carga. */
    default void onBallCollected() {}

    /** Uma bola saiu: a simulação do top perde um pouco de RPM. */
    default void onBallLaunched() {}

    // #endregion
}
