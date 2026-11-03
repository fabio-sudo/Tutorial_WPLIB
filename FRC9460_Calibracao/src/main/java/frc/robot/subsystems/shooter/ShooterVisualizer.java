package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color8Bit;

// #endregion

/**
 * Desenhos do shooter no AdvantageScope. O giro do shooter no modelo 3D
 * fica em frc.robot.RobotComponentsVisualizer, junto com o intake.
 *
 * Publica:
 * - um desenho 2D visto de cima (shooter, hub e limites de ±130°);
 * - linhas no campo 3D: para onde o shooter aponta e onde está o hub.
 */
public class ShooterVisualizer implements AutoCloseable {

    private static final int REAL_MECHANISM_PERIOD_CYCLES = 5;
    private int cycle;

    // #region DESENHO 2D (VISTO DE CIMA)

    // Tela de 1 x 1 com o robô no centro e a FRENTE DO ROBÔ PARA CIMA.
    private static final double CANVAS_SIZE = 1.0;
    private static final double CENTER = CANVAS_SIZE / 2.0;

    private final LoggedMechanism2d mechanism =
        new LoggedMechanism2d(CANVAS_SIZE, CANVAS_SIZE, new Color8Bit(32, 41, 52));
    private final LoggedMechanismLigament2d shooterLine;
    private final LoggedMechanismLigament2d hubLine;

    public ShooterVisualizer() {
        LoggedMechanismRoot2d root = mechanism.getRoot("CentroDoRobo", CENTER, CENTER);
        // Referências fixas: frente do robô e os dois limites da fiação.
        root.append(new LoggedMechanismLigament2d(
            "FrenteDoRobo", 0.18, canvasAngle(0.0), 3, new Color8Bit(220, 220, 220)));
        root.append(new LoggedMechanismLigament2d(
            "LimitePositivo", 0.42, canvasAngle(PIVOT_ZERO_FACING_DEGREES + PIVOT_MAX_ANGLE_DEGREES),
            2, new Color8Bit(229, 57, 53)));
        root.append(new LoggedMechanismLigament2d(
            "LimiteNegativo", 0.42, canvasAngle(PIVOT_ZERO_FACING_DEGREES - PIVOT_MAX_ANGLE_DEGREES),
            2, new Color8Bit(229, 57, 53)));
        // Móveis: direção do hub (verde) e do shooter (laranja).
        hubLine = root.append(new LoggedMechanismLigament2d(
            "DirecaoDoHub", 0.46, canvasAngle(PIVOT_ZERO_FACING_DEGREES), 4, new Color8Bit(59, 135, 90)));
        shooterLine = root.append(new LoggedMechanismLigament2d(
            "Shooter", 0.38, canvasAngle(PIVOT_ZERO_FACING_DEGREES), 9, new Color8Bit(228, 139, 50)));
    }

    /** Ângulo no robô (0° = frente, anti-horário) -> ângulo na tela (frente para cima). */
    private static double canvasAngle(double robotFrameDegrees) {
        return 90.0 + robotFrameDegrees;
    }

    // #endregion

    // #region ATUALIZAÇÃO

    /**
     * @param robotPose pose fundida do Drive (odometria + Limelight)
     * @param pivotDegrees ângulo do pivô lido do encoder
     * @param hubAim mira até o hub (para a linha verde)
     * @param hub centro do hub da aliança
     * @param virtualTarget alvo compensado do tiro andando
     */
    public void update(Pose2d robotPose, double pivotDegrees, ShotCalculator.Aim hubAim,
            Translation2d hub, Translation2d virtualTarget) {
        // 2D visto de cima.
        shooterLine.setAngle(canvasAngle(PIVOT_ZERO_FACING_DEGREES + pivotDegrees));
        hubLine.setAngle(canvasAngle(hubAim.robotFrameAngleDegrees()));
        // Cada gravação escreve dezenas de campos: no robô real, 10 Hz bastam para ver.
        if (!RobotBase.isReal() || cycle++ % REAL_MECHANISM_PERIOD_CYCLES == 0) {
            Logger.recordOutput("Shooter/Visualizacao/Mecanismo2d", mechanism);
        }

        // Linhas no campo 3D, saindo do ponto onde a bola sai (altura de saída).
        Translation2d shooter = ShotCalculator.shooterFieldPosition(robotPose);
        double heading = Math.toRadians(ShotCalculator.shooterFieldHeadingDegrees(robotPose, pivotDegrees));
        double length = Math.max(1.0, hubAim.distanceMeters());
        Translation2d pointing = shooter.plus(new Translation2d(length * Math.cos(heading), length * Math.sin(heading)));

        Logger.recordOutput("Shooter/Visualizacao/LinhaDoShooter", new Translation3d[] {
            atHeight(shooter, RELEASE_HEIGHT_METERS).getTranslation(),
            atHeight(pointing, TARGET_HEIGHT_METERS).getTranslation() });
        // Parado, o alvo virtual é o próprio hub; andando, fica deslocado.
        Logger.recordOutput("Shooter/Visualizacao/LinhaAteOAlvo", new Translation3d[] {
            atHeight(shooter, RELEASE_HEIGHT_METERS).getTranslation(),
            atHeight(virtualTarget, TARGET_HEIGHT_METERS).getTranslation() });
        Logger.recordOutput("Shooter/Visualizacao/Hub", atHeight(hub, TARGET_HEIGHT_METERS));
        Logger.recordOutput("Shooter/Visualizacao/AlvoVirtual", atHeight(virtualTarget, TARGET_HEIGHT_METERS));
    }

    private static Pose3d atHeight(Translation2d point, double heightMeters) {
        return new Pose3d(new Translation3d(point.getX(), point.getY(), heightMeters), new Rotation3d());
    }

    // #endregion

    // #region ENCERRAMENTO (TESTES)

    @Override
    public void close() {
        mechanism.close();
    }

    // #endregion
}
