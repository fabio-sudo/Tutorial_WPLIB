package frc.robot.subsystems.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

// #endregion

/** Visualização de bancada compatível com o asset Robot_Telescopio_9460. */
public class TelescopioVisualizer implements AutoCloseable {

    // #region MECANISMO 2D

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.5, 2.4, new Color8Bit(32, 41, 52));
    private final LoggedMechanismLigament2d stage2;
    private final LoggedMechanismLigament2d stage3;

    public TelescopioVisualizer() {
        var root = mechanism.getRoot("Base", 0.75, 0.1);
        var stage1 = root.append(new LoggedMechanismLigament2d(
                "N1", N1_VISIBLE_LENGTH_METERS, 90, 14, new Color8Bit(64, 136, 255)));
        stage2 = stage1.append(new LoggedMechanismLigament2d(
                "N2", N2_CLOSED_VISIBLE_LENGTH_METERS, 0, 10, new Color8Bit(255, 153, 0)));
        stage3 = stage2.append(new LoggedMechanismLigament2d(
                "N3", N3_CLOSED_VISIBLE_LENGTH_METERS, 0, 6, new Color8Bit(68, 221, 136)));
    }

    // #endregion

    // #region CURSO DOS ESTAGIOS

    /** Ambos avançam a mesma fração do curso, encerrando juntos (624 e 628 mm). */
    public static double stage2Extension(double extensionMeters) {
        return fraction(extensionMeters) * N2_TRAVEL_METERS;
    }

    public static double stage3RelativeExtension(double extensionMeters) {
        return fraction(extensionMeters) * N3_RELATIVE_TRAVEL_METERS;
    }

    private static double fraction(double extensionMeters) {
        return Double.isFinite(extensionMeters)
                ? MathUtil.clamp(extensionMeters / MAX_EXTENSION_METERS, 0, 1)
                : 0;
    }

    // #endregion

    // #region POSES 3D DOS COMPONENTES

    public static Pose3d[] componentPoses(double extensionMeters) {
        double n2 = stage2Extension(extensionMeters);
        double n3 = stage3RelativeExtension(extensionMeters);
        // As poses dos componentes são relativas à base, não uma à outra.
        return new Pose3d[] {
                new Pose3d(N2_PIVOT_X_METERS, N2_PIVOT_Y_METERS,
                        N2_CLOSED_PIVOT_Z_METERS + n2, new Rotation3d()),
                new Pose3d(N3_PIVOT_X_METERS, N3_PIVOT_Y_METERS,
                        N3_CLOSED_PIVOT_Z_METERS + n2 + n3, new Rotation3d())
        };
    }

    // #endregion

    // #region ATUALIZACAO E LOGGING

    public void update(double extensionMeters, boolean valid) {
        double n2 = stage2Extension(extensionMeters);
        double n3 = stage3RelativeExtension(extensionMeters);
        stage2.setLength(N2_CLOSED_VISIBLE_LENGTH_METERS + n2);
        stage3.setLength(N3_CLOSED_VISIBLE_LENGTH_METERS + n3);
        Logger.recordOutput("Telescopio/BasePose", new Pose2d());
        Logger.recordOutput("Telescopio/Componentes", componentPoses(extensionMeters));
        Logger.recordOutput("Telescopio/Mecanismo2d", mechanism);
        Logger.recordOutput("Telescopio/ExtensaoN2Metros", n2);
        Logger.recordOutput("Telescopio/ExtensaoRelativaN3Metros", n3);
        Logger.recordOutput("Telescopio/ExtensaoVisualMetros", n2 + n3);
        Logger.recordOutput("Telescopio/AlturaTotalMetros", CLOSED_HEIGHT_METERS + n2 + n3);
        Logger.recordOutput("Telescopio/VisualizacaoValida", valid);
    }

    // #endregion

    // #region ENCERRAMENTO

    @Override
    public void close() {
        mechanism.close();
    }

    // #endregion
}
