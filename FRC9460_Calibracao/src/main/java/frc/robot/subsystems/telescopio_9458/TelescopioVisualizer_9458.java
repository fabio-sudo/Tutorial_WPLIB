package frc.robot.subsystems.telescopio_9458;

// #region IMPORTS

import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

// #endregion

/** Visualização de bancada compatível com o asset Robot_Telescopio_9458 (cascata). */
public class TelescopioVisualizer_9458 implements AutoCloseable {

    // #region MECANISMO 2D

    // Cada tubo é uma linha vertical; a base de cada linha sobe com o estágio.
    private static final double TUBE_X_METERS = 0.6;
    private static final double FLOOR_Y_METERS = 0.1;
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(
            1.2, 0.3 + CLOSED_HEIGHT_METERS + MAX_EXTENSION_METERS, new Color8Bit(32, 41, 52));
    private final LoggedMechanismRoot2d tube3;
    private final LoggedMechanismRoot2d tube2;

    public TelescopioVisualizer_9458() {
        mechanism.getRoot("Estagio1Fixo", TUBE_X_METERS, FLOOR_Y_METERS).append(new LoggedMechanismLigament2d(
                "Tubo4", N1_LENGTH_METERS, 90, 18, new Color8Bit(64, 136, 255)));
        tube3 = mechanism.getRoot("Estagio2Tubo3", TUBE_X_METERS, FLOOR_Y_METERS + N2_CLOSED_BOTTOM_METERS);
        tube3.append(new LoggedMechanismLigament2d(
                "Tubo3", N2_LENGTH_METERS, 90, 12, new Color8Bit(255, 153, 0)));
        tube2 = mechanism.getRoot("Estagio3Tubo2", TUBE_X_METERS, FLOOR_Y_METERS + N3_CLOSED_BOTTOM_METERS);
        tube2.append(new LoggedMechanismLigament2d(
                "Tubo2", N3_LENGTH_METERS, 90, 6, new Color8Bit(68, 221, 136)));
    }

    // #endregion

    // #region CURSO DOS ESTAGIOS

    /** Cascata: cada estágio anda metade do deslocamento da ponta (até 0,498 m). */
    public static double stageExtension(double extensionMeters) {
        return Double.isFinite(extensionMeters)
                ? MathUtil.clamp(extensionMeters, MIN_EXTENSION_METERS, MAX_EXTENSION_METERS) / 2
                : 0;
    }

    // #endregion

    // #region POSES 3D DOS COMPONENTES

    public static Pose3d[] componentPoses(double extensionMeters) {
        double stage = stageExtension(extensionMeters);
        // Relativas à base: o tubo 2" soma o curso do tubo 3".
        return new Pose3d[] {
                new Pose3d(0, 0, stage, new Rotation3d()),
                new Pose3d(0, 0, 2 * stage, new Rotation3d())
        };
    }

    // #endregion

    // #region ATUALIZACAO E LOGGING

    public void update(double extensionMeters, boolean valid) {
        double stage = stageExtension(extensionMeters);
        tube3.setPosition(TUBE_X_METERS, FLOOR_Y_METERS + N2_CLOSED_BOTTOM_METERS + stage);
        tube2.setPosition(TUBE_X_METERS, FLOOR_Y_METERS + N3_CLOSED_BOTTOM_METERS + 2 * stage);
        Logger.recordOutput("Telescopio9458/Pose", new Pose3d());
        Logger.recordOutput("Telescopio9458/Componentes", componentPoses(extensionMeters));
        Logger.recordOutput("Telescopio9458/Mecanismo2d", mechanism);
        Logger.recordOutput("Telescopio9458/ExtensaoTubo3Metros", stage);
        Logger.recordOutput("Telescopio9458/ExtensaoTubo2Metros", stage);
        Logger.recordOutput("Telescopio9458/ExtensaoMetros", 2 * stage);
        Logger.recordOutput("Telescopio9458/MotorRotacoes", stage / STAGE_METERS_PER_MOTOR_ROTATION);
        Logger.recordOutput("Telescopio9458/AlturaTotalMetros", CLOSED_HEIGHT_METERS + 2 * stage);
        Logger.recordOutput("Telescopio9458/VisualizacaoValida", valid);
    }

    // #endregion

    // #region ENCERRAMENTO

    @Override
    public void close() {
        mechanism.close();
    }

    // #endregion
}
