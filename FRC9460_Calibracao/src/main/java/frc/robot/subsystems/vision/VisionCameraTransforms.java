package frc.robot.subsystems.vision;

// #region IMPORTS

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.Constants.LimelightConstants;

// #endregion

/**
 * Montagem das câmeras para a simulação.
 *
 * Reaproveita as medidas utilizadas pelas câmeras reais.
 * Não guarda uma segunda cópia dos valores de montagem.
 *
 * Os transforms representam:
 * centro do robô -> posição e orientação da câmera.
 */
public final class VisionCameraTransforms {

    // #region CÂMERA TRASEIRA

    public static final Transform3d ROBOT_TO_REAR =
        fromMounting(
            LimelightConstants.REAR_FORWARD,
            LimelightConstants.REAR_RIGHT,
            LimelightConstants.REAR_UP,
            LimelightConstants.REAR_ROLL,
            LimelightConstants.REAR_PITCH,
            LimelightConstants.REAR_YAW
        );

    // #endregion

    // #region CÂMERA FRONTAL

    // Os valores FRONT_* continuam provisórios,
    // até serem substituídos pelas medidas da montagem real.
    public static final Transform3d ROBOT_TO_FRONT =
        fromMounting(
            LimelightConstants.FRONT_FORWARD,
            LimelightConstants.FRONT_RIGHT,
            LimelightConstants.FRONT_UP,
            LimelightConstants.FRONT_ROLL,
            LimelightConstants.FRONT_PITCH,
            LimelightConstants.FRONT_YAW
        );

    // #endregion

    // #region CONVERSÃO PARA A GEOMETRIA DA SIMULAÇÃO

    /**
     * Convenção adotada para as constantes de montagem:
     *
     * forward: positivo para a frente do robô.
     * right: positivo para a direita do robô.
     * up: positivo para cima.
     * pitch: positivo aponta a câmera para cima.
     * yaw: zero aponta para a frente; 180 aponta para trás.
     *
     * Na WPILib:
     * X positivo aponta para a frente.
     * Y positivo aponta para a esquerda.
     * Z positivo aponta para cima.
     * Pitch positivo inclina o eixo X para baixo.
     *
     * Por isso, invertemos right e pitch nesta conversão.
     * As constantes e a configuração das câmeras reais
     * permanecem nos seus arquivos atuais.
     */
    private static Transform3d fromMounting(
        double forwardMeters,
        double rightMeters,
        double upMeters,
        double rollDegrees,
        double pitchUpDegrees,
        double yawDegrees
    ) {
        return new Transform3d(
            new Translation3d(
                forwardMeters,
                -rightMeters,
                upMeters
            ),
            new Rotation3d(
                Math.toRadians(rollDegrees),
                Math.toRadians(-pitchUpDegrees),
                Math.toRadians(yawDegrees)
            )
        );
    }

    // #endregion

    private VisionCameraTransforms() {}
}