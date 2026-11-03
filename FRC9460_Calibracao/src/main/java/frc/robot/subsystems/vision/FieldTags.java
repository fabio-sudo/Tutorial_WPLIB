package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Contas com as tags do mapa oficial (VisionFieldConstants.FIELD_LAYOUT).
 * Ficam aqui, e não nas classes de constantes, que guardam só valores.
 */
public final class FieldTags {
    // #region CENTRO DE UM ELEMENTO DO CAMPO

    /** X pela média das tags da frente e de trás; Y pela média das laterais. */
    public static Translation2d centerBetween(int frontTagX, int backTagX, int sideTagY, int otherSideTagY) {
        return new Translation2d(
            (xy(frontTagX).getX() + xy(backTagX).getX()) / 2.0,
            (xy(sideTagY).getY() + xy(otherSideTagY).getY()) / 2.0);
    }

    /** Posição (X, Y) da tag; falha ao iniciar se o mapa não tiver a tag. */
    public static Translation2d xy(int id) {
        return VisionFieldConstants.FIELD_LAYOUT.getTagPose(id)
            .orElseThrow(() -> new IllegalStateException("Tag " + id + " ausente no mapa do campo"))
            .toPose2d().getTranslation();
    }

    // #endregion
    private FieldTags() {}
}
