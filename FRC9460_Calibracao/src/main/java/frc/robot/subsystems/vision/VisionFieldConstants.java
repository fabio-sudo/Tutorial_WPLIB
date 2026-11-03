package frc.robot.subsystems.vision;

// #region IMPORTS

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

// #endregion

/**
 * Centraliza o mapa do campo usado pelo projeto.
 *
 * Guarda as posições nominais oficiais das AprilTags.
 * A montagem de cada câmera no robô continua sendo
 * uma configuração separada.
 */
public final class VisionFieldConstants {

    // #region SELEÇÃO DO CAMPO

    // Principal configuração para trocar o mapa da temporada.
    //
    // Escolhemos explicitamente a variante Welded para
    // corresponder ao modelo 2026 do AdvantageScope.
    public static final AprilTagFields SELECTED_FIELD =
        AprilTagFields.k2026RebuiltWelded;

    // #endregion

    // #region 📷 MAPA OFICIAL DAS TAGS

    // Carrega o mapa incluído na própria WPILib.
    // Não depende de internet durante a execução.
    //
    // A origem padrão é a origem azul do campo.
    // Mantemos esse referencial também na aliança vermelha,
    // consistente com as poses wpiBlue usadas no projeto.
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(SELECTED_FIELD);

    // Dimensões obtidas do mesmo mapa, em metros.
    public static final double FIELD_LENGTH_METERS =
        FIELD_LAYOUT.getFieldLength();

    public static final double FIELD_WIDTH_METERS =
        FIELD_LAYOUT.getFieldWidth();

    // #endregion

    // #region DADOS PARA VISUALIZAÇÃO

    // Todas as posições e orientações das tags.
    // Esta lista representa o mapa completo,
    // não apenas as tags visíveis para uma câmera.
    public static final Pose3d[] TAG_POSES =
        FIELD_LAYOUT.getTags()
            .stream()
            .map(tag -> tag.pose)
            .toArray(Pose3d[]::new);

    // IDs na mesma ordem das poses acima.
    public static final int[] TAG_IDS =
        FIELD_LAYOUT.getTags()
            .stream()
            .mapToInt(tag -> tag.ID)
            .toArray();

    // #endregion

    // Classe de constantes: não precisamos criar instâncias.
    private VisionFieldConstants() {}
}