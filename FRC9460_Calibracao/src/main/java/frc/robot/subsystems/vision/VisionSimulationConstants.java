package frc.robot.subsystems.vision;

/**
 * Propriedades das câmeras virtuais. Não altera os pipelines das Limelights reais.
 * Posição e orientação vêm de VisionCameraTransforms/Constants.LimelightConstants.
 */
public final class VisionSimulationConstants {
    // #region VALORES CONFIRMADOS NOS PRINTS DO PIPELINE

    public static final int IMAGE_WIDTH_PIXELS = 1280;
    public static final int IMAGE_HEIGHT_PIXELS = 800;
    public static final double CAPTURE_FPS = 60.0;
    public static final double EXPOSURE_MILLISECONDS = 0.66; // 66 * 0,01 ms.
    public static final double PIPELINE_MIN_TARGET_AREA_PERCENT = 0.0010;

    // Somente documentação do detector real: PhotonLib não reproduz o motor
    // U-Michigan, Quality Threshold, ganho ou a exposição de uma imagem real.
    // Downscale não é recorte e não reduz o FOV da lente pela metade.
    public static final int PIPELINE_DETECTOR_DOWNSCALE = 2;
    public static final double APRILTAG_SIZE_METERS = 0.1651;

    // #endregion

    // #region LENTE PADRÃO DA LIMELIGHT 4

    // FOV nominal da documentação oficial (lente original):
    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4
    // Não substitui a matriz intrínseca e a distorção medidas via ChArUco.
    public static final double HORIZONTAL_FOV_DEGREES = 82.0;
    public static final double VERTICAL_FOV_DEGREES = 56.2;

    // Os prints mostram X/Y-Crop de -1 a +1: utilizamos a imagem inteira.

    // #endregion

    // #region PARÂMETROS DE ENSAIO AINDA NÃO MEDIDOS NA CÂMERA REAL

    // Latência total captura -> entrega. O valor tl do pipeline sozinho não
    // inclui necessariamente todas as etapas. Mantemos o ensaio anterior.
    public static final double AVERAGE_LATENCY_MILLISECONDS = 30.0;
    public static final double LATENCY_STD_DEV_MILLISECONDS = 0.0;
    public static final double AVERAGE_PIXEL_ERROR = 0.25;
    public static final double PIXEL_ERROR_STD_DEV = 0.08;

    // O detector simulado usa uma área mínima adicional como aproximação da
    // resolução necessária para decodificar uma tag. Não é um limite oficial
    // da LL4: conservamos os 100 pixels do ensaio anterior da PhotonLib.
    public static final double MIN_DETECTABLE_TARGET_AREA_PIXELS = 100.0;

    // Não há alcance real medido. Um número finito pode ser usado para ensaios
    // de perda de visão, mas não deve ser apresentado como alcance da LL4.
    // O FOV, a face da tag e a área mínima já limitam as detecções na PhotonLib.
    public static final double MAX_SIGHT_RANGE_METERS = Double.POSITIVE_INFINITY;

    // #endregion

    // #region PERFIL COMPARTILHADO PELAS DUAS CÂMERAS

    /** Permite adotar outro perfil por câmera sem duplicar a implementação. */
    public record CameraProfile(
        int widthPixels,
        int heightPixels,
        double horizontalFovDegrees,
        double verticalFovDegrees,
        double framesPerSecond,
        double exposureMilliseconds,
        double averageLatencyMilliseconds,
        double latencyStdDevMilliseconds,
        double averagePixelError,
        double pixelErrorStdDev,
        double minTargetAreaPercent,
        double minDetectableAreaPixels,
        double maxSightRangeMeters
    ) {}

    public static final CameraProfile LIMELIGHT_4 = new CameraProfile(
        IMAGE_WIDTH_PIXELS,
        IMAGE_HEIGHT_PIXELS,
        HORIZONTAL_FOV_DEGREES,
        VERTICAL_FOV_DEGREES,
        CAPTURE_FPS,
        EXPOSURE_MILLISECONDS,
        AVERAGE_LATENCY_MILLISECONDS,
        LATENCY_STD_DEV_MILLISECONDS,
        AVERAGE_PIXEL_ERROR,
        PIXEL_ERROR_STD_DEV,
        PIPELINE_MIN_TARGET_AREA_PERCENT,
        MIN_DETECTABLE_TARGET_AREA_PIXELS,
        MAX_SIGHT_RANGE_METERS
    );

    // Mesmo modelo/pipeline: muda somente a montagem já definida no projeto.
    public static final CameraProfile REAR = LIMELIGHT_4;
    public static final CameraProfile FRONT = LIMELIGHT_4;

    // #endregion

    private VisionSimulationConstants() {}
}
