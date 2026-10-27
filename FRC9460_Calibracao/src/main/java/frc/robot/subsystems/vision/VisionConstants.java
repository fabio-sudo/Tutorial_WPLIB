package frc.robot.subsystems.vision;

/**
 * Configuração da aceitação e da confiança das poses de visão.
 * O Vision aplica estas regras; o Drive mantém o estimador de posição.
 * A óptica das câmeras virtuais fica em VisionSimulationConstants.
 */
public final class VisionConstants {
    // #region CÂMERAS FÍSICAS

    // Habilitação da frontal física. A simulação cria suas duas câmeras
    // explicitamente e independe desta opção.
    public static final boolean FRONT_CAMERA_ENABLED = false;

    // #endregion

    // #region FILA E ORDENAÇÃO DAS CAPTURAS

    // Segura as capturas por uma janela limitada para ordenar as duas câmeras.
    // A odometria continua atualizando normalmente durante essa espera.
    public static final double REORDER_SECONDS = 0.080;
    public static final double PAIR_WINDOW_SECONDS = 0.020;
    public static final double MAX_QUEUED_AGE_SECONDS = 0.500;
    public static final int MAX_PENDING_FRAMES = 128;
    public static final int MAX_REMEMBERED_TIMESTAMPS = 256;

    // Os limites de idade, timestamp futuro e giro já usados pela Limelight
    // continuam em Constants.LimelightConstants, compartilhados pelo Vision.

    // #endregion

    // #region ACEITAÇÃO DAS SOLUÇÕES PHOTONLIB

    // Valores iniciais do projeto, sujeitos à validação no robô físico.
    // Ambiguidade é avaliada quando a solução utiliza somente uma tag.
    public static final double MAX_PHOTON_SINGLE_TAG_AMBIGUITY = 0.20;
    public static final double MAX_PHOTON_HEIGHT_METERS = 0.30;
    public static final double MAX_PHOTON_TILT_RADIANS = Math.toRadians(20.0);

    // #endregion

    // #region COMPARAÇÃO ENTRE CÂMERAS

    // Limiar = margem + multiplicador * combinação dos desvios de X/Y.
    // Uma contradição grande rejeita o grupo; a próxima leitura pode recuperar.
    public static final double DISAGREEMENT_MARGIN_METERS = 0.15;
    public static final double DISAGREEMENT_STD_MULTIPLIER = 3.0;

    // #endregion

    // #region CONFIANÇA EM X/Y

    // Desvio menor significa maior peso no estimador. Estes números são
    // parâmetros de ajuste, não medições da precisão real da Limelight.
    public static final double NEAR_TAG_DISTANCE_METERS = 2.0;
    public static final double MEDIUM_TAG_DISTANCE_METERS = 3.0;
    public static final double MULTI_TAG_NEAR_STD_DEV_METERS = 0.25;
    public static final double MULTI_TAG_MEDIUM_STD_DEV_METERS = 0.40;
    public static final double MULTI_TAG_FAR_STD_DEV_METERS = 0.60;
    public static final double SINGLE_TAG_NEAR_STD_DEV_METERS = 0.60;
    public static final double SINGLE_TAG_MEDIUM_STD_DEV_METERS = 0.85;
    public static final double SINGLE_TAG_FAR_STD_DEV_METERS = 1.20;

    // #endregion

    private VisionConstants() {}
}
