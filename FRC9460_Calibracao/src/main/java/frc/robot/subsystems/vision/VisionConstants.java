package frc.robot.subsystems.vision;

/**
 * Configuração da aceitação e da confiança das poses de visão.
 * O Vision aplica estas regras; o Drive mantém o estimador de posição.
 * A óptica das câmeras virtuais fica em VisionSimulationConstants; nome e
 * montagem das Limelights físicas, em LimelightConstants.
 */
public final class VisionConstants {
    // #region 📷 CÂMERAS FÍSICAS

    // Habilitação da frontal física. A simulação cria suas duas câmeras
    // explicitamente e independe desta opção.
    public static final boolean FRONT_CAMERA_ENABLED = false;

    // #endregion

    // #region 📷 FILA E ORDENAÇÃO DAS CAPTURAS

    // Segura as capturas por uma janela limitada para ordenar as duas câmeras.
    // A odometria continua atualizando normalmente durante essa espera.
    public static final double REORDER_SECONDS = 0.080;
    public static final double PAIR_WINDOW_SECONDS = 0.020;
    public static final double MAX_QUEUED_AGE_SECONDS = 0.500;
    public static final int MAX_PENDING_FRAMES = 128;
    public static final int MAX_REMEMBERED_TIMESTAMPS = 256;

    // #endregion

    // #region 📷 ACEITAÇÃO - TEMPO, GIRO E DISTÂNCIA

    // Idade máxima da captura: 0.250 s = 250 ms.
    public static final double MAX_VISION_CAPTURE_AGE_SECONDS = 0.250;

    // Margem para timestamp à frente: 0.020 s = 20 ms.
    public static final double MAX_VISION_FUTURE_TOLERANCE_SECONDS = 0.020;

    // Acima desse giro a imagem pode sofrer blur e a pose do MegaTag2 fica
    // menos confiável. Unidade: graus por segundo (720 = usado nos testes).
    public static final double MAX_VISION_YAW_RATE_DEG_PER_SEC =
        720.0;

    // Referência de distância preferida no diagnóstico do Vision.
    // Não representa o alcance físico da lente nem rejeita a pose sozinha.
    public static final double MAX_TAG_DISTANCE_METERS = 4.0;

    // #endregion

    // #region 📷 ACEITAÇÃO DAS SOLUÇÕES PHOTONLIB

    // Valores iniciais do projeto, sujeitos à validação no robô físico.
    // Ambiguidade é avaliada quando a solução utiliza somente uma tag.
    public static final double MAX_PHOTON_SINGLE_TAG_AMBIGUITY = 0.20;
    public static final double MAX_PHOTON_HEIGHT_METERS = 0.30;
    public static final double MAX_PHOTON_TILT_RADIANS = Math.toRadians(20.0);

    // #endregion

    // #region 📷 COMPARAÇÃO ENTRE CÂMERAS

    // Limiar = margem + multiplicador * combinação dos desvios de X/Y.
    // Uma contradição grande rejeita o grupo; a próxima leitura pode recuperar.
    public static final double DISAGREEMENT_MARGIN_METERS = 0.15;
    public static final double DISAGREEMENT_STD_MULTIPLIER = 3.0;

    // #endregion

    // #region 📷 CONFIANÇA EM X/Y

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

    // #region 📷 INICIALIZAÇÃO AUTOMÁTICA DA POSE (MEGATAG1)

    // O MegaTag2 usa o heading do Pigeon como verdade: se ele estiver errado
    // (robô ligado virado, Start + A esquecido), a pose sai ESPELHADA em volta
    // das tags e ainda assim é aceita (teste de 28 e 29/10/2026: o hub aparecia
    // a 9-10 m). O MegaTag1 calcula o heading pelas próprias tags. Só com o
    // robô DESABILITADO e PARADO ele define X, Y e heading, depois de vários
    // quadros consistentes. Durante o jogo o heading nunca é alterado: só alerta.
    public static final boolean AUTO_INIT_ENABLED = true;
    // Com uma tag só o heading do MegaTag1 é ambíguo (solução espelhada).
    public static final int AUTO_INIT_MIN_TAGS = 2;
    public static final double AUTO_INIT_MAX_TAG_DISTANCE_METERS = 4.5;
    // Quadros novos seguidos que precisam concordar entre si antes de aplicar.
    public static final int AUTO_INIT_STABLE_FRAMES = 6;
    public static final double AUTO_INIT_MAX_FRAME_SPREAD_METERS = 0.15;
    public static final double AUTO_INIT_MAX_FRAME_SPREAD_DEGREES = 3.0;
    // Sem quadro novo por este tempo, a contagem recomeça.
    public static final double AUTO_INIT_FRAME_TIMEOUT_SECONDS = 0.5;
    // Só reaplica se a pose do estimador estiver pior que isto em relação às tags.
    // Com a pose já alinhada nada muda: não há reset repetido nem salto.
    public static final double AUTO_INIT_HEADING_TOLERANCE_DEGREES = 6.0;
    public static final double AUTO_INIT_POSITION_TOLERANCE_METERS = 0.75;
    // Heading divergente das tags por este tempo, com o robô parado, vira alerta.
    public static final double HEADING_ALERT_SECONDS = 1.0;

    // #endregion

    // #region 📷 DIAGNÓSTICO (Vision/Diagnostico)

    // "Corrigindo" = uma medição foi aplicada no último 1 s.
    public static final double DIAGNOSIS_RECENT_SECONDS = 1.0;
    // Um problema que dura isto vira aviso na Driver Station (uma vez por mudança).
    public static final double DIAGNOSIS_WARNING_SECONDS = 1.0;
    // O texto com números é refeito a 4 Hz.
    public static final double DIAGNOSIS_TEXT_PERIOD_SECONDS = 0.25;
    // Conferência do heading pelo MegaTag1: com uma tag só ele é ambíguo.
    public static final int MEGATAG1_HEADING_MIN_TAGS = 2;

    // #endregion

    private VisionConstants() {}
}
