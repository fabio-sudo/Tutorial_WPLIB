package frc.robot;

/**
 * Configuração da gravação e do replay, separada dos filtros da visão.
 * Aqui ficam só os valores; a lógica que os usa fica em LoggingSetup.
 */
public final class LoggingConstants {
    // #region MODOS DE EXECUÇÃO

    public enum Mode { REAL, SIM, REPLAY }

    // O desktop continua iniciando a simulação óptica por padrão.
    // -PreplayLog no Gradle seleciona REPLAY somente naquela execução.
    public static final Mode DEFAULT_DESKTOP_MODE = Mode.SIM;
    public static final String MODE_ENVIRONMENT_VARIABLE = "FRC9460_MODE";

    // #endregion

    // #region 📊 ARQUIVOS DE LOG

    // Pasta e prefixo próprios: no AdvantageScope o nome já diz que é simulação.
    // O replay grava ao lado do log lido. O Git ignora a pasta (regra logs/).
    public static final String SIM_LOG_DIRECTORY = "logs/simulacao";
    public static final String SIM_LOG_PREFIX = "Simulacao_";
    public static final String SIM_LOG_TIME_PATTERN = "yy-MM-dd_HH-mm-ss";
    public static final String REPLAY_OUTPUT_SUFFIX = "_replay";

    // Fonte dos painéis do Elastic. O deploy copia para deploy/elastic no robô.
    public static final String ELASTIC_LAYOUTS_DIRECTORY = "config/elastic/layouts";

    // #endregion
    private LoggingConstants() {}
}
