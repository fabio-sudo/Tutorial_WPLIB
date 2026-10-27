package frc.robot;

import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Locale;

import edu.wpi.first.wpilibj.RobotBase;

/** Configuração da gravação e do replay, separada dos filtros da visão. */
public final class LoggingConstants {
    // #region MODOS DE EXECUÇÃO

    public enum Mode { REAL, SIM, REPLAY }

    // O desktop continua iniciando a simulação óptica por padrão.
    // -PreplayLog no Gradle seleciona REPLAY somente naquela execução.
    public static final Mode DEFAULT_DESKTOP_MODE = Mode.SIM;
    public static final String MODE_ENVIRONMENT_VARIABLE = "FRC9460_MODE";
    public static final String SIM_LOG_DIRECTORY = "logs/simulacao";
    public static final String SIM_LOG_PREFIX = "Simulacao_";
    public static final String REPLAY_OUTPUT_SUFFIX = "_replay";
    private static final DateTimeFormatter SIM_LOG_TIME =
        DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss");

    /** Pasta e prefixo próprios: no AdvantageScope o nome já diz que é simulação. */
    public static String simulationLogPath(LocalDateTime start) {
        return Path.of(SIM_LOG_DIRECTORY, SIM_LOG_PREFIX + SIM_LOG_TIME.format(start) + ".wpilog")
            .toString();
    }

    public static Mode getMode() {
        return resolveMode(RobotBase.isReal(), System.getenv(MODE_ENVIRONMENT_VARIABLE));
    }

    static Mode resolveMode(boolean realRobot, String requestedMode) {
        // Uma variável do computador nunca transforma o roboRIO em replay.
        if (realRobot) return Mode.REAL;
        if (requestedMode == null || requestedMode.isBlank()) return DEFAULT_DESKTOP_MODE;
        Mode mode = Mode.valueOf(requestedMode.strip().toUpperCase(Locale.ROOT));
        if (mode == Mode.REAL) {
            throw new IllegalArgumentException("No desktop, use FRC9460_MODE=SIM ou REPLAY.");
        }
        return mode;
    }

    // #endregion

    // #region ARQUIVO DE ENTRADA DO REPLAY

    /** Evita iniciar o Logger com caminho vazio, CSV, pasta ou arquivo ausente. */
    public static Path requireReplayLog(String suppliedPath) {
        if (suppliedPath == null || suppliedPath.isBlank()) {
            throw new IllegalArgumentException("Selecione um log AdvantageKit .wpilog para replay.");
        }
        String value = suppliedPath.strip();
        if ((value.startsWith("\"") && value.endsWith("\""))
                || (value.startsWith("'") && value.endsWith("'"))) {
            value = value.substring(1, value.length() - 1);
        }
        Path path = Path.of(value).toAbsolutePath().normalize();
        if (!path.toString().endsWith(".wpilog") || !Files.isRegularFile(path)
                || !Files.isReadable(path)) {
            throw new IllegalArgumentException("Log AdvantageKit .wpilog não encontrado: " + path);
        }
        return path;
    }

    // #endregion
    private LoggingConstants() {}
}
