package frc.robot;

import static frc.robot.LoggingConstants.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Locale;
import java.util.Properties;
import java.io.IOException;

import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.LoggingConstants.Mode;

/**
 * Escolhe o modo de execução e os arquivos de log do AdvantageKit.
 * Os valores (modo padrão, pasta, prefixo, sufixo) ficam em LoggingConstants.
 */
public final class LoggingSetup {
    /** Metadados do artefato compilado; nunca usa data/versão do log de entrada. */
    static Properties buildInfo() {
        Properties info = new Properties();
        try (var stream = LoggingSetup.class.getResourceAsStream("/frc9460-build.properties")) {
            if (stream != null) info.load(stream);
        } catch (IOException ignored) {
            // O diagnóstico continua possível com artefatos antigos sem o recurso.
        }
        return info;
    }
    // #region MODOS DE EXECUÇÃO

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

    // #region 🧪 ARQUIVO DE SAÍDA DA SIMULAÇÃO

    private static final DateTimeFormatter SIM_LOG_TIME = DateTimeFormatter.ofPattern(SIM_LOG_TIME_PATTERN);

    /** Pasta e prefixo próprios: no AdvantageScope o nome já diz que é simulação. */
    public static String simulationLogPath(LocalDateTime start) {
        return Path.of(SIM_LOG_DIRECTORY, SIM_LOG_PREFIX + SIM_LOG_TIME.format(start) + ".wpilog")
            .toString();
    }

    // #endregion

    // #region 📊 ARQUIVO DE ENTRADA DO REPLAY

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

    // #region 📊 ARQUIVOS DOS PAINÉIS DO ELASTIC

    /** No robô, o deploy copia os painéis para deploy/elastic. No computador, valem os da fonte. */
    public static Path elasticLayoutsDirectory(Mode mode, Path deployDirectory) {
        return mode == Mode.REAL
            ? deployDirectory.resolve("elastic")
            : Path.of(ELASTIC_LAYOUTS_DIRECTORY).toAbsolutePath();
    }

    // #endregion

    private LoggingSetup() {}
}
