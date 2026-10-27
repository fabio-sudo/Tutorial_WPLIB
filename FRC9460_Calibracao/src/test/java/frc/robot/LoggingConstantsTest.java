package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class LoggingConstantsTest {
    @Test void desktopStaysInSimulationUnlessReplayIsExplicit() {
        assertEquals(LoggingConstants.Mode.SIM, LoggingConstants.resolveMode(false, null));
        assertEquals(LoggingConstants.Mode.SIM, LoggingConstants.resolveMode(false, " "));
        assertEquals(LoggingConstants.Mode.REPLAY, LoggingConstants.resolveMode(false, " replay "));
    }

    @Test void simulationLogsGoToTheirOwnFolderWithAPrefix() {
        assertEquals(Path.of("logs", "simulacao", "Simulacao_26-10-21_14-05-09.wpilog").toString(),
            LoggingConstants.simulationLogPath(java.time.LocalDateTime.of(2026, 10, 21, 14, 5, 9)));
    }

    @Test void realRobotCannotBeChangedToReplayByDesktopConfiguration() {
        assertEquals(LoggingConstants.Mode.REAL, LoggingConstants.resolveMode(true, "REPLAY"));
    }

    @Test void invalidDesktopModesDoNotSilentlyStartAnotherMode() {
        assertThrows(IllegalArgumentException.class, () -> LoggingConstants.resolveMode(false, "REAL"));
        assertThrows(IllegalArgumentException.class, () -> LoggingConstants.resolveMode(false, "TYPO"));
    }

    @Test void replayRequiresAReadableWpilogAndAcceptsQuotedPaths(@TempDir Path directory) throws Exception {
        Path log = Files.write(directory.resolve("coleta com espaço.wpilog"), new byte[0]);
        assertEquals(log, LoggingConstants.requireReplayLog("\"" + log + "\""));
        assertThrows(IllegalArgumentException.class, () -> LoggingConstants.requireReplayLog(""));
        assertThrows(IllegalArgumentException.class, () -> LoggingConstants.requireReplayLog(directory.toString()));
        assertThrows(IllegalArgumentException.class,
            () -> LoggingConstants.requireReplayLog(directory.resolve("ausente.wpilog").toString()));
        Path csv = Files.writeString(directory.resolve("log.csv"), "x,y");
        assertThrows(IllegalArgumentException.class, () -> LoggingConstants.requireReplayLog(csv.toString()));
    }
}
