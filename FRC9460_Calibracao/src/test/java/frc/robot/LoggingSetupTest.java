package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class LoggingSetupTest {
    @Test void compiledArtifactIdentifiesItsSourcesAndBuildTime() {
        var info = LoggingSetup.buildInfo();
        assertTrue(info.getProperty("SourceSHA256", "").matches("[0-9a-f]{64}"));
        assertDoesNotThrow(() -> java.time.Instant.parse(info.getProperty("BuiltAtUTC")));
    }
    @Test void desktopStaysInSimulationUnlessReplayIsExplicit() {
        assertEquals(LoggingConstants.Mode.SIM, LoggingSetup.resolveMode(false, null));
        assertEquals(LoggingConstants.Mode.SIM, LoggingSetup.resolveMode(false, " "));
        assertEquals(LoggingConstants.Mode.REPLAY, LoggingSetup.resolveMode(false, " replay "));
    }

    @Test void simulationLogsGoToTheirOwnFolderWithAPrefix() {
        assertEquals(Path.of("logs", "simulacao", "Simulacao_26-10-21_14-05-09.wpilog").toString(),
            LoggingSetup.simulationLogPath(java.time.LocalDateTime.of(2026, 10, 21, 14, 5, 9)));
    }

    @Test void elasticPanelsComeFromDeployOnTheRobotAndFromTheSourceOnTheComputer() {
        Path deploy = Path.of("deploy");
        assertEquals(deploy.resolve("elastic"),
            LoggingSetup.elasticLayoutsDirectory(LoggingConstants.Mode.REAL, deploy));
        Path source = LoggingSetup.elasticLayoutsDirectory(LoggingConstants.Mode.SIM, deploy);
        assertTrue(source.isAbsolute());
        assertTrue(Files.isRegularFile(source.resolve("FRC9460-Diagnostico.json")), source.toString());
    }

    @Test void theDeployCopiesTheElasticPanelsToTheFolderTheRobotServes() throws Exception {
        // Sem esta cópia, File > Download Layout pararia de achar os painéis no robô.
        String gradle = Files.readString(Path.of("build.gradle"));
        assertTrue(gradle.contains("project.fileTree('" + LoggingConstants.ELASTIC_LAYOUTS_DIRECTORY + "')"));
        assertTrue(gradle.contains("directory = '/home/lvuser/deploy/elastic'"));
    }

    @Test void realRobotCannotBeChangedToReplayByDesktopConfiguration() {
        assertEquals(LoggingConstants.Mode.REAL, LoggingSetup.resolveMode(true, "REPLAY"));
    }

    @Test void invalidDesktopModesDoNotSilentlyStartAnotherMode() {
        assertThrows(IllegalArgumentException.class, () -> LoggingSetup.resolveMode(false, "REAL"));
        assertThrows(IllegalArgumentException.class, () -> LoggingSetup.resolveMode(false, "TYPO"));
    }

    @Test void replayRequiresAReadableWpilogAndAcceptsQuotedPaths(@TempDir Path directory) throws Exception {
        Path log = Files.write(directory.resolve("coleta com espaço.wpilog"), new byte[0]);
        assertEquals(log, LoggingSetup.requireReplayLog("\"" + log + "\""));
        assertThrows(IllegalArgumentException.class, () -> LoggingSetup.requireReplayLog(""));
        assertThrows(IllegalArgumentException.class, () -> LoggingSetup.requireReplayLog(directory.toString()));
        assertThrows(IllegalArgumentException.class,
            () -> LoggingSetup.requireReplayLog(directory.resolve("ausente.wpilog").toString()));
        Path csv = Files.writeString(directory.resolve("log.csv"), "x,y");
        assertThrows(IllegalArgumentException.class, () -> LoggingSetup.requireReplayLog(csv.toString()));
    }
}
