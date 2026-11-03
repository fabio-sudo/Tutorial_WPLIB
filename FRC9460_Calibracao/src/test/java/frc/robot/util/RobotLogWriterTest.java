package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.littletonrobotics.junction.LogTable;
import edu.wpi.first.util.datalog.DataLogReader;

class RobotLogWriterTest {
    @TempDir Path temp;

    @Test void usbWritesAReadableWpilogAndPreservesExistingFiles() throws Exception {
        Path usb = Files.createDirectory(temp.resolve("usb"));
        Path logs = Files.createDirectory(usb.resolve("logs"));
        Path previous = Files.writeString(logs.resolve("anterior.wpilog"), "preservar");
        RobotLogWriter writer = new RobotLogWriter(usb, temp.resolve("internal"));
        writer.start();
        LogTable table = new LogTable(20_000);
        table.put("Teste", 123.0);
        writer.putTable(table);
        assertEquals("GRAVANDO_USB", RobotLogWriter.status().state());
        Path path = Path.of(RobotLogWriter.status().path());
        writer.end();
        // Lê uma cópia em memória: o leitor por nome de arquivo mapeia o
        // arquivo e o Windows não deixa o JUnit apagar a pasta temporária.
        assertTrue(new DataLogReader(ByteBuffer.wrap(Files.readAllBytes(path))).isValid());
        assertEquals("preservar", Files.readString(previous));
        assertFalse(Files.exists(temp.resolve("internal")));
    }

    @Test void missingUsbUsesInternalWithoutCreatingFakeUsbDirectory() throws Exception {
        Path usb = temp.resolve("absent-usb");
        RobotLogWriter writer = new RobotLogWriter(usb, temp.resolve("internal"));
        writer.start();
        writer.putTable(new LogTable(20_000));
        assertEquals("GRAVANDO_INTERNO", RobotLogWriter.status().state());
        assertTrue(RobotLogWriter.status().writing());
        assertFalse(RobotLogWriter.status().failed());
        writer.end();
        assertFalse(Files.exists(usb));
    }

    @Test void reserveStaysBelowWhereTheRevLogsStartCleaningUp() {
        // Com 100 MiB, o disco ficou em 84-99 MB e 5 sessões não gravaram nada: a
        // REVLib só apaga os logs dela abaixo de 50 MB, e o disco parava antes disso.
        assertTrue(RobotLogWriter.RESERVE_BYTES < 50_000_000L);
    }

    @Test void logNameSortsByDate() {
        assertEquals("FRC9460_26-10-30_15-38-07.wpilog",
            RobotLogWriter.datedName(java.time.LocalDateTime.of(2026, 10, 30, 15, 38, 7)));
    }

    @Test void startingIsNotAFailureAndAStoppedLogIs() {
        assertFalse(new RobotLogWriter.Status("NAO_INICIADO", "", -1, false).failed());
        assertFalse(new RobotLogWriter.Status("VERIFICANDO_ARQUIVO", "x", -1, false).failed());
        assertTrue(new RobotLogWriter.Status("RESERVA_DE_DISCO_ATINGIDA", "x", 29, false).failed());
        assertTrue(new RobotLogWriter.Status("FALHA_DE_GRAVACAO", "x", -1, false).failed());
    }

    @Test void driverStationMessageSaysWhyAndWhatToDo() {
        String message = RobotHealth.logFailureMessage(
            new RobotLogWriter.Status("SEM_ESPACO_OU_SEM_PERMISSAO", "", 84.4, false));
        assertTrue(message.contains("SEM_ESPACO_OU_SEM_PERMISSAO"), message);
        assertTrue(message.contains("84 MiB"), message);
        assertTrue(message.contains("pendrive"), message);
    }

    @Test void unavailableStorageDoesNotCrashTelemetryThread() throws Exception {
        Path blocked = Files.writeString(temp.resolve("not-a-folder"), "preservar");
        RobotLogWriter writer = new RobotLogWriter(temp.resolve("missing"), blocked.resolve("logs"));
        writer.start();
        assertDoesNotThrow(() -> writer.putTable(new LogTable(20_000)));
        assertDoesNotThrow(writer::end);
        assertFalse(RobotLogWriter.status().writing());
        assertTrue(RobotLogWriter.status().failed());
        assertEquals("SEM_ESPACO_OU_SEM_PERMISSAO", RobotLogWriter.status().state());
        assertEquals("preservar", Files.readString(blocked));
    }
}
