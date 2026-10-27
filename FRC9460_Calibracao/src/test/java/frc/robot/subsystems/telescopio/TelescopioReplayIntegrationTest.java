package frc.robot.subsystems.telescopio;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Recalcula o controle com sensores/painel gravados, sem integrar a física. */
class TelescopioReplayIntegrationTest {
    @Test void replayReproducesControlAndPreservesSourceLog() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        Path directory = Path.of(System.getProperty("telescopio.validationDir"));
        Path source = directory.resolve("Telescopio9460-Simulacao.wpilog");
        Path output = directory.resolve("Telescopio9460-Replay.wpilog");
        byte[] original = Files.readAllBytes(source);
        int frames = countFrames(source);
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.setReplaySource(new WPILOGReader(source.toString()));
        Logger.addDataReceiver(new WPILOGWriter(output.toString(),
            WPILOGWriter.AdvantageScopeOpenBehavior.NEVER));
        Logger.start();
        var io = Telescopio.createIO();
        assertFalse(io instanceof TelescopioIOSim);
        assertFalse(io instanceof TelescopioIOTalonFX);
        var telescope = new Telescopio(io);
        new TelescopioDashboard(telescope);
        var scheduler = CommandScheduler.getInstance();
        try {
            // O leitor usa o timestamp seguinte para fechar o ciclo. No EOF o
            // Logger encerra a JVM; deixamos só o último frame fora do ensaio.
            for (int cycle = 0; cycle < frames - 1; cycle++) {
                if (cycle > 0) Logger.AdvancedHooks.invokePeriodicBeforeUser();
                scheduler.run();
                Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
            }
        } finally {
            scheduler.cancelAll();
            telescope.close();
            scheduler.getDefaultButtonLoop().clear();
            Logger.end();
        }
        assertArrayEquals(original, Files.readAllBytes(source));
        Map<Long, Double> expected = samples(output, "/RealOutputs/Telescopio/TensaoSolicitadaVolts");
        Map<Long, Double> actual = samples(output, "/ReplayOutputs/Telescopio/TensaoSolicitadaVolts");
        int compared = 0;
        for (var sample : expected.entrySet()) {
            assertTrue(actual.containsKey(sample.getKey()), "Ciclo original deve ter saída recalculada");
            assertEquals(sample.getValue(), actual.get(sample.getKey()), 1e-8,
                "Controle deve ser idêntico usando apenas as entradas gravadas");
            compared++;
        }
        assertTrue(compared >= 900);
        assertTrue(expected.values().stream().anyMatch(v -> v > 1.0));
    }
    private static int countFrames(Path path) throws Exception {
        var names = new HashMap<Integer, String>();
        int count = 0;
        for (var record : new DataLogReader(path.toString())) {
            if (record.isStart()) names.put(record.getStartData().entry, record.getStartData().name);
            if (!record.isControl() && "/Timestamp".equals(names.get(record.getEntry()))) count++;
        }
        return count;
    }
    private static Map<Long, Double> samples(Path path, String field) throws Exception {
        var names = new HashMap<Integer, String>();
        var result = new HashMap<Long, Double>();
        Long timestamp = null;
        Double value = null;
        for (var record : new DataLogReader(path.toString())) {
            if (record.isStart()) names.put(record.getStartData().entry, record.getStartData().name);
            if (record.isControl()) continue;
            String name = names.get(record.getEntry());
            if ("/Timestamp".equals(name)) {
                if (timestamp != null && value != null) result.put(timestamp, value);
                timestamp = record.getInteger();
            } else if (field.equals(name)) {
                value = record.getDouble();
            }
        }
        if (timestamp != null && value != null) result.put(timestamp, value);
        return result;
    }
}
