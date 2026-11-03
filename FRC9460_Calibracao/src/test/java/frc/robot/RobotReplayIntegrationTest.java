package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Replay do robô inteiro, incluindo controles, visão, movimento e disparo. */
class RobotReplayIntegrationTest {
    @Test void replaysDrivingAndShootingWithoutRunningAnotherSimulation() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        RobotTestClock.configure();
        Path directory = Path.of(System.getProperty("cicloDaBola.validationDir"));
        Path source = directory.resolve("CicloDaBola-Teste.wpilog");
        Path output = directory.resolve("CicloDaBola-Teste_replay.wpilog");
        byte[] original = Files.readAllBytes(source);
        int frames = countFrames(source);
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.setReplaySource(new WPILOGReader(source.toString()));
        Logger.addDataReceiver(new WPILOGWriter(output.toString(), WPILOGWriter.AdvantageScopeOpenBehavior.NEVER));
        Logger.start();
        var scheduler = CommandScheduler.getInstance();
        try {
            var container = new RobotContainer();
            for (String name : new String[] {"visionSimulation", "fuelSimulation"}) {
                var field = RobotContainer.class.getDeclaredField(name);
                field.setAccessible(true);
                assertNull(field.get(container), "Replay não deve criar outro mundo físico");
            }
            // O EOF do Logger encerra a JVM. O último frame fica fora deste ensaio.
            for (int cycle = 0; cycle < frames - 1; cycle++) {
                if (cycle > 0) Logger.AdvancedHooks.invokePeriodicBeforeUser();
                scheduler.run();
                container.updateVisionSimulation();
                container.updateFuelSimulation();
                Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
            }
        } finally {
            scheduler.cancelAll();
            scheduler.getDefaultButtonLoop().clear();
            scheduler.unregisterAllSubsystems();
            Logger.end();
        }
        assertArrayEquals(original, Files.readAllBytes(source), "Log original preservado");
        // Com o mesmo código, sensores e controles, os pedidos precisam se repetir.
        for (String key : new String[] {"Drive/Diagnostic/DesiredVx", "Drive/Diagnostic/DesiredVy",
                "Shooter/Top/AlvoRpm", "Shooter/Tower/AlvoRpm", "Shooter/Pivo/AlvoGraus",
                "Fuel/Contagem/DisparosDetectados"}) {
            compare(output, key);
        }
        // A constância é estatística entre disparos: grava só quando há tiro, então
        // vale o valor FINAL. O replay recalcula tudo dos sensores gravados e precisa
        // chegar exatamente aos mesmos números do robô.
        for (String key : new String[] {"Fuel/Consistencia/TirosNaJanela",
                "Fuel/Consistencia/VelocidadeSaidaMps/Media", "Fuel/Consistencia/VelocidadeSaidaMps/Desvio",
                "Fuel/Consistencia/ErroMiraGraus/Media", "Fuel/Consistencia/ErroQuedaMetros/Media",
                "Fuel/Consistencia/TaxaAcertoSensores", "Fuel/Consistencia/TaxaAcertoPlanejado"}) {
            compareFinal(output, key);
        }
    }

    private static void compareFinal(Path path, String key) throws Exception {
        Map<Long, Double> expected = samples(path, "/RealOutputs/" + key);
        Map<Long, Double> actual = samples(path, "/ReplayOutputs/" + key);
        assertFalse(expected.isEmpty(), key + ": o ensaio precisa ter disparos");
        long lastCycle = expected.keySet().stream().mapToLong(Long::longValue).max().getAsLong();
        assertNotNull(actual.get(lastCycle), key + ": o replay não gravou o valor final");
        assertEquals(expected.get(lastCycle), actual.get(lastCycle), 1e-6, key + ": valor final diferente");
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

    private static void compare(Path path, String key) throws Exception {
        Map<Long, Double> expected = samples(path, "/RealOutputs/" + key);
        Map<Long, Double> actual = samples(path, "/ReplayOutputs/" + key);
        assertTrue(expected.size() > 900, key + ": precisa cobrir todo o roteiro");
        assertTrue(expected.values().stream().anyMatch(v -> Math.abs(v) > 0.1), key + ": ensaio não pode ser vazio");
        for (var sample : expected.entrySet()) {
            assertNotNull(actual.get(sample.getKey()), key + ": ciclo ausente " + sample.getKey());
            assertEquals(sample.getValue(), actual.get(sample.getKey()), 1e-6,
                key + ": diferença no timestamp " + sample.getKey());
        }
    }

    private static Map<Long, Double> samples(Path path, String field) throws Exception {
        var names = new HashMap<Integer, String>();
        var types = new HashMap<Integer, String>();
        var result = new HashMap<Long, Double>();
        Long timestamp = null;
        Double value = null;
        for (var record : new DataLogReader(path.toString())) {
            if (record.isStart()) {
                names.put(record.getStartData().entry, record.getStartData().name);
                types.put(record.getStartData().entry, record.getStartData().type);
            }
            if (record.isControl()) continue;
            String name = names.get(record.getEntry());
            if ("/Timestamp".equals(name)) {
                if (timestamp != null && value != null) result.put(timestamp, value);
                timestamp = record.getInteger();
            } else if (field.equals(name)) {
                value = "int64".equals(types.get(record.getEntry())) ? (double) record.getInteger() : record.getDouble();
            }
        }
        if (timestamp != null && value != null) result.put(timestamp, value);
        return result;
    }
}
