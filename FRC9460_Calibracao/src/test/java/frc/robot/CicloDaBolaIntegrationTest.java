package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.fuel.FuelSimulation;
import frc.robot.subsystems.fuel.FuelTracker;

/**
 * Processo isolado (o Logger é global): o robô INTEIRO na simulação, com um
 * controle só e os botões de jogo do piloto. Liga em (2, 2), dá um toque em
 * START + R1 (liga o rolete), segura R2 (desce o intake) e anda até as bolas
 * do centro; outro toque em START + R1 desliga o rolete. Depois um toque em
 * START + B liga o tiro automático no hub azul; outro toque desliga.
 * Parado, segura L1 (abre o telescópio 9460) e depois L2 (fecha).
 *
 * Confere que:
 * - bolas foram coletadas, lançadas e acertaram o hub;
 * - os sensores (FuelTracker) contaram o mesmo que a simulação;
 * - L1 abriu e L2 fechou o telescópio 9460;
 * - toda chave usada pelos layouts do AdvantageScope (Shooter, CicloDaBola,
 *   telescópio 9460) existe no log;
 * - todo tópico dos layouts do Elastic (config/elastic/layouts) existe no
 *   log com o tipo que o widget espera.
 */
class CicloDaBolaIntegrationTest {

    private static final String[] LAYOUTS = {
        "config/advantagescope/layouts/FRC9460-ValidacaoRobo.json",
        "config/advantagescope/layouts/FRC9460-Replay.json",
        "config/advantagescope/layouts/FRC9460-ConsistenciaDeTiro.json",
        "config/advantagescope/layouts/FRC9460-PoseLimelight.json",
        "config/advantagescope/layouts/FRC9460-Campo2D3D.json",
        "config/advantagescope/layouts/FRC9460-TesteFisicoVsModelo.json",
        "config/advantagescope/layouts/FRC9460-Shooter.json",
        "config/advantagescope/layouts/FRC9460-CicloDaBola.json",
        "config/advantagescope/layouts/Telescopio9460-Simulacao.json",
        "config/advantagescope/layouts/Telescopio9460-TesteControle.json"
    };

    private static final Path ELASTIC_DIR = Path.of("config/elastic/layouts");

    // Tipo do log (AdvantageKit) -> tipo do widget no Elastic.
    private static final Map<String, String> ELASTIC_TYPES = Map.of(
        "double", "double", "float", "float", "boolean", "boolean", "string", "string", "int64", "int",
        "double[]", "double[]", "int64[]", "int[]");

    /**
     * Eixo do joystick que dá esta velocidade, com a zona morta de 0,10 e a
     * curva quadrática do RobotContainer. O roteiro foi escrito com
     * kMaxLinearSpeed = 2,0 m/s; com 2,5 m/s (29/10/2026) o eixo cheio levava
     * o robô até x = 10,3 m, além das bolas, e só 4 eram coletadas.
     */
    private static double driverAxisFor(double speedMetersPerSecond) {
        return 0.10 + 0.90 * Math.sqrt(speedMetersPerSecond / DriveConstants.kMaxLinearSpeed);
    }

    private static Object field(Object owner, String name) throws Exception {
        Field f = owner.getClass().getDeclaredField(name);
        f.setAccessible(true);
        return f.get(owner);
    }

    @Test
    void wholeRobotCollectsShootsAndSensorsAgreeWithTheSimulation() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        RobotTestClock.configure();
        Path directory = Path.of(System.getProperty("cicloDaBola.validationDir"));
        Files.createDirectories(directory);
        Path output = directory.resolve("CicloDaBola-Teste.wpilog");

        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.addDataReceiver(new WPILOGWriter(output.toString(), WPILOGWriter.AdvantageScopeOpenBehavior.NEVER));
        Logger.start();

        RobotContainer container = new RobotContainer();
        FuelSimulation sim = (FuelSimulation) field(container, "fuelSimulation");
        FuelTracker tracker = (FuelTracker) field(container, "fuelTracker");
        frc.robot.subsystems.telescopio.Telescopio telescopio =
            (frc.robot.subsystems.telescopio.Telescopio) field(container, "telescopio");
        double telescopioMaxMeters = 0.0;
        var scheduler = CommandScheduler.getInstance();
        DriverStationSim.setJoystickAxisCount(0, 6);
        DriverStationSim.setJoystickButtonCount(0, 10);
        DriverStationSim.setJoystickPOVCount(0, 1);
        DriverStationSim.setJoystickAxisCount(1, 6);
        DriverStationSim.setJoystickButtonCount(1, 10);
        DriverStationSim.setJoystickPOVCount(1, 1);
        // Velocidades do roteiro original: eixo cheio e eixo 0,6 a 2,0 m/s.
        double forwardAxis = driverAxisFor(2.0);
        double leftAxis = driverAxisFor(0.617);
        try {
            for (int cycle = 0; cycle < 1350; cycle++) {
                if (cycle == 5) DriverStationSim.setEnabled(true);
                // Novo mapa: R1 desce intake, R2 puxa, B segura a sequência.
                DriverStationSim.setJoystickButton(0, 6, cycle >= 15 && cycle < 90);
                DriverStationSim.setJoystickAxis(0, 3, cycle >= 10 && cycle < 520 ? 1.0 : 0.0);
                DriverStationSim.setJoystickButton(0, 8, false);
                // Depois do tiro: BACK + A pede o teste de chegada a 2 m.
                // Os próprios botões ficam no log e são reaplicados no replay.
                DriverStationSim.setJoystickButton(0, 7, cycle >= 1000 && cycle < 1320);
                DriverStationSim.setJoystickButton(0, 1, cycle >= 1000 && cycle < 1320);
                DriverStationSim.setJoystickButton(0, 2, cycle >= 560 && cycle < 950);
                DriverStationSim.setJoystickAxis(0, 1, cycle >= 40 && cycle < 200 ? -forwardAxis : 0.0); // frente
                DriverStationSim.setJoystickAxis(0, 0, cycle >= 220 && cycle < 360 ? -leftAxis : 0.0);   // esquerda
                // L1 sobe intake; L2 só mira. O elevador é exercitado por comandos
                // na simulação, sem ocupar os botões operacionais do robô.
                DriverStationSim.setJoystickButton(0, 5, cycle >= 700 && cycle < 900);
                DriverStationSim.setJoystickAxis(0, 2, cycle >= 540 && cycle < 560 ? 1.0 : 0.0);
                if (cycle == 600) scheduler.schedule(
                    new frc.robot.commands.telescopio.TelescopioAbrirCommand(telescopio));
                if (cycle == 700) scheduler.schedule(
                    new frc.robot.commands.telescopio.TelescopioFecharCommand(telescopio));
                DriverStationSim.notifyNewData();
                SimHooks.stepTiming(0.02);
                Logger.AdvancedHooks.invokePeriodicBeforeUser();
                scheduler.run();
                container.updateVisionSimulation();
                container.updateFuelSimulation();
                Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
                telescopioMaxMeters = Math.max(telescopioMaxMeters, telescopio.getExtensionMeters());
            }
            assertTrue(telescopioMaxMeters > 0.2, "comando simulado abriu o telescópio 9460: " + telescopioMaxMeters);
            assertTrue(telescopio.getExtensionMeters() < 0.02, "comando simulado fechou: " + telescopio.getExtensionMeters());
            assertTrue(sim.getCollected() >= 10, "coletadas: " + sim.getCollected());
            assertTrue(sim.getScored() >= 10, "acertos: " + sim.getScored());
            assertEquals(sim.getCollected(), tracker.getCollections(), "coletas contadas pela corrente do rolo");
            assertEquals(sim.getLaunched(), tracker.getShots(), "disparos contados pela queda de RPM do top");
        } finally {
            scheduler.cancelAll();
            scheduler.getDefaultButtonLoop().clear();
            Logger.end();
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            SimHooks.resumeTiming();
        }

        var reader = new DataLogReader(output.toString());
        assertTrue(reader.isValid());
        Map<String, String> loggedTypes = new HashMap<>();
        Map<Integer, String> entryNames = new HashMap<>();
        double finalPositionError = Double.NaN;
        double finalForwardDistance = Double.NaN;
        boolean arrivalInterrupted = true;
        for (var record : reader) {
            if (record.isStart()) {
                loggedTypes.put(record.getStartData().name, record.getStartData().type);
                entryNames.put(record.getStartData().entry, record.getStartData().name);
            }
            if (record.isControl()) continue;
            String key = entryNames.get(record.getEntry());
            if ("/RealOutputs/Validation/Drive2m/PositionErrorMeters".equals(key)) finalPositionError = record.getDouble();
            if ("/RealOutputs/Validation/Drive2m/FinalDistanceMeters".equals(key)) finalForwardDistance = record.getDouble();
            if ("/RealOutputs/Validation/Drive2m/Interrupted".equals(key)) arrivalInterrupted = record.getBoolean();
        }
        assertFalse(arrivalInterrupted, "O teste de chegada deve concluir antes do timeout");
        // O comando de calibração atual controla só a distância longitudinal.
        // Não confundir esse contrato com chegada XY: o relatório guarda o erro
        // completo, inclusive quando excede 3 cm por desvio lateral.
        assertEquals(2.0, finalForwardDistance, 0.03, "Distância longitudinal da calibração");
        Set<String> logged = loggedTypes.keySet();
        Set<String> missing = new TreeSet<>();
        for (String layout : LAYOUTS) {
            for (String layoutKey : layoutKeys(Path.of(layout))) {
                // ReplayOutputs só existe num *_replay.wpilog; a chave é a mesma
                // de RealOutputs, então confere a versão gravada pelo robô.
                String key = layoutKey.replaceFirst("^/ReplayOutputs/", "/RealOutputs/");
                // Mechanism2d é gravado como tabela: Mecanismo2d/.type, .../angle...
                boolean present = logged.contains(key) || logged.stream().anyMatch(k -> k.startsWith(key + "/"));
                if (!present) missing.add(layout + " -> " + key);
            }
        }
        assertTrue(missing.isEmpty(), "Chaves dos layouts ausentes no log: " + missing);
        checkElasticLayouts(loggedTypes);
        Files.writeString(directory.resolve("validacao-resumo.json"), new ObjectMapper()
            .writerWithDefaultPrettyPrinter().writeValueAsString(Map.of(
                "origem", "SIMULACAO_AUTOMATIZADA", "coletas", sim.getCollected(),
                "disparos", sim.getLaunched(), "acertosSimulados", sim.getScored(),
                "disparosDetectados", tracker.getShots(), "erroChegadaMetros", finalPositionError,
                "chegadaInterrompida", arrivalInterrupted, "chegadaDentroDe3cmXY", finalPositionError <= 0.03,
                "distanciaLongitudinalMetros", finalForwardDistance, "chavesRegistradas", loggedTypes.size())) + "\n");
    }

    /**
     * Elastic: /AdvantageKit/RealOutputs/X é /RealOutputs/X no log. Os alertas
     * (/SmartDashboard/Alerts) são da WPILib e não passam pelo AdvantageKit; o
     * vídeo (/CameraPublisher) é publicado pela própria Limelight, só no robô real.
     */
    private static void checkElasticLayouts(Map<String, String> loggedTypes) throws Exception {
        Set<String> problems = new TreeSet<>();
        int widgets = 0;
        try (var files = Files.list(ELASTIC_DIR)) {
            for (Path file : files.filter(f -> f.toString().endsWith(".json")).toList()) {
                JsonNode root = new ObjectMapper().readTree(file.toFile());
                for (JsonNode tab : root.get("tabs")) {
                    for (JsonNode container : tab.get("grid_layout").get("containers")) {
                        widgets++;
                        JsonNode props = container.get("properties");
                        String topic = props.get("topic").asText();
                        if (topic.startsWith("/SmartDashboard/") || topic.startsWith("/CameraPublisher/")) continue;
                        String key = topic.replaceFirst("^/AdvantageKit", "");
                        // Widget Field: lê <tópico>/Robot; os outros filhos viram objetos no mapa.
                        if ("Field".equals(container.get("type").asText())) {
                            if (!"double[]".equals(loggedTypes.get(key + "/Robot"))) {
                                problems.add(file.getFileName() + " -> " + topic + "/Robot (mapa sem robô)");
                            }
                            continue;
                        }
                        String loggedType = loggedTypes.get(key);
                        String expected = props.get("data_type").asText();
                        if (loggedType == null) {
                            problems.add(file.getFileName() + " -> " + topic + " (não existe)");
                        } else if (!expected.equals(ELASTIC_TYPES.get(loggedType))) {
                            problems.add(file.getFileName() + " -> " + topic + " (log " + loggedType
                                + ", widget " + expected + ")");
                        }
                    }
                }
            }
        }
        assertTrue(widgets > 0, "Nenhum layout do Elastic em " + ELASTIC_DIR);
        assertTrue(problems.isEmpty(), "Tópicos do Elastic com problema: " + problems);
    }

    /** Chaves das fontes (logKey) e da tabela de entradas de um layout. */
    private static Set<String> layoutKeys(Path layout) throws Exception {
        JsonNode root = new ObjectMapper().readTree(layout.toFile());
        Set<String> keys = new TreeSet<>();
        for (JsonNode tab : root.get("hubs").get(0).get("state").get("tabs").get("tabs")) {
            collect(tab.get("controller"), keys);
        }
        return keys;
    }

    private static void collect(JsonNode node, Set<String> keys) {
        if (node == null) return;
        if (node.isObject()) {
            JsonNode key = node.get("logKey");
            if (key != null && key.isTextual()) keys.add(key.asText());
            node.elements().forEachRemaining(child -> collect(child, keys));
        } else if (node.isArray()) {
            node.elements().forEachRemaining(child -> {
                if (child.isTextual() && child.asText().startsWith("/")) keys.add(child.asText());
                else collect(child, keys);
            });
        }
    }
}
