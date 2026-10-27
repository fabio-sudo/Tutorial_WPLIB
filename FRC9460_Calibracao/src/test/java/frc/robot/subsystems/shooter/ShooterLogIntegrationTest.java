package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import java.util.TreeSet;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Processo isolado (o Logger é global): grava um WPILOG com o shooter
 * atirando pelo painel do AdvantageScope e confere que TODA chave do shooter
 * usada pelo layout FRC9460-Shooter.json existe no arquivo.
 */
class ShooterLogIntegrationTest {

    private static final Path LAYOUT = Path.of("config/advantagescope/layouts/FRC9460-Shooter.json");

    @Test
    void layoutKeysExistInALogOfARealShot() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        Path directory = Path.of(System.getProperty("shooter.validationDir"));
        Files.createDirectories(directory);
        Path output = directory.resolve("Shooter-Teste.wpilog");

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

        // Hub a 45° do zero do pivô, a 3 m do shooter.
        double d = 3.0 / Math.sqrt(2.0);
        Translation2d shooterAt = BLUE_HUB_CENTER.plus(new Translation2d(d, d));
        Pose2d pose = new Pose2d(shooterAt.minus(
            new Translation2d(SHOOTER_OFFSET_FORWARD_METERS, SHOOTER_OFFSET_LEFT_METERS)), Rotation2d.kZero);

        var shooter = new Shooter(new ShooterPivotIOSim(0.0), Shooter.createTopIO(), Shooter.createTowerIO(),
            Shooter.createCentrifugeIO(), () -> pose, ChassisSpeeds::new, () -> true);
        new ShooterDashboard(shooter);
        var scheduler = CommandScheduler.getInstance();
        var shoot = NetworkTableInstance.getDefault().getEntry("/Tuning/Shooter/MirarEAtirar");
        boolean fed = false;
        try {
            for (int cycle = 0; cycle < 200; cycle++) {
                if (cycle == 5) {
                    DriverStationSim.setEnabled(true);
                    DriverStationSim.notifyNewData();
                    shoot.setBoolean(true);
                }
                if (cycle == 180) {
                    shoot.setBoolean(false);
                }
                SimHooks.stepTiming(0.02);
                Logger.AdvancedHooks.invokePeriodicBeforeUser();
                scheduler.run();
                Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
                fed |= shooter.getCentrifugeTargetRpm() > 0.0;
            }
            assertTrue(fed, "O botão do painel deve disparar a sequência");
            assertEquals(45.0, shooter.getPivotDegrees(), 1.0);
            assertEquals(0.0, shooter.getCentrifugeTargetRpm(), "Soltar o botão para tudo");
        } finally {
            scheduler.cancelAll();
            shooter.close();
            scheduler.getDefaultButtonLoop().clear();
            Logger.end();
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            SimHooks.resumeTiming();
        }

        var reader = new DataLogReader(output.toString());
        assertTrue(reader.isValid());
        Set<String> logged = new HashSet<>();
        for (var record : reader) {
            if (record.isStart()) logged.add(record.getStartData().name);
        }

        // Chaves que pertencem ao shooter (inclusive as da simulação, pois este
        // teste usa os IOs simulados). Robot/Pose e Vision vêm do Drive e da
        // visão, que este teste não executa.
        Set<String> missing = new TreeSet<>();
        for (String key : layoutKeys()) {
            boolean ownedByShooter = key.startsWith("/RealOutputs/Shooter/") || key.startsWith("/Shooter/")
                || key.equals("/RealOutputs/Mecanismos/Componentes");
            // Mechanism2d é gravado como uma tabela: Mecanismo2d/.type, .../angle...
            boolean present = logged.contains(key) || logged.stream().anyMatch(k -> k.startsWith(key + "/"));
            if (ownedByShooter && !present) {
                missing.add(key);
            }
        }
        assertTrue(missing.isEmpty(), "Chaves do layout ausentes no log: " + missing);
    }

    /** Todas as chaves citadas pelo layout (fontes de gráficos, campos e tabela). */
    private static Set<String> layoutKeys() throws Exception {
        JsonNode root = new ObjectMapper().readTree(LAYOUT.toFile());
        Set<String> keys = new TreeSet<>();
        collect(root, keys);
        assertFalse(keys.isEmpty(), "Layout sem chaves: " + LAYOUT.toAbsolutePath());
        return keys;
    }

    private static void collect(JsonNode node, Set<String> keys) {
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
