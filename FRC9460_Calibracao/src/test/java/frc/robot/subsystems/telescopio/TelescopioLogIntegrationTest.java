package frc.robot.subsystems.telescopio;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Gera um log da física e dos comandos reais, em processo isolado. */
class TelescopioLogIntegrationTest {
    @Test void recordsPhysicsAndDashboardCommandsForAdvantageScope() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        Path directory = Path.of(System.getProperty("telescopio.validationDir"));
        Files.createDirectories(directory);
        Path output = directory.resolve("Telescopio9460-Simulacao.wpilog");
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        RoboRioSim.setVInVoltage(12);
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.recordMetadata("ProjectName", "FRC9460 - validacao telescopio");
        Logger.recordMetadata("Physics", "Massa concentrada; relacao cabo/ponta provisoria 1:1");
        Logger.addDataReceiver(new WPILOGWriter(output.toString(),
            WPILOGWriter.AdvantageScopeOpenBehavior.NEVER));
        Logger.start();
        var scheduler = CommandScheduler.getInstance();
        var telescope = new Telescopio(Telescopio.createIO());
        new TelescopioDashboard(telescope);
        var nt = NetworkTableInstance.getDefault();
        double maximum = 0;
        boolean returned = false;
        try {
            for (int cycle = 0; cycle < 1000; cycle++) {
                if (cycle == 10) {
                    DriverStationSim.setEnabled(true);
                    DriverStationSim.notifyNewData();
                    nt.getEntry("/Tuning/Telescopio/ControleAtivo").setBoolean(true);
                    nt.getEntry("/Tuning/Telescopio/CicloAutomatico").setBoolean(true);
                }
                if (cycle == 850) {
                    nt.getEntry("/Tuning/Telescopio/MetaMetros").setDouble(0.31);
                    nt.getEntry("/Tuning/Telescopio/CicloAutomatico").setBoolean(false);
                }
                if (cycle == 900) nt.getEntry("/Tuning/Telescopio/ControleAtivo").setBoolean(false);
                if (cycle == 925) {
                    DriverStationSim.setEnabled(false);
                    DriverStationSim.notifyNewData();
                }
                SimHooks.stepTiming(TelescopioConstants.LOOP_PERIOD_SECONDS);
                Logger.AdvancedHooks.invokePeriodicBeforeUser();
                scheduler.run();
                maximum = Math.max(maximum, telescope.getExtensionMeters());
                if (maximum > 1.24 && telescope.getExtensionMeters() < 0.01) returned = true;
                Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
            }
            assertTrue(maximum > 1.24, "O painel deve iniciar o ciclo e chegar ao topo");
            assertTrue(returned, "O ciclo deve retornar ao limite inferior");
            assertFalse(telescope.isPositionControlActive());
        } finally {
            scheduler.cancelAll();
            telescope.close();
            scheduler.getDefaultButtonLoop().clear();
            Logger.end();
            SimHooks.resumeTiming();
        }
        var reader = new DataLogReader(output.toString());
        assertTrue(reader.isValid());
        var names = new HashSet<String>();
        for (var record : reader) if (record.isStart()) names.add(record.getStartData().name);
        assertTrue(names.contains("/Telescopio/Inputs/ExtensionMeters"));
        assertTrue(names.contains("/RealOutputs/Telescopio/Componentes"));
        assertTrue(names.contains("/RealOutputs/Telescopio/Mecanismo2d/.type"));
        assertTrue(names.contains("/NetworkInputs/Tuning/Telescopio/ControleAtivo"));
    }
}
