package frc.robot.subsystems.telescopio_9458;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Processo isolado: verifica painel, adaptador real com CAN fictício, publicação NT e arquivo. */
class Telescopio9458LogIntegrationTest {
    @Test void publishesTwoDifferent3dModelsAndConsumesDashboardRequests() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        Path directory = Path.of(System.getProperty("telescopio9458.validationDir"));
        Files.createDirectories(directory);
        Path output = directory.resolve("FisicoVirtual9458-Teste.wpilog");
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        RoboRioSim.setVInVoltage(12);
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.addDataReceiver(new WPILOGWriter(output.toString(), WPILOGWriter.AdvantageScopeOpenBehavior.NEVER));
        Logger.addDataReceiver(new NT4Publisher());
        var nt = NetworkTableInstance.getDefault();
        String root = "/AdvantageKit/RealOutputs/Telescopio9458/";
        try (var physical = nt.getStructArrayTopic(root + "Componentes", Pose3d.struct).subscribe(new Pose3d[0]);
             var virtual = nt.getStructArrayTopic(root + "Modelo/Componentes", Pose3d.struct).subscribe(new Pose3d[0]);
             var virtualBase = nt.getStructTopic(root + "Modelo/Pose", Pose3d.struct).subscribe(new Pose3d());
             var motorTurns = nt.getDoubleTopic(root + "Voltas/Motor").subscribe(Double.NaN);
             var outputTurns = nt.getDoubleTopic(root + "Voltas/SaidaRedutor").subscribe(Double.NaN);
             var virtualOutputTurns = nt.getDoubleTopic(root + "Modelo/VoltasSaidaRedutor").subscribe(Double.NaN);
             var physicalCm = nt.getDoubleTopic(root + "Comparacao/FisicoCentimetros").subscribe(Double.NaN);
             var physicalM = nt.getDoubleTopic(root + "Comparacao/FisicoMetros").subscribe(Double.NaN);
             var modelCm = nt.getDoubleTopic(root + "Comparacao/SimuladoCentimetros").subscribe(Double.NaN);
             var modelM = nt.getDoubleTopic(root + "Comparacao/SimuladoMetros").subscribe(Double.NaN);
             var differenceCm = nt.getDoubleTopic(root + "Comparacao/DiferencaCentimetros").subscribe(Double.NaN);
             var marginCm = nt.getDoubleTopic(root + "Limites/MargemCentimetros").subscribe(Double.NaN);
             var operatingCm = nt.getDoubleTopic(root + "Limites/OperacionalCentimetros").subscribe(Double.NaN);
             var maxSpeed = nt.getDoubleTopic(root + "Controle/VelocidadeMaximaMetrosPorSeg").subscribe(Double.NaN)) {
            Logger.start();
            var device = new StationaryHardware();
            var telescope = new Telescopio_9458(new TelescopioIOTalonFX_9458(device));
            new TelescopioDashboard_9458(telescope);
            var scheduler = CommandScheduler.getInstance();
            String tuning = "/Tuning/Telescopio9458/";
            try {
                for (int cycle = 0; cycle < 100; cycle++) {
                    if (cycle == 5) {
                        nt.getEntry(tuning + "PID/kP").setDouble(25);
                        nt.getEntry(tuning + "PID/AplicarGanhos").setBoolean(true);
                    }
                    if (cycle == 10 || cycle == 65) {
                        DriverStationSim.setEnabled(true);
                        DriverStationSim.notifyNewData();
                        nt.getEntry(tuning + "ControleAtivo").setBoolean(true);
                        nt.getEntry(tuning + "MetaMetros").setDouble(0.7);
                    }
                    if (cycle == 35) {
                        nt.getEntry(tuning + "PID/kP").setDouble(1000);
                        nt.getEntry(tuning + "PID/AplicarGanhos").setBoolean(true);
                        nt.getEntry(tuning + "ConfirmarRecolhidoEZerar").setBoolean(true);
                    }
                    if (cycle == 60) {
                        DriverStationSim.setEnabled(false);
                        DriverStationSim.notifyNewData();
                    }
                    if (cycle >= 70) device.position = 0.4 * MOTOR_ROTATIONS_PER_METER;
                    SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
                    Logger.AdvancedHooks.invokePeriodicBeforeUser();
                    scheduler.run();
                    Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
                }
                assertEquals(25, telescope.getGains().kP(), "Pedido habilitado não pode ficar pendente");
                assertEquals(1, device.zeroCalls, "Pedido de zero habilitado deve ser recusado e consumido");
                assertEquals(0.4, telescope.getExtensionMeters(), 1e-9);
                assertTrue(telescope.getModelExtensionMeters() < 0.1,
                        "Com tensão medida zero, a previsão não acompanha a posição fictícia do encoder");
            } finally {
                scheduler.cancelAll();
                telescope.close();
                scheduler.getDefaultButtonLoop().clear();
                Logger.end();
                DriverStationSim.setEnabled(false);
                DriverStationSim.notifyNewData();
                SimHooks.resumeTiming();
            }
            assertEquals(2, physical.get().length);
            assertEquals(2, virtual.get().length);
            assertEquals(0.4, physical.get()[1].getZ(), 1e-9);
            assertTrue(virtual.get()[1].getZ() < 0.1);
            assertEquals(MODEL_VIEW_OFFSET_Y_METERS, virtualBase.get().getY(), 1e-9);
            // O encoder da fronteira CAN informa a extensão equivalente a 0,4 m.
            // Conferir os números publicados, não só a presença das chaves.
            assertEquals(31.496062992125985, motorTurns.get(), 1e-9);
            assertEquals(1.968503937007874, outputTurns.get(), 1e-9);
            assertEquals(virtual.get()[1].getZ() / 0.2032, virtualOutputTurns.get(), 1e-9);
            assertEquals(40, physicalCm.get(), 1e-9);
            assertEquals(0.4, physicalM.get(), 1e-9);
            assertEquals(virtual.get()[1].getZ(), modelM.get(), 1e-9);
            assertEquals(modelM.get() * 100, modelCm.get(), 1e-9);
            assertEquals(physicalCm.get() - modelCm.get(), differenceCm.get(), 1e-9);
            assertEquals(5, marginCm.get(), 1e-9);
            assertEquals(94.568, operatingCm.get(), 1e-9);
            assertEquals(MOTION_MAGIC_VELOCITY_METERS_PER_SEC, maxSpeed.get(), 1e-9);
        }
        var reader = new DataLogReader(output.toString());
        assertTrue(reader.isValid());
        var fields = new HashSet<String>();
        for (var record : reader) if (record.isStart()) fields.add(record.getStartData().name);
        for (String field : new String[] {
                "/Telescopio9458/Inputs/RotorPositionRotations", "/Telescopio9458/Ganhos/KP",
                "/RealOutputs/Telescopio9458/Componentes", "/RealOutputs/Telescopio9458/Modelo/Componentes",
                "/RealOutputs/Telescopio9458/Voltas/SaidaRedutor",
                "/RealOutputs/Telescopio9458/Modelo/VoltasSaidaRedutor",
                "/RealOutputs/Telescopio9458/Modelo/TensaoAplicadaVolts", "/RealOutputs/Telescopio9458/Diagnostico"}) {
            assertTrue(fields.contains(field), "Campo necessário ao layout/replay: " + field);
        }
    }

    private static class StationaryHardware implements TelescopioIOTalonFX_9458.Hardware {
        double position;
        int zeroCalls;
        @Override public StatusCode configure(TelescopioGains_9458 gains, TelescopioMotionProfile_9458.Limits profile) {
            return StatusCode.OK;
        }
        @Override public StatusCode configureGains(TelescopioGains_9458 gains) { return StatusCode.OK; }
        @Override public StatusCode configureMotionMagic(double velocity, double acceleration, double jerk, double timeout) {
            return timeout > 0 ? StatusCode.OK : StatusCode.TimeoutCannotBeZero;
        }
        @Override public StatusCode setMotionMagic(double rotorRotations, boolean limitForward, boolean limitReverse) {
            return StatusCode.OK;
        }
        @Override public StatusCode zeroPosition() { zeroCalls++; position = 0; return StatusCode.OK; }
        @Override public boolean hasResetOccurred() { return false; }
        @Override public StatusCode read(TelescopioIO_9458.TelescopioIOInputs_9458 inputs) {
            inputs.rotorPositionRotations = position;
            inputs.appliedVolts = 0;
            return StatusCode.OK;
        }
        @Override public StatusCode setVoltage(double volts) { return StatusCode.OK; }
        @Override public void close() {}
    }
}
