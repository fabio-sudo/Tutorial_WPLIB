package frc.robot.subsystems.centrifuga;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

class CentrifugaTest {
    private FakeIO io;
    private Centrifuga centrifuge;
    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        io = new FakeIO();
        centrifuge = new Centrifuga(io);
        centrifuge.periodic();
    }
    @AfterEach void cleanup() {
        CommandScheduler.getInstance().cancelAll(); centrifuge.close();
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }
    @Test void invalidRequestsAndLostSensorsCannotKeepMotorRunning() {
        centrifuge.setRpm(2300); assertEquals(2300, io.command);
        io.connected = false; centrifuge.periodic();
        assertEquals(0, io.command); assertEquals(0, centrifuge.getTargetRpm());
        centrifuge.setRpm(2300); assertEquals(0, io.command);
        io.connected = true; io.rpm = Double.NaN; centrifuge.periodic();
        assertFalse(centrifuge.isAtSpeed());
        io.rpm = 0; centrifuge.periodic(); centrifuge.setRpm(Double.NaN);
        assertEquals(0, io.command);
    }
    @Test void capsTheRequestedSpeedAndBrakesOnRelease() {
        centrifuge.setRpm(100000); assertEquals(CentrifugaConstants.MAX_COMMAND_RPM, io.command);
        centrifuge.stop(); assertEquals(0, io.command);
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
        centrifuge.setRpm(2300); assertEquals(0, io.command);
    }
    @Test void oneFailedCanRequestDoesNotLockTheCentrifugeUntilRestart() {
        // Antes: uma falha de setControl deixava a centrífuga parada até reiniciar.
        io.canAccepts = false;
        centrifuge.setRpm(2300); assertEquals(0, io.command, "pedido recusado corta a saída");
        centrifuge.periodic(); assertEquals(0, centrifuge.getTargetRpm());
        centrifuge.setRpm(2300); assertEquals(0, io.command, "recusa enquanto a falha aparece");
        centrifuge.periodic(); centrifuge.setRpm(2300);
        assertEquals(0, io.command, "CAN ainda recusando: continua parada");
        io.canAccepts = true;
        centrifuge.periodic(); // o stop() deste ciclo é aceito...
        centrifuge.setRpm(2300); assertEquals(0, io.command);
        centrifuge.periodic(); // ...e o próximo ciclo libera o RPM
        centrifuge.setRpm(2300); assertEquals(2300, io.command);
    }
    // Start do piloto: o mesmo teste que girou o motor 36 no Phoenix Tuner X.
    @Test void startTestSpinsForwardEvenWithInvalidSensors() {
        io.connected = false; // com trava, a centrífuga recusaria o pedido
        io.canAccepts = false;
        centrifuge.periodic();
        var scheduler = CommandScheduler.getInstance();
        var test = centrifuge.openLoopTestCommand();
        scheduler.schedule(test);
        for (int i = 0; i < 5; i++) {
            scheduler.run();
            assertEquals(CentrifugaConstants.OPEN_LOOP_TEST_PERCENT, io.percent, 1e-12,
                "sem trava: segue girando (ciclo " + i + ")");
        }
        assertTrue(centrifuge.isOpenLoopTestActive());
        scheduler.cancel(test);
        assertEquals(0, io.percent, "soltou o Start: para");
        assertFalse(centrifuge.isOpenLoopTestActive());
    }
    @Test void startTestStopsWhenTheRobotIsDisabled() {
        var scheduler = CommandScheduler.getInstance();
        scheduler.schedule(centrifuge.openLoopTestCommand());
        scheduler.run();
        assertEquals(CentrifugaConstants.OPEN_LOOP_TEST_PERCENT, io.percent, 1e-12);
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
        scheduler.run();
        assertEquals(0, io.percent, "desabilitado: o motor para");
    }
    @Test void startTestInSimulationOnlyPushesForward() {
        var sim = new CentrifugaIOSim();
        var inputs = new CentrifugaIO.CentrifugaIOInputs();
        sim.setPercentOutput(-0.5); // pedido para trás: vira 0
        sim.updateInputs(inputs);
        assertEquals(0.0, inputs.appliedVolts, 1e-9);
        sim.setPercentOutput(CentrifugaConstants.OPEN_LOOP_TEST_PERCENT);
        for (int i = 0; i < 50; i++) sim.updateInputs(inputs);
        assertTrue(inputs.velocityRpm > 500, "gira para a frente: " + inputs.velocityRpm);
        sim.stop(); sim.updateInputs(inputs);
        assertEquals(0.0, inputs.appliedVolts, 1e-9);
    }
    @Test void simulatedStartUsesAccelerationProfileInsteadOfAStep() {
        var sim = new CentrifugaIOSim();
        var inputs = new CentrifugaIO.CentrifugaIOInputs();
        sim.setVelocityRpm(2300); sim.updateInputs(inputs);
        assertEquals(72, inputs.referenceRpm, 1e-6);
        for (int i=0;i<100;i++) sim.updateInputs(inputs);
        assertEquals(2300, inputs.referenceRpm, 1e-6);
        assertTrue(inputs.velocityRpm > 2000);
        sim.setVelocityRpm(0); sim.updateInputs(inputs); assertEquals(0, inputs.referenceRpm);
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
        sim.updateInputs(inputs); assertEquals(0, inputs.appliedVolts);
    }
    /** Segue o contrato do CentrifugaIOTalonFX: todo pedido, inclusive o stop, guarda o resultado. */
    private static class FakeIO implements CentrifugaIO {
        boolean connected = true, canAccepts = true, lastRequestOk = true;
        double command, rpm;
        @Override public void updateInputs(CentrifugaIOInputs in) {
            in.connected = connected; in.velocityRpm = rpm; in.controlRequestOK = lastRequestOk;
        }
        double percent;
        @Override public void setVelocityRpm(double rpm) { lastRequestOk = canAccepts; command = canAccepts ? rpm : 0; percent = 0; }
        @Override public void setPercentOutput(double value) { percent = value; command = 0; }
        @Override public void stop() { lastRequestOk = canAccepts; command = 0; percent = 0; }
    }
}
