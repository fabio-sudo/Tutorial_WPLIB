package frc.robot.subsystems.centrifuga;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.centrifuga.CentrifugaConstants.*;
import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.shooter.CentrifugaDestravarCommand;

class CentrifugaDestravarTest {
    private Centrifuga centrifuga;
    private FakeIO io;
    private boolean shotStarted;
    private Command sequence;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        io = new FakeIO();
        centrifuga = new Centrifuga(io);
        tick(1);
    }
    @AfterEach void cleanup() {
        scheduler.cancelAll();
        centrifuga.close();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }
    private void tick(int count) {
        for (int i = 0; i < count; i++) { SimHooks.stepTiming(0.02); scheduler.run(); }
    }
    private void start() {
        sequence = new CentrifugaDestravarCommand(centrifuga)
            .andThen(Commands.runOnce(() -> shotStarted = true));
        scheduler.schedule(sequence);
    }

    @Test void quarterOutputTurnMeansOneAndAQuarterRotorTurnsAndStopsBeforeShot() {
        start(); tick(12);
        assertEquals(-UNJAM_ROTOR_RPM, io.command);
        io.position = -1.24; tick(1);
        assertFalse(shotStarted);
        io.position = -1.25; io.rpm = -100; tick(1);
        assertEquals(0, io.command);
        tick(8); assertFalse(shotStarted, "não passar ao tiro enquanto gira para trás");
        io.rpm = 0; tick(10);
        assertTrue(shotStarted);
    }
    // Antes (até 29/10/2026) estas falhas bloqueavam o tiro até soltar o B.
    // No robô, a reversão fraca falhava sempre e a centrífuga nunca empurrava a bola.
    @Test void stallTimeoutGivesUpOnceAndGoesToTheShot() {
        start(); tick(12);
        assertEquals(-UNJAM_ROTOR_RPM, io.command);
        tick(100); // bolas segurando: a reversão não chega a 1/4 de volta
        assertTrue(shotStarted, "sem conseguir destravar, segue para o tiro normal");
        assertEquals(0, io.command, "não fica revertendo");
        assertFalse(sequence.isScheduled(), "uma tentativa só, sem repetir sozinho");
    }
    @Test void sustainedCurrentSkipsTheUnjamWithoutPushingBackwards() {
        start(); tick(12); io.current = UNJAM_CURRENT_AMPS + 1; tick(10);
        assertTrue(shotStarted, "corrente alta: para de reverter e segue para o tiro");
        assertEquals(0, io.command);
    }
    @Test void sensorGlitchDuringTheUnjamDoesNotBlockTheShot() {
        // Um ciclo atrasado no CAN (laço lento) invalidava o sensor e travava o B.
        start(); tick(12);
        io.valid = false; tick(1);
        assertEquals(0, io.command, "sensor inválido: para de reverter na hora");
        io.valid = true; tick(1);
        assertTrue(shotStarted, "o tiro tem as próprias travas de sensor a cada ciclo");
    }
    @Test void releaseAndDisableStopReverseImmediately() {
        start(); tick(12); assertTrue(io.command < 0);
        sequence.cancel(); assertEquals(0, io.command);
        start(); tick(12); assertTrue(io.command < 0);
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData(); tick(1);
        assertEquals(0, io.command); assertFalse(shotStarted);
    }
    @Test void wrongDirectionNeverUnlocksTheShot() {
        start(); tick(12); io.position = 0.2; tick(1);
        assertEquals(0, io.command); assertFalse(shotStarted);
        tick(100); assertFalse(shotStarted);
    }
    @Test void physicalSimulationCompletesReverseAndBrake() {
        centrifuga.close();
        centrifuga = new Centrifuga(new CentrifugaIOSim());
        tick(2); start(); tick(100);
        assertTrue(shotStarted);
        assertTrue(centrifuga.getOutputRotations() <= -UNJAM_OUTPUT_ROTATIONS);
        assertTrue(centrifuga.getOutputRotations() > -0.40, "deslocamento e frenagem limitados na simulação");
    }
    private static class FakeIO implements CentrifugaIO {
        double position, rpm, command, current;
        boolean valid = true;
        @Override public void updateInputs(CentrifugaIOInputs in) {
            in.connected = valid; in.positionValid = valid;
            in.rotorPositionRotations = position; in.velocityRpm = rpm; in.currentAmps = current;
        }
        @Override public void setVelocityRpm(double rpm) { command = rpm; }
        @Override public void stop() { command = 0; }
    }
}
