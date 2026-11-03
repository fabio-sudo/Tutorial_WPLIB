package frc.robot.subsystems.telescopio;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.telescopio.TelescopioAbrirCommand;
import frc.robot.commands.telescopio.TelescopioCicloCommand;
import frc.robot.commands.telescopio.TelescopioFecharCommand;
import org.junit.jupiter.api.*;

class TelescopioControlTest {
    private Telescopio telescopio;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    @BeforeAll static void init() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        RoboRioSim.setVInVoltage(12);
        enabled(false);
        telescopio = new Telescopio(new TelescopioIOSim());
        tick(1);
        enabled(true);
    }
    @AfterEach void cleanup() {
        scheduler.cancelAll();
        telescopio.close();
        enabled(false);
        SimHooks.resumeTiming();
    }
    private void enabled(boolean value) {
        DriverStationSim.setEnabled(value);
        DriverStationSim.notifyNewData();
    }
    private void tick(int cycles) {
        for (int i = 0; i < cycles; i++) {
            SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
            scheduler.run();
            assertTrue(Math.abs(telescopio.getRequestedVolts()) <= MAX_CONTROL_VOLTS);
        }
    }
    @Test void reachesAndHoldsIntermediatePositionAgainstGravity() {
        telescopio.setGoalMeters(0.626);
        tick(200);
        assertTrue(telescopio.atGoal());
        double position = telescopio.getExtensionMeters();
        tick(100);
        assertEquals(position, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        assertTrue(telescopio.getRequestedVolts() > 0);
    }
    @Test void goalsAreBoundedAndRetractionCompletes() {
        telescopio.setGoalMeters(99);
        assertEquals(MAX_EXTENSION_METERS, telescopio.getGoalMeters());
        tick(300);
        assertTrue(telescopio.atGoal());
        telescopio.setGoalMeters(-5);
        tick(300);
        assertTrue(telescopio.atGoal());
        assertEquals(0, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        assertEquals(0, telescopio.getRequestedVolts());
    }
    @Test void disablingDoesNotRetainAnActiveTargetAfterEnable() {
        telescopio.setGoalMeters(0.5);
        tick(100);
        enabled(false);
        tick(1);
        assertFalse(telescopio.isPositionControlActive());
        assertEquals(0, telescopio.getRequestedVolts());
        enabled(true);
        tick(10);
        assertFalse(telescopio.isPositionControlActive());
    }
    @Test void nonFiniteGoalAndBadSensorsStopTheMotor() {
        telescopio.setGoalMeters(0.5);
        tick(20);
        telescopio.setGoalMeters(Double.NaN);
        assertFalse(telescopio.isPositionControlActive());
        telescopio.close();
        var fake = new FakeIO();
        telescopio = new Telescopio(fake);
        tick(1);
        telescopio.setGoalMeters(0.5);
        tick(2);
        assertTrue(fake.volts > 0);
        fake.valid = false;
        tick(1);
        assertFalse(telescopio.isPositionControlActive());
        assertEquals(0, fake.volts);
        fake.valid = true;
        fake.position = Double.NaN;
        tick(1);
        telescopio.setGoalMeters(0.5);
        assertFalse(telescopio.isPositionControlActive());
    }
    @Test void automaticCycleReachesBothEndsAndCancellationStopsControl() {
        var command = new TelescopioCicloCommand(telescopio);
        scheduler.schedule(command);
        double max = 0;
        boolean returned = false;
        for (int i = 0; i < 800; i++) {
            tick(1);
            max = Math.max(max, telescopio.getExtensionMeters());
            if (max > 1.24 && telescopio.getExtensionMeters() < 0.01) returned = true;
        }
        assertTrue(max > 1.24);
        assertTrue(returned);
        command.cancel();
        assertFalse(telescopio.isPositionControlActive());
        assertEquals(0, telescopio.getRequestedVolts());
    }
    @Test void holdingOpensReleasingHoldsAndHoldingCloseRetracts() {
        var open = new TelescopioAbrirCommand(telescopio);
        scheduler.schedule(open);
        tick(100);
        double released = telescopio.getExtensionMeters();
        assertTrue(released > 0.4);
        open.cancel(); // soltou o RB em movimento
        tick(100);
        assertTrue(telescopio.atGoal());
        double held = telescopio.getExtensionMeters();
        assertTrue(held >= released && held < released + 0.2, "freia sem voltar: " + held);
        tick(100);
        assertEquals(held, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        scheduler.schedule(open);
        tick(300);
        assertEquals(MAX_EXTENSION_METERS, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        open.cancel();
        var close = new TelescopioFecharCommand(telescopio);
        scheduler.schedule(close);
        tick(400);
        assertEquals(0, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        close.cancel();
        tick(10);
        assertEquals(0, telescopio.getRequestedVolts());
        assertFalse(telescopio.isJammed()); // Uso normal não aciona o travamento.
    }
    @Test void jamStopsPushingHoldsWeightAndClearsOnNextPress() {
        telescopio.close();
        var fake = new FakeIO();
        telescopio = new Telescopio(fake);
        tick(1);
        var open = new TelescopioAbrirCommand(telescopio);
        scheduler.schedule(open);
        tick(5);
        fake.current = STATOR_CURRENT_LIMIT_AMPS; // empurrando sem sair do lugar
        tick(5);
        assertFalse(telescopio.isJammed(), "0,1 s ainda não é travamento");
        tick((int) (JAM_TIME_SECONDS / LOOP_PERIOD_SECONDS));
        assertTrue(telescopio.isJammed());
        assertEquals(fake.position, telescopio.getGoalMeters(), 1e-9);
        assertEquals(SIM_KG, fake.volts, 1e-6); // só sustenta o peso
        fake.current = 12;
        tick(10);
        assertTrue(telescopio.isJammed(), "continua travado com o botão apertado");
        open.cancel();
        tick(1);
        assertEquals(fake.position, telescopio.getGoalMeters(), 1e-9);
        scheduler.schedule(new TelescopioAbrirCommand(telescopio)); // apertou de novo
        tick(1);
        assertFalse(telescopio.isJammed());
        assertEquals(MAX_EXTENSION_METERS, telescopio.getGoalMeters());
    }
    @Test void endStopsOnlyAllowGravityCompensationAtTheTop() {
        double top = MAX_EXTENSION_METERS;
        assertEquals(SIM_KG, Telescopio.limitAtEnds(3.0, top, top, 0.2)); // não empurra
        assertEquals(0.2, Telescopio.limitAtEnds(0.2, top, top, 0)); // abaixo do kG passa
        assertEquals(-1.0, Telescopio.limitAtEnds(-1.0, top, 1.0, -0.2)); // pode descer
        assertEquals(0, Telescopio.limitAtEnds(-2.0, 0, 0.5, 0)); // embaixo não empurra
        assertEquals(0, Telescopio.limitAtEnds(0.4, 0.001, 0, 0)); // recolhido apoia
        assertEquals(2.0, Telescopio.limitAtEnds(2.0, 0.6, 1.0, 0.3)); // meio do curso livre
    }
    @Test void disablingWhileHoldingCutsVoltage() {
        scheduler.schedule(new TelescopioAbrirCommand(telescopio));
        tick(50);
        enabled(false);
        tick(1);
        assertNull(telescopio.getCurrentCommand());
        assertFalse(telescopio.isPositionControlActive());
        assertEquals(0, telescopio.getRequestedVolts());
    }
    @Test void hardwarePlaceholderDoesNotPretendToHaveReferencedSensors() {
        telescopio.close();
        telescopio = new Telescopio(new TelescopioIO() {});
        tick(1);
        telescopio.setGoalMeters(0.5);
        assertFalse(telescopio.hasValidPosition());
        assertFalse(telescopio.isPositionControlActive());
    }
    private static class FakeIO implements TelescopioIO {
        boolean valid = true;
        double position = 0.1;
        double current;
        double volts;
        @Override public void updateInputs(TelescopioIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.positionValid = valid;
            inputs.extensionMeters = position;
            inputs.motorCurrentAmps = current;
        }
        @Override public void setVoltage(double value) { volts = value; }
    }
}
