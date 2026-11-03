package frc.robot.subsystems.telescopio;

import static frc.robot.subsystems.telescopio.TelescopioConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.*;

/**
 * Limites do telescópio 9460 na simulação física. Prova a lógica de limitar;
 * não prova que o motor real para no lugar certo (teste de hardware).
 */
class TelescopioSafetyTest {
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
            assertTrue(Math.abs(telescopio.getRequestedVolts()) <= MAX_CONTROL_VOLTS + 1e-9,
                "tensão acima do máximo: " + telescopio.getRequestedVolts());
        }
    }

    @Test void goalSentWhileDisabledIsRefusedAndNothingMovesOnEnable() {
        telescopio.setGoalMeters(0.6);
        assertFalse(telescopio.isPositionControlActive());
        enabled(true);
        tick(50);
        assertEquals(0.0, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        assertEquals(0.0, telescopio.getRequestedVolts());
    }

    @Test void goalsAtAndJustBeyondTheEndsAreClampedExactly() {
        enabled(true);
        tick(1);
        telescopio.setGoalMeters(MAX_EXTENSION_METERS);
        assertEquals(MAX_EXTENSION_METERS, telescopio.getGoalMeters());
        telescopio.setGoalMeters(MAX_EXTENSION_METERS + 1e-6);
        assertEquals(MAX_EXTENSION_METERS, telescopio.getGoalMeters());
        telescopio.setGoalMeters(MIN_EXTENSION_METERS - 1e-6);
        assertEquals(MIN_EXTENSION_METERS, telescopio.getGoalMeters());
    }

    @Test void rapidReversalsNeverExceedTheVoltageLimitAndStillSettle() {
        enabled(true);
        tick(1);
        for (int i = 0; i < 20; i++) {
            telescopio.setGoalMeters(i % 2 == 0 ? MAX_EXTENSION_METERS : MIN_EXTENSION_METERS);
            tick(5); // tick confere a tensão a cada ciclo
        }
        telescopio.setGoalMeters(0.5);
        tick(300);
        assertTrue(telescopio.atGoal(), "depois das reversões, chega na meta");
        assertTrue(telescopio.getExtensionMeters() >= MIN_EXTENSION_METERS - POSITION_TOLERANCE_METERS
            && telescopio.getExtensionMeters() <= MAX_EXTENSION_METERS + POSITION_TOLERANCE_METERS);
    }
}
