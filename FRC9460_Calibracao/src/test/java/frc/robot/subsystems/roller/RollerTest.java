package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkParameters;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Rolete do intake: malha aberta, para ao soltar e ao desabilitar. */
class RollerTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private FakeIO io;
    private Roller roller;

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        setEnabled(true);
        io = new FakeIO();
        roller = new Roller(io);
    }

    @AfterEach void cleanup() {
        scheduler.cancelAll();
        roller.close();
        setEnabled(false);
        SimHooks.resumeTiming();
    }

    private void setEnabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    private void tick() {
        SimHooks.stepTiming(0.02);
        scheduler.run();
    }

    @Test void holdingPullsAtTheOpenLoopPercentAndReleasingStops() {
        var pull = roller.pullCommand();
        scheduler.schedule(pull);
        tick();
        assertEquals(PULL_PERCENT, io.percent, 1e-12);
        assertTrue(roller.isPulling());
        scheduler.cancel(pull);
        assertEquals(0.0, io.percent);
        assertFalse(roller.isPulling());
    }

    @Test void disablingStopsEvenWithoutReleasingTheButton() {
        roller.pull();
        setEnabled(false);
        tick();
        assertEquals(0.0, io.percent);
        assertFalse(roller.isPulling());
    }

    @Test void sparkConfigurationHasTheCurrentLimitAndDirection() {
        var config = new SparkFlexConfig() {
            { apply(RollerIOSparkFlex.createConfig()); }
            Object read(SparkParameters parameter) { return getParameter(parameter.value); }
        };
        assertEquals(CURRENT_LIMIT_AMPS, ((Number) config.read(SparkParameters.kSmartCurrentStallLimit)).intValue());
        assertEquals(MOTOR_INVERTED, config.read(SparkParameters.kInverted));
    }

    private static class FakeIO implements RollerIO {
        double percent;
        @Override public void updateInputs(RollerIOInputs inputs) { inputs.connected = true; }
        @Override public void setPercentOutput(double value) { percent = value; }
        @Override public void stop() { percent = 0.0; }
    }
}
