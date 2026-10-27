package frc.robot.subsystems.telescopio;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class TelescopioSimulationTest {
    private TelescopioIOSim io;
    private TelescopioIO.TelescopioIOInputs inputs;

    @BeforeAll static void init() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        RoboRioSim.setVInVoltage(12);
        enabled(false);
        io = new TelescopioIOSim();
        inputs = new TelescopioIO.TelescopioIOInputs();
    }
    @AfterEach void cleanup() { enabled(false); RoboRioSim.setVInVoltage(12); }
    private void enabled(boolean value) {
        DriverStationSim.setEnabled(value);
        DriverStationSim.notifyNewData();
    }
    private void tick(int cycles) {
        for (int i = 0; i < cycles; i++) {
            io.updateInputs(inputs);
            assertTrue(Double.isFinite(inputs.motorCurrentAmps));
            assertTrue(inputs.extensionMeters >= MIN_EXTENSION_METERS);
            assertTrue(inputs.extensionMeters <= MAX_EXTENSION_METERS);
        }
    }
    @Test void disabledStartsRetractedAndClearsStaleVoltage() {
        io.setVoltage(12);
        tick(10);
        assertEquals(0, inputs.extensionMeters);
        assertEquals(0, inputs.appliedVolts);
        enabled(true);
        tick(1);
        assertEquals(0, inputs.appliedVolts);
        assertTrue(inputs.positionValid);
    }
    @Test void motorRespondsToVoltageAndBothStopsBoundTheMotion() {
        enabled(true);
        io.setVoltage(3);
        tick(30);
        assertTrue(inputs.extensionMeters > 0.01);
        io.setVoltage(12);
        tick(400);
        assertEquals(MAX_EXTENSION_METERS, inputs.extensionMeters, 1e-9);
        // Parado no batente com 12 V: o limite do estator segura a corrente em 40 A.
        assertEquals(STATOR_CURRENT_LIMIT_AMPS, inputs.motorCurrentAmps, 1e-6);
        assertTrue(inputs.statorCurrentLimitActive);
        assertTrue(inputs.appliedVolts < 12);
        io.setVoltage(-12);
        tick(400);
        assertEquals(MIN_EXTENSION_METERS, inputs.extensionMeters, 1e-9);
    }
    @Test void gravityContinuesAfterDisableAndVoltageRespectsBattery() {
        enabled(true);
        RoboRioSim.setVInVoltage(6);
        io.setVoltage(20);
        tick(20);
        assertEquals(6, inputs.appliedVolts);
        enabled(false);
        tick(15); // Deixa terminar a inércia ascendente.
        double before = inputs.extensionMeters;
        tick(80);
        assertEquals(0, inputs.appliedVolts);
        assertTrue(inputs.extensionMeters < before);
    }
    @Test void invalidVoltageAndStopRemainFinite() {
        enabled(true);
        io.setVoltage(Double.NaN);
        tick(1);
        assertEquals(0, inputs.appliedVolts);
        io.setVoltage(4);
        io.stop();
        tick(1);
        assertEquals(0, inputs.appliedVolts);
    }
}
