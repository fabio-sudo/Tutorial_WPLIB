package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.subsystems.centrifuga.CentrifugaIOInputsAutoLogged;
import frc.robot.subsystems.centrifuga.CentrifugaIOSim;

/** Regras do modelo em paralelo (as mesmas do telescópio 9458). */
class ParallelModelTest {

    // #region MODELO DE TESTE

    /** Física mínima: velocidade = tensão; registra o que recebeu. */
    private static final class FakeModel implements MechanismModel {
        double position;
        double velocity;
        double lastVolts = Double.NaN;
        int steps;

        @Override public void setModelState(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override public void stepModel(double volts) {
            lastVolts = volts;
            velocity = volts;
            position += velocity * 0.02;
            steps++;
        }

        @Override public double modelPosition() { return position; }
        @Override public double modelVelocity() { return velocity; }
        @Override public double modelCurrentAmps() { return 0.0; }
    }

    private FakeModel fake;
    private ParallelModel model;

    // #endregion

    // #region PREPARAÇÃO

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        SimHooks.pauseTiming();
        fake = new FakeModel();
        model = new ParallelModel("Teste", () -> fake,
            ParallelModel.Unit.of("Graus", 1.0), ParallelModel.Unit.of("GrausPorSeg", 1.0), 3.0, 2.0);
        setEnabled(false);
    }

    @AfterEach
    void cleanup() {
        setEnabled(false);
        SimHooks.resumeTiming();
    }

    private static void setEnabled(boolean enabled) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    private void cycle(double position, double velocity, double volts) {
        SimHooks.stepTiming(0.02);
        model.update(true, position, velocity, volts, 0.0);
    }

    // #endregion

    // #region REGRAS

    @Test
    void disabledAlignsTheModelWithTheRealAndDoesNotStep() {
        cycle(12.0, 0.0, 0.0);
        assertEquals(12.0, fake.position);
        assertEquals(0, fake.steps);
        assertFalse(model.isComparing());
    }

    @Test
    void enabledStepsWithTheMeasuredVoltage() {
        cycle(0.0, 0.0, 0.0);
        setEnabled(true);
        cycle(0.1, 5.0, 5.0);
        assertTrue(model.isComparing());
        assertEquals(5.0, fake.lastVolts);
        assertEquals(1, fake.steps);
    }

    @Test
    void invalidReadingNeverReachesTheModel() {
        setEnabled(true);
        SimHooks.stepTiming(0.02);
        model.update(false, 99.0, 0.0, 12.0, 0.0);
        assertEquals(0, fake.steps);
        assertEquals(0.0, fake.position);
        assertFalse(model.isComparing());
    }

    @Test
    void realStuckWhileTheModelMovesIsDivergenceAfterHalfASecond() {
        cycle(0.0, 0.0, 0.0);
        setEnabled(true);
        // Tensão aplicada, real parado (travado): o modelo anda 10°/s e se afasta.
        for (int i = 0; i < 20; i++) {
            cycle(0.0, 0.0, 10.0);
        }
        assertFalse(model.isDiverging(), "0,4 s ainda não conta");
        for (int i = 0; i < 20; i++) {
            cycle(0.0, 0.0, 10.0);
        }
        assertTrue(model.isDiverging());
    }

    @Test
    void bothStoppedRealignsSoEachMovementStartsFromTheReal() {
        cycle(0.0, 0.0, 0.0);
        setEnabled(true);
        // Movimento: o real anda mais devagar que o modelo e para em 0,5°.
        for (int i = 0; i < 10; i++) {
            cycle(0.05 * (i + 1), 2.5, 5.0);
        }
        assertEquals(1.0, fake.position, 1e-9);
        // Os dois param: realinha no real.
        cycle(0.5, 0.0, 0.0);
        assertEquals(0.5, fake.position, 1e-9);
    }

    // #endregion

    // #region COM A FÍSICA DE VERDADE

    /** A centrífuga simulada e o modelo, com a mesma tensão, ficam juntos. */
    @Test
    void flywheelModelFollowsTheSimulationWithTheMeasuredVoltage() {
        CentrifugaIOSim real = new CentrifugaIOSim();
        CentrifugaIOSim physics = new CentrifugaIOSim();
        ParallelModel centrifuge = new ParallelModel("TesteCentrifuga", () -> physics,
            null, ParallelModel.Unit.of("Rpm", 1.0), 300.0);
        CentrifugaIOInputsAutoLogged inputs = new CentrifugaIOInputsAutoLogged();
        setEnabled(true);
        real.setVelocityRpm(3000.0);
        for (int i = 0; i < 100; i++) {
            SimHooks.stepTiming(0.02);
            real.updateInputs(inputs);
            centrifuge.update(inputs.connected, 0.0, inputs.velocityRpm, inputs.appliedVolts, inputs.currentAmps);
        }
        assertTrue(inputs.velocityRpm > 2500.0, "girou: " + inputs.velocityRpm);
        assertEquals(inputs.velocityRpm, physics.modelVelocity(), 5.0);
        assertFalse(centrifuge.isDiverging());
    }

    // #endregion
}
