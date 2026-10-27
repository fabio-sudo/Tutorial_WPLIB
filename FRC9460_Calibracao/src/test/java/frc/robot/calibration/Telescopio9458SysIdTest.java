package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.telescopio_9458.TelescopioIO_9458;
import frc.robot.subsystems.telescopio_9458.TelescopioIOSim_9458;
import frc.robot.subsystems.telescopio_9458.Telescopio_9458;
import org.junit.jupiter.api.*;

/** R1 (quasistático) e L1 (dinâmico) na física simulada, no modo Test, começando fechado. */
class Telescopio9458SysIdTest {
    private Telescopio_9458 telescopio;
    private Telescopio9458SysId sysId;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private double highest;
    private double lowest;

    @BeforeAll static void init() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setTest(true);
        RoboRioSim.setVInVoltage(12);
        create(new TelescopioIOSim_9458());
    }
    @AfterEach void cleanup() {
        scheduler.cancelAll();
        telescopio.close();
        enabled(false);
        DriverStationSim.setTest(false);
        SimHooks.resumeTiming();
    }
    private void create(TelescopioIO_9458 io) {
        if (telescopio != null) telescopio.close();
        enabled(false);
        telescopio = new Telescopio_9458(io);
        sysId = new Telescopio9458SysId(telescopio, false);
        tick(1);
        enabled(true);
    }
    private void enabled(boolean value) {
        DriverStationSim.setEnabled(value);
        DriverStationSim.notifyNewData();
    }
    private void tick(int cycles) {
        for (int i = 0; i < cycles; i++) {
            SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
            scheduler.run();
            highest = Math.max(highest, telescopio.getExtensionMeters());
            lowest = Math.min(lowest, telescopio.getExtensionMeters());
        }
    }
    /** Segura o botão até a sequência terminar sozinha; devolve quantos ciclos levou. */
    private int holdUntilFinished(Command button) {
        highest = telescopio.getExtensionMeters();
        lowest = highest;
        scheduler.schedule(button);
        int cycles = 0;
        while (button.isScheduled()) {
            tick(1);
            assertTrue(++cycles < 1500, "a sequência deveria terminar sozinha");
        }
        tick(1);
        return cycles;
    }

    @Test void r1QuasistaticGoesUpPausesAndComesBackClosed() {
        Command r1 = sysId.quasistaticUpAndDown();
        scheduler.schedule(r1);
        tick(102); // 2 s de rampa (+2 ciclos: início da sequência e tensão aplicada no periodic seguinte)
        assertTrue(telescopio.isCharacterizationActive());
        assertEquals("SYSID", telescopio.getStatus());
        assertEquals(1.0, telescopio.getRequestedVolts(), 0.03);

        int cycles = 102 + holdUntilFinished(r1);
        assertTrue(cycles * LOOP_PERIOD_SECONDS < 2 * SYSID_TIMEOUT_SECONDS, "parou pela posição, não pelo tempo");
        assertTrue(highest >= SYSID_UPPER_STOP_METERS, "subiu até a parada: " + highest);
        assertTrue(highest < MAX_OPERATING_EXTENSION_METERS, "não passa do limite operacional: " + highest);
        assertEquals(SYSID_LOWER_STOP_METERS, telescopio.getExtensionMeters(), 0.03);
        assertTrue(lowest > MIN_EXTENSION_METERS, "não bate no fundo: " + lowest);
        assertFalse(telescopio.isCharacterizationActive());
        assertEquals(0, telescopio.getRequestedVolts());
    }

    @Test void l1DynamicUsesTwoVoltStepsUpAndDown() {
        Command l1 = sysId.dynamicUpAndDown();
        scheduler.schedule(l1);
        tick(10);
        assertEquals(SYSID_STEP_VOLTS, telescopio.getRequestedVolts(), 1e-9);
        while (telescopio.getExtensionMeters() < SYSID_UPPER_STOP_METERS) tick(1);
        tick(5);
        assertEquals(0, telescopio.getRequestedVolts(), "pausa em 0 V no alto");
        tick((int) (SYSID_PAUSE_SECONDS / LOOP_PERIOD_SECONDS) + 2);
        assertEquals(-SYSID_STEP_VOLTS, telescopio.getRequestedVolts(), 1e-9);

        holdUntilFinished(l1);
        assertEquals(SYSID_LOWER_STOP_METERS, telescopio.getExtensionMeters(), 0.03);
        assertEquals(0, telescopio.getRequestedVolts());
    }

    @Test void releasingTheButtonCutsTheVoltageAndDoesNotContinue() {
        Command l1 = sysId.dynamicUpAndDown();
        scheduler.schedule(l1);
        tick(10);
        assertEquals(SYSID_STEP_VOLTS, telescopio.getRequestedVolts(), 1e-9);
        l1.cancel(); // soltou o L1 no meio da subida
        tick(1);
        assertFalse(telescopio.isCharacterizationActive());
        assertEquals(0, telescopio.getRequestedVolts());
        assertEquals("PARADO", telescopio.getStatus());
        tick(100);
        assertEquals(0, telescopio.getRequestedVolts(), "não desce sozinho depois de soltar");
    }

    @Test void disablingDuringTheTestStopsTheMotor() {
        scheduler.schedule(sysId.quasistaticUpAndDown());
        tick(50);
        enabled(false);
        tick(1);
        assertFalse(telescopio.isCharacterizationActive());
        assertEquals(0, telescopio.getRequestedVolts());
        assertFalse(telescopio.isPositionControlActive());
    }

    @Test void stallStopsTheWholeSequenceAndTheNextPressTriesAgain() {
        var stuck = new StuckIO();
        create(stuck);
        Command r1 = sysId.quasistaticUpAndDown();
        int cycles = holdUntilFinished(r1);
        assertTrue(telescopio.isCharacterizationStalled());
        assertEquals(0, stuck.volts, "parou de empurrar");
        assertTrue(cycles * LOOP_PERIOD_SECONDS < SYSID_STALL_TIME_SECONDS + 0.5,
                "parou pelo travamento, sem rodar a descida: " + cycles);
        assertTrue(stuck.maxVolts < 1.0, "a rampa não continuou subindo: " + stuck.maxVolts);

        scheduler.schedule(sysId.quasistaticUpAndDown()); // apertou de novo
        tick(5);
        assertFalse(telescopio.isCharacterizationStalled());
        assertTrue(telescopio.isCharacterizationActive());
    }

    @Test void dataFromR1AndL1RecoversTheSimulatedGainsLikeSysId() {
        var recorder = new RecordingIO();
        create(recorder);
        holdUntilFinished(sysId.quasistaticUpAndDown());
        holdUntilFinished(sysId.dynamicUpAndDown());

        // Mesmo modelo do SysId (Elevator): V = kS·sinal(v) + kG + kV·v + kA·a
        double[] gains = fitElevator(recorder);
        assertEquals(SIM_KG, gains[1], 0.005, "kG");
        assertEquals(SIM_KV, gains[2], SIM_KV * 0.01, "kV");
        assertEquals(SIM_KS, gains[0], 0.01, "kS: atrito da simulação");
        // kA não é verificado: a constante de tempo do motor (kA/kV ≈ 2 ms) é muito menor
        // que o ciclo de 20 ms, e a aceleração quase não aparece nas amostras (sai ≈ 0).
    }

    /** Mínimos quadrados só com as amostras em movimento durante o SysId, como o SysId faz. */
    private static double[] fitElevator(RecordingIO data) {
        double[][] normal = new double[4][5];
        for (int i = 1; i < data.size; i++) {
            double v = data.velocity[i];
            if (!data.active[i] || !data.active[i - 1] || Math.abs(v) < 0.02) continue;
            double a = (v - data.velocity[i - 1]) / LOOP_PERIOD_SECONDS;
            double[] x = {Math.signum(v), 1, v, a};
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) normal[r][c] += x[r] * x[c];
                normal[r][4] += x[r] * data.volts[i];
            }
        }
        for (int p = 0; p < 4; p++) { // eliminação de Gauss
            for (int r = p + 1; r < 4; r++) {
                double f = normal[r][p] / normal[p][p];
                for (int c = p; c < 5; c++) normal[r][c] -= f * normal[p][c];
            }
        }
        double[] k = new double[4];
        for (int r = 3; r >= 0; r--) {
            double sum = normal[r][4];
            for (int c = r + 1; c < 4; c++) sum -= normal[r][c] * k[c];
            k[r] = sum / normal[r][r];
        }
        return k;
    }

    /** Física simulada que guarda a tensão aplicada e a velocidade medidas a cada ciclo. */
    private class RecordingIO extends TelescopioIOSim_9458 {
        final double[] volts = new double[3000];
        final double[] velocity = new double[3000];
        final boolean[] active = new boolean[3000];
        int size;
        @Override public void updateInputs(TelescopioIOInputs_9458 inputs) {
            super.updateInputs(inputs);
            volts[size] = inputs.appliedVolts;
            velocity[size] = inputs.velocityMetersPerSec;
            active[size] = telescopio != null && telescopio.isCharacterizationActive();
            size++;
        }
    }

    /** Telescópio preso em 0,3 m: qualquer tensão vira corrente alta sem movimento. */
    private static class StuckIO implements TelescopioIO_9458 {
        double volts;
        double maxVolts;
        @Override public void updateInputs(TelescopioIOInputs_9458 inputs) {
            inputs.motorConnected = true;
            inputs.positionValid = true;
            inputs.extensionMeters = 0.3;
            inputs.velocityMetersPerSec = 0;
            inputs.appliedVolts = volts;
            inputs.motorCurrentAmps = volts == 0 ? 0 : STATOR_CURRENT_LIMIT_AMPS;
            inputs.controlRequestOK = true;
        }
        @Override public void setVoltage(double value) {
            volts = value;
            maxVolts = Math.max(maxVolts, Math.abs(value));
        }
    }
}
