package frc.robot.subsystems.telescopio_9458;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.*;

/** Exercita o adaptador real, injetando apenas a fronteira CAN. */
class TelescopioHardware_9458Test {
    private final FakeHardware device = new FakeHardware();
    private TelescopioIOTalonFX_9458 io;
    private final TelescopioIO_9458.TelescopioIOInputs_9458 inputs =
            new TelescopioIO_9458.TelescopioIOInputs_9458();

    @BeforeAll static void init() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        enabled(false);
    }
    @AfterEach void cleanup() {
        if (io != null) io.close();
        enabled(false);
    }
    private void enabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    @Test void startsReferencedAndConvertsRawEncoderWithoutClamping() {
        io = new TelescopioIOTalonFX_9458(device);
        device.position = 78.4;
        io.updateInputs(inputs);
        assertEquals(1, device.zeroCalls);
        assertTrue(inputs.positionValid);
        assertEquals(78.4, inputs.rotorPositionRotations);
        assertEquals(MAX_EXTENSION_METERS, inputs.extensionMeters, 1e-9);
        device.position = -2;
        io.updateInputs(inputs);
        assertTrue(inputs.extensionMeters < 0, "A leitura bruta deve mostrar erros de zero/sentido");
    }

    @Test void resetBlocksOutputUntilDisabledOperatorRecovery() {
        io = new TelescopioIOTalonFX_9458(device);
        io.updateInputs(inputs);
        enabled(true);
        io.setVoltage(2);
        assertEquals(2, device.volts);
        device.reset = true;
        io.updateInputs(inputs);
        assertFalse(inputs.positionValid);
        assertTrue(inputs.referenceLostOnReset);
        assertEquals(0, device.volts);
        io.setVoltage(2);
        assertEquals(0, device.volts);
        assertFalse(io.confirmRetractedZero());
        assertEquals(1, device.zeroCalls, "Não zerar em movimento nem automaticamente após reset");
        enabled(false);
        assertTrue(io.confirmRetractedZero());
        io.updateInputs(inputs);
        assertTrue(inputs.positionValid);
        assertFalse(inputs.referenceLostOnReset);
        assertEquals(2, device.zeroCalls);
    }

    @Test void lateMotorConnectionNeedsExplicitReferenceAndCanRecover() {
        device.configureStatus = StatusCode.GeneralError;
        io = new TelescopioIOTalonFX_9458(device);
        assertEquals(CONFIG_ATTEMPTS, device.configureCalls);
        assertEquals(0, device.zeroCalls);
        device.configureStatus = StatusCode.OK;
        io.updateInputs(inputs);
        assertTrue(inputs.motorConnected);
        assertFalse(inputs.positionValid);
        assertTrue(io.confirmRetractedZero());
        io.updateInputs(inputs);
        assertTrue(inputs.positionValid);
    }

    @Test void failedZeroNeverPermitsVoltage() {
        device.zeroStatus = StatusCode.GeneralError;
        io = new TelescopioIOTalonFX_9458(device);
        io.updateInputs(inputs);
        assertTrue(inputs.configurationApplied);
        assertFalse(inputs.referenceEstablished);
        assertEquals(CONFIG_ATTEMPTS, device.zeroCalls);
        enabled(true);
        io.setVoltage(3);
        assertEquals(0, device.volts);
    }

    @Test void communicationLossStopsRequestsWithoutSilentlyRezeroing() {
        io = new TelescopioIOTalonFX_9458(device);
        io.updateInputs(inputs);
        enabled(true);
        device.readStatus = StatusCode.GeneralError;
        io.updateInputs(inputs);
        io.setVoltage(2);
        assertFalse(inputs.motorConnected);
        assertEquals(0, device.volts);
        device.readStatus = StatusCode.OK;
        io.updateInputs(inputs);
        assertTrue(inputs.positionValid);
        assertEquals(1, device.zeroCalls);
    }

    @Test void reportsRejectedRequestsAndBoundsVoltageAtTheIOBoundary() {
        io = new TelescopioIOTalonFX_9458(device);
        io.updateInputs(inputs);
        enabled(true);
        io.setVoltage(100);
        assertEquals(MAX_CONTROL_VOLTS, device.volts);
        io.setVoltage(Double.NaN);
        assertEquals(0, device.volts);
        device.controlStatus = StatusCode.GeneralError;
        io.setVoltage(1);
        io.updateInputs(inputs);
        assertFalse(inputs.controlRequestOK);
        assertEquals(StatusCode.GeneralError.toString(), inputs.controlStatus);
        enabled(false);
        io.setVoltage(2);
        assertEquals(0, device.volts);
    }

    @Test void motionMagicSendsRotorTargetsAndRewritesTheProfileOnlyWhenTheCruiseChanges() {
        io = new TelescopioIOTalonFX_9458(device);
        device.position = 0.3 * MOTOR_ROTATIONS_PER_METER;
        io.updateInputs(inputs);
        enabled(true);
        int configs = device.motionMagicConfigs;
        for (int i = 0; i < 10; i++) io.setMotionMagicPosition(0.5, MOTION_MAGIC_VELOCITY_METERS_PER_SEC);
        assertEquals(0.5 * MOTOR_ROTATIONS_PER_METER, device.motionMagicTarget, 1e-9);
        assertEquals(configs, device.motionMagicConfigs, "o cruzeiro da partida já está no Talon");
        io.setMotionMagicPosition(0.5, END_APPROACH_VELOCITY_METERS_PER_SEC);
        io.setMotionMagicPosition(0.5, END_APPROACH_VELOCITY_METERS_PER_SEC);
        assertEquals(configs + 1, device.motionMagicConfigs, "troca de trecho: uma escrita só");
        assertEquals(END_APPROACH_VELOCITY_METERS_PER_SEC, device.lastCruise, 1e-12);
        assertEquals(MOTION_CONFIG_TIMEOUT_SECONDS, device.lastTimeout, "em movimento, espera curta");
        io.setMotionMagicPosition(5, END_APPROACH_VELOCITY_METERS_PER_SEC);
        assertEquals(MAX_OPERATING_EXTENSION_METERS * MOTOR_ROTATIONS_PER_METER, device.motionMagicTarget, 1e-9);
        io.updateInputs(inputs);
        assertTrue(inputs.motionMagicActive);
    }

    @Test void retractedGoalRestsWithoutVoltageAndProfileChangesOnlyWhileDisabled() {
        io = new TelescopioIOTalonFX_9458(device);
        io.updateInputs(inputs);
        enabled(true);
        device.volts = -1;
        io.setMotionMagicPosition(0, END_APPROACH_VELOCITY_METERS_PER_SEC);
        assertEquals(0, device.volts, "recolhido apoia no batente sem tensão");
        assertTrue(Double.isNaN(device.motionMagicTarget));
        var limits = new TelescopioMotionProfile_9458.Limits(0.6, 2, 20);
        assertFalse(io.configureMotionProfile(limits), "perfil não muda em movimento");
        enabled(false);
        assertTrue(io.configureMotionProfile(limits));
        assertEquals(CONFIG_TIMEOUT_SECONDS, device.lastTimeout);
        assertEquals(0.6, device.lastCruise, 1e-12);
        enabled(true);
        device.position = 0.3 * MOTOR_ROTATIONS_PER_METER;
        io.updateInputs(inputs);
        int configs = device.motionMagicConfigs;
        io.setMotionMagicPosition(0.5, 0.9); // acima do perfil: fica nos 0,6 já gravados
        assertEquals(configs, device.motionMagicConfigs);
    }

    @Test void refusedSectionChangeKeepsMovingWithThePreviousCruiseAndDoesNotRetryEveryCycle() {
        // Subiu e parou (22/10): a troca para o trecho lento era enviada com timeout 0, o Talon
        // devolvia TimeoutCannotBeZero e o código bloqueava o telescópio.
        io = new TelescopioIOTalonFX_9458(device);
        device.position = 0.8 * MOTOR_ROTATIONS_PER_METER;
        io.updateInputs(inputs);
        enabled(true);
        device.configureStatus = StatusCode.GeneralError;
        int configs = device.motionMagicConfigs;
        for (int i = 0; i < 10; i++) {
            io.setMotionMagicPosition(MAX_OPERATING_EXTENSION_METERS, END_APPROACH_VELOCITY_METERS_PER_SEC);
            io.updateInputs(inputs);
        }
        assertEquals(configs + 1, device.motionMagicConfigs, "uma tentativa por troca, não uma por ciclo");
        assertTrue(inputs.positionValid, "a troca de trecho não é motivo para bloquear");
        assertTrue(inputs.motionMagicActive);
        assertEquals(MAX_OPERATING_EXTENSION_METERS * MOTOR_ROTATIONS_PER_METER, device.motionMagicTarget, 1e-9);
        assertTrue(inputs.configurationStatus.startsWith("TROCA_DE_TRECHO_RECUSADA"));
        device.configureStatus = StatusCode.OK;
        io.setMotionMagicPosition(0.5, MOTION_MAGIC_VELOCITY_METERS_PER_SEC); // pedido mudou: tenta de novo
        io.setMotionMagicPosition(0.5, END_APPROACH_VELOCITY_METERS_PER_SEC);
        assertEquals(END_APPROACH_VELOCITY_METERS_PER_SEC, device.lastCruise, 1e-12);
    }

    private static class FakeHardware implements TelescopioIOTalonFX_9458.Hardware {
        StatusCode configureStatus = StatusCode.OK;
        StatusCode zeroStatus = StatusCode.OK;
        StatusCode readStatus = StatusCode.OK;
        StatusCode controlStatus = StatusCode.OK;
        int configureCalls, zeroCalls, motionMagicConfigs;
        boolean reset;
        double position, volts, lastTimeout, lastCruise = Double.NaN, motionMagicTarget = Double.NaN;
        @Override public StatusCode configure(TelescopioGains_9458 gains, TelescopioMotionProfile_9458.Limits profile) {
            configureCalls++;
            return configureStatus;
        }
        @Override public StatusCode configureGains(TelescopioGains_9458 gains) { return configureStatus; }
        @Override public StatusCode configureMotionMagic(double velocity, double acceleration, double jerk, double timeout) {
            motionMagicConfigs++;
            lastTimeout = timeout;
            // Como a CTRE: configuração com timeout 0 não é aplicada.
            if (timeout <= 0) return StatusCode.TimeoutCannotBeZero;
            if (configureStatus.isOK()) lastCruise = velocity;
            return configureStatus;
        }
        @Override public StatusCode zeroPosition() {
            zeroCalls++;
            if (zeroStatus.isOK()) position = 0;
            return zeroStatus;
        }
        @Override public boolean hasResetOccurred() {
            boolean value = reset;
            reset = false;
            return value;
        }
        @Override public StatusCode read(TelescopioIO_9458.TelescopioIOInputs_9458 inputs) {
            inputs.rotorPositionRotations = position;
            inputs.appliedVolts = volts;
            return readStatus;
        }
        @Override public StatusCode setVoltage(double value) { volts = value; return controlStatus; }
        @Override public StatusCode setMotionMagic(double rotorRotations, boolean limitForward, boolean limitReverse) {
            motionMagicTarget = rotorRotations;
            return controlStatus;
        }
        @Override public void close() {}
    }
}
