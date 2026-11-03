package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

import frc.robot.util.MechanismDiagnostics;

/** Dispositivo do swerve fora do CAN precisa aparecer, dizendo qual procurar. */
class ModuleHardwareStatusTest {

    private static class FakeModule implements ModuleIO {
        boolean drive = true;
        boolean turn = true;
        boolean encoder = true;

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = drive;
            inputs.turnConnected = turn;
            inputs.encoderConnected = encoder;
        }
    }

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    private static String statusOf(FakeModule io) {
        Module module = new Module(io, 1);
        module.periodic();
        return module.getHardwareStatus();
    }

    @Test void allThreeDevicesRespondingIsOk() {
        assertEquals(MechanismDiagnostics.OK, statusOf(new FakeModule()));
    }

    @Test void theStatusSaysWhichDeviceToLookFor() {
        FakeModule noDrive = new FakeModule();
        noDrive.drive = false;
        assertEquals("DRIVE_SEM_RESPOSTA", statusOf(noDrive));

        FakeModule noSteer = new FakeModule();
        noSteer.turn = false;
        assertEquals("STEER_SEM_RESPOSTA", statusOf(noSteer));

        FakeModule noEncoder = new FakeModule();
        noEncoder.encoder = false;
        assertEquals("CANCODER_SEM_RESPOSTA", statusOf(noEncoder));
    }

    @Test void aDeviceThatReturnsClearsTheStatus() {
        FakeModule io = new FakeModule();
        io.turn = false;
        Module module = new Module(io, 2);
        module.periodic();
        assertEquals("STEER_SEM_RESPOSTA", module.getHardwareStatus());
        io.turn = true;
        module.periodic();
        assertEquals(MechanismDiagnostics.OK, module.getHardwareStatus());
    }

    @Test void theSimulatedModuleAlwaysResponds() {
        Module module = new Module(new ModuleIOSim(), 0);
        module.periodic();
        assertEquals(MechanismDiagnostics.OK, module.getHardwareStatus());
    }
}
