package frc.robot.subsystems.telescopio;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import java.nio.file.Path;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.math.geometry.Pose3d;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class TelescopioVisualizerTest {
    @Test void closedPivotsMatchTheExistingAssetDemonstration() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        var path = Path.of("config/advantagescope/mecanismos/Robot_Telescopio_9460/Telescopio_9460-Demo.wpilog");
        var reader = new DataLogReader(path.toString());
        var entries = new HashMap<Integer, String>();
        for (var record : reader) {
            if (record.isStart()) entries.put(record.getStartData().entry, record.getStartData().name);
            if (!record.isControl() && "/RealOutputs/Telescopio/Componentes".equals(entries.get(record.getEntry()))) {
                var bytes = ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
                var poses = TelescopioVisualizer.componentPoses(0);
                assertEquals(Pose3d.struct.unpack(bytes), poses[0]);
                assertEquals(Pose3d.struct.unpack(bytes), poses[1]);
                return;
            }
        }
        fail("Não encontrou os componentes no log de referência");
    }
    @Test void bothStagesMoveTogetherAndTheThirdIsCarriedByTheSecond() {
        assertEquals(0.312, TelescopioVisualizer.stage2Extension(MAX_EXTENSION_METERS / 2), 1e-9);
        assertEquals(0.314, TelescopioVisualizer.stage3RelativeExtension(MAX_EXTENSION_METERS / 2), 1e-9);
        var closed = TelescopioVisualizer.componentPoses(0);
        var opened = TelescopioVisualizer.componentPoses(MAX_EXTENSION_METERS);
        assertEquals(0.624, opened[0].getZ() - closed[0].getZ(), 1e-9);
        assertEquals(1.252, opened[1].getZ() - closed[1].getZ(), 1e-9);
        assertEquals(OPEN_HEIGHT_METERS, CLOSED_HEIGHT_METERS + MAX_EXTENSION_METERS, 1e-9);
    }
    @Test void loggedInputsPreserveValidityAndMeasuredValues() {
        var original = new TelescopioIOInputsAutoLogged();
        original.motorConnected = true;
        original.positionValid = false;
        original.extensionMeters = 0.52;
        original.velocityMetersPerSec = -0.1;
        original.appliedVolts = 2.3;
        original.motorCurrentAmps = 12.5;
        var table = new LogTable(1234);
        original.toLog(table);
        var replay = new TelescopioIOInputsAutoLogged();
        replay.fromLog(table);
        assertTrue(replay.motorConnected);
        assertFalse(replay.positionValid);
        assertEquals(original.extensionMeters, replay.extensionMeters);
        assertEquals(original.velocityMetersPerSec, replay.velocityMetersPerSec);
        assertEquals(original.appliedVolts, replay.appliedVolts);
        assertEquals(original.motorCurrentAmps, replay.motorCurrentAmps);
    }
}
