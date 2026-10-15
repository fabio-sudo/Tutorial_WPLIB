package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class SimulationPoseHistoryTest {
    @Test void comparesMovingFramesAtCaptureInsteadOfCurrentPosition() {
        var history = new SimulationPoseHistory();
        history.add(10.0, new Pose2d(2, 2, new Rotation2d()));
        history.add(10.1, new Pose2d(3, 2, new Rotation2d()));
        assertEquals(2.7, history.sample(10.07).orElseThrow().getX(), 1e-9);
        assertTrue(history.sample(9.99).isEmpty());
        assertTrue(history.sample(10.11).isEmpty());
        assertTrue(history.sample(Double.NaN).isEmpty());
    }

    @Test void expiredHistoryAndClockRestartDoNotReuseOldPoses() {
        var history = new SimulationPoseHistory();
        history.add(10, new Pose2d());
        history.add(13, new Pose2d());
        assertTrue(history.sample(10).isEmpty());
        history.add(1, new Pose2d(2, 2, new Rotation2d()));
        assertTrue(history.sample(13).isEmpty());
        assertEquals(2, history.sample(1).orElseThrow().getX());
    }
}
