package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

/** Entrega das medições entre o cenário, as entradas do Vision e o log. */
class VisionIOSimTest {
    private static PoseObservation observation(double captureTimestamp, double x) {
        return new PoseObservation(
            captureTimestamp,
            new Pose3d(x, 2.0, 0.03, new Rotation3d(0.01, -0.02, 0.3)),
            0.08,
            2,
            1.8,
            PoseObservationType.PHOTONVISION
        );
    }

    @Test
    void deliversEveryObservationOnceWithOriginalCaptureTimeAndFullPose() {
        var io = new VisionIOSim(() -> 10.0);
        var inputs = new VisionIO.VisionIOInputs();
        var first = observation(9.97, 1.0);
        var second = observation(9.93, 1.1);
        var third = observation(9.99, 1.2);

        // A ordenação temporal será feita pelo Vision. O IO conserva o lote
        // completo na ordem de chegada, inclusive quando a captura veio atrasada.
        io.publishObservation(first, 25, 30.0);
        io.publishObservation(second, 26, 70.0);
        io.publishObservation(third, 25, 10.0);
        io.updateInputs(inputs);
        assertTrue(inputs.usesObservationBatch);
        assertArrayEquals(new PoseObservation[] {first, second, third}, inputs.poseObservations);
        assertEquals(0.03, inputs.poseObservations[0].pose().getZ());
        assertEquals(9.99, inputs.captureTimestampSeconds);
        assertEquals(10.0, inputs.readTimestampSeconds);
        assertEquals(3.0, inputs.heartbeat);

        PoseObservation[] delivered = inputs.poseObservations;
        io.updateInputs(inputs);
        assertEquals(0, inputs.poseObservations.length, "Uma nova leitura não repete o lote");

        var next = observation(10.01, 1.3);
        io.publishObservation(next, 25, 30.0);
        io.updateInputs(inputs);
        assertArrayEquals(new PoseObservation[] {next}, inputs.poseObservations);
        assertArrayEquals(new PoseObservation[] {first, second, third}, delivered,
            "Publicar outro lote não modifica os dados que já foram entregues");
    }

    @Test
    void disconnectDiscardsOldDataAndDoesNotAffectTheOtherCamera() {
        var rear = new VisionIOSim(() -> 10.0);
        var front = new VisionIOSim(() -> 10.0);
        var rearInputs = new VisionIO.VisionIOInputs();
        var frontInputs = new VisionIO.VisionIOInputs();
        var frontMeasurement = observation(9.97, 2.0);
        rear.publishObservation(observation(9.97, 1.0), 9, 30.0);
        front.publishObservation(frontMeasurement, 25, 30.0);

        rear.setConnected(false);
        rear.publishObservation(observation(9.98, 1.1), 9, 20.0);
        rear.updateInputs(rearInputs);
        front.updateInputs(frontInputs);
        assertFalse(rearInputs.connected);
        assertFalse(rearInputs.hasTarget);
        assertFalse(rearInputs.hasPoseEstimate);
        assertFalse(rearInputs.hasPose);
        assertEquals(-1, rearInputs.tagId);
        assertEquals(0, rearInputs.tagCount);
        assertEquals(0.0, rearInputs.captureTimestampSeconds);
        assertEquals(0, rearInputs.poseObservations.length);
        assertArrayEquals(new PoseObservation[] {frontMeasurement}, frontInputs.poseObservations);

        rear.setConnected(true);
        rear.updateInputs(rearInputs);
        assertTrue(rearInputs.connected);
        assertFalse(rearInputs.hasPose);
        assertEquals(0, rearInputs.poseObservations.length,
            "Reconectar não deve restaurar uma observação anterior");

        var recovered = observation(9.99, 1.2);
        rear.publishObservation(recovered, 9, 10.0);
        rear.updateInputs(rearInputs);
        assertTrue(rearInputs.hasPose);
        assertArrayEquals(new PoseObservation[] {recovered}, rearInputs.poseObservations);
    }

    @Test
    void controlledFramesKeepCompatibilityAndFramesWithoutPoseDoNotInventObservations() {
        var io = new VisionIOSim(() -> 10.0);
        var inputs = new VisionIO.VisionIOInputs();
        var pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30));
        io.publishFrame(pose, 2, 25, 1.8, 9.97, 30.0);
        io.updateInputs(inputs);
        assertEquals(pose, inputs.pose);
        assertEquals(1, inputs.poseObservations.length);
        assertEquals(new Pose3d(pose), inputs.poseObservations[0].pose());
        assertEquals(PoseObservationType.MEGATAG_2, inputs.poseObservations[0].type());

        io.publishFrame(null, 2, 25, 1.8, 9.98, 20.0);
        io.updateInputs(inputs);
        assertTrue(inputs.hasTarget);
        assertFalse(inputs.hasPoseEstimate);
        assertEquals(0, inputs.poseObservations.length);

        io.publishFrame(pose, 0, -1, 0.0, 9.99, 10.0);
        io.updateInputs(inputs);
        assertFalse(inputs.hasTarget);
        assertFalse(inputs.hasPose);
        assertEquals(0, inputs.poseObservations.length);
    }

    @Test
    void leavesNumericalQualityChecksToVision() {
        var io = new VisionIOSim(() -> 10.0);
        var inputs = new VisionIO.VisionIOInputs();
        var invalid = new PoseObservation(
            Double.NaN, new Pose3d(), Double.NaN, 0, -1.0, PoseObservationType.PHOTONVISION);
        io.publishObservation(invalid, -1, 30.0);
        io.updateInputs(inputs);
        assertArrayEquals(new PoseObservation[] {invalid}, inputs.poseObservations);
        assertFalse(inputs.hasPoseEstimate);
    }

    @Test
    void advantageKitRoundTripPreservesBatchAndFollowingEmptyCycle() {
        var io = new VisionIOSim(() -> 10.0);
        var live = new VisionIOInputsAutoLogged();
        var replay = new VisionIOInputsAutoLogged();
        var measured = observation(9.97, 1.0);
        io.publishObservation(measured, 25, 30.0);
        io.updateInputs(live);

        var table = new LogTable(10_000_000);
        live.toLog(table);
        replay.fromLog(table);
        assertTrue(replay.usesObservationBatch);
        assertArrayEquals(new PoseObservation[] {measured}, replay.poseObservations);
        assertEquals(live.captureTimestampSeconds, replay.captureTimestampSeconds);
        assertEquals(live.readTimestampSeconds, replay.readTimestampSeconds);

        io.updateInputs(live);
        var nextTable = new LogTable(10_020_000);
        live.toLog(nextTable);
        replay.fromLog(nextTable);
        assertTrue(replay.usesObservationBatch);
        assertEquals(0, replay.poseObservations.length,
            "Replay de um ciclo vazio não pode repetir o lote anterior");
    }
}
