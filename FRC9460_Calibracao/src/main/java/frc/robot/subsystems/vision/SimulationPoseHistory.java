package frc.robot.subsystems.vision;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

/** Histórico exclusivo da referência virtual, nunca da pose corrigida pela visão. */
final class SimulationPoseHistory {
    private final TimeInterpolatableBuffer<Pose2d> poses =
            TimeInterpolatableBuffer.createBuffer(2.0);

    void add(double timestamp, Pose2d pose) {
        var samples = poses.getInternalBuffer();
        // Uma reinicialização do relógio não pode misturar duas execuções.
        if (!samples.isEmpty() && timestamp < samples.lastKey()) {
            poses.clear();
        }
        poses.addSample(timestamp, pose);
    }

    Optional<Pose2d> sample(double timestamp) {
        var samples = poses.getInternalBuffer();
        // A WPILib limita consultas às pontas do buffer. Para medir erro,
        // precisamos de cobertura real, sem substituir a captura pelo presente.
        if (!Double.isFinite(timestamp) || samples.isEmpty()
                || timestamp < samples.firstKey() || timestamp > samples.lastKey()) {
            return Optional.empty();
        }
        return poses.getSample(timestamp);
    }
}
