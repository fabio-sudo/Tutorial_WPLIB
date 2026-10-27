package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Evita dar confiança de multitag a uma solução que utilizou apenas uma tag. */
class VisionPhotonTargetsTest {
    private static PhotonTrackedTarget target(int id, double ambiguity) {
        var target = new PhotonTrackedTarget();
        target.fiducialId = id;
        target.poseAmbiguity = ambiguity;
        return target;
    }

    @Test
    void fallbackSelectsOneKnownTagWithLowestAvailableAmbiguity() {
        var chosen = target(26, 0.05);
        var frame = new PhotonPipelineResult(1, 1_000_000, 1_030_000, 0,
                List.of(target(25, 0.3), chosen, target(9, -1), target(999, 0.01)));
        var used = VisionSimulationScenario.getUsedTargets(frame, false);
        assertEquals(1, used.size(), "Quatro detecções não transformam o fallback em multitag");
        assertSame(chosen, used.get(0));
    }

    @Test
    void multitagUsesOnlyTheIdsReportedByTheSolver() {
        var first = target(25, 0.2);
        var second = target(26, 0.2);
        var multiTag = new MultiTargetPNPResult();
        multiTag.fiducialIDsUsed = List.of((short) 25, (short) 26);
        var frame = new PhotonPipelineResult(1, 1_000_000, 1_030_000, 0,
                List.of(first, target(9, 0.01), second), Optional.of(multiTag));
        assertEquals(List.of(first, second), VisionSimulationScenario.getUsedTargets(frame, true));
    }
}
