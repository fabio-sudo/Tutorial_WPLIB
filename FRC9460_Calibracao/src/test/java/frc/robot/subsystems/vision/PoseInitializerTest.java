package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.PoseInitializer.Candidate;
import frc.robot.subsystems.vision.PoseInitializer.State;

/**
 * Quando a pose pode ser definida pelas tags. O caso que motivou tudo: o
 * Pigeon ligado virado, o MegaTag2 espelhando a pose em volta das tags (28 e
 * 29/10/2026) e o shooter confiando nela.
 */
class PoseInitializerTest {

    private final PoseInitializer initializer = new PoseInitializer();
    private double now = 1.0;
    private double stamp = 1.0;

    private static Pose2d pose(double x, double y, double headingDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
    }

    private Candidate frame(double x, double y, double headingDegrees) {
        stamp += 0.033;
        return new Candidate(pose(x, y, headingDegrees), 2, 2.5, stamp);
    }

    /** Alimenta quadros novos, um por ciclo de ~33 ms, e devolve o último resultado. */
    private Optional<Pose2d> feed(int count, boolean eligible, double x, double y, double heading, Pose2d current) {
        Optional<Pose2d> result = Optional.empty();
        for (int i = 0; i < count; i++) {
            now += 0.033;
            result = initializer.update(now, eligible, frame(x, y, heading), current);
        }
        return result;
    }

    @Test
    void neverActsWhenItIsNotSafe() {
        // Habilitado, em movimento ou gyro ruim: quem chama diz eligible = false.
        Optional<Pose2d> result = feed(20, false, 3.5, 4.0, 180.0, pose(0, 0, 0));
        assertTrue(result.isEmpty());
        assertEquals(State.INELEGIVEL, initializer.state());
        assertEquals(0, initializer.framesCollected());
    }

    @Test
    void mirroredHeadingIsFixedOnlyAfterEnoughStableFrames() {
        Pose2d wrong = pose(0, 0, 0); // Pigeon ligado virado: o estimador acha que olha para +X.
        Optional<Pose2d> early = feed(AUTO_INIT_STABLE_FRAMES - 1, true, 3.5, 4.0, 180.0, wrong);
        assertTrue(early.isEmpty(), "ainda juntando quadros");
        assertEquals(State.JUNTANDO_FRAMES, initializer.state());

        Optional<Pose2d> applied = feed(1, true, 3.5, 4.0, 180.0, wrong);
        assertTrue(applied.isPresent());
        assertEquals(State.APLICADA, initializer.state());
        assertEquals(3.5, applied.get().getX(), 1e-9);
        assertEquals(4.0, applied.get().getY(), 1e-9);
        assertEquals(180.0, Math.abs(applied.get().getRotation().getDegrees()), 1e-6);
        assertEquals(180.0, Math.abs(initializer.headingErrorDegrees()), 1e-6);
    }

    @Test
    void alreadyAlignedPoseIsNeverTouchedAndNeverReset() {
        Pose2d good = pose(3.45, 4.02, 179.0);
        for (int i = 0; i < 60; i++) {
            assertTrue(feed(1, true, 3.5, 4.0, 180.0, good).isEmpty(), "ciclo " + i);
        }
        assertEquals(State.ALINHADA, initializer.state());
        assertTrue(Math.abs(initializer.headingErrorDegrees()) < AUTO_INIT_HEADING_TOLERANCE_DEGREES);
    }

    @Test
    void differencesInsideTheToleranceAreIgnored() {
        // 4° de heading e 0,5 m: abaixo de 6° e 0,75 m.
        Pose2d close = pose(3.0, 4.0, 176.0);
        assertTrue(feed(AUTO_INIT_STABLE_FRAMES + 3, true, 3.5, 4.0, 180.0, close).isEmpty());
        assertEquals(State.ALINHADA, initializer.state());
    }

    @Test
    void farPositionIsAppliedEvenWithTheRightHeading() {
        Pose2d lost = pose(1.0, 1.0, 180.0);
        Optional<Pose2d> applied = feed(AUTO_INIT_STABLE_FRAMES, true, 3.5, 4.0, 180.0, lost);
        assertTrue(applied.isPresent());
        assertEquals(State.APLICADA, initializer.state());
    }

    @Test
    void afterApplyingItDoesNotApplyAgain() {
        Pose2d wrong = pose(0, 0, 0);
        Optional<Pose2d> applied = feed(AUTO_INIT_STABLE_FRAMES, true, 3.5, 4.0, 180.0, wrong);
        assertTrue(applied.isPresent());
        // O Drive passou a ter a pose aplicada; os mesmos quadros continuam chegando.
        for (int i = 0; i < 40; i++) {
            assertTrue(feed(1, true, 3.5, 4.0, 180.0, applied.get()).isEmpty(), "ciclo " + i);
        }
        assertEquals(State.ALINHADA, initializer.state());
    }

    @Test
    void theSameCameraFrameRepeatedCountsOnce() {
        Candidate repeated = new Candidate(pose(3.5, 4.0, 180.0), 2, 2.5, 5.0);
        for (int i = 0; i < 40; i++) {
            now += 0.02;
            assertTrue(initializer.update(now, true, repeated, pose(0, 0, 0)).isEmpty());
        }
        // A Limelight repete o último valor entre quadros: não vira estabilidade.
        assertTrue(initializer.framesCollected() <= 1);
    }

    @Test
    void framesThatDisagreeAreNotApplied() {
        Pose2d wrong = pose(0, 0, 0);
        Optional<Pose2d> result = Optional.empty();
        // Heading alternando 10° entre quadros: câmera tremendo ou tags trocando.
        for (int i = 0; i < AUTO_INIT_STABLE_FRAMES + 4; i++) {
            result = feed(1, true, 3.5, 4.0, i % 2 == 0 ? 175.0 : 185.0, wrong);
        }
        assertTrue(result.isEmpty());
        assertEquals(State.INSTAVEL, initializer.state());
    }

    @Test
    void weakReadingsAreIgnored() {
        Pose2d wrong = pose(0, 0, 0);
        // Uma tag só: o heading do MegaTag1 é ambíguo.
        for (int i = 0; i < 20; i++) {
            now += 0.033;
            stamp += 0.033;
            assertTrue(initializer.update(now, true,
                new Candidate(pose(3.5, 4.0, 180.0), 1, 2.5, stamp), wrong).isEmpty());
        }
        assertEquals(State.SEM_LEITURA_BOA, initializer.state());
        // Longe demais, e fora do campo.
        for (int i = 0; i < 20; i++) {
            now += 0.033;
            stamp += 0.033;
            assertTrue(initializer.update(now, true,
                new Candidate(pose(3.5, 4.0, 180.0), 3, AUTO_INIT_MAX_TAG_DISTANCE_METERS + 1.0, stamp),
                wrong).isEmpty());
            assertTrue(initializer.update(now, true,
                new Candidate(pose(-2.0, 4.0, 180.0), 3, 2.5, stamp), wrong).isEmpty());
        }
        assertEquals(0, initializer.framesCollected());
    }

    @Test
    void gapWithoutNewFramesRestartsTheCount() {
        Pose2d wrong = pose(0, 0, 0);
        feed(AUTO_INIT_STABLE_FRAMES - 2, true, 3.5, 4.0, 180.0, wrong);
        now += AUTO_INIT_FRAME_TIMEOUT_SECONDS + 0.2; // câmera sumiu
        // Só faltavam 2 quadros, mas os antigos já não valem.
        Optional<Pose2d> result = feed(2, true, 3.5, 4.0, 180.0, wrong);
        assertTrue(result.isEmpty());
        assertEquals(2, initializer.framesCollected());
    }

    @Test
    void headingIsAveragedAsAnAngleNotAsANumber() {
        // 359° e 1° estão a 2° um do outro. Média simples dá 180° (a oposta).
        Pose2d facingFar = pose(0, 0, 90.0);
        Optional<Pose2d> result = Optional.empty();
        for (int i = 0; i < AUTO_INIT_STABLE_FRAMES; i++) {
            result = feed(1, true, 3.5, 4.0, i % 2 == 0 ? 359.0 : 1.0, facingFar);
        }
        assertTrue(result.isPresent());
        assertEquals(0.0, result.get().getRotation().getDegrees(), 1.0);
    }
}
