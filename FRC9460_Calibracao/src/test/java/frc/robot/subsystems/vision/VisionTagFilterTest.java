package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;

import org.junit.jupiter.api.Test;

/**
 * A simulação não aplica o filtro de IDs da Limelight: uma lista curta passa em
 * todos os testes e só aparece no robô, como o tiro errado na diagonal de 30/10/2026
 * (só 25, 26, 9 e 10 aceitas). Restringir a lista precisa ser uma decisão consciente.
 */
class VisionTagFilterTest {

    private static void assertAcceptsEveryFieldTag(int[] accepted) {
        for (var tag : VisionFieldConstants.FIELD_LAYOUT.getTags()) {
            assertTrue(Arrays.stream(accepted).anyMatch(id -> id == tag.ID), "tag " + tag.ID + " fora do filtro");
        }
    }

    @Test void rearCameraAcceptsEveryTagOfTheField() {
        assertAcceptsEveryFieldTag(LimelightConstants.VALID_TAG_IDS);
    }

    @Test void frontCameraAcceptsEveryTagOfTheField() {
        assertAcceptsEveryFieldTag(LimelightConstants.FRONT_VALID_TAG_IDS);
    }
}
