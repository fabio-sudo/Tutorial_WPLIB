package frc.robot.subsystems.fuel;

import static frc.robot.subsystems.fuel.FuelConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.fuel.ShotConsistency.Shot;

/** O diagnóstico separa viés (erro sempre para o mesmo lado) de dispersão (cada tiro diferente). */
class ShotConsistencyTest {

    private int number;

    /** Disparo com os valores relevantes; o resto fica neutro. */
    private Shot shot(double exitSpeed, double aimError, double miss, double topBefore, double topTarget) {
        return new Shot(++number, 3.0, topBefore, topTarget, 150.0, exitSpeed, aimError, miss,
            0.0, 0.5, true, true);
    }

    private Shot good(int i) {
        // Pequenas variações em torno de um tiro bom.
        double wobble = (i % 3 - 1) * 0.02;
        return shot(8.0 + wobble, 0.2 * (i % 3 - 1), 0.10, 3000.0 - 20.0, 3000.0);
    }

    @Test
    void fewShotsSayHowManyAreMissing() {
        ShotConsistency consistency = new ShotConsistency();
        assertEquals("POUCOS_TIROS_0_DE_" + CONSISTENCY_MIN_SHOTS, consistency.diagnosis());
        for (int i = 0; i < CONSISTENCY_MIN_SHOTS - 1; i++) {
            consistency.record(good(i));
        }
        assertEquals("POUCOS_TIROS_" + (CONSISTENCY_MIN_SHOTS - 1) + "_DE_" + CONSISTENCY_MIN_SHOTS,
            consistency.diagnosis());
    }

    @Test
    void repeatableShotsAreConstant() {
        ShotConsistency consistency = new ShotConsistency();
        for (int i = 0; i < 8; i++) {
            consistency.record(good(i));
        }
        assertEquals("CONSTANTE", consistency.diagnosis());
        assertEquals(8.0, consistency.exitSpeed().mean(), 0.02);
    }

    @Test
    void spreadOfExitSpeedIsFlagged() {
        ShotConsistency consistency = new ShotConsistency();
        double[] speeds = {7.0, 8.0, 9.0, 7.5, 8.5, 8.0};
        for (double speed : speeds) {
            consistency.record(shot(speed, 0.0, 0.1, 2980.0, 3000.0));
        }
        assertTrue(consistency.diagnosis().contains("VELOCIDADE_INCONSTANTE"), consistency.diagnosis());
    }

    @Test
    void aimBiasIsNotTheSameAsAimSpread() {
        ShotConsistency consistency = new ShotConsistency();
        // Sempre 2° para o mesmo lado, quase sem variar: erro sistemático (heading, offset).
        for (int i = 0; i < 6; i++) {
            consistency.record(shot(8.0, 2.0 + 0.05 * (i % 2), 0.1, 2980.0, 3000.0));
        }
        assertTrue(consistency.diagnosis().contains("MIRA_COM_VIES"), consistency.diagnosis());
        assertFalse(consistency.diagnosis().contains("MIRA_INCONSTANTE"), consistency.diagnosis());

        // Média zero mas variando muito: dispersão (pivô que não assenta, pose ruidosa).
        ShotConsistency scattered = new ShotConsistency();
        double[] errors = {-3.0, 3.0, -2.5, 2.5, -3.0, 3.0};
        for (double error : errors) {
            scattered.record(shot(8.0, error, 0.1, 2980.0, 3000.0));
        }
        assertTrue(scattered.diagnosis().contains("MIRA_INCONSTANTE"), scattered.diagnosis());
        assertFalse(scattered.diagnosis().contains("MIRA_COM_VIES"), scattered.diagnosis());
    }

    @Test
    void topBelowTheRequestedRpmIsFlagged() {
        ShotConsistency consistency = new ShotConsistency();
        for (int i = 0; i < 6; i++) {
            consistency.record(shot(8.0, 0.0, 0.1, 2700.0, 3000.0));
        }
        assertTrue(consistency.diagnosis().contains("TOP_ABAIXO_DO_PEDIDO"), consistency.diagnosis());
    }

    @Test
    void topAboveTheRequestedRpmIsFlaggedToo() {
        ShotConsistency consistency = new ShotConsistency();
        for (int i = 0; i < 6; i++) {
            consistency.record(shot(8.0, 0.0, 0.1, 3300.0, 3000.0));
        }
        assertTrue(consistency.diagnosis().contains("TOP_ACIMA_DO_PEDIDO"), consistency.diagnosis());
        assertFalse(consistency.diagnosis().contains("TOP_ABAIXO_DO_PEDIDO"), consistency.diagnosis());
    }

    @Test
    void highMissDistanceIsFlagged() {
        ShotConsistency consistency = new ShotConsistency();
        double tooFar = CONSISTENCY_MISS_MAX_MEAN_FRACTION_OF_RADIUS * HUB_OPENING_RADIUS_METERS + 0.1;
        for (int i = 0; i < 6; i++) {
            consistency.record(shot(8.0, 0.0, tooFar, 2980.0, 3000.0));
        }
        assertTrue(consistency.diagnosis().contains("ERRO_DE_QUEDA_ALTO"), consistency.diagnosis());
    }

    @Test
    void ballThatNeverReachesTheHubHeightIsLeftOutOfTheMissAverage() {
        ShotConsistency consistency = new ShotConsistency();
        for (int i = 0; i < 4; i++) {
            consistency.record(shot(8.0, 0.0, -1.0, 2980.0, 3000.0));
        }
        assertEquals(0, consistency.miss().count());
        assertTrue(Double.isNaN(consistency.miss().mean()));
        assertEquals("CONSTANTE", consistency.diagnosis(), "sem média de queda não inventa problema");
    }

    @Test
    void onlyTheLastWindowCounts() {
        ShotConsistency consistency = new ShotConsistency();
        for (int i = 0; i < CONSISTENCY_WINDOW_SHOTS; i++) {
            consistency.record(shot(8.0, 4.0, 0.1, 2980.0, 3000.0));
        }
        assertTrue(consistency.diagnosis().contains("MIRA_COM_VIES"));
        // Depois do ajuste, só tiros bons: a janela esquece os ruins.
        for (int i = 0; i < CONSISTENCY_WINDOW_SHOTS; i++) {
            consistency.record(good(i));
        }
        assertEquals("CONSTANTE", consistency.diagnosis());
        assertEquals(2 * CONSISTENCY_WINDOW_SHOTS, consistency.shotsRecorded());
    }
}
