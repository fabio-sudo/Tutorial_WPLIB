package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Média e desvio da janela móvel, conferidos com contas à mão. */
class RollingStatsTest {

    @Test
    void meanAndSampleStdDevMatchTheHandCalculation() {
        RollingStats stats = new RollingStats(10);
        for (double value : new double[] {2, 4, 4, 4, 5, 5, 7, 9}) {
            stats.add(value);
        }
        // Média 5; soma dos quadrados dos desvios = 9+1+1+1+0+0+4+16 = 32.
        assertEquals(5.0, stats.mean(), 1e-12);
        // Desvio amostral (n-1): raiz de 32/7. O populacional daria exatamente 2.
        assertEquals(Math.sqrt(32.0 / 7.0), stats.stdDev(), 1e-12);
        assertEquals(2.0, stats.min());
        assertEquals(9.0, stats.max());
        assertEquals(9.0, stats.last());
        assertEquals(8, stats.count());
    }

    @Test
    void theOldestValueLeavesWhenTheWindowFills() {
        RollingStats stats = new RollingStats(3);
        for (double value = 1; value <= 5; value++) {
            stats.add(value);
        }
        // Ficam 3, 4 e 5.
        assertEquals(3, stats.count());
        assertEquals(4.0, stats.mean(), 1e-12);
        assertEquals(1.0, stats.stdDev(), 1e-12);
        assertEquals(3.0, stats.min());
    }

    @Test
    void nonFiniteValuesAreIgnored() {
        RollingStats stats = new RollingStats(5);
        stats.add(10.0);
        stats.add(Double.NaN);
        stats.add(Double.POSITIVE_INFINITY);
        stats.add(20.0);
        assertEquals(2, stats.count());
        assertEquals(15.0, stats.mean(), 1e-12);
    }

    @Test
    void notEnoughDataGivesNaNInsteadOfAFakeZero() {
        RollingStats stats = new RollingStats(5);
        assertTrue(Double.isNaN(stats.mean()));
        assertTrue(Double.isNaN(stats.last()));
        assertTrue(Double.isNaN(stats.min()));
        stats.add(3.0);
        assertEquals(3.0, stats.mean());
        assertTrue(Double.isNaN(stats.stdDev()), "um valor só não tem desvio");
    }

    @Test
    void windowSmallerThanOneIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> new RollingStats(0));
    }
}
