package frc.robot.subsystems.fuel;

// #region IMPORTS

import java.util.ArrayDeque;
import java.util.Deque;

// #endregion

/**
 * Estatística dos últimos N valores (janela móvel): média, desvio e extremos.
 *
 * Sem robô, sem log e sem relógio: o mesmo resultado no robô real, na
 * simulação e no replay, e testável com contas à mão. Valores NaN ou
 * infinitos são ignorados (um tiro sem leitura não entra na média).
 */
public final class RollingStats {

    // #region ESTADO

    private final int window;
    private final Deque<Double> values = new ArrayDeque<>();

    public RollingStats(int window) {
        if (window < 1) {
            throw new IllegalArgumentException("A janela precisa ter ao menos 1 valor: " + window);
        }
        this.window = window;
    }

    // #endregion

    // #region ENTRADA

    /** Acrescenta um valor; o mais antigo sai quando a janela enche. */
    public void add(double value) {
        if (!Double.isFinite(value)) {
            return;
        }
        values.addLast(value);
        if (values.size() > window) {
            values.removeFirst();
        }
    }

    // #endregion

    // #region RESULTADOS (NaN = sem dados suficientes)

    public int count() {
        return values.size();
    }

    /** Valor mais recente da janela. */
    public double last() {
        return values.isEmpty() ? Double.NaN : values.peekLast();
    }

    public double mean() {
        if (values.isEmpty()) {
            return Double.NaN;
        }
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    /**
     * Desvio padrão AMOSTRAL (divide por n-1): a janela é uma amostra dos tiros
     * que o robô daria, não todos eles. Precisa de 2 valores.
     */
    public double stdDev() {
        if (values.size() < 2) {
            return Double.NaN;
        }
        double mean = mean();
        double squares = 0.0;
        for (double value : values) {
            squares += (value - mean) * (value - mean);
        }
        return Math.sqrt(squares / (values.size() - 1));
    }

    public double min() {
        return values.stream().mapToDouble(Double::doubleValue).min().orElse(Double.NaN);
    }

    public double max() {
        return values.stream().mapToDouble(Double::doubleValue).max().orElse(Double.NaN);
    }

    // #endregion
}
