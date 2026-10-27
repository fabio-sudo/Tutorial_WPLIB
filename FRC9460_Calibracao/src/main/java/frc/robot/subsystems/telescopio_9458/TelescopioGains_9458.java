package frc.robot.subsystems.telescopio_9458;

import org.littletonrobotics.junction.AutoLog;

/** Ganhos em volts e metros da ponta, não em voltas do motor. */
public record TelescopioGains_9458(
        double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    public TelescopioGains_9458 {
        for (double value : new double[] {kP, kI, kD, kS, kG, kV, kA}) {
            if (!Double.isFinite(value) || value < 0) {
                throw new IllegalArgumentException("Ganhos devem ser finitos e nao negativos");
            }
        }
    }

    public double feedforward(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kG + kV * velocity + kA * acceleration;
    }

    /** Os parâmetros efetivamente usados são entradas do replay, inclusive após ajuste no painel. */
    @AutoLog
    public static class TelescopioGainsInputs {
        public double kP, kI, kD, kS, kG, kV, kA;

        public void set(TelescopioGains_9458 gains) {
            kP = gains.kP();
            kI = gains.kI();
            kD = gains.kD();
            kS = gains.kS();
            kG = gains.kG();
            kV = gains.kV();
            kA = gains.kA();
        }

        public TelescopioGains_9458 get() {
            return new TelescopioGains_9458(kP, kI, kD, kS, kG, kV, kA);
        }
    }
}
