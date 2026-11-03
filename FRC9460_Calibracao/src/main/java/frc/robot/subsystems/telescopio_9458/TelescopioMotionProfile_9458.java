package frc.robot.subsystems.telescopio_9458;

// #region IMPORTS

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import org.littletonrobotics.junction.AutoLog;

// #endregion

/**
 * Perfil de movimento com jerk limitado (curva em S): velocidade, aceleração e jerk
 * máximos, o mesmo tipo de perfil que o Motion Magic gera dentro do TalonFX.
 *
 * Usos:
 * - subsistema: calcula onde a referência consegue parar (ao soltar o gatilho) e
 *   quando precisa começar a frear para entrar no trecho lento perto dos batentes;
 * - simulação: emula o Motion Magic, para o PC se comportar como o Talon.
 *
 * Unidades: metros da ponta, m/s, m/s² e m/s³.
 */
public final class TelescopioMotionProfile_9458 {

    // #region TIPOS

    /** Limites do perfil. */
    public record Limits(double velocity, double acceleration, double jerk) {
        public Limits {
            if (!Double.isFinite(velocity) || !Double.isFinite(acceleration) || !Double.isFinite(jerk)
                    || velocity <= 0 || acceleration <= 0 || jerk <= 0) {
                throw new IllegalArgumentException("Limites do perfil devem ser finitos e positivos");
            }
        }
    }

    /** Referência do perfil: posição, velocidade e aceleração. */
    public record State(double position, double velocity, double acceleration) {}

    /** Limites efetivamente usados, gravados no log para o replay (como os ganhos). */
    @AutoLog
    public static class TelescopioProfileInputs {
        public double velocity, acceleration, jerk;

        public void set(Limits limits) {
            velocity = limits.velocity();
            acceleration = limits.acceleration();
            jerk = limits.jerk();
        }

        public Limits get() {
            return new Limits(velocity, acceleration, jerk);
        }
    }

    // #endregion

    // #region CONSTANTES NUMÉRICAS

    // Passo da conta da distância de frenagem (0,5 ms: erro < 0,1 mm a 0,9 m/s).
    private static final double BRAKING_STEP_SECONDS = 0.0005;
    private static final int MAX_BRAKING_STEPS = 20_000;

    // Abaixo disto a referência é considerada parada no alvo.
    private static final double SETTLED_METERS = 1e-4;
    private static final double SETTLED_METERS_PER_SEC = 2e-3;

    // Folga numérica para "não dá para parar antes do alvo" (o TrapezoidProfile mantém
    // a rampa de frenagem exata; a folga só evita disparar por arredondamento).
    private static final double OVERRUN_TOLERANCE_METERS = 1e-7;

    // #endregion

    private TelescopioMotionProfile_9458() {
    }

    // #region 🔧 DISTÂNCIA DE FRENAGEM

    /**
     * Distância para reduzir a velocidade de {@code speed} até {@code finalSpeed}, partindo
     * da aceleração {@code acceleration} (positiva = acelerando no sentido do movimento),
     * com aceleração e jerk limitados e terminando com aceleração zero.
     * Velocidades no sentido do movimento (speed ≥ 0).
     */
    public static double brakingDistance(double speed, double acceleration, double finalSpeed, Limits limits) {

        double v = speed;
        double a = acceleration;
        double distance = 0;
        double jerk = limits.jerk();

        for (int i = 0; i < MAX_BRAKING_STEPS && v > finalSpeed; i++) {
            // Freia com jerk máximo; solta o freio a tempo de chegar com aceleração zero.
            boolean release = a < 0 && v - finalSpeed <= a * a / (2 * jerk);
            double next = release
                    ? Math.min(0, a + jerk * BRAKING_STEP_SECONDS)
                    : Math.max(-limits.acceleration(), a - jerk * BRAKING_STEP_SECONDS);
            double nextV = v + (a + next) / 2 * BRAKING_STEP_SECONDS;
            distance += (v + Math.max(nextV, finalSpeed)) / 2 * BRAKING_STEP_SECONDS;
            v = nextV;
            a = next;
        }
        return distance;
    }

    /** Distância até parar (velocidade final zero). */
    public static double stoppingDistance(double speed, double acceleration, Limits limits) {
        return brakingDistance(speed, acceleration, 0, limits);
    }

    /**
     * Distância até parar no {@link Generator} (média móvel de janela W = A/J): só o
     * trapezoide interno, que anda à frente da saída, precisa frear (v_i²/2A).
     *
     * Acelerando, a saída está na rampa de partida (o interno saltou para A há a·W/A s):
     * v_i = v + a·W − a²·W/2A e o interno está v·W/2 + a²·W²/4A − a³·W²/6A² à frente.
     * Com a = A são os valores da janela linear (v + A·W/2 e v·W/2 + A·W²/12); perto do
     * cruzeiro a conta sobra, nunca falta. Freando, a janela não é linear (o interno já
     * chegou ao cruzeiro menor): conta como velocidade constante, também com sobra.
     * Abaixo de A²/J a média móvel para mais longe que o perfil ótimo.
     */
    public static double filteredStoppingDistance(double speed, double acceleration, Limits limits) {
        double a = Math.min(Math.max(acceleration, 0), limits.acceleration());
        double maxAcceleration = limits.acceleration();
        double window = maxAcceleration / limits.jerk();
        double internalSpeed = Math.min(Math.max(speed, limits.velocity()),
                speed + a * window - a * a * window / (2 * maxAcceleration));
        double lead = speed * window / 2 + a * a * window * window / (4 * maxAcceleration)
                - a * a * a * window * window / (6 * maxAcceleration * maxAcceleration);
        return lead + internalSpeed * internalSpeed / (2 * maxAcceleration);
    }

    /**
     * Distância da referência lida até onde ela consegue parar ao soltar o gatilho.
     *
     * A meta nova só chega ao Talon depois de {@code delaySeconds}; até lá a referência
     * segue o pedido anterior (cruzeiro {@code cruise}). A estimativa nunca fica curta:
     * - no atraso, a aceleração pode crescer com jerk máximo (inclusive partindo do
     *   repouso) e a velocidade não cai abaixo do cruzeiro atual;
     * - a parada usa o maior entre o perfil ótimo e o da média móvel, para não depender
     *   de como o Talon monta a curva em S.
     * Meta além do necessário só prolonga o movimento por alguns milímetros; aquém, a
     * referência passaria da meta e voltaria.
     */
    public static double releaseDistance(double speed, double acceleration, double cruise,
            double delaySeconds, Limits limits) {

        double nextAcceleration = Math.min(limits.acceleration(), acceleration + limits.jerk() * delaySeconds);
        double top = Math.max(speed, cruise);
        double bottom = Math.min(speed, cruise);
        double nextSpeed = Math.max(bottom,
                Math.min(top, speed + (acceleration + nextAcceleration) / 2 * delaySeconds));
        if (nextSpeed >= top || nextSpeed <= bottom) {
            nextAcceleration = 0;
        }
        double travel = Math.max(speed, nextSpeed) * delaySeconds;
        return travel + Math.max(
                stoppingDistance(nextSpeed, nextAcceleration, limits),
                filteredStoppingDistance(nextSpeed, nextAcceleration, limits));
    }

    // #endregion

    // #region 🎯🚀 GERADOR DE REFERÊNCIA (EMULA O MOTION MAGIC)

    /**
     * Emula o Motion Magic com jerk: um perfil trapezoidal (velocidade e aceleração) cuja
     * velocidade passa por uma média móvel de A/J segundos. A média transforma as quinas
     * do trapezoide em rampas de aceleração com jerk J (curva em S), preserva a distância
     * percorrida e termina exatamente no alvo, sem oscilar.
     *
     * O trapezoide interno anda à frente da saída (v·A/2J em cruzeiro); a distância até
     * parar é {@link #filteredStoppingDistance}. Alvo novo perto demais para parar: freia
     * com A, passa do alvo e volta, como o Talon. Nunca acelera além de A.
     */
    public static final class Generator {

        private final double dtSeconds;
        private double[] window = new double[1];
        private int index;
        private double sum;
        private TrapezoidProfile.State internal = new TrapezoidProfile.State();
        private State reference = new State(0, 0, 0);

        public Generator(double dtSeconds) {
            this.dtSeconds = dtSeconds;
        }

        /** Começa da posição e da velocidade medidas, como o Talon ao receber o pedido. */
        public void reset(double position, double velocity, Limits limits) {
            int samples = Math.max(1, (int) Math.round(limits.acceleration() / limits.jerk() / dtSeconds));
            window = new double[samples];
            java.util.Arrays.fill(window, velocity);
            index = 0;
            sum = velocity * samples;
            // A saída filtrada fica (n − 1)·dt/2 atrás do trapezoide interno.
            internal = new TrapezoidProfile.State(
                    position + velocity * (samples - 1) * dtSeconds / 2, velocity);
            reference = new State(position, velocity, 0);
        }

        public State reference() {
            return reference;
        }

        /** Um passo de dtSeconds rumo ao alvo (parado), com os limites atuais. */
        public State step(double target, Limits limits) {

            double v = internal.velocity;
            double toTarget = target - internal.position;
            // Alvo à frente mais perto que a frenagem: o TrapezoidProfile da WPILib saltaria
            // a velocidade (aceleração acima de A) para parar nele. O Talon não faz isso.
            boolean overrun = v * toTarget > 0
                    && v * v / (2 * limits.acceleration()) > Math.abs(toTarget) + OVERRUN_TOLERANCE_METERS;
            if (overrun || Math.abs(v) > limits.velocity() + 1e-9) {
                // Passa do alvo, ou cruzeiro reduzido (trecho lento): freia o trapezoide
                // com a aceleração máxima, sem salto de velocidade.
                double floor = overrun ? 0 : limits.velocity();
                double next = v - Math.signum(v)
                        * Math.min(Math.abs(v) - floor, limits.acceleration() * dtSeconds);
                internal = new TrapezoidProfile.State(internal.position + (v + next) / 2 * dtSeconds, next);
            } else {
                internal = new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(limits.velocity(), limits.acceleration()))
                        .calculate(dtSeconds, internal, new TrapezoidProfile.State(target, 0));
            }

            // Média móvel da velocidade (soma corrente).
            sum += internal.velocity - window[index];
            window[index] = internal.velocity;
            index = (index + 1) % window.length;
            double velocity = sum / window.length;

            double position = reference.position() + (reference.velocity() + velocity) / 2 * dtSeconds;
            double acceleration = (velocity - reference.velocity()) / dtSeconds;

            // Filtro vazio e trapezoide parado no alvo: elimina o resíduo numérico da integração.
            if (Math.abs(velocity) <= 1e-12 && internal.velocity == 0
                    && Math.abs(internal.position - target) <= SETTLED_METERS
                    && Math.abs(position - target) <= SETTLED_METERS) {
                position = target;
                velocity = 0;
                acceleration = 0;
            }
            reference = new State(position, velocity, acceleration);
            return reference;
        }
    }

    // #endregion
}
