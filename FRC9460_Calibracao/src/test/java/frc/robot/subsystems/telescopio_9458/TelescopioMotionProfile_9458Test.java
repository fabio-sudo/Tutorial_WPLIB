package frc.robot.subsystems.telescopio_9458;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.telescopio_9458.TelescopioMotionProfile_9458.Generator;
import frc.robot.subsystems.telescopio_9458.TelescopioMotionProfile_9458.Limits;
import frc.robot.subsystems.telescopio_9458.TelescopioMotionProfile_9458.State;
import org.junit.jupiter.api.Test;

/** Curva em S (jerk limitado): distância de frenagem e gerador que emula o Motion Magic. */
class TelescopioMotionProfile_9458Test {
    private static final Limits LIMITS = new Limits(0.75, 2.5, 25);
    private static final double DT = 0.001;

    @Test void stoppingDistanceMatchesTheClosedForm() {
        // v ≥ A²/J: rampa de jerk, aceleração constante, rampa de jerk.
        double v = 0.75;
        assertEquals(v * v / (2 * 2.5) + v * 2.5 / (2 * 25), TelescopioMotionProfile_9458.stoppingDistance(v, 0, LIMITS), 1e-3);
        // v < A²/J: a aceleração não chega ao máximo (perfil triangular).
        v = 0.1;
        assertEquals(v * Math.sqrt(v / 25), TelescopioMotionProfile_9458.stoppingDistance(v, 0, LIMITS), 1e-3);
        // Acelerando no momento de soltar: precisa de mais espaço que em velocidade constante.
        assertTrue(TelescopioMotionProfile_9458.stoppingDistance(0.5, 2.0, LIMITS)
                > TelescopioMotionProfile_9458.stoppingDistance(0.5, 0, LIMITS));
        assertEquals(0, TelescopioMotionProfile_9458.stoppingDistance(0, 0, LIMITS));
    }

    @Test void referenceRespectsTheLimitsAndStopsOnTheTargetWithoutOvershoot() {
        var generator = new Generator(DT);
        generator.reset(0, 0, LIMITS);
        double target = 0.9;
        double peak = 0;
        State previous = generator.reference();
        int steps = 0;
        for (; steps < 4000 && !(previous.position() == target && previous.velocity() == 0); steps++) {
            var next = generator.step(target, LIMITS);
            assertTrue(Math.abs(next.velocity()) <= LIMITS.velocity() + 1e-9, "velocidade: " + next.velocity());
            assertTrue(Math.abs(next.acceleration()) <= LIMITS.acceleration() + 1e-6, "aceleração: " + next.acceleration());
            boolean finalSample = next.position() == target && next.velocity() == 0;
            assertTrue(finalSample || Math.abs(next.acceleration() - previous.acceleration()) <= LIMITS.jerk() * DT + 1e-6,
                    "jerk: " + (next.acceleration() - previous.acceleration()) / DT);
            peak = Math.max(peak, next.position());
            previous = next;
        }
        assertEquals(target, previous.position(), 1e-9);
        assertEquals(0, previous.velocity(), 1e-9);
        assertTrue(peak <= target + 1e-4, "passou do alvo: " + peak);
        // Trapezoide 0,9 m a 0,75 m/s e 2,5 m/s² leva 1,5 s; o jerk soma A/J = 0,1 s.
        assertEquals(1.6, steps * DT, 0.02);
    }

    @Test void releasingAtCruiseStopsAtTheComputedPointWithoutGoingBack() {
        // É o que o subsistema faz ao soltar o gatilho: alvo = referência + distância de parada.
        var generator = new Generator(DT);
        generator.reset(0, 0, LIMITS);
        for (int i = 0; i < 700; i++) generator.step(1.0, LIMITS);
        var cruising = generator.reference();
        assertEquals(0.75, cruising.velocity(), 1e-9);
        double target = cruising.position()
                + TelescopioMotionProfile_9458.stoppingDistance(cruising.velocity(), cruising.acceleration(), LIMITS);
        double previous = cruising.position();
        double highest = previous;
        for (int i = 0; i < 2000; i++) {
            var state = generator.step(target, LIMITS);
            assertTrue(state.position() >= previous - 1e-9, "não volta: " + state.position());
            previous = state.position();
            highest = Math.max(highest, previous);
        }
        assertEquals(target, previous, 1e-9);
        assertTrue(highest <= target + 1e-3, "passou do ponto de parada: " + (highest - target));
    }

    @Test void loweringTheCruiseSpeedMidwayBrakesSmoothly() {
        var generator = new Generator(DT);
        generator.reset(0, 0, LIMITS);
        for (int i = 0; i < 800; i++) generator.step(1.0, LIMITS);
        var state = generator.reference();
        assertEquals(LIMITS.velocity(), state.velocity(), 1e-9);
        var slow = new Limits(0.06, LIMITS.acceleration(), LIMITS.jerk());
        for (int i = 0; i < 500; i++) {
            var next = generator.step(1.0, slow);
            assertTrue(Math.abs(next.acceleration()) <= LIMITS.acceleration() + 1e-6, "sem frear além de A");
            assertTrue(Math.abs(next.acceleration() - state.acceleration()) <= LIMITS.jerk() * DT + 1e-6, "sem tranco");
            state = next;
        }
        assertEquals(0.06, state.velocity(), 1e-9);
    }

    @Test void newTargetBehindTheMotionStopsThenReturns() {
        var generator = new Generator(DT);
        generator.reset(0.5, 0.75, LIMITS);
        double lowest = Double.POSITIVE_INFINITY;
        double highest = 0;
        State state = generator.reference();
        for (int i = 0; i < 4000; i++) {
            state = generator.step(0.4, LIMITS);
            highest = Math.max(highest, state.position());
            lowest = Math.min(lowest, state.position());
        }
        assertEquals(0.5 + TelescopioMotionProfile_9458.stoppingDistance(0.75, 0, LIMITS), highest, 2e-3);
        assertEquals(0.4, state.position(), 1e-9);
        assertTrue(lowest >= 0.4 - 1e-4, "não passa do alvo na volta: " + lowest);
    }

    @Test void targetCloserThanTheBrakingDistanceOvershootsAndReturnsWithinTheLimits() {
        // Como o Talon: não salta a velocidade para parar num alvo impossível.
        var generator = new Generator(DT);
        generator.reset(0.2, 0.75, LIMITS);
        double target = 0.25; // precisa de ~15 cm para parar
        State previous = generator.reference();
        double highest = previous.position();
        for (int i = 0; i < 4000; i++) {
            var next = generator.step(target, LIMITS);
            assertTrue(Math.abs(next.acceleration()) <= LIMITS.acceleration() + 1e-6, "aceleração: " + next.acceleration());
            highest = Math.max(highest, next.position());
            previous = next;
        }
        assertEquals(0.2 + TelescopioMotionProfile_9458.filteredStoppingDistance(0.75, 0, LIMITS), highest, 2e-3);
        assertEquals(target, previous.position(), 1e-9);
    }

    @Test void releaseTargetCoversTheCommandDelayInEveryPhase() {
        // O subsistema lê a referência, calcula a meta e ela só vale 20 ms depois.
        // Partida, cruzeiro, frenagem para 6 cm/s e trecho lento: nunca volta.
        var slow = new Limits(0.06, LIMITS.acceleration(), LIMITS.jerk());
        int delaySteps = (int) Math.round(0.02 / DT);
        for (int release = 20; release <= 1400; release += 20) {
            var generator = new Generator(DT);
            generator.reset(0, 0, LIMITS);
            double previousVelocity = 0;
            for (int i = 0; i < release; i++) {
                if (i % delaySteps == 0) previousVelocity = generator.reference().velocity();
                generator.step(2.0, i < 700 ? LIMITS : slow);
            }
            var read = generator.reference();
            double acceleration = (read.velocity() - previousVelocity) / 0.02;
            double cruise = release <= 700 ? LIMITS.velocity() : slow.velocity();
            double target = read.position() + TelescopioMotionProfile_9458.releaseDistance(
                    read.velocity(), acceleration, cruise, 0.03, LIMITS);
            var limits = release <= 700 ? LIMITS : slow;
            for (int i = 0; i < delaySteps; i++) generator.step(2.0, limits);
            double previous = generator.reference().position();
            for (int i = 0; i < 3000; i++) {
                double position = generator.step(target, limits).position();
                assertTrue(position >= previous - 1e-9, "voltou ao soltar no passo " + release);
                previous = position;
            }
            assertEquals(target, previous, 1e-9);
        }
    }

    @Test void startingWhileMovingKeepsTheMeasuredStateContinuous() {
        var generator = new Generator(DT);
        generator.reset(0.3, 0.2, LIMITS);
        var first = generator.step(0.8, LIMITS);
        assertEquals(0.3 + 0.2 * DT, first.position(), 1e-5);
        assertEquals(0.2, first.velocity(), 0.01);
    }
}
