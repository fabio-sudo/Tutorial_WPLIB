package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

/**
 * Tempo para o pivô mirar, na simulação (Motion Magic emulado).
 *
 * Teste de 29/10/2026: "o pivô tem que ficar mais rápido para mirar". Sem kV
 * o Talon só andava pelo erro (kP): ficava graus atrás do perfil e depois
 * rastejava até a tolerância. Mede o tempo até entrar e FICAR na tolerância
 * de mira (0,75°) e o quanto passa do alvo.
 */
class ShooterPivotAimSpeedTest {

    private static final double LOOP_SECONDS = 0.02;

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** Resultado de um movimento: tempo até ficar na tolerância e quanto passou do alvo. */
    record Move(double settleSeconds, double overshootDegrees) {}

    static Move move(double fromDegrees, double toDegrees) {
        var io = new ShooterPivotIOSim(0.0);
        var inputs = new ShooterPivotIOInputsAutoLogged();
        // Leva o pivô ao ponto de partida e espera parar.
        io.setTargetRotations(ShotCalculator.pivotDegreesToMotorRotations(fromDegrees));
        for (int i = 0; i < 200; i++) io.updateInputs(inputs);

        double target = ShotCalculator.pivotDegreesToMotorRotations(toDegrees);
        double direction = Math.signum(toDegrees - fromDegrees);
        io.setTargetRotations(target);
        double settled = Double.NaN;
        double overshoot = 0.0;
        for (int i = 1; i <= 250; i++) {
            io.updateInputs(inputs);
            double errorRotations = inputs.motorPositionRotations - target;
            overshoot = Math.max(overshoot, direction * errorRotations * PIVOT_DEGREES_PER_MOTOR_ROTATION);
            boolean inside = Math.abs(errorRotations) <= PIVOT_TOLERANCE_MOTOR_ROTATIONS;
            if (inside && Double.isNaN(settled)) settled = i * LOOP_SECONDS;
            if (!inside) settled = Double.NaN;
        }
        return new Move(settled, overshoot);
    }

    // Medido na simulação em 29/10/2026 (antes -> depois do kV e do perfil):
    //   30°: 0,72 -> 0,30 s | 90°: 1,30 -> 0,54 s | 180°: 2,12 -> 0,88 s
    // Os limites deixam folga de ~25% sobre o medido.
    @Test
    void aimsQuicklyAndWithoutOvershootingTheTolerance() {
        double[][] moves = {{0, 30, 0.40}, {0, 90, 0.70}, {-90, 90, 1.10}, {-120, 120, 1.40}};
        for (double[] m : moves) {
            Move result = move(m[0], m[1]);
            System.out.printf("PIVO %5.0f -> %5.0f graus: mira em %.2f s, passou %.2f graus%n",
                m[0], m[1], result.settleSeconds(), result.overshootDegrees());
            assertTrue(result.settleSeconds() <= m[2],
                "pivô levou " + result.settleSeconds() + " s para " + (m[1] - m[0]) + "°");
            // Passar do alvo menos que a tolerância: não sai e volta da mira.
            assertTrue(result.overshootDegrees() < PIVOT_TOLERANCE_MOTOR_ROTATIONS * PIVOT_DEGREES_PER_MOTOR_ROTATION,
                "pivô passou " + result.overshootDegrees() + "° do alvo");
        }
    }

    @Test
    void neutralBandSwallowsTheKsAndEndsInsideTheAimTolerance() {
        double bandVolts = PIVOT_NEUTRAL_DEADBAND * NOMINAL_VOLTAGE;
        // Maior que o kS: +kS e -kS com erro perto de zero viram neutro, sem tremer.
        assertTrue(bandVolts > PIVOT_KS_VOLTS);
        // O pivô para dentro da banda: ela precisa terminar antes da tolerância.
        assertTrue((bandVolts - PIVOT_KS_VOLTS) / PIVOT_KP < PIVOT_TOLERANCE_MOTOR_ROTATIONS);
        // Acima do atrito medido (3%): fora da banda o motor se move, não zumbe parado.
        assertTrue(PIVOT_NEUTRAL_DEADBAND > 0.03);
    }

    @Test
    void afterEachSmallAimChangeItStopsAndStaysAtZeroVolts() {
        // Botão de mirar segurado: a pose muda a mira só um pouco (acima do reenvio
        // de 0,3°, que o Shooter filtra). O pivô chega e fica sem tensão, sem caçar.
        var io = new ShooterPivotIOSim(0.0);
        var inputs = new ShooterPivotIOInputsAutoLogged();
        for (double aimDegrees : new double[] {45.0, 45.35, 44.95, 45.4, 45.0}) {
            io.setTargetRotations(ShotCalculator.pivotDegreesToMotorRotations(aimDegrees));
            for (int i = 0; i < 100; i++) io.updateInputs(inputs);
            for (int i = 0; i < 200; i++) {
                io.updateInputs(inputs);
                assertEquals(0.0, inputs.appliedVolts, "mira " + aimDegrees + ", ciclo " + i);
            }
            double errorDegrees = ShotCalculator.motorRotationsToPivotDegrees(inputs.motorPositionRotations) - aimDegrees;
            assertTrue(Math.abs(errorDegrees) < PIVOT_TOLERANCE_MOTOR_ROTATIONS * PIVOT_DEGREES_PER_MOTOR_ROTATION,
                "parou fora da mira: " + errorDegrees);
        }
    }

    @Test
    void retargetThresholdIsBelowTheAimTolerance() {
        // Com o limiar acima da tolerância, o alvo enviado ao Talon ficava
        // até 1,2° atrás da mira certa e o pivô parava no lugar errado.
        assertTrue(PIVOT_RETARGET_MIN_MOTOR_ROTATIONS < PIVOT_TOLERANCE_MOTOR_ROTATIONS);
    }
}
