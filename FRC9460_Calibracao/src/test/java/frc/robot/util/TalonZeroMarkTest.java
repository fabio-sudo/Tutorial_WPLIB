package frc.robot.util;

import static frc.robot.util.TalonZeroMark.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Zero que sobrevive ao deploy: decide pela posição lida no Talon ao iniciar.
 * Não prova o comportamento do Talon real (ligar dentro de uma volta do rotor);
 * isso se confere no Tuner X depois de ligar o robô.
 */
class TalonZeroMarkTest {
    private static final double PIVOT_MAX = ShooterConstants.PIVOT_MAX_MOTOR_ROTATIONS;

    private static Decision pivot(double talonRotations) {
        return onStartup(talonRotations, -PIVOT_MAX, PIVOT_MAX);
    }

    private static Decision intake(double talonRotations) {
        return onStartup(talonRotations, IntakeConstants.MECHANICAL_OPEN_ROTATIONS,
            IntakeConstants.CLOSED_ROTATIONS);
    }

    @Test void talonJustPoweredOnZerosWhereTheMechanismIs() {
        // Ligado agora: dentro de uma volta do rotor, qualquer sinal.
        assertEquals(Decision.ZERAR_AQUI, pivot(0.0));
        assertEquals(Decision.ZERAR_AQUI, pivot(0.92));
        assertEquals(Decision.ZERAR_AQUI, pivot(-0.085));
        assertEquals(Decision.ZERAR_AQUI, intake(0.4));
    }

    @Test void codeRestartKeepsTheZeroWithTheMechanismAnywhereInItsCourse() {
        // O caso que antes mudava o centro: deploy com o shooter virado.
        assertEquals(Decision.MANTER_ZERO, pivot(toTalon(PIVOT_MAX)));
        assertEquals(Decision.MANTER_ZERO, pivot(toTalon(-PIVOT_MAX)));
        assertEquals(Decision.MANTER_ZERO, pivot(toTalon(0.3)));
        // Deploy com o intake aberto.
        assertEquals(Decision.MANTER_ZERO, intake(toTalon(IntakeConstants.OPEN_ROTATIONS)));
        assertEquals(Decision.MANTER_ZERO, intake(toTalon(-10.0)));
    }

    @Test void meaninglessPositionsAreNeverTrusted() {
        assertEquals(Decision.POSICAO_DESCONHECIDA, pivot(Double.NaN));
        // Zero antigo (sem marca) com o shooter virado 74°: não dá para saber o centro.
        assertEquals(Decision.POSICAO_DESCONHECIDA, pivot(5.0));
        // Muito além do curso, mesmo contando a folga.
        assertEquals(Decision.POSICAO_DESCONHECIDA, pivot(toTalon(PIVOT_MAX + TRAVEL_MARGIN_ROTATIONS + 0.5)));
        assertEquals(Decision.POSICAO_DESCONHECIDA, intake(toTalon(10.0)));
    }

    @Test void freshBootWindowNeverOverlapsAKeptZero() {
        // A menor posição que mantém o zero fica longe da janela de Talon recém-ligado.
        double pivotLowest = toTalon(-PIVOT_MAX - TRAVEL_MARGIN_ROTATIONS);
        double intakeLowest = toTalon(IntakeConstants.MECHANICAL_OPEN_ROTATIONS - TRAVEL_MARGIN_ROTATIONS);
        assertTrue(pivotLowest > FRESH_BOOT_WINDOW_ROTATIONS + 10.0, "pivô: " + pivotLowest);
        assertTrue(intakeLowest > FRESH_BOOT_WINDOW_ROTATIONS + 10.0, "intake: " + intakeLowest);
    }

    @Test void conversionsAreExactInverses() {
        for (double rotations : new double[] {-22.205078, -8.843920681818181, 0.0, 0.051, 8.843920681818181}) {
            assertEquals(rotations, toMechanism(toTalon(rotations)), 1e-12);
        }
        assertEquals(100.0, toTalon(0.0), "no Tuner X o zero aparece como 100");
    }
}
