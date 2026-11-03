package frc.robot.util;

import static frc.robot.util.MechanismDiagnostics.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** O diagnóstico mostra primeiro o problema mais grave. */
class MechanismDiagnosticsTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    @AfterEach void cleanup() {
        scheduler.cancelAll();
        scheduler.unregisterAllSubsystems();
    }

    @Test void mostSevereProblemWins() {
        assertEquals(MOTOR_SEM_RESPOSTA, result(false, false, true), "sem motor vem antes de tudo");
        assertEquals(ZERO_PERDIDO, result(true, false, true), "sem zero vem antes do modelo");
        assertEquals(REAL_DIFERENTE_DO_MODELO, result(true, true, true));
        assertEquals(OK, result(true, true, false));
    }

    @Test void onlyProblemsThatNeedAnActionGoToTheDriverStation() {
        assertTrue(shouldReport(MOTOR_SEM_RESPOSTA, OK));
        assertTrue(shouldReport(ZERO_PERDIDO, OK));
        assertFalse(shouldReport(MOTOR_SEM_RESPOSTA, MOTOR_SEM_RESPOSTA), "uma mensagem por problema");
        assertFalse(shouldReport(OK, MOTOR_SEM_RESPOSTA));
        // Eram mais de 100 por sessão: seguem no log e no Elastic.
        assertFalse(shouldReport(REAL_DIFERENTE_DO_MODELO, OK));
    }

    @Test void theZeroTextNeverSaysDoneWithAnInvalidZero() {
        assertEquals("ZERADO_PELA_EQUIPE", zeroResult(true, "ZERADO_PELA_EQUIPE"));
        assertEquals(ZERO_PERDIDO, zeroResult(false, "ZERADO_PELA_EQUIPE"), "zerar falhou");
        assertEquals(ZERO_PERDIDO, zeroResult(false, "ZERADO_AO_LIGAR"), "posição desconhecida ou Talon reiniciado");
        assertEquals("RECUSADO_ROBO_HABILITADO", zeroResult(false, "RECUSADO_ROBO_HABILITADO"),
            "a recusa continua visível");
    }

    @Test void commandNameShowsWhoIsUsingTheSubsystem() {
        SubsystemBase subsystem = new SubsystemBase() {};
        assertEquals("NENHUM", commandName(subsystem));
        // ignoringDisable: no teste a Driver Station simulada está desabilitada.
        var command = subsystem.run(() -> {}).ignoringDisable(true).withName("Teste - segurando");
        scheduler.schedule(command);
        assertEquals("Teste - segurando", commandName(subsystem));
        scheduler.cancel(command);
        assertEquals("NENHUM", commandName(subsystem));
    }
}
