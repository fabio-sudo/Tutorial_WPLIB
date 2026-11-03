package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.hal.HAL;

/** Configuração dos TalonFX com novas tentativas (partida com o CAN cheio). */
class PhoenixUtilTest {

    private final List<String> steps = new ArrayList<>();

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    private Supplier<StatusCode> step(String name, StatusCode result) {
        return () -> {
            steps.add(name);
            return result;
        };
    }

    @Test
    void asksForTheUsedSignalsBeforeSlowingTheRest() {
        StatusCode result = PhoenixUtil.applySignalRates("teste", true,
            step("usados", StatusCode.OK), step("desacelera", StatusCode.OK), step("restaura", StatusCode.OK));
        assertTrue(result.isOK());
        assertEquals(List.of("usados", "desacelera"), steps);
    }

    @Test
    void neverSlowsTheRestIfAUsedSignalWasNotConfirmed() {
        // Desacelerar agora derrubaria para 4 Hz um sinal que o código usa.
        StatusCode result = PhoenixUtil.applySignalRates("teste", true,
            step("usados", StatusCode.RxTimeout), step("desacelera", StatusCode.OK), step("restaura", StatusCode.OK));
        assertFalse(result.isOK());
        assertFalse(steps.contains("desacelera"), steps.toString());
    }

    @Test
    void failingToSlowTheRestDoesNotCountAsAMechanismFailure() {
        // O Pigeon marca configurationOk com este resultado: tráfego padrão não é defeito.
        StatusCode result = PhoenixUtil.applySignalRates("teste", true,
            step("usados", StatusCode.OK), step("desacelera", StatusCode.RxTimeout), step("restaura", StatusCode.OK));
        assertTrue(result.isOK());
    }

    @Test
    void theSwitchRestoresDefaultsBeforeAskingForTheUsedSignals() {
        // Restaurar apaga também o que foi pedido: nesta ordem, os usados ficam valendo.
        StatusCode result = PhoenixUtil.applySignalRates("teste", false,
            step("usados", StatusCode.OK), step("desacelera", StatusCode.OK), step("restaura", StatusCode.OK));
        assertTrue(result.isOK());
        assertEquals(List.of("restaura", "usados"), steps);
    }

    @Test
    void aMissingDeviceDoesNotHoldTheRobotStartup() {
        StatusCode result = PhoenixUtil.applySignalRates("teste", true,
            step("usados", StatusCode.RxTimeout), step("desacelera", StatusCode.OK), step("restaura", StatusCode.OK));
        assertFalse(result.isOK());
        assertEquals(2, steps.size(), "cada tentativa espera até 100 ms por sinal");
    }

    @Test
    void retriesUntilTheConfigurationIsAccepted() {
        int[] calls = {0};
        StatusCode result = PhoenixUtil.tryUntilOk(() -> ++calls[0] < 3 ? StatusCode.RxTimeout : StatusCode.OK);
        assertTrue(result.isOK());
        assertEquals(3, calls[0]);
    }

    @Test
    void givesUpAfterTheLastAttemptAndReturnsTheError() {
        int[] calls = {0};
        StatusCode result = PhoenixUtil.tryUntilOk(() -> {
            calls[0]++;
            return StatusCode.RxTimeout;
        });
        assertFalse(result.isOK());
        assertEquals(PhoenixUtil.CONFIG_ATTEMPTS, calls[0]);
    }

    @Test
    void doesNotRepeatWhenTheFirstAttemptWorks() {
        int[] calls = {0};
        assertTrue(PhoenixUtil.tryUntilOk(() -> {
            calls[0]++;
            return StatusCode.OK;
        }).isOK());
        assertEquals(1, calls[0]);
    }
}
