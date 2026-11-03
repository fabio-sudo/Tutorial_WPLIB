package frc.robot.util;

// #region IMPORTS

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.DriverStation;

// #endregion

/**
 * Ajudas do Phoenix 6 para os IOs reais dos mecanismos.
 *
 * Na partida do robô o CAN está cheio (22 dispositivos) e uma configuração
 * pode expirar. Por isso cada configuração tenta algumas vezes. Se ainda
 * falhar, o mecanismo fica parado por segurança e a mensagem aparece na
 * Driver Station, mesmo sem AdvantageScope conectado.
 */
public final class PhoenixUtil {

    // #region TENTATIVAS

    public static final int CONFIG_ATTEMPTS = 5;

    /** Repete até dar certo ou acabar as tentativas; devolve o último resultado. */
    public static StatusCode tryUntilOk(Supplier<StatusCode> action) {
        return tryUntilOk(action, CONFIG_ATTEMPTS);
    }

    private static StatusCode tryUntilOk(Supplier<StatusCode> action, int attempts) {
        StatusCode result = action.get();
        for (int attempt = 1; attempt < attempts && !result.isOK(); attempt++) {
            result = action.get();
        }
        return result;
    }

    // #endregion

    // #region TRÁFEGO DO CAN

    // 🛑 Interruptor de emergência. A otimização fica gravada no dispositivo até ele
    // reiniciar, então só apagar a chamada não basta: com false, o próximo deploy
    // devolve as frequências padrão sem precisar desligar o robô.
    public static final boolean OPTIMIZE_CAN_BUS = true;

    // Sinal que o código não usa cai para 4 Hz, nunca para 0 Hz (desligado): o
    // hasResetOccurred, o Tuner X e o log do Phoenix continuam funcionando.
    public static final double UNUSED_SIGNAL_HZ = 4.0;

    // Cada tentativa espera até 100 ms por sinal: com um dispositivo fora do CAN,
    // tentar mais só atrasa a partida do robô.
    private static final int SIGNAL_RATE_ATTEMPTS = 2;

    /**
     * Pede a frequência dos sinais que o código usa e desacelera o resto.
     *
     * A ordem é a segurança: se pedir os sinais usados falhar, o resto não é
     * desacelerado, porque um sinal usado poderia cair para 4 Hz junto. Nesse
     * caso, e se desacelerar falhar, o mecanismo continua normal, com o tráfego
     * padrão do Phoenix: é só um aviso, não uma falha de configuração.
     *
     * @return o resultado dos sinais usados, o único que importa ao mecanismo.
     */
    public static StatusCode applySignalRates(
            String what, Supplier<StatusCode> setUsedRates, ParentDevice... devices) {
        return applySignalRates(what, OPTIMIZE_CAN_BUS, setUsedRates,
            () -> ParentDevice.optimizeBusUtilizationForAll(UNUSED_SIGNAL_HZ, devices),
            () -> ParentDevice.resetSignalFrequenciesForAll(devices));
    }

    static StatusCode applySignalRates(String what, boolean optimize, Supplier<StatusCode> setUsedRates,
            Supplier<StatusCode> slowTheRest, Supplier<StatusCode> restoreDefaults) {
        if (!optimize) {
            // Antes de pedir os usados: restaurar apaga também as frequências pedidas.
            reportTrafficWarning(tryUntilOk(restoreDefaults, SIGNAL_RATE_ATTEMPTS), what, "restaurar as frequências");
            return tryUntilOk(setUsedRates, SIGNAL_RATE_ATTEMPTS);
        }
        StatusCode used = tryUntilOk(setUsedRates, SIGNAL_RATE_ATTEMPTS);
        reportTrafficWarning(used, what, "pedir a frequência dos sinais usados");
        if (used.isOK()) {
            reportTrafficWarning(tryUntilOk(slowTheRest, SIGNAL_RATE_ATTEMPTS), what, "desacelerar os sinais sem uso");
        }
        return used;
    }

    private static void reportTrafficWarning(StatusCode result, String what, String step) {
        if (!result.isOK()) {
            DriverStation.reportWarning("[FRC9460] CAN: " + what + ": " + step + " falhou (" + result
                + "). O mecanismo segue normal, com o tráfego padrão.", false);
        }
    }

    // #endregion

    // #region 📊 MENSAGENS NA DRIVER STATION

    /** Falhou mesmo depois das tentativas: avisa como erro. */
    public static void reportIfFailed(StatusCode result, String what) {
        if (!result.isOK()) {
            DriverStation.reportError("[FRC9460] " + what + " falhou (" + result
                + "). Mecanismo parado por segurança.", false);
        }
    }

    /**
     * Falhou mesmo depois das tentativas, mas o dispositivo segue com a última
     * configuração gravada nele. Vale para o swerve: parar um módulo no meio da
     * partida seria pior que rodar com a configuração anterior.
     */
    public static void reportKeptRunning(StatusCode result, String what) {
        if (!result.isOK()) {
            DriverStation.reportError("[FRC9460] " + what + " falhou (" + result
                + "). Segue com a última configuração gravada nele: conferir antes de jogar.", false);
        }
    }

    /** O controlador reiniciou durante o uso: o zero manual se perdeu. */
    public static void reportReset(String what) {
        DriverStation.reportWarning("[FRC9460] " + what
            + " reiniciou: zero perdido, mecanismo parado. Desabilite e zere de novo.", false);
    }

    // #endregion

    private PhoenixUtil() {}
}
