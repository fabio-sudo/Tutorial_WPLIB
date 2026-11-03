package frc.robot.util;

// #region IMPORTS

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.DriverStation;

// #endregion

/**
 * Ajudas da REVLib para os IOs reais com Spark Flex.
 *
 * O Spark só guarda o que for pedido. Sem gravar, um reinício por queda de
 * tensão volta ao que estiver salvo nele: sentido, limite de corrente e PID. No
 * top, um motor com o sentido errado briga com o outro no mesmo eixo.
 */
public final class SparkUtil {

    // #region 🛑 GRAVAR A CONFIGURAÇÃO

    /**
     * Grava a configuração já aplicada. É um passo separado de propósito: uma
     * falha aqui só avisa, e o mecanismo segue com o que foi aplicado. Junto com a
     * configuração, um tempo esgotado ao gravar deixaria o shooter sem atirar.
     */
    public static void persist(SparkBase spark, SparkBaseConfig config) {
        String result;
        try {
            result = spark.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters).name();
        } catch (IllegalStateException failure) {
            // A REVLib lança exceção nos erros que ela considera graves.
            result = failure.getMessage();
        }
        if (!REVLibError.kOk.name().equals(result)) {
            DriverStation.reportWarning("[FRC9460] Spark Flex " + spark.getDeviceId()
                + ": gravar a configuração falhou (" + result
                + "). Ele segue normal até reiniciar.", false);
        }
    }

    // #endregion

    // #region 🛑 REINÍCIO DO SPARK

    /** Descarta o aviso de reinício da própria ligação; reinícios futuros contam. */
    public static void clearStartupReset(SparkBase spark) {
        spark.clearFaults();
    }

    /** O Spark reiniciou durante o uso: avisa uma vez por reinício. */
    public static void reportIfReset(SparkBase spark) {
        if (spark.getStickyWarnings().hasReset) {
            spark.clearFaults();
            DriverStation.reportWarning("[FRC9460] Spark Flex " + spark.getDeviceId()
                + " reiniciou. Conferir bateria e fiação: voltou com a configuração gravada nele.", false);
        }
    }

    // #endregion

    private SparkUtil() {}
}
