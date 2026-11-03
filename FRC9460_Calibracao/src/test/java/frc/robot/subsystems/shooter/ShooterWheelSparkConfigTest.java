package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkParameters;

/**
 * O que vai para cada Spark Flex do top e da tower: SEGURANÇA (corrente)
 * e CONTROLE (PID de velocidade). Os números ficam em ShooterConstants;
 * este teste impede que a configuração esqueça algum deles.
 */
class ShooterWheelSparkConfigTest {

    /** Lê os parâmetros que a REVLib vai gravar no Spark (PID e kV ficam em subpartes). */
    private static final class Reader extends SparkFlexConfig {
        Object read(BaseConfig from, SparkParameters parameter) {
            return getParameter(from, parameter.value);
        }
    }

    private static final Reader READER = new Reader();

    private static Number number(BaseConfig from, SparkParameters parameter) {
        Object value = READER.read(from, parameter);
        assertNotNull(value, parameter + " não foi configurado");
        return (Number) value;
    }

    @Test void safetyCoastAndCurrentLimit() {
        SparkFlexConfig config = ShooterWheelIOSparkFlex.createConfig(false);
        assertEquals(0, number(config, SparkParameters.kIdleMode).intValue(), "Coast = 0");
        assertEquals(SPARK_CURRENT_LIMIT_AMPS, number(config, SparkParameters.kSmartCurrentStallLimit).intValue());
    }

    @Test void velocityPidRampAndInversion() {
        SparkFlexConfig config = ShooterWheelIOSparkFlex.createConfig(true);
        assertEquals(Boolean.TRUE, READER.read(config, SparkParameters.kInverted));
        assertEquals(Boolean.FALSE, READER.read(ShooterWheelIOSparkFlex.createConfig(false), SparkParameters.kInverted));
        assertEquals((float) SPARK_KP, number(config.closedLoop, SparkParameters.kP_0).floatValue());
        assertEquals((float) SPARK_KV_VOLTS_PER_RPM,
            number(config.closedLoop.feedForward, SparkParameters.kV_0).floatValue());
        // A REVLib grava a rampa invertida: 0,25 s até o máximo = 4 por segundo.
        assertEquals((float) (1.0 / SPARK_CLOSED_LOOP_RAMP_SECONDS),
            number(config, SparkParameters.kClosedLoopRampRate).floatValue());
    }
}
