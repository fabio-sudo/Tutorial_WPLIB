package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.util.SparkUtil;

// #endregion

/**
 * Rodas reais com Spark Flex: top (dois motores) e tower (um motor).
 * Configuração igual à da competição: Coast, rampa, compensação de tensão,
 * limite de corrente e controle de velocidade dentro do Spark.
 */
public class ShooterWheelIOSparkFlex implements ShooterWheelIO {

    // #region MOTORES

    private final SparkFlex[] motors;
    private final RelativeEncoder[] encoders;
    private final SparkClosedLoopController[] controllers;
    private final boolean configured;

    // #endregion

    // #region CONFIGURAÇÃO

    /** ids e inverted na mesma ordem: um item por motor. */
    public ShooterWheelIOSparkFlex(int[] ids, boolean[] inverted) {
        if (ids.length == 0 || ids.length != inverted.length) {
            throw new IllegalArgumentException("Informe um sentido para cada motor");
        }
        motors = new SparkFlex[ids.length];
        encoders = new RelativeEncoder[ids.length];
        controllers = new SparkClosedLoopController[ids.length];
        boolean allOK = true;
        for (int i = 0; i < ids.length; i++) {
            motors[i] = new SparkFlex(ids[i], MotorType.kBrushless);
            SparkFlexConfig config = createConfig(inverted[i]);
            REVLibError result = motors[i].configure(config,
                ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            allOK &= result == REVLibError.kOk;
            // 🛑 Grava no Spark: sentido, corrente e PID sobrevivem a uma queda de tensão.
            if (result == REVLibError.kOk) {
                SparkUtil.persist(motors[i], config);
            }
            SparkUtil.clearStartupReset(motors[i]);
            encoders[i] = motors[i].getEncoder();
            controllers[i] = motors[i].getClosedLoopController();
        }
        configured = allOK;
    }

    /** Configuração da competição, separada por assunto. */
    static SparkFlexConfig createConfig(boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();

        // #region MOTOR, SENTIDO E ENCODER

        // Coast: ao parar, a roda gira livre até perder velocidade.
        config
            .inverted(inverted)
            .idleMode(IdleMode.kCoast);
        // Fator 1: o encoder já informa RPM do motor.
        config.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

        // #endregion

        // #region 🛑⚡ SEGURANÇA - LIMITE DE CORRENTE

        config.smartCurrentLimit(SPARK_CURRENT_LIMIT_AMPS);

        // #endregion

        // #region 🎯🚀 CONTROLE - PID DE VELOCIDADE, kV E RAMPA

        config
            .closedLoopRampRate(SPARK_CLOSED_LOOP_RAMP_SECONDS)
            .voltageCompensation(NOMINAL_VOLTAGE);
        config.closedLoop
            .p(SPARK_KP)
            .i(0.0)
            .d(0.0)
            .outputRange(-1.0, 1.0);
        config.closedLoop.feedForward
            .kV(SPARK_KV_VOLTS_PER_RPM);

        // #endregion

        return config;
    }

    // #endregion

    // #region LEITURAS

    @Override
    public void updateInputs(ShooterWheelIOInputs inputs) {
        int count = motors.length;
        if (inputs.velocityRpm.length != count) {
            inputs.velocityRpm = new double[count];
            inputs.appliedVolts = new double[count];
            inputs.currentAmps = new double[count];
        }
        boolean allOK = configured;
        for (int i = 0; i < count; i++) {
            SparkUtil.reportIfReset(motors[i]);
            inputs.velocityRpm[i] = encoders[i].getVelocity();
            inputs.appliedVolts[i] = motors[i].getAppliedOutput() * motors[i].getBusVoltage();
            inputs.currentAmps[i] = motors[i].getOutputCurrent();
            allOK &= motors[i].getLastError() == REVLibError.kOk;
        }
        inputs.connected = allOK;
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setVelocityRpm(double rpm) {
        if (!Double.isFinite(rpm)) {
            stop();
            return;
        }
        for (SparkClosedLoopController controller : controllers) {
            controller.setSetpoint(rpm, ControlType.kVelocity);
        }
    }

    @Override
    public void stop() {
        for (SparkFlex motor : motors) {
            motor.stopMotor();
        }
    }

    // #endregion
}
