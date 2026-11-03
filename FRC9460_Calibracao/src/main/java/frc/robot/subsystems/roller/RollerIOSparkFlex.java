package frc.robot.subsystems.roller;

// #region IMPORTS

import static frc.robot.subsystems.roller.RollerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;

import frc.robot.util.SparkUtil;

// #endregion

/** Rolete real: Spark Flex no CAN 31, configuração mínima da competição. */
public class RollerIOSparkFlex implements RollerIO {

    // #region MOTOR

    private final SparkFlex motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final boolean configured;

    public RollerIOSparkFlex() {
        SparkFlexConfig config = createConfig();
        configured = motor.configure(config, ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters) == REVLibError.kOk;
        // 🛑 Grava no Spark: sentido e corrente sobrevivem a uma queda de tensão.
        if (configured) {
            SparkUtil.persist(motor, config);
        }
        SparkUtil.clearStartupReset(motor);
    }

    /** Malha aberta: não há PID, só sentido e o limite de corrente. */
    static SparkFlexConfig createConfig() {
        SparkFlexConfig config = new SparkFlexConfig();

        // #region MOTOR E SENTIDO

        config.inverted(MOTOR_INVERTED);

        // #endregion

        // #region 🛑⚡ SEGURANÇA - LIMITE DE CORRENTE

        config.smartCurrentLimit(CURRENT_LIMIT_AMPS);

        // #endregion

        return config;
    }

    // #endregion

    // #region LEITURAS E COMANDOS

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        SparkUtil.reportIfReset(motor);
        inputs.velocityRpm = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.connected = configured && motor.getLastError() == REVLibError.kOk;
    }

    @Override
    public void setPercentOutput(double percent) {
        motor.set(Double.isFinite(percent) ? MathUtil.clamp(percent, -1.0, 1.0) : 0.0);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    // #endregion
}
