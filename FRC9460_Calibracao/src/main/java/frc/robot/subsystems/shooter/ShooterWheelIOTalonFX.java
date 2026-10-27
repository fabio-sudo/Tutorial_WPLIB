package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

// #endregion

/**
 * Centrífuga real: TalonFX no CAN 36, controle VelocityVoltage.
 * O Phoenix 6 trabalha em rotações por SEGUNDO; aqui convertemos de/para RPM.
 */
public class ShooterWheelIOTalonFX implements ShooterWheelIO {

    // #region MOTOR E SINAIS

    private static final double SIGNAL_UPDATE_HZ = 50.0;

    private final TalonFX motor = new TalonFX(CENTRIFUGE_MOTOR_ID);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<Current> current = motor.getStatorCurrent();
    private final boolean configured;

    // #endregion

    // #region CONFIGURAÇÃO

    public ShooterWheelIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = CENTRIFUGE_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CENTRIFUGE_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = CENTRIFUGE_STATOR_CURRENT_LIMIT_AMPS;

        // Tensão = kS + kV × alvo + kP × erro (alvo e erro em rot/s).
        config.Slot0.kP = CENTRIFUGE_KP;
        config.Slot0.kS = CENTRIFUGE_KS;
        config.Slot0.kV = CENTRIFUGE_KV;

        StatusCode result = motor.getConfigurator().apply(config);
        configured = result.isOK();
        BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ, velocity, voltage, current);
    }

    // #endregion

    // #region LEITURAS

    @Override
    public void updateInputs(ShooterWheelIOInputs inputs) {
        StatusCode read = BaseStatusSignal.refreshAll(velocity, voltage, current);
        inputs.connected = configured && read.isOK();
        inputs.velocityRpm = new double[] { velocity.getValueAsDouble() * 60.0 };
        inputs.appliedVolts = new double[] { voltage.getValueAsDouble() };
        inputs.currentAmps = new double[] { current.getValueAsDouble() };
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setVelocityRpm(double rpm) {
        if (!Double.isFinite(rpm)) {
            stop();
            return;
        }
        motor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    // #endregion
}
