package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

// #endregion

/**
 * Pivô real: TalonFX no CAN 32, com a mesma configuração da competição.
 *
 * O Motion Magic roda DENTRO do TalonFX (1 kHz). O código só envia a posição
 * final; o Talon gera o perfil e segura a posição com o kP.
 */
public class ShooterPivotIOTalonFX implements ShooterPivotIO {

    // #region MOTOR E SINAIS

    private static final int CONFIG_ATTEMPTS = 5;
    private static final double SIGNAL_UPDATE_HZ = 50.0;

    private final TalonFX motor = new TalonFX(PIVOT_MOTOR_ID);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);

    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<Current> current = motor.getStatorCurrent();
    private final StatusSignal<Double> reference = motor.getClosedLoopReference();
    private final StatusSignal<Boolean> forwardLimit = motor.getFault_ForwardSoftLimit();
    private final StatusSignal<Boolean> reverseLimit = motor.getFault_ReverseSoftLimit();

    private boolean configured;
    private boolean zeroValid;

    // #endregion

    // #region CONFIGURAÇÃO E ZERO AO LIGAR

    public ShooterPivotIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake: parado, o motor resiste a ser girado.
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = PIVOT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = PIVOT_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_CURRENT_LIMIT_AMPS;

        config.Slot0.kP = PIVOT_KP;
        config.Slot0.kI = PIVOT_KI;
        config.Slot0.kD = PIVOT_KD;

        config.MotionMagic.MotionMagicCruiseVelocity = PIVOT_CRUISE_ROTATIONS_PER_SEC;
        config.MotionMagic.MotionMagicAcceleration = PIVOT_ACCELERATION_ROTATIONS_PER_SEC2;
        config.MotionMagic.MotionMagicJerk = PIVOT_JERK_ROTATIONS_PER_SEC3;

        // Segunda proteção do curso: mesmo que algum código peça mais, o Talon
        // recusa passar de ±6 voltas. Só vale se o zero estiver no centro.
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PIVOT_MAX_MOTOR_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -PIVOT_MAX_MOTOR_ROTATIONS;

        configured = retry(() -> motor.getConfigurator().apply(config)).isOK();
        BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ,
            position, velocity, voltage, current, reference, forwardLimit, reverseLimit);

        // Descarta o aviso de reset da própria ligação; resets futuros contam.
        motor.hasResetOccurred();

        // Igual ao código de competição: o shooter foi centralizado à mão,
        // então a posição atual vira 0.
        zeroHere();
    }

    private static StatusCode retry(Supplier<StatusCode> action) {
        StatusCode result = action.get();
        for (int attempt = 1; attempt < CONFIG_ATTEMPTS && !result.isOK(); attempt++) {
            result = action.get();
        }
        return result;
    }

    @Override
    public void zeroHere() {
        zeroValid = configured && retry(() -> motor.setPosition(0.0)).isOK();
        if (!zeroValid) {
            motor.stopMotor();
        }
    }

    // #endregion

    // #region LEITURAS

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        StatusCode read = BaseStatusSignal.refreshAll(
            position, velocity, voltage, current, reference, forwardLimit, reverseLimit);

        // Um reset do Talon (queda de bateria, por exemplo) zera o encoder onde
        // o shooter estiver. O centro deixa de ser conhecido: para o motor até
        // a equipe centralizar e zerar de novo.
        if (motor.hasResetOccurred()) {
            zeroValid = false;
            motor.stopMotor();
        }

        inputs.connected = read.isOK();
        inputs.zeroValid = configured && zeroValid;
        inputs.motorPositionRotations = position.getValueAsDouble();
        inputs.motorVelocityRotationsPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.statorCurrentAmps = current.getValueAsDouble();
        inputs.profileReferenceRotations = reference.getValueAsDouble();
        inputs.forwardLimitActive = forwardLimit.getValue();
        inputs.reverseLimitActive = reverseLimit.getValue();
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setTargetRotations(double motorRotations) {
        if (!configured || !zeroValid || !Double.isFinite(motorRotations)) {
            stop();
            return;
        }
        motor.setControl(positionRequest.withPosition(
            MathUtil.clamp(motorRotations, -PIVOT_MAX_MOTOR_ROTATIONS, PIVOT_MAX_MOTOR_ROTATIONS)));
    }

    @Override
    public void setPercentOutput(double percent) {
        if (!configured || !zeroValid || !Double.isFinite(percent)) {
            stop();
            return;
        }
        motor.setControl(percentRequest.withOutput(MathUtil.clamp(percent, -1.0, 1.0)));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    // #endregion
}
