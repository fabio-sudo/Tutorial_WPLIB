package frc.robot.subsystems.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

// #endregion

/**
 * Telescópio 9460 real: um TalonFX (Kraken X44) com redução 5:1 e tambor de
 * 35 mm. O PID e o perfil rodam no roboRIO (subsistema Telescopio); aqui só
 * se aplica a tensão pedida e se leem os sensores.
 *
 * Só é criado quando frc.robot.CanIds.TELESCOPIO_9460 tem um ID de verdade.
 * Sem ID, Telescopio.createIO usa um IO vazio: nenhum dispositivo CAN é
 * criado com número inventado. Antes de ligar no robô, conferir também
 * MOTOR_INVERTED e a relação cabo/ponta (SIM_CABLE_TO_TIP_RATIO).
 *
 * Zero manual: ao ligar, a posição vira 0. O telescópio precisa estar
 * RECOLHIDO. Se o Talon reiniciar, o zero se perde e a posição fica inválida.
 */
public class TelescopioIOTalonFX implements TelescopioIO {

    // #region MOTOR E SINAIS

    private static final double SIGNAL_UPDATE_HZ = 50.0;

    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Boolean> statorLimit;
    private final boolean configured;
    private boolean zeroValid;

    // #endregion

    // #region 🧭 CONFIGURAÇÃO E ZERO AO LIGAR

    public TelescopioIOTalonFX(int canId) {
        motor = new TalonFX(canId);
        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        statorCurrent = motor.getStatorCurrent();
        statorLimit = motor.getFault_StatorCurrLimit();

        TalonFXConfiguration config = createConfiguration();
        config.MotorOutput.Inverted = MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        StatusCode configResult = tryUntilOk(() -> motor.getConfigurator().apply(config));
        configured = configResult.isOK();
        reportIfFailed(configResult, "Telescópio 9460: configuração do TalonFX " + canId);
        applySignalRates("Telescópio 9460", () -> BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ,
            position, velocity, voltage, statorCurrent, statorLimit), motor);

        // Descarta o aviso de reset da própria ligação; resets futuros contam.
        motor.hasResetOccurred();
        zeroValid = configured && tryUntilOk(() -> motor.setPosition(0.0)).isOK();
    }

    /**
     * Proteções do motor (Phoenix 6), dentro das faixas de frc.robot.MotorLimits.
     * Os soft limits só valem depois de zerar com o telescópio recolhido.
     */
    public static TalonFXConfiguration createConfiguration() {
        var config = new TalonFXConfiguration();

        // #region MOTOR E SENTIDO (MODO NEUTRO)

        // Brake: ao desabilitar, o motor freia em vez de deixar o telescópio cair livre.
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // #endregion

        // #region 🛑⚡ SEGURANÇA - LIMITES DE CORRENTE

        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME_SECONDS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // #endregion

        // #region 🛑📏 SEGURANÇA - SOFT LIMITS (CURSO)

        // Em voltas do rotor (SensorToMechanismRatio = 1, padrão).
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            (MAX_EXTENSION_METERS + SOFT_LIMIT_MARGIN_METERS) * MOTOR_ROTATIONS_PER_METER;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            (MIN_EXTENSION_METERS - SOFT_LIMIT_MARGIN_METERS) * MOTOR_ROTATIONS_PER_METER;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // #endregion

        return config;
    }

    // #endregion

    // #region LEITURAS

    @Override
    public void updateInputs(TelescopioIOInputs inputs) {
        StatusCode read = BaseStatusSignal.refreshAll(position, velocity, voltage, statorCurrent, statorLimit);
        if (motor.hasResetOccurred()) {
            zeroValid = false;
            motor.stopMotor();
            reportReset("Telescópio 9460");
        }
        inputs.motorConnected = read.isOK();
        inputs.positionValid = configured && zeroValid && read.isOK();
        inputs.extensionMeters = position.getValueAsDouble() / MOTOR_ROTATIONS_PER_METER;
        inputs.velocityMetersPerSec = velocity.getValueAsDouble() / MOTOR_ROTATIONS_PER_METER;
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.motorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.statorCurrentLimitActive = statorLimit.getValue();
    }

    // #endregion

    // #region COMANDOS DO MOTOR

    @Override
    public void setVoltage(double volts) {
        // Segurança: sem configuração, sem zero ou pedido inválido, não move.
        // O PID que calcula a tensão roda no roboRIO (Telescopio).
        if (!configured || !zeroValid || !Double.isFinite(volts)) {
            motor.stopMotor();
            return;
        }
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    // #endregion
}
