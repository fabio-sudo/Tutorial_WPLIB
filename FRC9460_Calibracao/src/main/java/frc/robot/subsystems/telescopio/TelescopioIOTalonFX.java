package frc.robot.subsystems.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

// #endregion

/**
 * Reserva para a integração física, deliberadamente sem criar dispositivo CAN.
 * Faltam ID/barramento, inversão, sensores de referência e transmissão confirmada.
 * O robô real informa indisponibilidade e não usa os parâmetros ideais da simulação.
 */
public class TelescopioIOTalonFX implements TelescopioIO {

    // #region LEITURAS (HARDWARE NAO CONFIGURADO)

    @Override
    public void updateInputs(TelescopioIOInputs inputs) {
        inputs.motorConnected = false;
        inputs.positionValid = false;
        inputs.extensionMeters = 0.0;
        inputs.velocityMetersPerSec = 0.0;
        inputs.appliedVolts = 0.0;
        inputs.motorCurrentAmps = 0.0;
        inputs.statorCurrentLimitActive = false;
    }

    // #endregion

    // #region CONFIGURACAO DO TALONFX (PHOENIX 6)

    /**
     * Proteções do motor para aplicar quando o TalonFX for criado (Phoenix 6).
     * Ainda não é aplicada: faltam o ID CAN e a inversão. Os soft limits só valem
     * depois de zerar a posição com o telescópio recolhido.
     */
    public static TalonFXConfiguration createConfiguration() {
        var config = new TalonFXConfiguration();

        // #region MODO NEUTRO

        // Brake: ao desabilitar, o motor freia em vez de deixar o telescópio cair livre.
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // #endregion

        // #region LIMITES DE CORRENTE

        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME_SECONDS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // #endregion

        // #region LIMITES DE CURSO (SOFT LIMITS)

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
}
