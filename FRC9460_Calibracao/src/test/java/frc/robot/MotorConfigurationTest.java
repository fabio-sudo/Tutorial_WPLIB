package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.centrifuga.CentrifugaConstants;
import frc.robot.subsystems.centrifuga.CentrifugaIOTalonFX;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterPivotIOTalonFX;
import frc.robot.util.TalonZeroMark;

/**
 * A configuração que vai para cada TalonFX usa as constantes certas:
 * SEGURANÇA (limites de corrente e soft limits) e CONTROLE (PID e perfil).
 * Os valores ficam nas constantes; este teste impede que a configuração
 * esqueça de aplicar algum deles.
 */
class MotorConfigurationTest {

    // #region SEGURANÇA COMUM

    private static void assertCurrentLimits(TalonFXConfiguration config, double supplyAmps, double statorAmps) {
        assertTrue(config.CurrentLimits.SupplyCurrentLimitEnable, "limite de supply desligado");
        assertEquals(supplyAmps, config.CurrentLimits.SupplyCurrentLimit);
        assertTrue(config.CurrentLimits.StatorCurrentLimitEnable, "limite de estator desligado");
        assertEquals(statorAmps, config.CurrentLimits.StatorCurrentLimit);
    }

    // #endregion

    // #region MECANISMOS

    @Test
    void shooterPivot() {
        TalonFXConfiguration config = ShooterPivotIOTalonFX.createConfiguration();
        assertEquals(NeutralModeValue.Brake, config.MotorOutput.NeutralMode);
        // Segurança: corrente e curso calibrado, em voltas do rotor.
        assertCurrentLimits(config, ShooterConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS,
            ShooterConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS);
        assertTrue(config.SoftwareLimitSwitch.ForwardSoftLimitEnable);
        assertTrue(config.SoftwareLimitSwitch.ReverseSoftLimitEnable);
        // No Talon o centro é a marca do zero (100 voltas): limites 100 ± 8,8439.
        assertEquals(TalonZeroMark.toTalon(ShooterConstants.PIVOT_MAX_MOTOR_ROTATIONS),
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold);
        assertEquals(TalonZeroMark.toTalon(-ShooterConstants.PIVOT_MAX_MOTOR_ROTATIONS),
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold);
        assertEquals(8.843920681818181,
            TalonZeroMark.toMechanism(config.SoftwareLimitSwitch.ForwardSoftLimitThreshold), 1e-9);
        assertEquals(1.0, config.Feedback.SensorToMechanismRatio);
        // Controle: PID e perfil do Motion Magic.
        assertEquals(ShooterConstants.PIVOT_NEUTRAL_DEADBAND, config.MotorOutput.DutyCycleNeutralDeadband);
        assertEquals(ShooterConstants.PIVOT_KP, config.Slot0.kP);
        assertEquals(ShooterConstants.PIVOT_KI, config.Slot0.kI);
        assertEquals(ShooterConstants.PIVOT_KD, config.Slot0.kD);
        assertEquals(0.30, config.Slot0.kS, 1e-9);
        assertEquals(com.ctre.phoenix6.signals.StaticFeedforwardSignValue.UseClosedLoopSign,
            config.Slot0.StaticFeedforwardSign);
        // kV do Kraken X44 (~0,092 V por volta/s): sem ele o pivô não seguia o perfil.
        assertEquals(0.092, config.Slot0.kV, 0.002);
        assertEquals(ShooterConstants.PIVOT_CRUISE_ROTATIONS_PER_SEC, config.MotionMagic.MotionMagicCruiseVelocity);
        assertEquals(ShooterConstants.PIVOT_ACCELERATION_ROTATIONS_PER_SEC2, config.MotionMagic.MotionMagicAcceleration);
        assertEquals(ShooterConstants.PIVOT_JERK_ROTATIONS_PER_SEC3, config.MotionMagic.MotionMagicJerk);
    }

    @Test
    void centrifuge() {
        TalonFXConfiguration config = CentrifugaIOTalonFX.createConfiguration();
        assertEquals(NeutralModeValue.Brake, config.MotorOutput.NeutralMode);
        assertCurrentLimits(config, CentrifugaConstants.SUPPLY_CURRENT_LIMIT_AMPS,
            CentrifugaConstants.STATOR_CURRENT_LIMIT_AMPS);
        assertEquals(CentrifugaConstants.KP, config.Slot0.kP);
        assertEquals(CentrifugaConstants.KS, config.Slot0.kS);
        assertEquals(CentrifugaConstants.KV, config.Slot0.kV);
        assertEquals(0.336, config.Slot0.kS, 1e-9);
        assertEquals(CentrifugaConstants.ACCELERATION_ROTATIONS_PER_SEC2,
            config.MotionMagic.MotionMagicAcceleration);
    }

    @Test
    void intakeLeaderHasLimitsAndHoldGainsFollowerOnlyCurrent() {
        TalonFXConfiguration leader = IntakeIOTalonFX.createLeaderConfiguration();
        assertEquals(NeutralModeValue.Brake, leader.MotorOutput.NeutralMode);
        assertCurrentLimits(leader, IntakeConstants.CURRENT_LIMIT_AMPS, IntakeConstants.CURRENT_LIMIT_AMPS);
        // Segurança: fechado e aberto são os soft limits.
        assertEquals(IntakeConstants.CLOSE_LIMIT_ENABLED, leader.SoftwareLimitSwitch.ForwardSoftLimitEnable);
        assertEquals(TalonZeroMark.toTalon(IntakeConstants.CLOSED_ROTATIONS),
            leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold);
        assertTrue(leader.SoftwareLimitSwitch.ReverseSoftLimitEnable);
        assertEquals(TalonZeroMark.toTalon(IntakeConstants.OPEN_ROTATIONS),
            leader.SoftwareLimitSwitch.ReverseSoftLimitThreshold);
        assertEquals(81.0, IntakeConstants.GEAR_REDUCTION);
        assertTrue(TalonZeroMark.toMechanism(leader.SoftwareLimitSwitch.ReverseSoftLimitThreshold)
            > IntakeConstants.MECHANICAL_OPEN_ROTATIONS, "limite deve parar antes do extremo medido");
        assertEquals(1.0, leader.Feedback.SensorToMechanismRatio,
            "os limites e o IO continuam em voltas do rotor, sem aplicar 81 duas vezes");
        // Controle: PID de segurar com kG.
        assertEquals(IntakeConstants.HOLD_KP, leader.Slot0.kP);
        assertEquals(IntakeConstants.HOLD_KG, leader.Slot0.kG);
        assertEquals(IntakeConstants.HOLD_MAX_VOLTS, leader.Voltage.PeakForwardVoltage);
        assertEquals(-IntakeConstants.HOLD_MAX_VOLTS, leader.Voltage.PeakReverseVoltage);
        assertEquals(GravityTypeValue.Elevator_Static, leader.Slot0.GravityType);

        TalonFXConfiguration follower = IntakeIOTalonFX.createFollowerConfiguration();
        assertEquals(IntakeConstants.HOLD_NEUTRAL_DEADBAND, leader.MotorOutput.DutyCycleNeutralDeadband);
        assertEquals(IntakeConstants.HOLD_NEUTRAL_DEADBAND, follower.MotorOutput.DutyCycleNeutralDeadband);
        assertEquals(NeutralModeValue.Brake, follower.MotorOutput.NeutralMode);
        assertCurrentLimits(follower, IntakeConstants.CURRENT_LIMIT_AMPS, IntakeConstants.CURRENT_LIMIT_AMPS);
        assertFalse(follower.SoftwareLimitSwitch.ForwardSoftLimitEnable, "o seguidor não tem soft limit próprio");
        assertEquals(0.0, follower.Slot0.kP, "o seguidor não roda PID");
    }

    // #endregion
}
