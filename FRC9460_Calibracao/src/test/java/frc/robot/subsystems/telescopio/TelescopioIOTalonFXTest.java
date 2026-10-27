package frc.robot.subsystems.telescopio;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import org.junit.jupiter.api.Test;

class TelescopioIOTalonFXTest {
    @Test void configurationLimitsCurrentAndTravel() {
        var config = TelescopioIOTalonFX.createConfiguration();
        assertEquals(NeutralModeValue.Brake, config.MotorOutput.NeutralMode);
        assertTrue(config.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(40, config.CurrentLimits.StatorCurrentLimit);
        assertTrue(config.CurrentLimits.SupplyCurrentLimitEnable);
        assertEquals(30, config.CurrentLimits.SupplyCurrentLimit);
        assertTrue(config.CurrentLimits.SupplyCurrentLowerLimit
            <= config.CurrentLimits.SupplyCurrentLimit);
        // 5:1 e tambor de 35 mm: 45,47 voltas por metro; 1,262 m = 57,4 voltas.
        assertEquals(45.47, MOTOR_ROTATIONS_PER_METER, 0.01);
        assertTrue(config.SoftwareLimitSwitch.ForwardSoftLimitEnable);
        assertEquals(57.38, config.SoftwareLimitSwitch.ForwardSoftLimitThreshold, 0.01);
        assertTrue(config.SoftwareLimitSwitch.ReverseSoftLimitEnable);
        assertEquals(-0.45, config.SoftwareLimitSwitch.ReverseSoftLimitThreshold, 0.01);
    }
}
