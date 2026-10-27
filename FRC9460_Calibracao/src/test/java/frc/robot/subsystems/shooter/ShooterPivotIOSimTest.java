package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

/** Física do pivô: Motion Magic emulado, soft limits, batente da fiação e zero manual. */
class ShooterPivotIOSimTest {

    private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

    // O DCMotorSim limita a tensão pela bateria simulada, que vem da HAL.
    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    private void run(ShooterPivotIOSim io, double seconds) {
        for (int i = 0; i < Math.round(seconds / 0.02); i++) {
            io.updateInputs(inputs);
        }
    }

    @Test
    void reachesAndHoldsAnAngle() {
        var io = new ShooterPivotIOSim(0.0);
        io.setTargetRotations(ShotCalculator.pivotDegreesToMotorRotations(45.0));
        run(io, 2.0);
        assertEquals(3.0, inputs.motorPositionRotations, PIVOT_TOLERANCE_MOTOR_ROTATIONS);
        assertEquals(45.0, io.physicalDegrees(), 1.0);
        run(io, 1.0);
        assertEquals(3.0, inputs.motorPositionRotations, PIVOT_TOLERANCE_MOTOR_ROTATIONS);
    }

    @Test
    void targetBeyondTheCourseIsClampedToTheSoftLimit() {
        var io = new ShooterPivotIOSim(0.0);
        io.setTargetRotations(50.0);
        run(io, 3.0);
        assertEquals(PIVOT_MAX_MOTOR_ROTATIONS, inputs.motorPositionRotations, PIVOT_TOLERANCE_MOTOR_ROTATIONS);
        assertTrue(Math.abs(io.physicalDegrees()) <= PIVOT_SIM_WIRING_LIMIT_DEGREES + 1e-9);
    }

    @Test
    void manualOutputStopsAtTheSoftLimit() {
        var io = new ShooterPivotIOSim(0.0);
        io.setPercentOutput(1.0);
        run(io, 4.0);
        assertTrue(inputs.forwardLimitActive);
        assertTrue(inputs.motorPositionRotations <= PIVOT_MAX_MOTOR_ROTATIONS + 0.3);
        io.setPercentOutput(-1.0);
        run(io, 0.5);
        assertTrue(inputs.motorPositionRotations < PIVOT_MAX_MOTOR_ROTATIONS, "o sentido de volta continua livre");
    }

    @Test
    void wrongManualZeroHitsTheWiringBeforeTheSoftLimit() {
        // Robô ligado com o shooter 20° fora do centro: o encoder começa em 0.
        var io = new ShooterPivotIOSim(20.0);
        run(io, 0.1);
        assertEquals(0.0, inputs.motorPositionRotations, 1e-9);
        assertEquals(20.0, io.physicalDegrees(), 1e-9);

        // O código pede +90° pelo encoder, mas a fiação para em 90° reais
        // (70° no encoder). O código nunca chega ao alvo.
        io.setTargetRotations(PIVOT_MAX_MOTOR_ROTATIONS);
        run(io, 3.0);
        assertEquals(PIVOT_SIM_WIRING_LIMIT_DEGREES, io.physicalDegrees(), 0.5);
        assertEquals(70.0 / PIVOT_DEGREES_PER_MOTOR_ROTATION, inputs.motorPositionRotations, 0.05);
    }

    @Test
    void zeroHereKeepsThePhysicalAngle() {
        var io = new ShooterPivotIOSim(0.0);
        io.setTargetRotations(3.0);
        run(io, 2.0);
        double physical = io.physicalDegrees();
        io.zeroHere();
        run(io, 0.04);
        assertEquals(0.0, inputs.motorPositionRotations, 0.01);
        assertEquals(physical, io.physicalDegrees(), 0.2);
    }
}
