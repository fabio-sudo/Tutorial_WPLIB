package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * O modelo em paralelo do intake (a mesma física recebendo a tensão medida)
 * acompanha o mecanismo. Ele roda também no robô real a cada ciclo, então
 * precisa ser preciso E barato: este teste segura a precisão quando a
 * física do modelo muda (desde 29/10/2026 ele usa util/ArmModel, sem objetos).
 */
class IntakeModelAccuracyTest {

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    @BeforeEach void enable() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        // Bateria de 12 V: sem isto o braço simulado não recebe tensão no teste.
        RoboRioSim.setVInVoltage(12.0);
    }

    @AfterEach void disable() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test void modelFollowsTheMechanismWhileOpeningHoldingAndClosing() {
        IntakeIOSim mechanism = new IntakeIOSim();
        IntakeIOSim model = new IntakeIOSim();
        IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
        mechanism.updateInputs(inputs);
        model.setModelState(inputs.positionRotations, inputs.velocityRotationsPerSec);

        double maxErrorDegrees = 0.0;
        for (int cycle = 0; cycle < 250; cycle++) {
            // 2 s abrindo, 1 s segurando no meio do caminho (PID + kG), 2 s fechando.
            if (cycle < 100) mechanism.setPercentOutput(OPEN_PERCENT);
            else if (cycle == 100) mechanism.holdPosition(inputs.positionRotations);
            else if (cycle >= 150) mechanism.setPercentOutput(CLOSE_PERCENT);
            mechanism.updateInputs(inputs);
            model.stepModel(inputs.appliedVolts);
            double error = Math.abs(inputs.positionRotations - model.modelPosition()) * DEGREES_PER_MOTOR_ROTATION;
            maxErrorDegrees = Math.max(maxErrorDegrees, error);
        }
        System.out.printf("MEDIDA erro maximo do modelo do intake: %.3f graus%n", maxErrorDegrees);
        assertTrue(maxErrorDegrees < MODEL_TOLERANCE_DEGREES / 2.0,
            "modelo se afastou " + maxErrorDegrees + "° (tolerância do diagnóstico: " + MODEL_TOLERANCE_DEGREES + "°)");
    }
}
