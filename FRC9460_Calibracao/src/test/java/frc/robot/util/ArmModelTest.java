package frc.robot.util;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.lang.management.ManagementFactory;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * ArmModel faz a mesma física do SingleJointedArmSim da WPILib, sem criar
 * objetos. Usa o braço do intake (2 motores, 81:1, gravidade e batentes).
 */
class ArmModelTest {
    private static final double MOI = SingleJointedArmSim.estimateMOI(SIM_ARM_LENGTH_METERS, SIM_MASS_KG);
    private static final double MIN = Math.toRadians(-30.0);
    private static final double MAX = Math.toRadians(90.0);

    @BeforeAll static void hal() {
        assertTrue(HAL.initialize(500, 0));
        // O SingleJointedArmSim limita a tensão à bateria; sem isto ela vale 0 V no teste.
        RoboRioSim.setVInVoltage(12.0);
    }

    @Test void followsTheWpilibArmThroughGravityAndBothEndStops() {
        SingleJointedArmSim wpilib = new SingleJointedArmSim(
            SIM_MOTORS, GEAR_REDUCTION, MOI, SIM_ARM_LENGTH_METERS, MIN, MAX, true, 0.0);
        ArmModel model = new ArmModel(SIM_MOTORS, GEAR_REDUCTION, MOI, SIM_ARM_LENGTH_METERS, MIN, MAX);
        model.setState(0.0, 0.0);
        double maxAngleError = 0.0, maxCurrentError = 0.0;
        for (int cycle = 0; cycle < 300; cycle++) {
            // 2 s sem tensão (cai pela gravidade até o batente de baixo),
            // 2 s a +6 V (sobe até o batente de cima), 2 s a -2 V.
            double volts = cycle < 100 ? 0.0 : cycle < 200 ? 6.0 : -2.0;
            wpilib.setInputVoltage(volts);
            model.setInputVoltage(volts);
            for (int i = 0; i < 20; i++) {
                wpilib.update(0.001);
                model.update(0.001);
            }
            maxAngleError = Math.max(maxAngleError, Math.abs(wpilib.getAngleRads() - model.getAngleRads()));
            maxCurrentError = Math.max(maxCurrentError,
                Math.abs(wpilib.getCurrentDrawAmps() - model.getCurrentDrawAmps()));
        }
        // A WPILib integra com passo adaptativo; aqui é RK4 fixo de 1 ms. A diferença
        // (~0,07°, na batida nos batentes) é bem menor que a tolerância do diagnóstico (5°).
        assertTrue(maxAngleError < Math.toRadians(0.2), "ângulo diferiu " + Math.toDegrees(maxAngleError) + "°");
        assertTrue(maxCurrentError < 0.5, "corrente diferiu " + maxCurrentError + " A");
        assertEquals(wpilib.getAngleRads(), model.getAngleRads(), Math.toRadians(0.2));
    }

    @Test void stepsWithoutCreatingObjects() {
        ArmModel model = new ArmModel(SIM_MOTORS, GEAR_REDUCTION, MOI, SIM_ARM_LENGTH_METERS, MIN, MAX);
        model.setInputVoltage(3.0);
        for (int i = 0; i < 20_000; i++) model.update(0.001); // aquece o JIT
        var mx = (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
        long thread = Thread.currentThread().getId();
        long before = mx.getThreadAllocatedBytes(thread);
        for (int i = 0; i < 20_000; i++) {
            model.setInputVoltage(i % 2 == 0 ? 3.0 : -3.0);
            model.update(0.001);
        }
        long allocated = mx.getThreadAllocatedBytes(thread) - before;
        // O SingleJointedArmSim criava ~50 KB por passo; aqui 20.000 passos cabem em 1 KB.
        assertTrue(allocated < 1024, "alocou " + allocated + " bytes em 20.000 passos");
    }
}
