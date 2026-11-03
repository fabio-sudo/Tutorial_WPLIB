package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSparkFlex;
import frc.robot.subsystems.shooter.ShooterWheelIO;
import frc.robot.subsystems.shooter.ShooterWheelIOSparkFlex;

/**
 * Cria os IOs reais com Spark Flex no simulador da REVLib: aplicar, gravar a
 * configuração e conferir reinício não podem lançar nada nem deixar o mecanismo
 * sem resposta. Um erro aqui derrubaria o código do robô na partida.
 */
class SparkUtilTest {

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    @Test void theWheelStartsPersistsAndKeepsReading() {
        ShooterWheelIO io = new ShooterWheelIOSparkFlex(new int[] {61, 62}, new boolean[] {true, false});
        var inputs = new ShooterWheelIO.ShooterWheelIOInputs();
        io.updateInputs(inputs);
        assertTrue(inputs.connected);
        assertEquals(2, inputs.velocityRpm.length);
    }

    @Test void theRollerStartsPersistsAndKeepsReading() {
        RollerIO io = new RollerIOSparkFlex();
        var inputs = new RollerIO.RollerIOInputs();
        io.updateInputs(inputs);
        assertTrue(inputs.connected);
    }
}
