package frc.robot;

import static frc.robot.MotorLimits.*;
import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.centrifuga.CentrifugaConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.telescopio.TelescopioConstants;

/** Mapa de IDs CAN sem repetição e limites de corrente dentro do recomendado. */
class HardwareMapTest {

    // #region IDS CAN

    @Test
    void everyCanIdIsUniqueAndListed() throws Exception {
        Map<Integer, String> seen = new HashMap<>();
        for (CanIds.Device device : CanIds.ALL) {
            String previous = seen.put(device.id(), device.name());
            assertNull(previous, "ID " + device.id() + " repetido: " + previous + " e " + device.name());
        }
        // Toda constante int de CanIds com ID definido precisa estar na lista.
        // IDs ainda NOT_DEFINED (ex.: telescópio 9460) ficam de fora até serem escolhidos.
        Set<Integer> listed = new HashSet<>(seen.keySet());
        for (Field field : CanIds.class.getFields()) {
            if (Modifier.isStatic(field.getModifiers()) && field.getType() == int.class
                && field.getInt(null) != CanIds.NOT_DEFINED) {
                assertTrue(listed.contains(field.getInt(null)), "Falta na lista ALL: " + field.getName());
            }
        }
        assertFalse(CanIds.ELEVATORS_HARDWARE_ENABLED,
            "elevadores ainda exclusivos de simulação: não criar CAN no robô atual");
        assertNotEquals(CanIds.TELESCOPIO_9458, CanIds.TELESCOPIO_9460);
        assertEquals(23, CanIds.ALL.size());
    }

    // #endregion

    // #region LIMITES DE CORRENTE

    private static void assertTalonLimits(String name, double supplyAmps, double statorAmps) {
        assertTrue(supplyAmps >= TALON_SUPPLY_MIN_AMPS && supplyAmps <= TALON_SUPPLY_MAX_AMPS,
            name + ": supply fora de 20-80 A (" + supplyAmps + ")");
        // Sem queda programada para 40 A, o supply não pode passar do disjuntor.
        assertTrue(supplyAmps <= BREAKER_AMPS, name + ": supply acima do disjuntor (" + supplyAmps + ")");
        assertTrue(statorAmps >= supplyAmps && statorAmps <= TALON_STATOR_MAX_AMPS,
            name + ": estator fora da faixa (" + statorAmps + ")");
    }

    @Test
    void talonFxMechanismsFollowTheCtreRecommendation() {
        assertTalonLimits("Pivô do shooter",
            ShooterConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS, ShooterConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS);
        assertTalonLimits("Intake", IntakeConstants.CURRENT_LIMIT_AMPS, IntakeConstants.CURRENT_LIMIT_AMPS);
        assertTalonLimits("Centrífuga",
            CentrifugaConstants.SUPPLY_CURRENT_LIMIT_AMPS, CentrifugaConstants.STATOR_CURRENT_LIMIT_AMPS);
        assertTalonLimits("Telescópio 9460",
            TelescopioConstants.SUPPLY_CURRENT_LIMIT_AMPS, TelescopioConstants.STATOR_CURRENT_LIMIT_AMPS);
    }

    @Test
    void sparkFlexMechanismsFollowTheRevRecommendation() {
        assertTrue(ShooterConstants.SPARK_CURRENT_LIMIT_AMPS <= SPARK_FLEX_MAX_AMPS, "Top e tower acima de 60 A");
        assertTrue(RollerConstants.CURRENT_LIMIT_AMPS >= SPARK_NEO_MIN_AMPS
            && RollerConstants.CURRENT_LIMIT_AMPS <= SPARK_FLEX_MAX_AMPS, "Rolete fora de 40-60 A");
        assertTrue(SPARK_FLEX_MAX_AMPS <= SPARK_NEO_MAX_AMPS);
    }

    // #endregion
}
