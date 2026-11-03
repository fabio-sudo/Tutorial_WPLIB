package frc.robot.subsystems.fuel;

import static frc.robot.subsystems.fuel.FuelConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Detecção pelos sensores: pico de corrente do rolo e queda de RPM do top. */
class FuelTrackerTest {

    private static final class Signals implements FuelRobotState {
        boolean rollerOn = true;
        double current = 2.0;
        double rollerRpm = 2000.0;
        double top = 3000.0;
        boolean centrifugeOn = true;

        @Override public Pose2d pose() { return new Pose2d(); }
        @Override public ChassisSpeeds fieldSpeeds() { return new ChassisSpeeds(); }
        @Override public boolean intakeReadyToCollect() { return true; }
        @Override public boolean rollerPulling() { return rollerOn; }
        @Override public double rollerRpm() { return rollerRpm; }
        @Override public double rollerCurrentAmps() { return current; }
        @Override public double topRpm() { return top; }
        @Override public boolean topCommanded() { return true; }
        @Override public double towerRpm() { return 3000.0; }
        @Override public double centrifugeRpm() { return centrifugeOn ? 2500.0 : 0.0; }
        @Override public boolean centrifugeRunning() { return centrifugeOn; }
        @Override public Translation2d shooterFieldPosition() { return new Translation2d(8.0, 4.0); }
        @Override public double shooterHeadingDegrees() { return 180.0; }
    }

    private Signals signals;
    private FuelTracker tracker;
    private double now;

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        signals = new Signals();
        now = 0.0;
        tracker = new FuelTracker(signals, () -> now);
    }

    @AfterEach
    void cleanup() {
        tracker.close();
    }

    private void cycles(int count) {
        for (int i = 0; i < count; i++) {
            now += 0.02;
            tracker.periodic();
        }
    }

    @Test
    void oneCurrentSpikeIsOneCollection() {
        cycles(20);
        signals.current = COLLECT_CURRENT_AMPS + 5.0;
        cycles(6);
        assertEquals(1, tracker.getCollections(), "pico longo conta uma vez só");
        signals.current = 2.0;
        cycles(3);
        signals.current = COLLECT_CURRENT_AMPS + 5.0;
        cycles(4);
        assertEquals(2, tracker.getCollections());
    }

    @Test
    void startingCurrentIsIgnored() {
        // Rolo ligando: corrente alta enquanto acelera (partida do motor).
        signals.rollerRpm = 600.0;
        signals.current = 35.0;
        cycles(10);
        // Chegou na velocidade e a corrente caiu: nada foi coletado.
        signals.rollerRpm = 2000.0;
        signals.current = 2.0;
        cycles(5);
        assertEquals(0, tracker.getCollections());
        // Agora sim, uma bola.
        signals.current = 20.0;
        cycles(3);
        assertEquals(1, tracker.getCollections());
    }

    @Test
    void shortSpikeOrStoppedRollerDoesNotCount() {
        cycles(20);
        signals.current = COLLECT_CURRENT_AMPS + 5.0;
        cycles(1);
        signals.current = 2.0;
        cycles(3);
        signals.rollerOn = false;
        signals.current = 30.0;
        cycles(5);
        assertEquals(0, tracker.getCollections());
    }

    @Test
    void suddenTopDropWithCentrifugeIsAShot() {
        cycles(10);
        signals.top = 3000.0 * 0.94;
        cycles(2);
        assertEquals(1, tracker.getShots());
        // A roda recupera devagar: isso não é outro disparo.
        for (int i = 0; i < 10; i++) {
            signals.top += 15.0;
            cycles(1);
        }
        assertEquals(1, tracker.getShots());
    }

    @Test
    void everyDetectedShotFeedsTheConsistencyStatistics() {
        cycles(10);
        for (int shot = 0; shot < 5; shot++) {
            signals.top = 3000.0;
            cycles(20);
            signals.top = 3000.0 * 0.94;
            cycles(2);
        }
        assertEquals(5, tracker.getShots());
        assertEquals(5, tracker.getConsistency().shotsRecorded(), "um registro de constância por disparo");
        // Cada tiro saiu com ~3000 RPM e sempre igual: sem dispersão na velocidade.
        assertEquals(0.0, tracker.getConsistency().exitSpeed().stdDev(), 1e-9);
        assertTrue(tracker.getConsistency().exitSpeed().mean() > 1.0);
    }

    @Test
    void dropWithoutCentrifugeIsNotAShot() {
        signals.centrifugeOn = false;
        cycles(10);
        signals.top = 2500.0;
        cycles(3);
        assertEquals(0, tracker.getShots());
    }
}
