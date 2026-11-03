package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.shooter.ShooterTrackAndShootCommand;
import frc.robot.subsystems.centrifuga.Centrifuga;
import frc.robot.subsystems.centrifuga.CentrifugaIOSim;
import frc.robot.util.MechanismDiagnostics;

/**
 * Subsistema e sequência de tiro com os IOs simulados.
 * A pose é controlada pelo teste, no lugar do Drive.
 */
class ShooterTest {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private Shooter shooter;
    private Centrifuga centrifuga;
    private Pose2d pose;
    private boolean visionCorrected;

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(false);
        setEnabled(false);
        visionCorrected = true;
        pose = robotWithShooterAt(7.5, BLUE_HUB_CENTER.getY(), 0.0);
        shooter = new Shooter(
            new ShooterPivotIOSim(0.0),
            Shooter.createTopIO(),
            Shooter.createTowerIO(),
            () -> pose,
            ChassisSpeeds::new,
            () -> visionCorrected);
        centrifuga = new Centrifuga(new CentrifugaIOSim());
        tick(1);
        setEnabled(true);
    }

    @AfterEach
    void cleanup() {
        scheduler.cancelAll();
        shooter.close();
        centrifuga.close();
        setEnabled(false);
        SimHooks.resumeTiming();
    }

    private static Pose2d robotWithShooterAt(double x, double y, double headingDegrees) {
        Rotation2d heading = Rotation2d.fromDegrees(headingDegrees);
        Translation2d offset = new Translation2d(SHOOTER_OFFSET_FORWARD_METERS, SHOOTER_OFFSET_LEFT_METERS)
            .rotateBy(heading);
        return new Pose2d(new Translation2d(x, y).minus(offset), heading);
    }

    private void setEnabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    private void tick(double seconds) {
        for (int i = 0; i < Math.max(1, Math.round(seconds / 0.02)); i++) {
            SimHooks.stepTiming(0.02);
            scheduler.run();
        }
    }

    /** Roda o tiro e diz se a centrífuga chegou a ser ligada. */
    private boolean shootFor(double seconds) {
        Command shoot = new ShooterTrackAndShootCommand(shooter, centrifuga);
        scheduler.schedule(shoot);
        boolean fed = false;
        for (int i = 0; i < Math.round(seconds / 0.02); i++) {
            tick(0.02);
            fed |= centrifuga.getTargetRpm() > 0.0;
        }
        return fed;
    }

    @Test
    void waitsForTheLimelightBeforeShooting() {
        visionCorrected = false;
        assertFalse(shootFor(2.0));
        assertEquals(Shooter.PivotMode.PARADO, shooter.getPivotMode());
    }

    @Test
    void shootsWhenTheHubIsBehindTheRobot() {
        assertTrue(shootFor(2.5));
        assertEquals(0.0, shooter.getPivotDegrees(), 1.0);
        // A tower (modelo NEO) chega ao RPM com o kV da competição. O top, com o
        // modelo NEO Vortex, fica acima da tolerância: o tiro sai pelo tempo.
        assertTrue(shooter.isTowerAtSpeed());
    }

    @Test
    void turnsThePivotToAHubAtTheSide() {
        // Hub a 45° do zero: shooter a 3 m na diagonal.
        double d = 3.0 / Math.sqrt(2.0);
        pose = robotWithShooterAt(BLUE_HUB_CENTER.getX() + d, BLUE_HUB_CENTER.getY() + d, 0.0);
        assertTrue(shootFor(3.0));
        assertEquals(45.0, shooter.getPivotDegrees(), 1.0);
    }

    @Test
    void neverFeedsWhenTheHubIsOutOfThePivotCourse() {
        // Robô de costas para o hub: o shooter teria de girar 180°.
        pose = robotWithShooterAt(2.0, BLUE_HUB_CENTER.getY(), 0.0);
        assertFalse(shootFor(3.0));
        assertEquals(PIVOT_MAX_ANGLE_DEGREES, Math.abs(shooter.getPivotDegrees()), 1.0);
        // Segurança: sem alcance, as rodas nem aceleram.
        assertEquals(0.0, shooter.getTopTargetRpm());
    }

    @Test
    void wheelsStayStoppedWhenTheHubIsTooFar() {
        // Hub atrás do robô (no curso), mas a 8 m: além da curva de tiro.
        pose = robotWithShooterAt(BLUE_HUB_CENTER.getX() + 8.0, BLUE_HUB_CENTER.getY(), 0.0);
        Command shoot = new ShooterTrackAndShootCommand(shooter, centrifuga);
        scheduler.schedule(shoot);
        double maxTopTarget = 0.0;
        for (int i = 0; i < 100; i++) {
            tick(0.02);
            maxTopTarget = Math.max(maxTopTarget, shooter.getTopTargetRpm());
            assertEquals(0.0, centrifuga.getTargetRpm(), "a centrífuga não pode ligar");
        }
        assertEquals(0.0, maxTopTarget, "o top não pode acelerar com o hub longe");
    }

    @Test
    void practiceAimsAtTheBlueHubEvenWithTheDefaultRedAlliance() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        tick(0.02);
        assertEquals(BLUE_HUB_CENTER, shooter.getHubTarget());
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        tick(0.02);
        assertEquals(RED_HUB_CENTER, shooter.getHubTarget());
    }

    @Test
    void stuckWheelNeverFeedsByTime() {
        // Top conectado, mas parado (travado ou sem força): o "pronto por tempo"
        // não pode liberar a bola.
        ShooterWheelIO stuckTop = new ShooterWheelIO() {
            @Override
            public void updateInputs(ShooterWheelIOInputs inputs) {
                inputs.connected = true;
                inputs.velocityRpm = new double[] {0.0, 0.0};
                inputs.appliedVolts = new double[] {0.0, 0.0};
                inputs.currentAmps = new double[] {0.0, 0.0};
            }
        };
        Shooter stuck = new Shooter(new ShooterPivotIOSim(0.0), stuckTop, Shooter.createTowerIO(),
            () -> pose, ChassisSpeeds::new, () -> true);
        try {
            scheduler.schedule(new ShooterTrackAndShootCommand(stuck, centrifuga));
            for (int i = 0; i < 150; i++) {
                tick(0.02);
                assertEquals(0.0, centrifuga.getTargetRpm(), "roda parada não libera a bola");
            }
        } finally {
            scheduler.cancelAll();
            stuck.close();
        }
    }

    @Test
    void releasingTheButtonStopsEverything() {
        Command shoot = new ShooterTrackAndShootCommand(shooter, centrifuga);
        scheduler.schedule(shoot);
        tick(2.0);
        scheduler.cancel(shoot);
        tick(0.04);
        assertEquals(0.0, centrifuga.getTargetRpm());
        assertEquals(Shooter.PivotMode.PARADO, shooter.getPivotMode());
    }

    @Test
    void lostOrNonFiniteWheelDataCutsFeedWithoutTheGracePeriod() {
        int[] fault = {0};
        ShooterWheelIO measuredTop = new ShooterWheelIO() {
            double requested;
            @Override public void updateInputs(ShooterWheelIOInputs inputs) {
                inputs.connected = fault[0] != 1;
                double rpm = fault[0] == 2 ? Double.NaN : requested;
                inputs.velocityRpm = new double[] {rpm, rpm};
            }
            @Override public void setVelocityRpm(double rpm) { requested = rpm; }
            @Override public void stop() { requested = 0.0; }
        };
        Shooter checked = new Shooter(new ShooterPivotIOSim(0.0), measuredTop, Shooter.createTowerIO(),
            () -> pose, ChassisSpeeds::new, () -> true);
        try {
            scheduler.schedule(new ShooterTrackAndShootCommand(checked, centrifuga));
            tick(2.0);
            assertTrue(centrifuga.isRunning());
            fault[0] = 1;
            tick(0.02);
            assertFalse(centrifuga.isRunning(), "perda de conexão corta sem esperar 80 ms");
            fault[0] = 2;
            tick(1.0);
            assertFalse(checked.isTopAtSpeed(), "NaN não conta como RPM correto");
            assertFalse(centrifuga.isRunning(), "o tempo não pode liberar NaN");
        } finally {
            scheduler.cancelAll();
            checked.close();
        }
    }

    @Test
    void zeroIsOnlyAcceptedWithTheRobotDisabled() {
        assertFalse(shooter.zeroPivotHere());
        setEnabled(false);
        tick(0.02);
        assertTrue(shooter.zeroPivotHere());
    }

    @Test
    void aZeroThatFailedOrWasLostIsNeverLoggedAsDone() {
        boolean[] zeroOk = {true};
        ShooterPivotIO pivot = new ShooterPivotIO() {
            @Override public void updateInputs(ShooterPivotIOInputs inputs) {
                inputs.connected = true;
                inputs.zeroValid = zeroOk[0];
            }
        };
        Shooter checked = new Shooter(pivot, Shooter.createTopIO(), Shooter.createTowerIO(),
            () -> pose, ChassisSpeeds::new, () -> true);
        try {
            setEnabled(false);
            tick(0.02);
            assertEquals("ZERADO_AO_LIGAR", checked.getZeroResult());
            // Talon reiniciou: o texto de antes não pode continuar dizendo "zerado".
            zeroOk[0] = false;
            tick(0.02);
            assertEquals(MechanismDiagnostics.ZERO_PERDIDO, checked.getZeroResult());
            // A equipe zera, mas o motor não aceita.
            assertTrue(checked.zeroPivotHere());
            tick(0.02);
            assertEquals(MechanismDiagnostics.ZERO_PERDIDO, checked.getZeroResult());
            zeroOk[0] = true;
            tick(0.02);
            assertEquals("ZERADO_PELA_EQUIPE", checked.getZeroResult());
        } finally {
            scheduler.cancelAll();
            checked.close();
        }
    }
}
