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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.shooter.ShooterTrackAndShootCommand;
import frc.robot.subsystems.centrifuga.Centrifuga;
import frc.robot.subsystems.centrifuga.CentrifugaIOSim;

/**
 * Tempos e travas da sequência de tiro, com pivô e rodas ideais (respondem
 * na hora). Assim o teste mede só a lógica: quando a centrífuga pode ligar.
 * No robô, o tempo real depende do PID e do atrito (ver Shooter/Sequencia/Estado).
 */
class ShooterSequenceTimingTest {
    private static final double DT = 0.02;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private FakePivot pivot;
    private FakeWheel top, tower;
    private Shooter shooter;
    private Centrifuga centrifuga;
    private Pose2d pose;

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
        setEnabled(false);
        // Hub atrás do robô, a ~2,9 m: dentro da curva e com o pivô em 0°.
        pose = robotWithShooterAt(7.5, BLUE_HUB_CENTER.getY(), 0.0);
        pivot = new FakePivot();
        top = new FakeWheel(2);
        tower = new FakeWheel(1);
        shooter = new Shooter(pivot, top, tower, () -> pose, ChassisSpeeds::new, () -> true);
        centrifuga = new Centrifuga(new CentrifugaIOSim());
        tick(1);
        setEnabled(true);
        tick(1);
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

    private void tick(int cycles) {
        for (int i = 0; i < cycles; i++) {
            SimHooks.stepTiming(DT);
            scheduler.run();
        }
    }

    private boolean feeding() {
        return centrifuga.getTargetRpm() > 0.0;
    }

    /** Agenda o tiro e roda até a centrífuga ligar; devolve o tempo desde o 1º execute. */
    private double shootUntilFeeding() {
        scheduler.schedule(new ShooterTrackAndShootCommand(shooter, centrifuga));
        double firstExecute = Timer.getTimestamp() + DT;
        for (int i = 0; i < 100 && !feeding(); i++) {
            tick(1);
        }
        assertTrue(feeding(), "com tudo ideal, a centrífuga tem de ligar");
        return Timer.getTimestamp() - firstExecute;
    }

    // #region SEQUÊNCIA DE TIRO

    @Test void pivotHeadStartKeepsWheelsAndCentrifugeStopped() {
        scheduler.schedule(new ShooterTrackAndShootCommand(shooter, centrifuga));
        int headStartCycles = (int) Math.floor(PIVOT_AIM_HEADSTART_SECONDS / DT);
        for (int i = 0; i < headStartCycles; i++) {
            tick(1);
            assertEquals(0.0, shooter.getTopTargetRpm(), "rodas paradas enquanto o pivô mira primeiro");
            assertFalse(feeding());
        }
    }

    @Test void centrifugeWaitsForTheHeadStartAndAStableReadiness() {
        double elapsed = shootUntilFeeding();
        // Antes: vantagem do pivô (0,25 s) + prontidão estável (0,10 s).
        assertTrue(elapsed >= PIVOT_AIM_HEADSTART_SECONDS + FEED_READY_STABLE_SECONDS - 1e-9,
            "ligou cedo demais: " + elapsed + " s");
        assertTrue(elapsed <= PIVOT_AIM_HEADSTART_SECONDS + FEED_READY_STABLE_SECONDS + 4 * DT,
            "demorou demais com tudo ideal: " + elapsed + " s");
    }

    @Test void losingTheAimCutsTheFeedInTheSameCycle() {
        shootUntilFeeding();
        // O hub passa para o lado (~20°) e o pivô não acompanha.
        pivot.frozen = true;
        pose = robotWithShooterAt(7.5, BLUE_HUB_CENTER.getY() + 1.0, 0.0);
        tick(1);
        assertFalse(feeding(), "mira perdida não ganha a margem de 80 ms");
    }

    @Test void oneCycleWheelDipDoesNotPulseTheCentrifuge() {
        shootUntilFeeding();
        tick(2);
        // A bola passando derruba o RPM do top por um ciclo.
        top.dipFraction = 0.5;
        tick(1);
        top.dipFraction = 1.0;
        for (int i = 0; i < 10; i++) {
            tick(1);
            assertTrue(feeding(), "uma queda de um ciclo não pode desligar a centrífuga (ciclo " + i + ")");
        }
    }

    @Test void pivotThatNeverReachesTheAimNeverFeedsAndSaysWhy() {
        // O log do robô em 29/10: top girando, centrífuga nunca ligou, sem aviso.
        pivot.frozen = true; // pivô parado em 0°, sem chegar no alvo
        pose = robotWithShooterAt(7.5, BLUE_HUB_CENTER.getY() + 1.0, 0.0); // hub a ~20°
        ShooterTrackAndShootCommand shoot = new ShooterTrackAndShootCommand(shooter, centrifuga);
        scheduler.schedule(shoot);
        for (int i = 0; i < 75; i++) {
            tick(1);
            assertFalse(feeding(), "sem mira não alimenta");
        }
        assertTrue(shooter.getTopTargetRpm() > 0.0, "as rodas aceleram: o hub está ao alcance");
        assertEquals("PIVO_BUSCANDO_ALVO", shoot.getLastState());
        scheduler.cancel(shoot);
        assertFalse(shoot.centrifugeRanThisPress(), "o aviso na Driver Station sai ao soltar");
        assertEquals(0, shoot.feedRequests(), "a sequência nem chegou a pedir a centrífuga");
    }

    @Test void centrifugeThatRefusesIsReportedInsteadOfLookingLikeAShot() {
        // O robô em 29/10: o Start (sem trava) girava, o B não. A sequência pedia e a
        // centrífuga recusava (sensor/Talon sem resposta), mas o estado dizia DISPARANDO.
        Centrifuga refusing = new Centrifuga(new frc.robot.subsystems.centrifuga.CentrifugaIO() {
            @Override public void updateInputs(CentrifugaIOInputs in) {
                in.connected = false; // como o Talon 36 sem resposta para o código com trava
                in.configurationStatus = "RESET_REINICIE_DESABILITADO";
            }
        });
        try {
            ShooterTrackAndShootCommand shoot = new ShooterTrackAndShootCommand(shooter, refusing);
            scheduler.schedule(shoot);
            tick(75);
            assertTrue(shoot.feedRequests() > 0, "pivô e rodas ideais: a sequência pede para alimentar");
            assertFalse(shoot.centrifugeRanThisPress(), "a centrífuga recusou todos os pedidos");
            assertEquals("CENTRIFUGA_RECUSOU", shoot.getLastState());
            assertEquals(0.0, refusing.getTargetRpm());
            scheduler.cancel(shoot); // ao soltar, a Driver Station mostra o estado da centrífuga
        } finally {
            refusing.close();
        }
    }

    @Test void longWheelDropStopsTheFeed() {
        shootUntilFeeding();
        // Antes dos 0,75 s do "pronto por tempo": só a tolerância de RPM vale.
        top.dipFraction = 0.5;
        tick((int) Math.ceil(CENTRIFUGE_READY_GRACE_SECONDS / DT) + 2);
        assertFalse(feeding(), "roda abaixo do RPM por mais que a margem: para de alimentar");
    }

    // #endregion

    // #region PIVÔ E RODAS

    /** Alvo a 3 m do shooter, num ângulo do pivô (0° = para trás do robô). */
    private Translation2d targetAtPivotDegrees(double pivotDegrees) {
        Translation2d shooterAt = ShotCalculator.shooterFieldPosition(pose);
        Rotation2d direction = pose.getRotation()
            .plus(Rotation2d.fromDegrees(PIVOT_ZERO_FACING_DEGREES + pivotDegrees));
        return shooterAt.plus(new Translation2d(3.0, direction));
    }

    @Test void smallAimChangesDoNotRestartTheMotionMagicProfile() {
        shooter.aimAt(targetAtPivotDegrees(10.0));
        assertEquals(1, pivot.targetCalls);
        shooter.aimAt(targetAtPivotDegrees(10.2));
        assertEquals(1, pivot.targetCalls, "0,2° de ruído não reenvia o alvo");
        // 29/10/2026: com limiar de 1,2° (acima da tolerância de 0,75°) o
        // alvo enviado ficava até 1,2° atrás da mira certa. Agora 0,6° reenvia.
        shooter.aimAt(targetAtPivotDegrees(10.6));
        assertEquals(2, pivot.targetCalls, "0,6° de mudança reenvia");
        shooter.aimAt(targetAtPivotDegrees(12.0));
        assertEquals(3, pivot.targetCalls, "1,4° de mudança reenvia");
        assertEquals(ShotCalculator.pivotDegreesToMotorRotations(12.0), pivot.target, 1e-9);
    }

    @Test void manualPivotIsSlowAndClamped() {
        shooter.manualPivot(1.0);
        assertEquals(PIVOT_MANUAL_PERCENT, pivot.percent, 1e-12);
        shooter.manualPivot(5.0);
        assertEquals(PIVOT_MANUAL_PERCENT, pivot.percent, 1e-12, "eixo fora de ±1 não acelera");
        shooter.manualPivot(-1.0);
        assertEquals(-PIVOT_MANUAL_PERCENT, pivot.percent, 1e-12);
    }

    @Test void pivotWithoutZeroRefusesAimAndManual() {
        pivot.zeroValid = false;
        tick(1);
        shooter.aimAt(targetAtPivotDegrees(30.0));
        shooter.manualPivot(1.0);
        assertEquals(0, pivot.targetCalls);
        assertEquals(0.0, pivot.percent);
        assertEquals(Shooter.PivotMode.PARADO, shooter.getPivotMode());
    }

    @Test void stoppingLetsTheTopCoastButBrakesTheTower() {
        shooter.setShotSpeeds(3000.0, 3500.0);
        top.stopCalls = 0;
        tower.stopCalls = 0;
        shooter.stopWheels();
        assertTrue(top.stopCalls > 0, "top: solto (Coast)");
        assertEquals(0.0, tower.requested, "tower: alvo 0 RPM, freando");
        assertEquals(0, tower.stopCalls, "tower não fica solta");
    }

    // #endregion

    // #region FALSOS

    /** Pivô ideal: chega no alvo no ciclo seguinte (a menos que congelado). */
    private static class FakePivot implements ShooterPivotIO {
        boolean zeroValid = true, frozen;
        double position, target, percent;
        int targetCalls;

        @Override public void updateInputs(ShooterPivotIOInputs inputs) {
            if (!frozen) position = target;
            inputs.connected = true;
            inputs.zeroValid = zeroValid;
            inputs.motorPositionRotations = position;
        }
        @Override public void setTargetRotations(double motorRotations) { target = motorRotations; targetCalls++; }
        @Override public void setPercentOutput(double value) { percent = value; }
        @Override public void stop() { target = position; }
    }

    /** Roda ideal: mede o RPM pedido (vezes dipFraction para simular a bola passando). */
    private static class FakeWheel implements ShooterWheelIO {
        private final int motors;
        double requested, dipFraction = 1.0;
        int stopCalls;

        FakeWheel(int motors) { this.motors = motors; }

        @Override public void updateInputs(ShooterWheelIOInputs inputs) {
            inputs.connected = true;
            inputs.velocityRpm = new double[motors];
            inputs.appliedVolts = new double[motors];
            inputs.currentAmps = new double[motors];
            java.util.Arrays.fill(inputs.velocityRpm, requested * dipFraction);
        }
        @Override public void setVelocityRpm(double rpm) { requested = rpm; }
        @Override public void stop() { requested = 0.0; stopCalls++; }
    }

    // #endregion
}
