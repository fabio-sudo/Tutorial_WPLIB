package frc.robot.subsystems.telescopio_9458;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.*;

class Telescopio_9458Test {
    private Telescopio_9458 telescopio;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    // Chegada nos fins de curso: medida na metade do trecho lento, onde a velocidade já
    // deve ser a lenta. Acompanha END_APPROACH_* (2 cm a 6 cm/s: 1 cm, até 0,08 m/s).
    private static final double ARRIVAL_CHECK_METERS = END_APPROACH_DISTANCE_METERS / 2;
    private static final double ARRIVAL_MAX_SPEED = END_APPROACH_VELOCITY_METERS_PER_SEC + 0.02;
    // Com leituras atrasadas (20 e 40 ms) o mecanismo segue a referência com folga maior.
    private static final double DELAYED_ARRIVAL_MAX_SPEED = END_APPROACH_VELOCITY_METERS_PER_SEC + 0.04;
    @BeforeAll static void init() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        RoboRioSim.setVInVoltage(12);
        enabled(false);
        telescopio = new Telescopio_9458(new TelescopioIOSim_9458());
        tick(1);
        enabled(true);
    }
    @AfterEach void cleanup() {
        scheduler.cancelAll();
        telescopio.close();
        enabled(false);
        SimHooks.resumeTiming();
    }
    private void enabled(boolean value) {
        DriverStationSim.setEnabled(value);
        DriverStationSim.notifyNewData();
    }
    private void tick(int cycles) {
        for (int i = 0; i < cycles; i++) {
            SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
            scheduler.run();
            assertTrue(Math.abs(telescopio.getRequestedVolts()) <= MAX_CONTROL_VOLTS);
        }
    }
    @Test void reachesAndHoldsIntermediatePositionAgainstGravity() {
        telescopio.setGoalMeters(0.5);
        tick(200);
        assertTrue(telescopio.atGoal());
        double position = telescopio.getExtensionMeters();
        tick(100);
        assertEquals(position, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        assertTrue(telescopio.getRequestedVolts() > 0);
    }
    @Test void holdingOpensReleasingHoldsAndHoldingCloseRetracts() {
        var open = telescopio.extendWhileHeldCommand();
        scheduler.schedule(open);
        tick(60);
        double released = telescopio.getExtensionMeters();
        assertTrue(released > 0.3);
        open.cancel(); // soltou o RT em movimento
        tick(100);
        assertTrue(telescopio.atGoal());
        double held = telescopio.getExtensionMeters();
        // A frenagem segue v²/2a; tolerância adicional cobre erro de rastreamento.
        double stoppingDistance = MAX_VELOCITY_METERS_PER_SEC * MAX_VELOCITY_METERS_PER_SEC
                / (2 * MAX_ACCELERATION_METERS_PER_SEC_SQ);
        assertTrue(held >= released && held < released + stoppingDistance + 0.025,
            "freia sem voltar: soltou em " + released + ", parou em " + held);
        tick(100);
        assertEquals(held, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        scheduler.schedule(open);
        tick(300);
        // Para na margem de 5 cm, não no batente físico.
        assertEquals(MAX_OPERATING_EXTENSION_METERS, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        open.cancel();
        var close = telescopio.retractWhileHeldCommand();
        scheduler.schedule(close);
        tick(400);
        assertEquals(0, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        close.cancel();
        tick(10);
        assertEquals(0, telescopio.getRequestedVolts());
        assertFalse(telescopio.isJammed()); // Uso normal não aciona o travamento.
    }
    @Test void testModeStepsMoveTenCentimetersPerPressAndStayWithinTheTravel() {
        var up = telescopio.stepUpWhileHeldCommand();
        var down = telescopio.stepDownWhileHeldCommand();
        scheduler.schedule(up);
        tick(100);
        up.cancel();
        tick(20);
        assertEquals(TEST_STEP_METERS, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        // Segundo aperto: mais um passo a partir da meta mantida.
        scheduler.schedule(up);
        tick(100);
        up.cancel();
        tick(20);
        assertEquals(2 * TEST_STEP_METERS, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        scheduler.schedule(down);
        tick(100);
        down.cancel();
        tick(20);
        assertEquals(TEST_STEP_METERS, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        // Soltou no meio do passo: freia e para antes do alvo, sem voltar.
        scheduler.schedule(up);
        tick(8);
        up.cancel();
        double released = telescopio.getExtensionMeters();
        tick(60);
        double stopped = telescopio.getExtensionMeters();
        assertTrue(stopped >= released && stopped <= 2 * TEST_STEP_METERS + POSITION_TOLERANCE_METERS,
                "soltou em " + released + ", parou em " + stopped);
        // Perto do topo, o passo não passa do curso de operação.
        telescopio.setGoalMeters(MAX_OPERATING_EXTENSION_METERS - 0.03);
        tick(150);
        scheduler.schedule(up);
        tick(100);
        up.cancel();
        tick(20);
        assertEquals(MAX_OPERATING_EXTENSION_METERS, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
    }
    @Test void approachesBothEndsSlowlyInsteadOfHittingThem() {
        // Na metade do trecho lento, a velocidade já é a lenta.
        var open = telescopio.extendWhileHeldCommand();
        scheduler.schedule(open);
        double topArrival = speedWhenCrossing(MAX_OPERATING_EXTENSION_METERS - ARRIVAL_CHECK_METERS, true);
        tick(50);
        assertEquals(MAX_OPERATING_EXTENSION_METERS, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        open.cancel();
        var close = telescopio.retractWhileHeldCommand();
        scheduler.schedule(close);
        double bottomArrival = speedWhenCrossing(ARRIVAL_CHECK_METERS, false);
        tick(50);
        assertEquals(0, telescopio.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        close.cancel();
        assertTrue(topArrival <= ARRIVAL_MAX_SPEED, "chegada no topo: " + topArrival + " m/s");
        assertTrue(bottomArrival <= ARRIVAL_MAX_SPEED, "chegada no fundo: " + bottomArrival + " m/s");
    }
    private double speedWhenCrossing(double meters, boolean up) {
        for (int i = 0; i < 500; i++) {
            tick(1);
            double position = telescopio.getExtensionMeters();
            if (up ? position >= meters : position <= meters) {
                return Math.abs(telescopio.getVelocityMetersPerSecond());
            }
        }
        return fail("não chegou em " + meters);
    }
    @Test void disablingWhileHoldingCutsVoltage() {
        scheduler.schedule(telescopio.extendWhileHeldCommand());
        tick(50);
        enabled(false);
        tick(1);
        assertNull(telescopio.getCurrentCommand());
        assertFalse(telescopio.isPositionControlActive());
        assertEquals(0, telescopio.getRequestedVolts());
    }
    @Test void nearZeroRoundingCannotReenergizeTheRetractedMechanism() {
        telescopio.close();
        var fake = new FakeIO();
        fake.position = -0.000341064453125; // Registro real em 106,390921 s.
        telescopio = new Telescopio_9458(fake);
        tick(1);
        telescopio.setGoalMeters(1.734723475976807e-18); // Meta do mesmo registro.
        tick(30);
        assertEquals(0, telescopio.getGoalMeters());
        assertEquals(0, fake.volts, "recolhido deve apoiar, sem pulso positivo por arredondamento");
        assertEquals(0, Telescopio_9458.limitAtEnds(0.33, fake.position,
                1.734723475976807e-18, 0));
        telescopio.setGoalMeters(0.1);
        tick(10);
        assertTrue(fake.volts > 0, "a proteção deve permitir abrir novamente");
    }

    @Test void realGainsApproachEndsSlowlyWithDelayedMeasurements() {
        // Exercita atrasos de 20 e 40 ms; não substitui medição com a carga real.
        for (int delay : new int[] {1, 2}) {
            telescopio.close();
            var delayed = new DelayedSimIO(delay);
            telescopio = new Telescopio_9458(delayed);
            tick(delay + 2);
            telescopio.setGoalMeters(10); // Nem meta externa ultrapassa a margem.
            assertEquals(MAX_OPERATING_EXTENSION_METERS, telescopio.getGoalMeters());
            double peak = 0;
            double topSpeed = Double.NaN;
            for (int cycle = 0; cycle < 400; cycle++) {
                tick(1);
                peak = Math.max(peak, delayed.latest.extensionMeters);
                if (Double.isNaN(topSpeed) && delayed.latest.extensionMeters >= MAX_OPERATING_EXTENSION_METERS - ARRIVAL_CHECK_METERS)
                    topSpeed = Math.abs(delayed.latest.velocityMetersPerSec);
            }
            assertTrue(topSpeed <= DELAYED_ARRIVAL_MAX_SPEED, "chegada superior com atraso: " + topSpeed);
            assertTrue(peak < UPPER_SOFT_LIMIT_METERS, "controle normal não deve depender do corte Talon: " + peak);
            telescopio.setGoalMeters(0);
            double bottomSpeed = Double.NaN;
            for (int cycle = 0; cycle < 400; cycle++) {
                tick(1);
                if (Double.isNaN(bottomSpeed) && delayed.latest.extensionMeters <= ARRIVAL_CHECK_METERS)
                    bottomSpeed = Math.abs(delayed.latest.velocityMetersPerSec);
            }
            assertTrue(bottomSpeed <= DELAYED_ARRIVAL_MAX_SPEED, "chegada inferior com atraso: " + bottomSpeed);
            assertEquals(0, telescopio.getRequestedVolts());
        }
    }
    @Test void finalApproachCompletesPromptlyWithRealGainsAndSensorDelay() {
        for (int delay : new int[] {1, 2}) {
            telescopio.close();
            var delayed = new DelayedSimIO(delay);
            telescopio = new Telescopio_9458(delayed);
            tick(delay + 2);
            for (double goal : new double[] {MAX_OPERATING_EXTENSION_METERS, 0}) {
                telescopio.setGoalMeters(goal);
                int entered = -1;
                int settled = -1;
                int stableCycles = 0;
                for (int cycle = 0; cycle < 300; cycle++) {
                    tick(1);
                    double error = Math.abs(delayed.latest.extensionMeters - goal);
                    double speed = Math.abs(delayed.latest.velocityMetersPerSec);
                    if (entered < 0 && error <= 0.03) entered = cycle;
                    stableCycles = error <= POSITION_TOLERANCE_METERS
                            && speed <= VELOCITY_TOLERANCE_METERS_PER_SEC ? stableCycles + 1 : 0;
                    if (stableCycles >= 10) { settled = cycle - 9; break; }
                }
                assertTrue(entered >= 0 && settled >= entered, "precisa chegar e estabilizar na meta");
                double finalSeconds = (settled - entered) * LOOP_PERIOD_SECONDS;
                System.out.printf("Aproximacao: atraso=%d ms, meta=%.5f m, ultimos 3 cm=%.3f s%n",
                        delay * 20, goal, finalSeconds);
                assertTrue(finalSeconds <= 0.52, "demora no trecho final: " + finalSeconds);
            }
        }
    }
    @Test void jamStopsPushingHoldsWeightAndClearsOnNextPress() {
        telescopio.close();
        var fake = new FakeIO();
        telescopio = new Telescopio_9458(fake);
        tick(1);
        var open = telescopio.extendWhileHeldCommand();
        scheduler.schedule(open);
        tick(5);
        fake.current = STATOR_CURRENT_LIMIT_AMPS; // empurrando sem sair do lugar
        tick((int) (JAM_TIME_SECONDS / LOOP_PERIOD_SECONDS) + 2);
        assertTrue(telescopio.isJammed());
        assertEquals(fake.position, telescopio.getGoalMeters(), 1e-9);
        assertEquals(SIM_KG, fake.volts, 1e-6); // só sustenta o peso
        fake.current = 5;
        open.cancel();
        tick(1);
        assertTrue(telescopio.isJammed(), "soltar não destrava");
        scheduler.schedule(telescopio.extendWhileHeldCommand()); // apertou de novo
        tick(1);
        assertFalse(telescopio.isJammed());
        assertEquals(MAX_OPERATING_EXTENSION_METERS, telescopio.getGoalMeters());
    }
    @Test void smallOvershootAboveTheTopStillLetsL2Lower() {
        telescopio.close();
        var fake = new FakeIO();
        fake.position = MAX_EXTENSION_METERS + 0.008; // passou 8 mm do topo (0,6 volta)
        telescopio = new Telescopio_9458(fake);
        tick(1);
        assertTrue(telescopio.hasValidPosition());
        scheduler.schedule(telescopio.retractWhileHeldCommand()); // segurou L2
        tick(5);
        assertTrue(telescopio.isPositionControlActive());
        assertEquals(MIN_EXTENSION_METERS, telescopio.getGoalMeters());
        assertTrue(fake.volts < SIM_KG, "deve descer: " + fake.volts);
        fake.position = MAX_EXTENSION_METERS + 0.10; // leitura absurda continua bloqueada
        tick(1);
        assertFalse(telescopio.hasValidPosition());
        assertEquals(0, fake.volts);
    }
    @Test void parallelModelMatchesTheSimulationAndResyncsWhenDisabled() {
        scheduler.schedule(telescopio.extendWhileHeldCommand());
        tick(100);
        assertTrue(telescopio.getExtensionMeters() > 0.5);
        // Com Motion Magic o modelo recebe a média da tensão de cada ciclo (a do Talon muda a
        // cada 1 ms): a diferença fica abaixo de 0,1 mm.
        assertEquals(telescopio.getExtensionMeters(), telescopio.getModelExtensionMeters(), 1e-4);
        scheduler.cancelAll();
        tick(50);
        assertEquals(telescopio.getExtensionMeters(), telescopio.getModelExtensionMeters(), 1e-4);
    }
    @Test void parallelModelDivergesWhenTheRealMechanismDoesNotMove() {
        telescopio.close();
        var fake = new FakeIO(); // "real" travado em 0,1 m
        telescopio = new Telescopio_9458(fake);
        tick(1);
        enabled(false);
        tick(1); // desabilitado: modelo alinhado com o real
        assertEquals(fake.position, telescopio.getModelExtensionMeters(), 0.001);
        enabled(true);
        telescopio.setGoalMeters(0.6);
        tick(50);
        assertTrue(telescopio.getModelExtensionMeters() > fake.position + 0.2,
            "o modelo sobe com a mesma tensão: " + telescopio.getModelExtensionMeters());
        enabled(false);
        tick(1);
        assertEquals(fake.position, telescopio.getModelExtensionMeters(), 0.001);
    }
    @Test void endStopsOnlyAllowGravityCompensationAtTheTop() {
        double top = MAX_EXTENSION_METERS;
        assertEquals(SIM_KG, Telescopio_9458.limitAtEnds(3.0, top, top, 0.2));
        assertEquals(-1.0, Telescopio_9458.limitAtEnds(-1.0, top, 0.5, -0.2));
        assertEquals(0, Telescopio_9458.limitAtEnds(-2.0, 0, 0.5, 0));
        assertEquals(0, Telescopio_9458.limitAtEnds(0.4, 0.001, 0, 0));
        assertEquals(2.0, Telescopio_9458.limitAtEnds(2.0, 0.5, 0.9, 0.3));
    }
    @Test void simulationLimitsStatorCurrentAndStopsAtTheTalonSoftLimitLikeTheRealRobot() {
        var io = new TelescopioIOSim_9458();
        var inputs = new TelescopioIO_9458.TelescopioIOInputs_9458();
        io.setVoltage(12);
        io.updateInputs(inputs); // partida: o limite do estator reduz a tensão aplicada
        assertTrue(inputs.statorCurrentLimitActive);
        assertTrue(inputs.appliedVolts < 12, "tensão limitada: " + inputs.appliedVolts);
        assertTrue(inputs.motorCurrentAmps <= STATOR_CURRENT_LIMIT_AMPS + 1e-6);
        double highest = 0;
        for (int i = 0; i < 300; i++) {
            io.updateInputs(inputs);
            highest = Math.max(highest, inputs.extensionMeters);
        }
        // Como no TalonFX real, o soft limit segura a abertura em volta de 0,956 m
        // (corta, escorrega um pouco, volta), sem chegar ao batente físico.
        assertEquals(UPPER_SOFT_LIMIT_METERS, inputs.extensionMeters, 0.01);
        assertTrue(highest < MAX_EXTENSION_METERS - 0.02, "não bate no batente: " + highest);
        // No sentido contrário continua liberado (só o limite do estator atua na partida).
        io.setVoltage(-2);
        io.updateInputs(inputs);
        assertTrue(inputs.appliedVolts < 0, "descer continua liberado: " + inputs.appliedVolts);
    }

    @Test void simulationAdvancesTheWholeLoopPeriodAndFrictionHoldsAtRest() {
        // O rkdp da WPILib perdia ~9% do ciclo: a posição andava menos que velocidade × 20 ms.
        var io = new TelescopioIOSim_9458();
        var inputs = new TelescopioIO_9458.TelescopioIOInputs_9458();
        double steady = (2.0 - SIM_KS - SIM_KG) / SIM_KV;
        io.setState(0.3, steady);
        io.setMeasuredVoltage(2.0);
        io.updateInputs(inputs);
        assertEquals(steady, inputs.velocityMetersPerSec, 1e-6);
        assertEquals(0.3 + steady * LOOP_PERIOD_SECONDS, inputs.extensionMeters, 1e-6);
        // Parado com tensão dentro do atrito em torno de kG: não se mexe.
        io.setState(0.5, 0);
        io.setMeasuredVoltage(SIM_KG + 0.8 * SIM_KS);
        io.updateInputs(inputs);
        assertEquals(0.5, inputs.extensionMeters, 1e-9);
        assertEquals(0, inputs.velocityMetersPerSec);
    }

    @Test void simulatedSoftLimitsCutOnlyTowardTheLimit() {
        assertEquals(0, TelescopioIOSim_9458.softLimit(3, UPPER_SOFT_LIMIT_METERS));
        assertEquals(-3, TelescopioIOSim_9458.softLimit(-3, UPPER_SOFT_LIMIT_METERS));
        assertEquals(3, TelescopioIOSim_9458.softLimit(3, UPPER_SOFT_LIMIT_METERS - 0.001));
        assertEquals(0, TelescopioIOSim_9458.softLimit(-3, MIN_EXTENSION_METERS));
        assertEquals(3, TelescopioIOSim_9458.softLimit(3, MIN_EXTENSION_METERS));
        // Mesmo limiar do TalonFX, em voltas do rotor.
        assertEquals(TelescopioIOTalonFX_9458.createConfiguration().SoftwareLimitSwitch.ForwardSoftLimitThreshold,
                UPPER_SOFT_LIMIT_METERS * MOTOR_ROTATIONS_PER_METER, 1e-9);
    }
    @Test void cascadeMovesEachStageHalfOfTheTipAndMatchesTheMeasuredTurns() {
        var poses = TelescopioVisualizer_9458.componentPoses(MAX_EXTENSION_METERS);
        assertEquals(N2_TRAVEL_METERS, poses[0].getZ(), 1e-9);
        assertEquals(MAX_EXTENSION_METERS, poses[1].getZ(), 1e-9);
        assertEquals(0.498, N2_TRAVEL_METERS, 0.001);
        assertEquals(MOTOR_ROTATIONS_AT_LIMIT,
            TelescopioVisualizer_9458.stageExtension(MAX_EXTENSION_METERS) / STAGE_METERS_PER_MOTOR_ROTATION, 1e-9);
        assertEquals(0, TelescopioVisualizer_9458.stageExtension(Double.NaN));
    }
    @Test void talonConfigurationUsesTheKrakenX60Limits() {
        assertEquals(5, MOTOR_CAN_ID);
        var config = TelescopioIOTalonFX_9458.createConfiguration();
        assertEquals(MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive,
            config.MotorOutput.Inverted);
        assertEquals(NeutralModeValue.Brake, config.MotorOutput.NeutralMode);
        assertEquals(30, config.CurrentLimits.StatorCurrentLimit);
        assertTrue(config.CurrentLimits.StatorCurrentLimitEnable);
        // 4 cm antes do batente: 78,4 − 0,04 × 78,74 = 75,25 voltas do rotor.
        assertEquals(75.25, config.SoftwareLimitSwitch.ForwardSoftLimitThreshold, 0.01);
        assertTrue(config.SoftwareLimitSwitch.ForwardSoftLimitEnable);
    }
    @Test void gainsOnlyApplyDisabledAndInvalidEditsDoNotChangeTheController() {
        var original = telescopio.getGains();
        assertFalse(telescopio.applyGains(25, 1, 2, 0.2, 0.4, 4, 0.1));
        assertEquals(original, telescopio.getGains());
        enabled(false);
        assertFalse(telescopio.applyGains(Double.NaN, 0, 0, 0, 0, 0, 0));
        assertFalse(telescopio.applyGains(10, -1, 0, 0, 0, 0, 0));
        assertFalse(telescopio.applyGains(10, 0, 0, 0, MAX_CONTROL_VOLTS + 1, 0, 0));
        assertEquals(original, telescopio.getGains());
        assertTrue(telescopio.applyGains(25, 1, 2, 0.2, 0.4, 4, 0.1));
        tick(1);
        assertEquals(new TelescopioGains_9458(25, 1, 2, 0.2, 0.4, 4, 0.1), telescopio.getGains());
        assertEquals(SIM_KG, SIM_GAINS.kG(), "Ajuste de controle não modifica a física");
    }

    @Test void calibratedGravityIsUsedForHoldingAndTopLimit() {
        telescopio.close();
        var fake = new FakeIO();
        telescopio = new Telescopio_9458(fake);
        enabled(false);
        tick(1);
        assertTrue(telescopio.applyGains(18, 0, 0, 0, 1.1, 0, 0));
        enabled(true);
        telescopio.setGoalMeters(fake.position);
        tick(2);
        assertEquals(1.1, fake.volts, 1e-9);
        assertEquals(1.1, Telescopio_9458.limitAtEnds(4, MAX_EXTENSION_METERS,
                MAX_EXTENSION_METERS, 0.2, 1.1), 1e-9);
    }

    @Test void hardwareModelUsesAppliedVoltageEvenWhenRequestedVoltageIsDifferent() {
        telescopio.close();
        var fake = new FakeIO();
        fake.realHardware = true;
        fake.position = 0.3;
        fake.appliedVolts = 0; // Talon não está aplicando a tensão solicitada.
        telescopio = new Telescopio_9458(fake);
        enabled(false);
        tick(1);
        assertEquals(REAL_GAINS, telescopio.getGains());
        enabled(true);
        telescopio.setGoalMeters(0.8);
        tick(30);
        assertTrue(telescopio.getRequestedVolts() > 1);
        assertTrue(telescopio.getModelExtensionMeters() < 0.3,
                "O modelo deve seguir 0 V medidos, não subir com a solicitação");
    }

    @Test void invalidSensorCannotPoisonTheModelAndRecoveryResynchronizesIt() {
        telescopio.close();
        var fake = new FakeIO();
        telescopio = new Telescopio_9458(fake);
        tick(1);
        fake.position = Double.NaN;
        tick(2);
        assertEquals("LEITURA_NAO_FINITA", telescopio.getDiagnostic());
        assertTrue(Double.isFinite(telescopio.getModelExtensionMeters()));
        assertEquals(0, fake.volts);
        fake.position = 0.4;
        tick(1);
        assertEquals(0.4, telescopio.getModelExtensionMeters(), 1e-9);
    }

    @Test void failedControlRequestCancelsPositionControlAndZeroIsDisabledOnly() {
        telescopio.close();
        var fake = new FakeIO();
        telescopio = new Telescopio_9458(fake);
        tick(1);
        telescopio.setGoalMeters(0.6);
        fake.controlRequestOK = false;
        tick(1);
        assertFalse(telescopio.isPositionControlActive());
        assertEquals(0, fake.volts);
        assertEquals("FALHA_COMANDO_MOTOR", telescopio.getDiagnostic());
        assertFalse(telescopio.confirmRetractedZero());
        assertEquals(0, fake.zeroCalls);
        enabled(false);
        assertTrue(telescopio.confirmRetractedZero());
        assertEquals(1, fake.zeroCalls);
    }

    @Test void statusLeavesDisabledAfterEnablingAndKeepsGoalRefusalVisible() {
        enabled(false);
        tick(1);
        assertEquals("DESABILITADO", telescopio.getStatus());
        enabled(true);
        tick(1);
        assertEquals("PARADO", telescopio.getStatus(), "Habilitado e ocioso não é DESABILITADO");
        scheduler.schedule(telescopio.followGoalCommand(() -> Double.NaN));
        tick(2);
        assertEquals("META_RECUSADA", telescopio.getStatus());
        scheduler.cancelAll();
        tick(1);
        assertEquals("PARADO", telescopio.getStatus());
    }

    @Test void sysIdVoltageIsCutOnlyTowardTheNearestEnd() {
        assertEquals(0, Telescopio_9458.limitCharacterization(2, SYSID_UPPER_STOP_METERS));
        assertEquals(-2, Telescopio_9458.limitCharacterization(-2, SYSID_UPPER_STOP_METERS));
        assertEquals(0, Telescopio_9458.limitCharacterization(-2, SYSID_LOWER_STOP_METERS));
        assertEquals(2, Telescopio_9458.limitCharacterization(2, SYSID_LOWER_STOP_METERS));
        assertEquals(MAX_CONTROL_VOLTS, Telescopio_9458.limitCharacterization(MAX_CONTROL_VOLTS + 1, 0.4));
    }

    @Test void sysIdIsRefusedWhileDisabledAndAGoalLeavesSysIdMode() {
        enabled(false);
        telescopio.runCharacterization(1.0);
        assertFalse(telescopio.isCharacterizationActive());
        enabled(true);
        tick(1);
        telescopio.runCharacterization(1.0);
        tick(1);
        assertTrue(telescopio.isCharacterizationActive());
        assertEquals(1.0, telescopio.getRequestedVolts(), 1e-9);
        telescopio.setGoalMeters(0.3);
        assertFalse(telescopio.isCharacterizationActive());
        assertTrue(telescopio.isPositionControlActive());
    }

    // #region MOTION MAGIC (EMULADO NA SIMULAÇÃO, COMO NO TALON)

    @Test void motionMagicIsFasterThanTheRoboRioProfileAndStillLandsSoftly() {
        assertTrue(telescopio.usesMotionMagic());
        double motionMagicSeconds = fullTravel(telescopio);

        telescopio.close();
        telescopio = new Telescopio_9458(new RoboRioSimIO());
        tick(1);
        assertFalse(telescopio.usesMotionMagic());
        double roboRioSeconds = fullTravel(telescopio);

        System.out.printf("Curso 0 -> %.3f m: Motion Magic %.2f s, roboRIO %.2f s%n",
                MAX_OPERATING_EXTENSION_METERS, motionMagicSeconds, roboRioSeconds);
        // Medido: 2,12 s x 2,54 s. O trecho lento final (6 cm/s) limita o ganho total.
        assertTrue(motionMagicSeconds < 0.9 * roboRioSeconds,
                "Motion Magic " + motionMagicSeconds + " s x roboRIO " + roboRioSeconds + " s");
    }

    /** Segura RT até o topo e depois LT até o fundo; mede o tempo de subida e as chegadas. */
    private double fullTravel(Telescopio_9458 subject) {
        var open = subject.extendWhileHeldCommand();
        scheduler.schedule(open);
        int cycles = 0;
        double topArrival = Double.NaN;
        while (Math.abs(subject.getExtensionMeters() - MAX_OPERATING_EXTENSION_METERS) > POSITION_TOLERANCE_METERS
                || Math.abs(subject.getVelocityMetersPerSecond()) > VELOCITY_TOLERANCE_METERS_PER_SEC) {
            tick(1);
            if (Double.isNaN(topArrival) && subject.getExtensionMeters() >= MAX_OPERATING_EXTENSION_METERS - ARRIVAL_CHECK_METERS)
                topArrival = Math.abs(subject.getVelocityMetersPerSecond());
            assertTrue(++cycles < 500, "não chegou ao topo");
        }
        open.cancel();
        var close = subject.retractWhileHeldCommand();
        scheduler.schedule(close);
        double bottomArrival = speedWhenCrossing(ARRIVAL_CHECK_METERS, false);
        tick(60);
        close.cancel();
        assertEquals(0, subject.getExtensionMeters(), POSITION_TOLERANCE_METERS);
        assertTrue(topArrival <= ARRIVAL_MAX_SPEED, "chegada no topo: " + topArrival + " m/s");
        assertTrue(bottomArrival <= ARRIVAL_MAX_SPEED, "chegada no fundo: " + bottomArrival + " m/s");
        return cycles * LOOP_PERIOD_SECONDS;
    }

    @Test void releasingAtFullSpeedBrakesInSCurveWithoutGoingBackOrOscillating() {
        var open = telescopio.extendWhileHeldCommand();
        scheduler.schedule(open);
        tick(45); // em velocidade de cruzeiro, longe do trecho lento
        assertEquals(MOTION_MAGIC_VELOCITY_METERS_PER_SEC, telescopio.getVelocityMetersPerSecond(), 0.02);
        double released = telescopio.getExtensionMeters();
        open.cancel();
        // A meta conta os 20 ms até ela chegar ao Talon e a idade da leitura.
        double expectedStop = released + TelescopioMotionProfile_9458.releaseDistance(
                MOTION_MAGIC_VELOCITY_METERS_PER_SEC, 0, MOTION_MAGIC_VELOCITY_METERS_PER_SEC,
                RELEASE_DELAY_SECONDS, telescopio.getProfileLimits());
        double previous = released;
        for (int i = 0; i < 50; i++) {
            tick(1);
            assertTrue(telescopio.getExtensionMeters() >= previous - 0.001, "voltou depois de soltar");
            previous = telescopio.getExtensionMeters();
        }
        assertEquals(expectedStop, telescopio.getExtensionMeters(), 0.01);
        assertTrue(telescopio.atGoal());
        double held = telescopio.getExtensionMeters();
        for (int i = 0; i < 50; i++) {
            tick(1);
            assertEquals(held, telescopio.getExtensionMeters(), 0.001, "oscilou parado");
        }
    }

    @Test void releasingAtAnyMomentStopsWithoutGoingBackOrExceedingTheAcceleration() {
        // Simulação de 22/10: soltar perto do fundo fazia a referência ir e voltar.
        // Solta em cada fase (partida, cruzeiro, frenagem para o trecho lento, trecho
        // lento), subindo e descendo: a referência nunca volta e respeita 2,5 m/s².
        for (boolean up : new boolean[] {true, false}) {
            for (int heldCycles = 2; heldCycles <= 98; heldCycles += 4) {
                telescopio.close();
                var io = new ReferenceRecordingSimIO();
                telescopio = new Telescopio_9458(io);
                tick(1);
                if (!up) {
                    io.setState(MAX_OPERATING_EXTENSION_METERS, 0);
                    tick(1);
                }
                var hold = up ? telescopio.extendWhileHeldCommand() : telescopio.retractWhileHeldCommand();
                scheduler.schedule(hold);
                tick(heldCycles);
                hold.cancel();
                String when = (up ? "subindo" : "descendo") + ", solto após " + heldCycles + " ciclos";
                double direction = up ? 1 : -1;
                double farthest = io.reference;
                double previousVelocity = io.referenceVelocity;
                for (int i = 0; i < 100; i++) {
                    tick(1);
                    farthest = direction > 0 ? Math.max(farthest, io.reference) : Math.min(farthest, io.reference);
                    double back = direction * (farthest - io.reference);
                    assertTrue(back <= 1e-5, "voltou " + back * 1000 + " mm: " + when + ", ciclo " + i);
                    assertTrue(Math.abs(io.referenceVelocity - previousVelocity) / LOOP_PERIOD_SECONDS
                            <= MOTION_MAGIC_ACCELERATION_METERS_PER_SEC_SQ + 1e-6, "aceleração acima do limite: " + when);
                    previousVelocity = io.referenceVelocity;
                }
                assertTrue(telescopio.atGoal(), "parado na meta: " + when);
            }
        }
    }

    /** Simulação que guarda a referência do Motion Magic emulado. */
    private static class ReferenceRecordingSimIO extends TelescopioIOSim_9458 {
        double reference, referenceVelocity;
        @Override public void updateInputs(TelescopioIOInputs_9458 inputs) {
            super.updateInputs(inputs);
            reference = inputs.motionMagicActive ? inputs.closedLoopReferenceMeters : inputs.extensionMeters;
            referenceVelocity = inputs.motionMagicActive ? inputs.closedLoopReferenceVelocityMetersPerSec : 0;
        }
    }

    @Test void profileTuningOnlyWhileDisabledAndWithinTheSafeRange() {
        assertFalse(telescopio.applyProfile(0.5, 2, 20), "habilitado: recusado");
        enabled(false);
        assertFalse(telescopio.applyProfile(PROFILE_MAX_VELOCITY_METERS_PER_SEC + 0.1, 2, 20));
        assertFalse(telescopio.applyProfile(0.5, 2, PROFILE_MIN_JERK_METERS_PER_SEC_CUBED - 1));
        assertFalse(telescopio.applyProfile(Double.NaN, 2, 20));
        assertEquals(MOTION_MAGIC_LIMITS, telescopio.getProfileLimits());
        assertTrue(telescopio.applyProfile(0.5, 2, 20));
        assertEquals(new TelescopioMotionProfile_9458.Limits(0.5, 2, 20), telescopio.getProfileLimits());
        enabled(true);
        tick(1);
        scheduler.schedule(telescopio.extendWhileHeldCommand());
        double fastest = 0;
        for (int i = 0; i < 60; i++) {
            tick(1);
            fastest = Math.max(fastest, telescopio.getVelocityMetersPerSecond());
        }
        assertEquals(0.5, fastest, 0.02, "o perfil novo vale no movimento seguinte");
    }

    @Test void gainsRefusedByTheTalonAreNotApplied() {
        telescopio.close();
        telescopio = new Telescopio_9458(new TelescopioIOSim_9458() {
            @Override public boolean configureGains(TelescopioGains_9458 gains) { return gains.kI() == 0; }
        });
        enabled(false);
        tick(1);
        var original = telescopio.getGains();
        assertFalse(telescopio.applyGains(40, 1, 0, 0.04, 0.2, 9.3, 0));
        assertEquals(original, telescopio.getGains());
        assertTrue(telescopio.applyGains(40, 0, 1, 0.04, 0.2, 9.3, 0));
    }

    @Test void replayFollowsTheExecutorRecordedInTheLog() {
        // No replay o IO é vazio; o executor vem das entradas gravadas.
        telescopio.close();
        telescopio = new Telescopio_9458(new FakeIO() {
            @Override public void updateInputs(TelescopioIOInputs_9458 inputs) {
                super.updateInputs(inputs);
                inputs.motionMagicAvailable = true;
            }
        });
        tick(1);
        assertTrue(telescopio.usesMotionMagic());
    }

    /** Física simulada controlada pelo perfil no roboRIO (sem Motion Magic). */
    private static class RoboRioSimIO extends TelescopioIOSim_9458 {
        @Override public boolean supportsMotionMagic() { return false; }
    }

    // #endregion

    private static class DelayedSimIO implements TelescopioIO_9458 {
        private final TelescopioIOSim_9458 plant = new TelescopioIOSim_9458();
        private final java.util.ArrayDeque<TelescopioIOInputs_9458> history = new java.util.ArrayDeque<>();
        private final int delay;
        TelescopioIOInputs_9458 latest;
        DelayedSimIO(int delay) { this.delay = delay; }
        @Override public boolean isRealHardware() { return true; }
        @Override public void setVoltage(double volts) { plant.setVoltage(volts); }
        @Override public void updateInputs(TelescopioIOInputs_9458 inputs) {
            latest = new TelescopioIOInputs_9458();
            plant.updateInputs(latest);
            history.addLast(latest);
            var sampled = history.size() > delay ? history.removeFirst() : history.getFirst();
            inputs.motorConnected = true;
            inputs.positionValid = true;
            inputs.extensionMeters = sampled.extensionMeters;
            inputs.velocityMetersPerSec = sampled.velocityMetersPerSec;
            inputs.appliedVolts = sampled.appliedVolts;
            inputs.motorCurrentAmps = sampled.motorCurrentAmps;
            inputs.rotorPositionRotations = sampled.rotorPositionRotations;
        }
    }

    private static class FakeIO implements TelescopioIO_9458 {
        double position = 0.1;
        double current;
        double volts;
        double appliedVolts;
        boolean realHardware;
        boolean controlRequestOK = true;
        int zeroCalls;
        @Override public void updateInputs(TelescopioIOInputs_9458 inputs) {
            inputs.motorConnected = true;
            inputs.positionValid = true;
            inputs.extensionMeters = position;
            inputs.motorCurrentAmps = current;
            inputs.appliedVolts = appliedVolts;
            inputs.controlRequestOK = controlRequestOK;
        }
        @Override public void setVoltage(double value) { volts = value; }
        @Override public boolean isRealHardware() { return realHardware; }
        @Override public boolean confirmRetractedZero() { zeroCalls++; position = 0; return true; }
    }
}
