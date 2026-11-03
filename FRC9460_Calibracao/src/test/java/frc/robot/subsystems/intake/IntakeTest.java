package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.util.MechanismDiagnostics;

/** Articulador com o braço simulado (gravidade, soft limits e segurar posição). */
class IntakeTest {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private Intake intake;

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        intake = new Intake(new IntakeIOSim());
        tick(0.04);
    }

    @AfterEach
    void cleanup() {
        scheduler.cancelAll();
        intake.close();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    private void tick(double seconds) {
        for (int i = 0; i < Math.max(1, Math.round(seconds / 0.02)); i++) {
            SimHooks.stepTiming(0.02);
            scheduler.run();
        }
    }

    private void hold(Command command, double seconds) {
        scheduler.schedule(command);
        tick(seconds);
        scheduler.cancel(command);
        tick(0.02);
    }

    @Test
    void startsClosedAndStaysThere() {
        tick(2.0);
        assertTrue(intake.isClosed());
        assertEquals(CLOSED_RAISE_DEGREES, intake.getRaiseDegrees(), 1.0);
    }

    @Test
    void opensToTheSoftLimitAndClosesBack() {
        hold(intake.openCommand(), 4.0);
        assertTrue(intake.isOpen(), "abrir para no fim do curso");
        // Só Brake no batente: assenta na margem de segurança (0,5 volta = 2,2°) abaixo do limite.
        assertEquals(-OPEN_SAFETY_MARGIN_ROTATIONS * DEGREES_PER_MOTOR_ROTATION,
            intake.getRaiseDegrees(), 0.6);
        hold(intake.closeCommand(), 4.0);
        assertTrue(intake.isClosed());
    }

    @Test
    void releasedHalfwayItHoldsAgainstGravity() {
        hold(intake.openCommand(), 0.4);
        double released = intake.getRaiseDegrees();
        assertTrue(released > 15.0 && released < 85.0, "parou no meio do curso: " + released);
        tick(2.0);
        // Sem o PID de segurar o braço cairia até o batente (0°).
        assertEquals(released, intake.getRaiseDegrees(), 3.0);
        // A banda neutra deixa o braço assentar até ~1,6° e o PID segura ali.
        tick(20.0);
        assertEquals(released, intake.getRaiseDegrees(), 2.0, "sem derrapar em 20 s");
    }

    @Test
    void closedStaysPutForALongTime() {
        // Só Brake deixaria o braço derrapar 7° em 30 s e cair em ~150 s (simulação).
        tick(30.0);
        assertEquals(CLOSED_RAISE_DEGREES, intake.getRaiseDegrees(), 2.0);
        assertTrue(intake.isClosed(), "a banda não pode tirar o braço da tolerância de fechado");
    }

    @Test
    void holdLeavesTheMotorsInBrakeMostOfTheTime() {
        // Sem a banda eram 100% dos ciclos com tensão: o motor parado zumbindo.
        // O modelo não tem o atrito da caixa 81:1, então no robô deve ser menos.
        var io = new IntakeIOSim();
        var inputs = new IntakeIOInputsAutoLogged();
        io.holdPosition(CLOSED_ROTATIONS);
        int cycles = 1500;
        int withVoltage = 0;
        for (int i = 0; i < cycles; i++) {
            io.updateInputs(inputs);
            if (inputs.appliedVolts != 0.0) withVoltage++;
        }
        System.out.printf("INTAKE segurando fechado: tensão em %.0f%% dos ciclos%n", 100.0 * withVoltage / cycles);
        assertTrue(withVoltage < cycles / 2, "ciclos com tensão: " + withVoltage + " de " + cycles);
    }

    @Test
    void neutralBandEndsInsideTheClosedTolerance() {
        // isClosed e isOpen não podem mudar por causa do Brake parado na banda.
        assertTrue(HOLD_NEUTRAL_DEADBAND * 12.0 / HOLD_KP <= TOLERANCE_ROTATIONS);
    }

    @Test
    void openedRestsOnTheStopWithoutDriftingAndClosesAgain() {
        hold(intake.openCommand(), 4.0);
        tick(10.0);
        // Só Brake: a gravidade encosta o braço no batente (no máximo ~3° abaixo do limite).
        assertTrue(intake.getRaiseDegrees() > -4.0 && intake.getRaiseDegrees() < 1.0,
            "aberto encostado no batente: " + intake.getRaiseDegrees());
        hold(intake.closeCommand(), 4.0);
        assertTrue(intake.isClosed(), "sai do batente e fecha");
    }

    @Test
    void zeroIsOnlyAcceptedDisabled() {
        assertFalse(intake.zeroClosed());
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        tick(0.02);
        assertTrue(intake.zeroClosed());
    }

    @Test
    void aZeroThatFailedIsNeverLoggedAsDone() {
        FakeIO io = replaceWithFake();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        io.valid = false;
        tick(0.02);
        assertTrue(intake.zeroClosed(), "o pedido foi aceito");
        tick(0.02);
        assertEquals(MechanismDiagnostics.ZERO_PERDIDO, intake.getZeroResult());
        io.valid = true;
        tick(0.02);
        assertEquals("ZERADO_PELA_EQUIPE", intake.getZeroResult());
    }

    @Test
    void aLostFollowerIsReportedAndTheIntakeKeepsWorking() {
        FakeIO io = replaceWithFake();
        tick(0.02);
        assertEquals(MechanismDiagnostics.OK, intake.getDiagnosis());
        io.followerOk = false;
        tick(0.02);
        assertEquals("SEGUIDOR_SEM_RESPOSTA", intake.getDiagnosis());
        intake.openSlowly();
        assertEquals(OPEN_PERCENT, io.percent, "é só um aviso: o pedido continua indo ao líder");
    }

    @Test
    void motorAndZeroProblemsComeBeforeTheFollower() {
        FakeIO io = replaceWithFake();
        io.followerOk = false;
        io.valid = false;
        tick(0.02);
        assertEquals(MechanismDiagnostics.MOTOR_SEM_RESPOSTA, intake.getDiagnosis());
    }

    @Test
    void theSimulatedIntakeHasItsFollowerConnected() {
        tick(0.02);
        assertNotEquals("SEGUIDOR_SEM_RESPOSTA", intake.getDiagnosis());
    }

    // Teste de 29/10/2026: a intake fazia barulho parada porque o PID relaxava e
    // retomava por histerese (20 a 40 trocas por segundo). Agora o modo só
    // depende de onde o braço está parado.
    @Test
    void holdAwayFromTheOpenStopIsContinuousAndNeverToggles() {
        FakeIO io = replaceWithFake();
        io.position = -5.0;
        tick(0.02);
        assertTrue(io.correcting, "fora do batente aberto o PID segura");
        assertEquals(-5.0, io.target, 1e-9);
        io.stopCalls = 0;
        int holdBefore = io.holdCalls;
        // Ruído do encoder e da velocidade em torno do alvo, incluindo o que
        // antes ficava dentro da faixa de relaxar e o que a ultrapassava.
        for (int i = 0; i < 200; i++) {
            io.position = -5.0 - 0.15 * Math.sin(i * 0.7);
            io.velocity = 2.0 * Math.sin(i * 1.3);
            tick(0.02);
            assertTrue(io.correcting, "ciclo " + i + ": PID não pode alternar com neutro");
        }
        assertEquals(0, io.stopCalls, "nenhuma troca para neutro");
        assertEquals(holdBefore + 200, io.holdCalls, "PID pedido em todos os ciclos");
        assertEquals(-5.0, io.target, 1e-9, "alvo fixo");
        intake.stop();
        assertEquals(-5.0, io.target, 1e-9, "stop repetido não arrasta o alvo");
    }

    @Test
    void atTheOpenStopOnlyBrakeNoPid() {
        FakeIO io = replaceWithFake();
        io.position = OPEN_ROTATIONS + 0.05;
        for (int i = 0; i < 50; i++) {
            tick(0.02);
        }
        assertFalse(io.correcting, "encostado no batente aberto: só Brake");
        assertEquals(0, io.holdCalls, "sem pedido de PID");
        assertTrue(io.stopCalls > 0);
    }

    @Test
    void brakeOnlyBandEndsWhereTheConstantSays() {
        FakeIO io = replaceWithFake();
        io.position = OPEN_ROTATIONS + BRAKE_ONLY_NEAR_OPEN_ROTATIONS - 0.01;
        tick(0.02);
        assertFalse(io.correcting, "dentro da faixa: só Brake");
        // Soltar um pouco mais longe do batente: o PID segura.
        intake.close();
        io = replaceWithFake();
        io.position = OPEN_ROTATIONS + BRAKE_ONLY_NEAR_OPEN_ROTATIONS + 0.05;
        tick(0.02);
        assertTrue(io.correcting, "fora da faixa: PID");
    }

    @Test
    void disabledAndInvalidSensorsStopAndReenableCapturesCurrentPosition() {
        FakeIO io = replaceWithFake();
        tick(0.02);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        io.position = -12.0; // Movida manualmente, com encoder ainda referenciado.
        tick(0.02);
        intake.stop();
        assertFalse(io.correcting);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        tick(0.02);
        assertTrue(io.correcting, "habilitar captura a posição atual e segura com PID");
        assertEquals(-12.0, io.target, 1e-9, "não retornar ao antigo zero ao habilitar");
        io.position = -12.25;
        tick(0.02);
        assertTrue(io.correcting);
        assertEquals(-12.0, io.target, 1e-9);
        io.valid = false;
        tick(0.02);
        assertFalse(io.correcting);
        intake.openSlowly();
        assertEquals(0.0, io.percent);
    }

    private FakeIO replaceWithFake() {
        intake.close();
        FakeIO io = new FakeIO();
        intake = new Intake(io);
        return io;
    }

    private static class FakeIO implements IntakeIO {
        double position;
        double velocity;
        double target;
        double percent;
        boolean correcting;
        boolean valid = true;
        boolean followerOk = true;
        int holdCalls;
        int stopCalls;

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
            inputs.connected = valid;
            inputs.followerConnected = followerOk;
            inputs.zeroValid = valid;
            inputs.positionRotations = position;
            inputs.velocityRotationsPerSec = velocity;
        }

        @Override
        public void holdPosition(double rotations) {
            target = rotations;
            correcting = true;
            holdCalls++;
        }

        @Override
        public void setPercentOutput(double output) {
            percent = output;
            correcting = false;
        }

        @Override
        public void stop() {
            percent = 0.0;
            correcting = false;
            stopCalls++;
        }
    }
}
