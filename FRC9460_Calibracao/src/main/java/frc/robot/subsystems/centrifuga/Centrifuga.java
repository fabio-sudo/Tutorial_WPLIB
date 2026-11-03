package frc.robot.subsystems.centrifuga;

// #region IMPORTS

import static frc.robot.subsystems.centrifuga.CentrifugaConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.MechanismDiagnostics;
import frc.robot.util.ParallelModel;

// #endregion

/**
 * Centrífuga: gira e empurra a bola do depósito para a tower do shooter.
 *
 * Quem decide QUANDO ela liga é a sequência de tiro do shooter: só depois
 * que pivô, top e tower estão prontos. Aqui fica só o controle de RPM.
 *
 * Ciclo (padrão do telescópio 9458): 1. sensores, 2. modelo em paralelo,
 * 3. segurança, 4. telemetria. O PID de velocidade roda no TalonFX.
 */
public class Centrifuga extends SubsystemBase implements AutoCloseable {

    // #region IO E ESTADO

    private final CentrifugaIO io;
    private final CentrifugaIOInputsAutoLogged inputs = new CentrifugaIOInputsAutoLogged();
    private double targetRpm;
    private String status = "PARADO";
    // Teste sem trava: saída direta, sem conferir sensores. Sem botão desde 31/10/2026.
    private boolean openLoopTest;

    // Física recebendo a tensão medida no motor (real x simulado) e diagnóstico.
    private final ParallelModel model = new ParallelModel("Centrifuga", CentrifugaIOSim::new,
        null, ParallelModel.Unit.of("Rpm", 1.0), MODEL_TOLERANCE_RPM);
    private final MechanismDiagnostics diagnostics =
        new MechanismDiagnostics("Centrifuga", "Centrífuga", "TALONFX_ID_" + MOTOR_ID);
    // Mesmo aviso do telescópio 9458: aparece no Elastic (aba Alerts).
    private final Alert controlAlert =
        new Alert("Centrífuga: falha ao enviar comando ao TalonFX " + MOTOR_ID + ".", AlertType.kError);

    public Centrifuga(CentrifugaIO io) {
        this.io = io;
    }

    /** Replay: IO vazio. Robô: TalonFX. Computador: física. */
    public static CentrifugaIO createIO() {
        if (Logger.hasReplaySource()) return new CentrifugaIO() {};
        if (RobotBase.isReal()) return new CentrifugaIOTalonFX();
        return new CentrifugaIOSim();
    }

    // #endregion

    // #region 🔁 PERIODIC - ORDEM DE CADA CICLO (20 ms)

    @Override
    public void periodic() {
        readSensors();
        stepParallelModel();
        applySafety();
        logTelemetry();
    }

    // #endregion

    // #region 🔁 1. SENSORES

    private void readSensors() {
        io.updateInputs(inputs);
        Logger.processInputs("Centrifuga/Inputs", inputs);
    }

    // #endregion

    // #region 🧪 2. MODELO EM PARALELO - PASSO DA FÍSICA

    private void stepParallelModel() {
        model.update(inputs.connected, 0.0, inputs.velocityRpm, inputs.appliedVolts, inputs.currentAmps);
    }

    // #endregion

    // #region 🛑 3. SEGURANÇA - DESABILITADO

    private void applySafety() {
        // Teste sem trava: só o robô desabilitado para o motor (o comando reenvia a saída).
        if (openLoopTest) {
            if (DriverStation.isDisabled()) {
                openLoopTest = false;
                status = "DESABILITADO";
                io.stop();
            }
            return;
        }
        // Desabilitado: não volta a girar sozinha ao habilitar.
        // Falha de comando: o stop() é o novo pedido; aceito, o próximo ciclo libera o RPM.
        if (DriverStation.isDisabled() || !sensorsValid()) {
            targetRpm = 0.0;
            status = DriverStation.isDisabled() ? "DESABILITADO"
                : !inputs.controlRequestOK ? "FALHA_COMANDO_MOTOR" : "SENSOR_INDISPONIVEL";
            io.stop();
        } else if (targetRpm == 0.0) {
            // Continua freando depois de soltar o B e alimenta o MotorSafety.
            // O alvo nunca volta a ser positivo sem um novo comando.
            io.setVelocityRpm(0.0);
        }
    }

    // #endregion

    // #region 📊 4. TELEMETRIA (NOMES USADOS PELOS LAYOUTS)

    private void logTelemetry() {
        Logger.recordOutput("Centrifuga/AlvoRpm", targetRpm);
        Logger.recordOutput("Centrifuga/Rpm", inputs.velocityRpm);
        Logger.recordOutput("Centrifuga/NoRpm", isAtSpeed());
        Logger.recordOutput("Centrifuga/Ligada", isRunning() || openLoopTest);
        Logger.recordOutput("Centrifuga/TesteSemTrava", openLoopTest);
        Logger.recordOutput("Centrifuga/Comando", MechanismDiagnostics.commandName(this));
        Logger.recordOutput("Centrifuga/Estado", status);
        Logger.recordOutput("Centrifuga/ErroRpm", targetRpm - inputs.velocityRpm);
        Logger.recordOutput("Centrifuga/ReferenciaRpm", inputs.referenceRpm);
        Logger.recordOutput("Centrifuga/ReducaoTotal", GEAR_REDUCTION);
        Logger.recordOutput("Centrifuga/SaidaRpm", inputs.velocityRpm / GEAR_REDUCTION);
        Logger.recordOutput("Centrifuga/SaidaVoltas", inputs.rotorPositionRotations / GEAR_REDUCTION);
        model.log();
        controlAlert.set(!inputs.controlRequestOK);
        diagnostics.update(inputs.connected,
            MechanismDiagnostics.result(inputs.connected, true, model.isDiverging()));
    }

    // #endregion

    // #region 🎯🚀 CONTROLE - RPM (PID DE VELOCIDADE NO TALONFX)

    // O PID roda dentro do TalonFX (ganhos em CentrifugaConstants,
    // configuração em CentrifugaIOTalonFX). Aqui só se escolhe o RPM.

    /** Pedido limitado ao envelope do mecanismo, com sensores válidos. */
    public void setRpm(double rpm) {
        openLoopTest = false;
        if (DriverStation.isDisabled() || !sensorsValid() || !Double.isFinite(rpm)) {
            targetRpm = 0.0;
            status = "PEDIDO_RECUSADO";
            io.stop();
            return;
        }
        targetRpm = MathUtil.clamp(rpm, 0.0, MAX_COMMAND_RPM);
        status = targetRpm > 0.0 ? "CONTROLE_RPM" : "FREANDO";
        io.setVelocityRpm(targetRpm);
    }

    // #endregion

    // #region 🛑 SEGURANÇA - PARADA FREANDO

    /**
     * Para FREANDO: pede 0 RPM no controle de velocidade, que aplica tensão
     * contra o giro. Só soltar (neutro/Brake) deixava a centrífuga girando
     * ~0,6 s e empurrando bolas depois do fim do tiro (visto no log de
     * 27/10, com bolas saindo com o pivô ainda mirando).
     */
    public void stop() {
        openLoopTest = false;
        targetRpm = 0.0;
        status = "PARADO";
        if (DriverStation.isDisabled() || !sensorsValid()) io.stop();
        else io.setVelocityRpm(0.0);
    }

    // #endregion

    // #region CONSULTAS

    public boolean isAtSpeed() {
        return sensorsValid() && Math.abs(targetRpm - inputs.velocityRpm) <= VELOCITY_TOLERANCE_RPM;
    }

    private boolean sensorsValid() {
        return inputs.connected && inputs.controlRequestOK && Double.isFinite(inputs.velocityRpm)
            && Double.isFinite(inputs.appliedVolts) && Double.isFinite(inputs.currentAmps);
    }

    /** Pediram RPM maior que zero: a centrífuga está empurrando bolas. */
    public boolean isRunning() {
        return targetRpm > 0.0;
    }

    public double getRpm() {
        return inputs.velocityRpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public boolean canUnjam() {
        return sensorsValid() && inputs.positionValid
            && Double.isFinite(inputs.rotorPositionRotations) && !DriverStation.isDisabled();
    }

    public double getOutputRotations() { return inputs.rotorPositionRotations / GEAR_REDUCTION; }
    public double getCurrentAmps() { return Math.abs(inputs.currentAmps); }

    /** Reversão restrita à baixa velocidade da preparação; setRpm normal continua positivo. */
    public void reverseForUnjam() {
        if (!canUnjam()) { stop(); return; }
        openLoopTest = false;
        targetRpm = -UNJAM_ROTOR_RPM;
        status = "DESTRAVANDO";
        io.setVelocityRpm(targetRpm);
    }

    // #endregion

    // #region COMANDOS

    /**
     * Sem botão desde 31/10/2026. Saída direta para a FRENTE, como no Tuner X,
     * sem as travas do tiro nem de sensor. Só para testar motor e sentido.
     * Ao terminar: freia. Desabilitado: para.
     */
    public Command openLoopTestCommand() {
        return runEnd(this::runOpenLoopTest, this::stop).withName("Centrifuga - teste sem trava (Start)");
    }

    private void runOpenLoopTest() {
        if (DriverStation.isDisabled()) {
            stop();
            return;
        }
        openLoopTest = true;
        targetRpm = 0.0;
        status = "TESTE_SEM_TRAVA";
        io.setPercentOutput(OPEN_LOOP_TEST_PERCENT);
    }

    /** Estado da centrífuga ("CONTROLE_RPM", "PEDIDO_RECUSADO"...), para os avisos. */
    public String getStatus() {
        return status;
    }

    /** Resultado da configuração do Talon (ou do último reinício dele). */
    public String getConfigurationStatus() {
        return inputs.configurationStatus;
    }

    public boolean isOpenLoopTestActive() {
        return openLoopTest;
    }

    /** Segurar: gira no RPM de teste da competição (2300). */
    public Command testCommand() {
        return runEnd(() -> setRpm(TEST_RPM), this::stop).withName("Centrifuga - teste 2300 RPM");
    }

    // #endregion

    // #region ENCERRAMENTO (TESTES)

    @Override
    public void close() {
        controlAlert.close();
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    // #endregion
}
