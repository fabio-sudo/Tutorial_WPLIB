package frc.robot.subsystems.roller;

// #region IMPORTS

import static frc.robot.subsystems.roller.RollerConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.MechanismDiagnostics;
import frc.robot.util.ParallelModel;

// #endregion

/**
 * Rolete do intake: puxa a bola para dentro do robô.
 * Malha aberta a 30%, como na competição. A corrente do motor fica gravada:
 * é por ela que o FuelTracker conta as coletas.
 *
 * Ciclo (padrão do telescópio 9458): 1. sensores, 2. modelo em paralelo,
 * 3. segurança, 4. telemetria. Sem PID: malha aberta.
 */
public class Roller extends SubsystemBase implements AutoCloseable {

    // #region IO E ESTADO

    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
    private double commandedPercent;

    // Física recebendo a tensão medida no motor (real x simulado) e diagnóstico.
    // O modelo não tem bolas: a diferença de corrente mostra cada coleta.
    private final ParallelModel model = new ParallelModel("Roller", RollerIOSim::new,
        null, ParallelModel.Unit.of("Rpm", 1.0), MODEL_TOLERANCE_RPM);
    private final MechanismDiagnostics diagnostics =
        new MechanismDiagnostics("Roller", "Rolete do intake", "SPARKFLEX_ID_" + MOTOR_ID);

    public Roller(RollerIO io) {
        this.io = io;
    }

    /** Replay: IO vazio. Robô: Spark Flex. Computador: física. */
    public static RollerIO createIO() {
        if (Logger.hasReplaySource()) return new RollerIO() {};
        if (RobotBase.isReal()) return new RollerIOSparkFlex();
        return new RollerIOSim();
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
        Logger.processInputs("Roller/Inputs", inputs);
    }

    // #endregion

    // #region 🧪 2. MODELO EM PARALELO - PASSO DA FÍSICA

    private void stepParallelModel() {
        model.update(inputs.connected, 0.0, inputs.velocityRpm, inputs.appliedVolts, inputs.currentAmps);
    }

    // #endregion

    // #region 🛑 3. SEGURANÇA - DESABILITADO

    private void applySafety() {
        // Desabilitado: não volta a puxar sozinho ao habilitar.
        if (DriverStation.isDisabled()) {
            stop();
        }
    }

    // #endregion

    // #region 📊 4. TELEMETRIA (NOMES USADOS PELOS LAYOUTS)

    private void logTelemetry() {
        Logger.recordOutput("Roller/SaidaComandada", commandedPercent);
        Logger.recordOutput("Roller/Puxando", isPulling());
        Logger.recordOutput("Roller/Rpm", inputs.velocityRpm);
        Logger.recordOutput("Roller/CorrenteAmps", inputs.currentAmps);
        Logger.recordOutput("Roller/Comando", MechanismDiagnostics.commandName(this));
        model.log();
        diagnostics.update(inputs.connected,
            MechanismDiagnostics.result(inputs.connected, true, model.isDiverging()));
    }

    // #endregion

    // #region 🚀 CONTROLE - PUXAR (MALHA ABERTA, SEM PID)

    public void pull() {
        commandedPercent = PULL_PERCENT;
        io.setPercentOutput(commandedPercent);
    }

    // #endregion

    // #region 🛑 SEGURANÇA - PARADA

    public void stop() {
        commandedPercent = 0.0;
        io.stop();
    }

    // #endregion

    // #region CONSULTAS E SIMULAÇÃO

    /** Pediram para puxar (saída positiva). */
    public boolean isPulling() {
        return commandedPercent > 0.0;
    }

    public double getRpm() {
        return inputs.velocityRpm;
    }

    public double getCurrentAmps() {
        return inputs.currentAmps;
    }

    /** Simulação: uma bola entrou e freia o rolo (a corrente sobe). */
    public void simulateBallIntake() {
        io.simulateBallIntake();
    }

    // #endregion

    // #region COMANDOS

    /** Segurar: puxa bolas. Soltou: para. */
    public Command pullCommand() {
        return runEnd(this::pull, this::stop).withName("Roller - puxar");
    }

    // #endregion

    // #region ENCERRAMENTO (TESTES)

    @Override
    public void close() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    // #endregion
}
