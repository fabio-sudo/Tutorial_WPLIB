package frc.robot.subsystems.intake;

// #region IMPORTS

import static frc.robot.subsystems.intake.IntakeConstants.*;

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
 * Articulador do intake: abre (desce) e fecha (sobe) o intake.
 *
 * Mesma lógica da competição: movimento lento enquanto o botão está
 * apertado, parada nos fins de curso e, sem comando, trava na posição atual
 * para não cair pelo peso: PID contínuo no meio do curso e no fechado; só
 * Brake encostado no batente aberto, onde a gravidade já segura o braço. Perto
 * do alvo o próprio Talon troca o PID por Brake (HOLD_NEUTRAL_DEADBAND), sem ruído.
 * O rolete que puxa a bola é o subsistema Roller.
 *
 * Ciclo (padrão do telescópio 9458): 1. sensores, 2. modelo em paralelo,
 * 3. segurança, 4. telemetria. O PID de segurar roda no TalonFX.
 */
public class Intake extends SubsystemBase implements AutoCloseable {

    // #region IO E ESTADO

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double commandedPercent;
    private double holdTarget = CLOSED_ROTATIONS;
    private boolean holding;
    // Retenção só com Brake (batente aberto); false = PID segurando a posição.
    private boolean brakeOnly;
    private String zeroStatus = "ZERADO_AO_LIGAR_FECHADO";
    private static final String FOLLOWER_LOST = "SEGUIDOR_SEM_RESPOSTA";

    // Física recebendo a tensão medida no líder (real x simulado) e diagnóstico.
    // Ângulo no log igual a AnguloSubidaGraus: 0° aberto, 90° fechado.
    private final ParallelModel model = new ParallelModel("Intake", IntakeIOSim::new,
            new ParallelModel.Unit("Graus", DEGREES_PER_MOTOR_ROTATION, OPEN_ROTATIONS),
            ParallelModel.Unit.of("GrausPorSeg", DEGREES_PER_MOTOR_ROTATION),
            MODEL_TOLERANCE_DEGREES, MODEL_REST_DEGREES_PER_SEC);
    private final MechanismDiagnostics diagnostics = new MechanismDiagnostics("Intake", "Intake",
            "TALONFX_ID_" + LEFT_MOTOR_ID + "_" + RIGHT_MOTOR_ID);

    public Intake(IntakeIO io) {
        this.io = io;
    }

    /** Replay: IO vazio. Robô: dois TalonFX. Computador: braço com gravidade. */
    public static IntakeIO createIO() {
        if (Logger.hasReplaySource())
            return new IntakeIO() {
            };
        if (RobotBase.isReal())
            return new IntakeIOTalonFX();
        return new IntakeIOSim();
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
        Logger.processInputs("Intake/Inputs", inputs);
    }

    // #endregion

    // #region 🧪 2. MODELO EM PARALELO - PASSO DA FÍSICA

    private void stepParallelModel() {
        model.update(inputs.connected && inputs.zeroValid, inputs.positionRotations,
                inputs.velocityRotationsPerSec, inputs.appliedVolts, inputs.currentAmps);
    }

    // #endregion

    // #region 🛑📏 3. SEGURANÇA - FIM DE CURSO E ZERO PERDIDO

    private void applySafety() {
        // Não manter um alvo antigo durante Disabled: ao habilitar, capturar
        // a posição atual, sem retornar automaticamente ao zero anterior.
        if (DriverStation.isDisabled() || !sensorsValid()) {
            commandedPercent = 0.0;
            holding = false;
            brakeOnly = false;
            io.stop();
            return;
        }
        // Proteção do código antigo: chegou no fim do curso, para e segura.
        if ((commandedPercent < 0.0 && reachedOpenLimit())
                || (commandedPercent > 0.0 && reachedCloseLimit())) {
            stop();
        }
        if (commandedPercent == 0.0) {
            if (!holding)
                stop();
            else
                updateHold();
        }
    }

    /**
     * Sem histerese: o modo depende só de ONDE o braço está parado. No batente
     * aberto a gravidade o encosta no batente e o Brake basta; nas demais
     * posições o PID segue pedido, e o Talon só aplica tensão fora da banda neutra.
     */
    private void updateHold() {
        brakeOnly = holdTarget <= OPEN_ROTATIONS + BRAKE_ONLY_NEAR_OPEN_ROTATIONS;
        if (brakeOnly)
            io.stop();
        else
            io.holdPosition(holdTarget);
    }

    // #endregion

    // #region 📊 4. TELEMETRIA (NOMES USADOS PELOS LAYOUTS)

    private void logTelemetry() {
        Logger.recordOutput("Intake/PosicaoVoltas", inputs.positionRotations);
        Logger.recordOutput("Intake/AnguloSubidaGraus", getRaiseDegrees());
        Logger.recordOutput("Intake/Aberto", isOpen());
        Logger.recordOutput("Intake/Fechado", isClosed());
        Logger.recordOutput("Intake/Segurando", holding);
        Logger.recordOutput("Intake/AlvoSegurarVoltas", holdTarget);
        Logger.recordOutput("Intake/RetencaoEmRepouso", holding && brakeOnly);
        Logger.recordOutput("Intake/CorrecaoPosicaoAtiva", holding && !brakeOnly);
        Logger.recordOutput("Intake/ErroRetencaoVoltas", holdTarget - inputs.positionRotations);
        Logger.recordOutput("Intake/SaidaComandada", commandedPercent);
        Logger.recordOutput("Intake/ZeroValido", inputs.zeroValid);
        Logger.recordOutput("Intake/ZeroResultado", getZeroResult());
        Logger.recordOutput("Intake/ReducaoTotal", GEAR_REDUCTION);
        Logger.recordOutput("Intake/VoltasSaida", inputs.positionRotations / GEAR_REDUCTION);
        Logger.recordOutput("Intake/LimiteMecanicoVoltasRotor", MECHANICAL_OPEN_ROTATIONS);
        Logger.recordOutput("Intake/LimiteOperacionalVoltasRotor", OPEN_ROTATIONS);
        Logger.recordOutput("Intake/MargemSegurancaGraus",
                OPEN_SAFETY_MARGIN_ROTATIONS * DEGREES_PER_MOTOR_ROTATION);
        Logger.recordOutput("Intake/Comando", MechanismDiagnostics.commandName(this));
        model.log();
        diagnostics.update(inputs.connected && inputs.followerConnected, getDiagnosis());
    }

    // #endregion

    // #region 🚀 CONTROLE - ABRIR E FECHAR (MALHA ABERTA)

    // Antes de mover, a segurança confere o fim de curso (região abaixo).
    public void openSlowly() {
        if (reachedOpenLimit()) {
            stop();
            return;
        }
        move(OPEN_PERCENT);
    }

    public void closeSlowly() {
        if (reachedCloseLimit()) {
            stop();
            return;
        }
        move(CLOSE_PERCENT);
    }

    private void move(double percent) {
        if (!sensorsValid() || DriverStation.isDisabled()) {
            holding = false;
            commandedPercent = 0.0;
            io.stop();
            return;
        }
        commandedPercent = percent;
        holding = false;
        brakeOnly = false;
        io.setPercentOutput(percent);
    }

    // #endregion

    // #region 🎯 CONTROLE - PARAR E SEGURAR (PID NO TALONFX)

    /** Para e trava onde está (ou só desliga, se o "segurar" estiver desligado). */
    public void stop() {
        commandedPercent = 0.0;
        if (!HOLD_ENABLED || !sensorsValid() || DriverStation.isDisabled()) {
            holding = false;
            brakeOnly = false;
            io.stop();
            return;
        }
        // Repetir stop no limite não deve arrastar o alvo a cada ciclo.
        if (!holding) {
            holdTarget = edu.wpi.first.math.MathUtil.clamp(inputs.positionRotations,
                    OPEN_ROTATIONS, CLOSED_ROTATIONS);
            holding = true;
            brakeOnly = false;
        }
        updateHold();
    }

    // #endregion

    // #region 🛑📏 SEGURANÇA - FIM DE CURSO

    private boolean reachedCloseLimit() {
        return CLOSE_LIMIT_ENABLED
                && inputs.positionRotations >= CLOSED_ROTATIONS - CLOSE_BUFFER_ROTATIONS;
    }

    private boolean sensorsValid() {
        return inputs.connected && inputs.zeroValid && Double.isFinite(inputs.positionRotations)
                && Double.isFinite(inputs.velocityRotationsPerSec)
                && Double.isFinite(inputs.appliedVolts) && Double.isFinite(inputs.currentAmps);
    }

    private boolean reachedOpenLimit() {
        return inputs.positionRotations <= OPEN_ROTATIONS + OPEN_BUFFER_ROTATIONS;
    }

    // #endregion

    // #region CONSULTAS

    /**
     * Aberto = no limite operacional ou além dele. Encostado no batente só com
     * Brake, o braço assenta até OPEN_SAFETY_MARGIN_ROTATIONS (0,5 volta) além
     * do limite: um teste de dois lados (±0,40) o daria como "não aberto".
     */
    public boolean isOpen() {
        return inputs.positionRotations <= OPEN_ROTATIONS + TOLERANCE_ROTATIONS;
    }

    public boolean isClosed() {
        return Math.abs(inputs.positionRotations - CLOSED_ROTATIONS) <= TOLERANCE_ROTATIONS;
    }

    /** 0° = aberto (posição do CAD), 90° = fechado. Usado no desenho 3D. */
    public double getRaiseDegrees() {
        return (inputs.positionRotations - OPEN_ROTATIONS) * DEGREES_PER_MOTOR_ROTATION;
    }

    public double getModeledRaiseDegrees() {
        return model.getModeledPositionConverted();
    }

    /** Mais grave primeiro: sem líder ou sem zero o intake para; sem seguidor ele só enfraquece. */
    public String getDiagnosis() {
        if (inputs.connected && inputs.zeroValid && !inputs.followerConnected) {
            return FOLLOWER_LOST;
        }
        return MechanismDiagnostics.result(inputs.connected, inputs.zeroValid, model.isDiverging());
    }

    // #endregion

    // #region 🧭 ZERO MANUAL

    /**
     * Robô desabilitado e intake fechado à mão: a posição atual vira 0.
     * true = pedido aceito; o resultado real aparece em getZeroResult().
     */
    public boolean zeroClosed() {
        if (!DriverStation.isDisabled()) {
            zeroStatus = "RECUSADO_ROBO_HABILITADO";
            return false;
        }
        io.zeroClosed();
        holdTarget = CLOSED_ROTATIONS;
        zeroStatus = "ZERADO_PELA_EQUIPE";
        return true;
    }

    /** 🧭 Resultado do zero para o log: nunca diz "zerado" com o zero inválido. */
    public String getZeroResult() {
        return MechanismDiagnostics.zeroResult(inputs.zeroValid, zeroStatus);
    }

    // #endregion

    // #region COMANDOS

    /** Segurar: abre devagar. Soltou: trava onde está. */
    public Command openCommand() {
        return runEnd(this::openSlowly, this::stop).withName("Intake - abrir");
    }

    /** Segurar: fecha devagar. Soltou: trava onde está. */
    public Command closeCommand() {
        return runEnd(this::closeSlowly, this::stop).withName("Intake - fechar");
    }

    public Command zeroClosedCommand() {
        return runOnce(this::zeroClosed).ignoringDisable(true).withName("Intake - zerar fechado");
    }

    // #endregion

    // #region ENCERRAMENTO (TESTES)

    @Override
    public void close() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    // #endregion
}
