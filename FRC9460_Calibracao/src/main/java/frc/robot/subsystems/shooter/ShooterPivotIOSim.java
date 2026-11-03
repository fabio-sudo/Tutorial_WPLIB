package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.util.MechanismModel;

// #endregion

/**
 * Pivô no computador: física do motor + emulação do que o TalonFX faz.
 *
 * Separa duas coisas que no robô real também são diferentes:
 * - o ângulo FÍSICO do shooter (onde ele realmente está);
 * - o encoder do motor, que começa em 0 onde o shooter estava ao ligar.
 *
 * Se a equipe ligar o robô com o shooter fora do centro
 * (PIVOT_SIM_MANUAL_PLACEMENT_ERROR_DEGREES), o código continua achando que
 * o zero é o centro. A simulação mostra o efeito: mira deslocada e o batente
 * da fiação aparecendo antes do soft limit.
 *
 * Também é o modelo em paralelo do pivô, criado SEM erro de colocação: na
 * simulação, a diferença real x modelo mostra o efeito do zero manual errado.
 */
public class ShooterPivotIOSim implements ShooterPivotIO, MechanismModel {

    // #region 🧪 FÍSICA

    // Um ciclo do robô e o passo interno (o Talon controla a 1 kHz).
    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.001;
    private static final double BATTERY_VOLTS = 12.0;

    // Estado do DCMotorSim: ângulo do eixo de SAÍDA (depois da redução), em
    // radianos, medido a partir do ponto em que o encoder foi zerado.
    private final DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(PIVOT_SIM_MOTOR, PIVOT_SIM_MOI_KG_M2, PIVOT_GEAR_REDUCTION),
        PIVOT_SIM_MOTOR);

    // Ângulo físico (graus) em que o encoder vale zero. Começa no erro de
    // colocação: 0 = a equipe centralizou certo.
    private double encoderZeroPhysicalDegrees;

    // #endregion

    // #region 🎯🚀 CONTROLE - ESTADO DO TALONFX EMULADO (MOTION MAGIC)

    private enum Mode { STOPPED, PERCENT, MOTION_MAGIC }

    // O TrapezoidProfile limita velocidade e aceleração, como o Motion Magic.
    // O jerk (PIVOT_JERK_*) não entra: a curva real é um pouco mais suave.
    private final TrapezoidProfile profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(PIVOT_CRUISE_ROTATIONS_PER_SEC, PIVOT_ACCELERATION_ROTATIONS_PER_SEC2));

    private Mode mode = Mode.STOPPED;
    private TrapezoidProfile.State reference = new TrapezoidProfile.State();
    private double goalRotations;
    private double percent;
    private double appliedVolts;

    // #endregion

    // #region 🛑 SEGURANÇA - ESTADO DOS LIMITES

    private boolean forwardLimit;
    private boolean reverseLimit;
    private boolean onWiringStop;

    // #endregion

    // #region CONSTRUÇÃO

    public ShooterPivotIOSim() {
        this(PIVOT_SIM_MANUAL_PLACEMENT_ERROR_DEGREES);
    }

    /** Usado nos testes para simular um zero manual mal feito. */
    public ShooterPivotIOSim(double manualPlacementErrorDegrees) {
        encoderZeroPhysicalDegrees = manualPlacementErrorDegrees;
    }

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        double voltsSum = 0.0;
        for (int i = 0; i < steps; i++) {
            // Controle pede, a segurança do Talon limita, a física responde.
            appliedVolts = applySoftLimits(controllerVolts());
            sim.setInputVoltage(appliedVolts);
            sim.update(SUBSTEP_SECONDS);
            applyWiringStop();
            voltsSum += appliedVolts;
        }

        inputs.connected = true;
        inputs.zeroValid = true;
        inputs.motorPositionRotations = motorRotations();
        inputs.motorVelocityRotationsPerSec = motorVelocityRotationsPerSec();
        // Média do ciclo: é a tensão que o modelo em paralelo recebe.
        inputs.appliedVolts = voltsSum / steps;
        inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
        inputs.profileReferenceRotations = mode == Mode.MOTION_MAGIC ? reference.position : motorRotations();
        inputs.forwardLimitActive = forwardLimit;
        inputs.reverseLimitActive = reverseLimit;

        // Só existe na simulação: o ângulo que o robô real não consegue medir.
        Logger.recordOutput("Shooter/Simulacao/AnguloFisicoGraus", physicalDegrees());
        Logger.recordOutput("Shooter/Simulacao/ErroDoZeroManualGraus", encoderZeroPhysicalDegrees);
        Logger.recordOutput("Shooter/Simulacao/NoBatenteDaFiacao", onWiringStop);
    }

    // #endregion

    // #region 🎯🚀 CONTROLE - EMULAÇÃO DO MOTION MAGIC (PID DO TALONFX)

    /** Tensão que o controle do Talon pede neste passo, antes das proteções. */
    private double controllerVolts() {
        double position = motorRotations();
        return switch (mode) {
            case STOPPED -> 0.0;
            case PERCENT -> percent * BATTERY_VOLTS;
            case MOTION_MAGIC -> {
                // Perfil avança 1 ms; o kP corrige a diferença até a referência.
                reference = profile.calculate(SUBSTEP_SECONDS, reference,
                    new TrapezoidProfile.State(goalRotations, 0.0));
                // Este modelo não contém o atrito seco identificado no hardware.
                // Não injetar o kS físico sozinho em uma planta sem esse atrito.
                // O kV sim: a força contraeletromotriz existe no modelo do motor.
                double volts = PIVOT_KV_VOLTS_PER_ROTATION_PER_SEC * reference.velocity
                    + PIVOT_KP * (reference.position - position)
                    + PIVOT_KD * (reference.velocity - motorVelocityRotationsPerSec());
                // O Talon compara com a banda neutra o pedido COM kS. Fora dela, o
                // kS só cancela o atrito seco que este modelo não tem.
                yield Math.abs(volts) + PIVOT_KS_VOLTS < PIVOT_NEUTRAL_DEADBAND * BATTERY_VOLTS ? 0.0 : volts;
            }
        };
    }

    // #endregion

    // #region 🛑📏 SEGURANÇA - SOFT LIMITS DO TALONFX E BATENTE DA FIAÇÃO

    /** Soft limits do Talon: no fim do curso, bloqueia só o sentido que sai. */
    private double applySoftLimits(double volts) {
        double position = motorRotations();
        forwardLimit = position >= PIVOT_MAX_MOTOR_ROTATIONS;
        reverseLimit = position <= -PIVOT_MAX_MOTOR_ROTATIONS;
        if ((forwardLimit && volts > 0.0) || (reverseLimit && volts < 0.0)) {
            volts = 0.0;
        }
        return MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    /** Envelope da fiação no modelo, relativo ao centro REAL, não ao encoder. */
    private void applyWiringStop() {
        double physical = physicalDegrees();
        double limit = PIVOT_SIM_WIRING_LIMIT_DEGREES;
        double velocity = sim.getAngularVelocityRadPerSec() * PIVOT_DIRECTION_SIGN;
        onWiringStop = false;
        if ((physical > limit && velocity >= 0.0) || (physical < -limit && velocity <= 0.0)) {
            double clampedPhysical = MathUtil.clamp(physical, -limit, limit);
            sim.setState(Math.toRadians((clampedPhysical - encoderZeroPhysicalDegrees) * PIVOT_DIRECTION_SIGN), 0.0);
            onWiringStop = true;
        }
    }

    // #endregion

    // #region CONVERSÕES

    private double motorRotations() {
        return sim.getAngularPositionRotations() * PIVOT_GEAR_REDUCTION;
    }

    private double motorVelocityRotationsPerSec() {
        return sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * PIVOT_GEAR_REDUCTION;
    }

    /** Ângulo real do shooter em relação ao centro mecânico. */
    public double physicalDegrees() {
        return encoderZeroPhysicalDegrees
            + Math.toDegrees(sim.getAngularPositionRad()) * PIVOT_DIRECTION_SIGN;
    }

    // #endregion

    // #region 🧪 MODELO EM PARALELO (frc.robot.util.ParallelModel)

    // A mesma física (e o batente da fiação), recebendo a tensão MEDIDA no
    // TalonFX. Os soft limits não entram: o Talon real já corta a tensão.
    // Um passo de 20 ms: com a tensão constante no ciclo, a física linear é
    // exata sem subpassos (e fica leve para rodar no roboRIO).

    @Override
    public void setModelState(double motorRotations, double motorRotationsPerSec) {
        sim.setState(Units.rotationsToRadians(motorRotations / PIVOT_GEAR_REDUCTION),
            Units.rotationsToRadians(motorRotationsPerSec / PIVOT_GEAR_REDUCTION));
    }

    @Override
    public void stepModel(double volts) {
        sim.setInputVoltage(MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS));
        sim.update(LOOP_SECONDS);
        applyWiringStop();
    }

    @Override
    public double modelPosition() {
        return motorRotations();
    }

    @Override
    public double modelVelocity() {
        return motorVelocityRotationsPerSec();
    }

    @Override
    public double modelCurrentAmps() {
        return Math.abs(sim.getCurrentDrawAmps());
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setTargetRotations(double motorRotations) {
        if (!Double.isFinite(motorRotations)) {
            stop();
            return;
        }
        if (mode != Mode.MOTION_MAGIC) {
            // Como o Talon: o perfil parte de onde o motor está agora.
            reference = new TrapezoidProfile.State(motorRotations(), motorVelocityRotationsPerSec());
        }
        mode = Mode.MOTION_MAGIC;
        goalRotations = MathUtil.clamp(motorRotations, -PIVOT_MAX_MOTOR_ROTATIONS, PIVOT_MAX_MOTOR_ROTATIONS);
    }

    @Override
    public void setPercentOutput(double percent) {
        mode = Mode.PERCENT;
        this.percent = Double.isFinite(percent) ? MathUtil.clamp(percent, -1.0, 1.0) : 0.0;
    }

    @Override
    public void stop() {
        mode = Mode.STOPPED;
        percent = 0.0;
    }

    @Override
    public void zeroHere() {
        // A posição física atual vira o novo zero do encoder.
        encoderZeroPhysicalDegrees = physicalDegrees();
        sim.setState(0.0, sim.getAngularVelocityRadPerSec());
        stop();
    }

    // #endregion
}
