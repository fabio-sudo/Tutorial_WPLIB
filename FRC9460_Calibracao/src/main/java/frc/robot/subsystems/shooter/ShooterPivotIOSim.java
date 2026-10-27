package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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
 */
public class ShooterPivotIOSim implements ShooterPivotIO {

    // #region FÍSICA

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

    // #region EMULAÇÃO DO TALONFX

    private enum Mode { STOPPED, PERCENT, MOTION_MAGIC }

    // O TrapezoidProfile limita velocidade e aceleração, como o Motion Magic.
    // O jerk (60 rot/s³) não entra: a curva real é um pouco mais suave.
    private final TrapezoidProfile profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(PIVOT_CRUISE_ROTATIONS_PER_SEC, PIVOT_ACCELERATION_ROTATIONS_PER_SEC2));

    private Mode mode = Mode.STOPPED;
    private TrapezoidProfile.State reference = new TrapezoidProfile.State();
    private double goalRotations;
    private double percent;
    private double appliedVolts;
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

    // #region CICLO DA SIMULAÇÃO

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        for (int i = 0; i < steps; i++) {
            appliedVolts = controlVolts();
            sim.setInputVoltage(appliedVolts);
            sim.update(SUBSTEP_SECONDS);
            applyWiringStop();
        }

        inputs.connected = true;
        inputs.zeroValid = true;
        inputs.motorPositionRotations = motorRotations();
        inputs.motorVelocityRotationsPerSec = motorVelocityRotationsPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
        inputs.profileReferenceRotations = mode == Mode.MOTION_MAGIC ? reference.position : motorRotations();
        inputs.forwardLimitActive = forwardLimit;
        inputs.reverseLimitActive = reverseLimit;

        // Só existe na simulação: o ângulo que o robô real não consegue medir.
        Logger.recordOutput("Shooter/Simulacao/AnguloFisicoGraus", physicalDegrees());
        Logger.recordOutput("Shooter/Simulacao/ErroDoZeroManualGraus", encoderZeroPhysicalDegrees);
        Logger.recordOutput("Shooter/Simulacao/NoBatenteDaFiacao", onWiringStop);
    }

    /** Tensão que o Talon aplicaria neste passo. */
    private double controlVolts() {
        double position = motorRotations();
        double volts = switch (mode) {
            case STOPPED -> 0.0;
            case PERCENT -> percent * BATTERY_VOLTS;
            case MOTION_MAGIC -> {
                // Perfil avança 1 ms; o kP corrige a diferença até a referência.
                reference = profile.calculate(SUBSTEP_SECONDS, reference,
                    new TrapezoidProfile.State(goalRotations, 0.0));
                yield PIVOT_KP * (reference.position - position)
                    + PIVOT_KD * (reference.velocity - motorVelocityRotationsPerSec());
            }
        };

        // Soft limits do Talon: no fim do curso, bloqueia só o sentido que sai.
        forwardLimit = position >= PIVOT_MAX_MOTOR_ROTATIONS;
        reverseLimit = position <= -PIVOT_MAX_MOTOR_ROTATIONS;
        if ((forwardLimit && volts > 0.0) || (reverseLimit && volts < 0.0)) {
            volts = 0.0;
        }
        return MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    /** A fiação física para o shooter em ±90° do centro REAL, não do encoder. */
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
