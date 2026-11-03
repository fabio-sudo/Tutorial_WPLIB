package frc.robot.subsystems.intake;

// #region IMPORTS

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.util.ArmModel;
import frc.robot.util.MechanismModel;

// #endregion

/**
 * Articulador no computador: braço com GRAVIDADE (SingleJointedArmSim) e a
 * mesma lógica do TalonFX (saída direta, PID de segurar e soft limits).
 *
 * Mostra o problema que o código antigo resolveu: sem o PID de segurar, o
 * intake aberto ou no meio do curso desce pelo próprio peso.
 * Também é o modelo em paralelo do intake (frc.robot.util.ParallelModel).
 */
public class IntakeIOSim implements IntakeIO, MechanismModel {

    // #region 🧪 FÍSICA

    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.001;
    private static final double BATTERY_VOLTS = 12.0;

    // Ângulo do braço em relação à horizontal (positivo para cima).
    // Aberto operacional: -10° (CAD). Fechado soma o curso calibrado.
    private static final double OPEN_ARM_RADIANS = Math.toRadians(SIM_OPEN_ARM_ANGLE_DEGREES);
    private static final double CLOSED_ARM_RADIANS = Math.toRadians(SIM_OPEN_ARM_ANGLE_DEGREES + CLOSED_RAISE_DEGREES);

    private static final double MOI_KG_M2 = SingleJointedArmSim.estimateMOI(SIM_ARM_LENGTH_METERS, SIM_MASS_KG);
    private static final double MIN_ARM_RADIANS = OPEN_ARM_RADIANS
            - Math.toRadians(OPEN_SAFETY_MARGIN_ROTATIONS * DEGREES_PER_MOTOR_ROTATION);

    // O robô liga com o intake fechado (zero manual).
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            SIM_MOTORS, GEAR_REDUCTION, MOI_KG_M2, SIM_ARM_LENGTH_METERS,
            MIN_ARM_RADIANS, CLOSED_ARM_RADIANS, true, CLOSED_ARM_RADIANS);

    // Modelo em paralelo: a mesma física, sem criar objetos (roda no robô real).
    private final ArmModel model = new ArmModel(
            SIM_MOTORS, GEAR_REDUCTION, MOI_KG_M2, SIM_ARM_LENGTH_METERS, MIN_ARM_RADIANS, CLOSED_ARM_RADIANS);

    // #endregion

    // #region 🎯 CONTROLE - ESTADO DO TALONFX EMULADO

    private enum Mode {
        STOPPED, PERCENT, HOLD
    }

    private Mode mode = Mode.STOPPED;
    private double percent;
    private double holdTarget = CLOSED_ROTATIONS;
    private double appliedVolts;

    // #endregion

    // #region 🛑📏 SEGURANÇA - ESTADO DOS SOFT LIMITS

    private boolean closeLimit;
    private boolean openLimit;

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        double voltsSum = 0.0;
        for (int i = 0; i < steps; i++) {
            appliedVolts = talonVolts();
            sim.setInputVoltage(appliedVolts);
            sim.update(SUBSTEP_SECONDS);
            voltsSum += appliedVolts;
        }
        inputs.connected = true;
        inputs.followerConnected = true;
        inputs.zeroValid = true;
        inputs.positionRotations = rotations();
        inputs.velocityRotationsPerSec = rotationsPerSec();
        // Média do ciclo: é a tensão que o modelo em paralelo recebe.
        inputs.appliedVolts = voltsSum / steps;
        // Corrente de UM motor (o líder), como o IntakeIOTalonFX lê.
        inputs.currentAmps = currentPerMotor();
        inputs.closeLimitActive = closeLimit;
        inputs.openLimitActive = openLimit;
    }

    /** O que o Talon aplica: segurança primeiro, depois o controle limitado. */
    private double talonVolts() {
        // Segurança: desabilitado, o Talon não aplica tensão (o pedido fica guardado).
        if (DriverStation.isDisabled()) {
            return 0.0;
        }
        return applySoftLimits(controllerVolts());
    }

    // #endregion

    // #region 🎯 CONTROLE - EMULAÇÃO DO TALONFX (SAÍDA DIRETA E PID DE SEGURAR)

    private double controllerVolts() {
        double position = rotations();
        return switch (mode) {
            case STOPPED -> 0.0;
            case PERCENT -> percent * BATTERY_VOLTS;
            case HOLD -> holdVolts(position);
        };
    }

    /** PositionVoltage com Elevator_Static: kP × erro + kG; dentro da banda, Brake. */
    private double holdVolts(double position) {
        double volts = MathUtil.clamp(HOLD_KP * (holdTarget - position) + HOLD_KG,
                -HOLD_MAX_VOLTS, HOLD_MAX_VOLTS);
        return Math.abs(volts) < HOLD_NEUTRAL_DEADBAND * BATTERY_VOLTS ? 0.0 : volts;
    }

    // #endregion

    // #region 🛑📏 SEGURANÇA - SOFT LIMITS DO TALONFX

    /** Soft limits: bloqueia só o sentido que sai do curso. */
    private double applySoftLimits(double volts) {
        double position = rotations();
        closeLimit = CLOSE_LIMIT_ENABLED && position >= CLOSED_ROTATIONS;
        openLimit = position <= OPEN_ROTATIONS;
        if ((closeLimit && volts > 0.0) || (openLimit && volts < 0.0)) {
            volts = 0.0;
        }
        return MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    // #endregion

    // #region CONVERSÕES

    /** Ângulo do braço convertido para as voltas do encoder do motor. */
    private double rotations() {
        return toRotations(sim.getAngleRads());
    }

    private double rotationsPerSec() {
        return toRotationsPerSec(sim.getVelocityRadPerSec());
    }

    private double currentPerMotor() {
        return Math.abs(sim.getCurrentDrawAmps()) / SIM_MOTOR_COUNT;
    }

    private static double toRotations(double armRadians) {
        return OPEN_ROTATIONS + Math.toDegrees(armRadians - OPEN_ARM_RADIANS) / DEGREES_PER_MOTOR_ROTATION;
    }

    private static double toRotationsPerSec(double armRadPerSec) {
        return Math.toDegrees(armRadPerSec) / DEGREES_PER_MOTOR_ROTATION;
    }

    // #endregion

    // #region 🧪 MODELO EM PARALELO (frc.robot.util.ParallelModel)

    // O mesmo braço com gravidade, recebendo a tensão MEDIDA no líder (o
    // seguidor recebe a mesma). Os soft limits não entram: o Talon já corta.
    // Subpassos de 1 ms: com um passo só de 20 ms, a física não linear do braço
    // erra a posição (~0,3° por ciclo); IntakeModelAccuracyTest segura a precisão.
    // Usa ArmModel (sem objetos): com o SingleJointedArmSim, este modelo criava
    // ~0,5 MB por ciclo no robô real e o coletor de lixo estourava o laço.

    @Override
    public void setModelState(double rotations, double rotationsPerSec) {
        model.setState(
                OPEN_ARM_RADIANS + Math.toRadians((rotations - OPEN_ROTATIONS) * DEGREES_PER_MOTOR_ROTATION),
                Math.toRadians(rotationsPerSec * DEGREES_PER_MOTOR_ROTATION));
    }

    @Override
    public void stepModel(double volts) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        model.setInputVoltage(MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS));
        for (int i = 0; i < steps; i++) {
            model.update(SUBSTEP_SECONDS);
        }
    }

    @Override
    public double modelPosition() {
        return toRotations(model.getAngleRads());
    }

    @Override
    public double modelVelocity() {
        return toRotationsPerSec(model.getVelocityRadPerSec());
    }

    @Override
    public double modelCurrentAmps() {
        return Math.abs(model.getCurrentDrawAmps()) / SIM_MOTOR_COUNT;
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setPercentOutput(double percent) {
        mode = Mode.PERCENT;
        this.percent = Double.isFinite(percent) ? MathUtil.clamp(percent, -1.0, 1.0) : 0.0;
    }

    @Override
    public void holdPosition(double rotations) {
        mode = Mode.HOLD;
        holdTarget = rotations;
    }

    @Override
    public void stop() {
        mode = Mode.STOPPED;
    }

    // #endregion
}
