package frc.robot.subsystems.centrifuga;

// #region IMPORTS

import static frc.robot.subsystems.centrifuga.CentrifugaConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.util.MechanismModel;

// #endregion

/**
 * Centrífuga no computador: volante (FlywheelSim) + o mesmo cálculo de tensão
 * que o TalonFX faz no VelocityVoltage, com os ganhos da competição.
 * Também é o modelo em paralelo da centrífuga (frc.robot.util.ParallelModel).
 */
public class CentrifugaIOSim implements CentrifugaIO, MechanismModel {

    // #region 🧪 FÍSICA

    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.001;
    private static final double BATTERY_VOLTS = 12.0;

    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(SIM_MOTOR, SIM_MOI_KG_M2, GEAR_REDUCTION), SIM_MOTOR);

    private boolean active;
    private boolean openLoop;
    private double percent;
    private double targetRpm;
    private double appliedVolts;
    private double profileRpm;
    private double rotorPosition;

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    @Override
    public void updateInputs(CentrifugaIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        double voltsSum = 0.0;
        for (int i = 0; i < steps; i++) {
            profileRpm = targetRpm == 0.0 ? 0.0 : profileRpm + MathUtil.clamp(
                targetRpm - profileRpm, -ACCELERATION_ROTATIONS_PER_SEC2 * 60.0 * SUBSTEP_SECONDS,
                ACCELERATION_ROTATIONS_PER_SEC2 * 60.0 * SUBSTEP_SECONDS);
            // Alvo 0 RPM: o controle freia ativamente. Sem pedido (neutro), o
            // Brake só curto-circuita o motor (0 V) e a peça demora a parar.
            // Controle pede, a segurança do Talon limita a corrente.
            appliedVolts = !DriverStation.isEnabled() ? 0.0
                : openLoop ? limitStatorCurrent(percent * BATTERY_VOLTS)
                : active ? limitStatorCurrent(controlVolts(sim.getAngularVelocityRPM() * GEAR_REDUCTION)) : 0.0;
            sim.setInputVoltage(appliedVolts);
            sim.update(SUBSTEP_SECONDS);
            rotorPosition += sim.getAngularVelocityRPM() * GEAR_REDUCTION / 60.0 * SUBSTEP_SECONDS;
            voltsSum += appliedVolts;
        }
        inputs.connected = true;
        inputs.velocityRpm = sim.getAngularVelocityRPM() * GEAR_REDUCTION;
        inputs.rotorPositionRotations = rotorPosition;
        inputs.positionValid = true;
        // Média do ciclo: é a tensão que o modelo em paralelo recebe.
        inputs.appliedVolts = voltsSum / steps;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
        inputs.referenceRpm = profileRpm;
        inputs.configurationApplied = true;
        inputs.controlRequestOK = true;
        inputs.configurationStatus = "SIM";
        inputs.controlStatus = openLoop ? "SAIDA_DIRETA" : active ? "CONTROLE_RPM" : "NEUTRO";
    }

    // #endregion

    // #region 🎯 CONTROLE - EMULAÇÃO DO VELOCITYVOLTAGE (PID DO TALONFX)

    /** O mesmo cálculo do Slot0 do TalonFX, em rot/s. */
    private double controlVolts(double rpm) {
        double targetRps = profileRpm / 60.0;
        double volts = SIM_KS * Math.signum(targetRps) + KV * targetRps + KP * (targetRps - rpm / 60.0);
        return MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    // #endregion

    // #region 🛑⚡ SEGURANÇA - LIMITE DE CORRENTE DO ESTATOR

    /**
     * Limite de corrente do estator, como no TalonFX: V = R·I + ω/Kv.
     * Sem isso a partida chegava a 300 A; o Talon real fica em 60 A.
     */
    private double limitStatorCurrent(double volts) {
        double backEmfVolts = sim.getAngularVelocityRadPerSec() * GEAR_REDUCTION / SIM_MOTOR.KvRadPerSecPerVolt;
        double limitVolts = STATOR_CURRENT_LIMIT_AMPS * SIM_MOTOR.rOhms;
        return MathUtil.clamp(volts, backEmfVolts - limitVolts, backEmfVolts + limitVolts);
    }

    // #endregion

    // #region 🧪 MODELO EM PARALELO (frc.robot.util.ParallelModel)

    // A mesma física, recebendo a tensão MEDIDA no motor em vez do controle.
    // 0 V: o Brake curto-circuita o motor e freia, como o TalonFX em neutro.
    // Um passo de 20 ms: com a tensão constante no ciclo, a física linear é
    // exata sem subpassos (e fica leve para rodar no roboRIO).

    @Override
    public void setModelState(double position, double velocityRpm) {
        sim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm / GEAR_REDUCTION));
    }

    @Override
    public void stepModel(double volts) {
        sim.setInputVoltage(MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS));
        sim.update(LOOP_SECONDS);
    }

    @Override
    public double modelPosition() {
        return 0.0;
    }

    @Override
    public double modelVelocity() {
        return sim.getAngularVelocityRPM() * GEAR_REDUCTION;
    }

    @Override
    public double modelCurrentAmps() {
        return Math.abs(sim.getCurrentDrawAmps());
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setVelocityRpm(double rpm) {
        if (!Double.isFinite(rpm)) {
            stop();
            return;
        }
        active = true;
        openLoop = false;
        targetRpm = MathUtil.clamp(rpm, -UNJAM_ROTOR_RPM, MAX_COMMAND_RPM);
    }

    @Override
    public void setPercentOutput(double value) {
        if (!Double.isFinite(value)) {
            stop();
            return;
        }
        openLoop = true;
        active = false;
        percent = MathUtil.clamp(value, 0.0, 1.0);
    }

    @Override
    public void stop() {
        active = false;
        openLoop = false;
        targetRpm = 0.0;
        profileRpm = 0.0;
    }

    // #endregion
}
