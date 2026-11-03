package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import frc.robot.util.MechanismModel;
import frc.robot.util.ParallelModel;

// #endregion

/**
 * Roda do shooter no computador (top ou tower): física de volante
 * (FlywheelSim) + o controle de velocidade do Spark Flex, com os ganhos da
 * competição: saída = kV × alvo + kP × erro, rampa e Coast ao parar.
 *
 * Serve para ver o tempo de aceleração, se o RPM chega na tolerância e
 * quanto o RPM cai quando uma bola passa. As inércias são estimativas.
 * Também é o modelo em paralelo do top e da tower (sem bolas: a diferença de
 * RPM mostra a energia que cada bola leva).
 */
public class ShooterWheelIOSim implements ShooterWheelIO, MechanismModel {

    // #region 🧪 FÍSICA

    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.001;

    // Coast: sem tensão, a roda perde velocidade só por atrito (estimativa).
    private static final double COAST_TIME_CONSTANT_SECONDS = 1.5;

    private final FlywheelSim sim;
    private final DCMotor motors;
    private final int motorCount;

    // #endregion

    // #region 🎯 CONTROLE - ESTADO DO SPARK FLEX EMULADO

    private boolean active;
    private double targetRpm;
    private double appliedVolts;
    private double modelVolts;

    // #endregion

    // #region CONSTRUÇÃO

    /**
     * @param motorCount motores no mesmo eixo (top = 2). O modelo do motor já
     *                   deve ter sido criado com essa quantidade.
     */
    public ShooterWheelIOSim(int motorCount, DCMotor motors, double moiKgM2) {
        this.sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motors, moiKgM2, WHEEL_SIM_GEARING), motors);
        this.motors = motors;
        this.motorCount = motorCount;
    }

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    @Override
    public void updateInputs(ShooterWheelIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        double voltsSum = 0.0;
        for (int i = 0; i < steps; i++) {
            if (!active) {
                // Coast: motor desligado do circuito, a roda desacelera livre.
                appliedVolts = 0.0;
                coast(SUBSTEP_SECONDS);
                continue;
            }
            // Controle pede, a segurança do Spark limita a corrente.
            appliedVolts = limitCurrent(controlVolts(sim.getAngularVelocityRPM()));
            sim.setInputVoltage(appliedVolts);
            sim.update(SUBSTEP_SECONDS);
            voltsSum += appliedVolts;
        }

        // Os motores do top estão no mesmo eixo: mesma velocidade nos dois.
        // Tensão: média do ciclo, a que o modelo em paralelo recebe.
        inputs.connected = true;
        inputs.velocityRpm = filled(sim.getAngularVelocityRPM());
        inputs.appliedVolts = filled(voltsSum / steps);
        inputs.currentAmps = filled(Math.abs(sim.getCurrentDrawAmps()) / motorCount);
    }

    // #endregion

    // #region 🎯🚀 CONTROLE - EMULAÇÃO DO PID DE VELOCIDADE DO SPARK FLEX

    /**
     * Controle de velocidade do Spark Flex (kV × alvo + kP × erro), com a
     * rampa de 0,25 s a partir da última tensão aplicada.
     */
    private double controlVolts(double rpm) {
        double duty = SPARK_KV_VOLTS_PER_RPM * targetRpm / NOMINAL_VOLTAGE
            + SPARK_KP * (targetRpm - rpm);
        double desired = MathUtil.clamp(duty, -1.0, 1.0) * NOMINAL_VOLTAGE;
        double maxStep = NOMINAL_VOLTAGE / SPARK_CLOSED_LOOP_RAMP_SECONDS * SUBSTEP_SECONDS;
        return appliedVolts + MathUtil.clamp(desired - appliedVolts, -maxStep, maxStep);
    }

    // #endregion

    // #region 🛑⚡ SEGURANÇA - LIMITE DE CORRENTE DO SPARK FLEX

    /**
     * Limite de corrente do Spark Flex (60 A por motor): V = R·I + ω/Kv.
     * Sem isso o top passava de 100 A por motor na partida.
     */
    private double limitCurrent(double volts) {
        double backEmfVolts = sim.getAngularVelocityRadPerSec() * WHEEL_SIM_GEARING / motors.KvRadPerSecPerVolt;
        double limitVolts = SPARK_CURRENT_LIMIT_AMPS * motorCount * motors.rOhms;
        return MathUtil.clamp(volts, backEmfVolts - limitVolts, backEmfVolts + limitVolts);
    }

    // #endregion

    // #region 🧪 FÍSICA AUXILIAR (COAST E LEITURAS)

    /** Motor solto: a roda perde velocidade só por atrito. */
    private void coast(double seconds) {
        sim.setAngularVelocity(sim.getAngularVelocityRadPerSec()
            * Math.exp(-seconds / COAST_TIME_CONSTANT_SECONDS));
    }

    private double[] filled(double value) {
        double[] values = new double[motorCount];
        Arrays.fill(values, value);
        return values;
    }

    // #endregion

    // #region 🧪 MODELO EM PARALELO (frc.robot.util.ParallelModel)

    // A mesma física, recebendo a tensão MEDIDA no Spark Flex (média dos motores).
    // Quase 0 V = saída neutra: Coast, como as rodas reais paradas.
    // Um passo de 20 ms: com a tensão constante no ciclo, a física linear é
    // exata sem subpassos (e fica leve para rodar no roboRIO).

    @Override
    public void setModelState(double position, double velocityRpm) {
        sim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm));
    }

    @Override
    public void stepModel(double volts) {
        modelVolts = Math.abs(volts) < ParallelModel.NEUTRAL_VOLTS
            ? 0.0
            : MathUtil.clamp(volts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
        if (modelVolts == 0.0) {
            coast(LOOP_SECONDS);
            return;
        }
        sim.setInputVoltage(modelVolts);
        sim.update(LOOP_SECONDS);
    }

    @Override
    public double modelPosition() {
        return 0.0;
    }

    @Override
    public double modelVelocity() {
        return sim.getAngularVelocityRPM();
    }

    @Override
    public double modelCurrentAmps() {
        return modelVolts == 0.0 ? 0.0 : Math.abs(sim.getCurrentDrawAmps()) / motorCount;
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
        targetRpm = rpm;
    }

    @Override
    public void stop() {
        active = false;
        targetRpm = 0.0;
    }

    /** A bola leva parte da energia: o RPM cai e o controle recupera. */
    @Override
    public void simulateBallPassing() {
        sim.setAngularVelocity(sim.getAngularVelocityRadPerSec() * (1.0 - SIM_RPM_LOSS_PER_BALL_FRACTION));
    }

    // #endregion
}
