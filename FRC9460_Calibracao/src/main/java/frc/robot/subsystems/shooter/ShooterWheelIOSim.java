package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// #endregion

/**
 * Roda no computador: física de volante (FlywheelSim) + emulação do
 * controlador de velocidade de cada marca, com os ganhos da competição.
 *
 * Serve para ver o tempo de aceleração e se o RPM chega na tolerância.
 * As inércias são estimativas; o tempo real depende do mecanismo físico.
 */
public class ShooterWheelIOSim implements ShooterWheelIO {

    // #region TIPO DE CONTROLADOR

    /** Cada controlador calcula a tensão de um jeito. */
    public enum Controller {
        // Spark Flex: saída = kV × alvo + kP × erro, com rampa e Coast ao parar.
        SPARK_FLEX,
        // TalonFX: tensão = kS + kV × alvo + kP × erro (rot/s), Brake ao parar.
        TALON_FX
    }

    // #endregion

    // #region FÍSICA

    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.001;
    private static final double BATTERY_VOLTS = 12.0;

    // Coast: sem tensão, a roda perde velocidade só por atrito (estimativa).
    private static final double COAST_TIME_CONSTANT_SECONDS = 1.5;

    private final FlywheelSim sim;
    private final Controller controller;
    private final int motorCount;

    // #endregion

    // #region ESTADO DO CONTROLE

    private boolean active;
    private double targetRpm;
    private double appliedVolts;

    // #endregion

    // #region CONSTRUÇÃO

    /**
     * @param motorCount motores no mesmo eixo (top = 2). O modelo do motor já
     *                   deve ter sido criado com essa quantidade.
     */
    public ShooterWheelIOSim(int motorCount, DCMotor motors, double moiKgM2, Controller controller) {
        this.sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motors, moiKgM2, WHEEL_SIM_GEARING), motors);
        this.controller = controller;
        this.motorCount = motorCount;
    }

    // #endregion

    // #region CICLO DA SIMULAÇÃO

    @Override
    public void updateInputs(ShooterWheelIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        for (int i = 0; i < steps; i++) {
            if (!active && controller == Controller.SPARK_FLEX) {
                // Coast: motor desligado do circuito, a roda desacelera livre.
                appliedVolts = 0.0;
                double decay = Math.exp(-SUBSTEP_SECONDS / COAST_TIME_CONSTANT_SECONDS);
                sim.setAngularVelocity(sim.getAngularVelocityRadPerSec() * decay);
                continue;
            }
            appliedVolts = controlVolts(sim.getAngularVelocityRPM());
            sim.setInputVoltage(appliedVolts);
            sim.update(SUBSTEP_SECONDS);
        }

        // Os motores do top estão no mesmo eixo: mesma velocidade nos dois.
        inputs.connected = true;
        inputs.velocityRpm = filled(sim.getAngularVelocityRPM());
        inputs.appliedVolts = filled(appliedVolts);
        inputs.currentAmps = filled(Math.abs(sim.getCurrentDrawAmps()) / motorCount);
    }

    private double controlVolts(double rpm) {
        if (!active) {
            return 0.0; // TalonFX em Brake: 0 V freia pela força contraeletromotriz.
        }
        double volts;
        if (controller == Controller.SPARK_FLEX) {
            double duty = SPARK_KV_VOLTS_PER_RPM * targetRpm / NOMINAL_VOLTAGE
                + SPARK_KP * (targetRpm - rpm);
            double desired = MathUtil.clamp(duty, -1.0, 1.0) * NOMINAL_VOLTAGE;
            // Rampa: de 0 a 12 V leva no mínimo SPARK_CLOSED_LOOP_RAMP_SECONDS.
            double maxStep = NOMINAL_VOLTAGE / SPARK_CLOSED_LOOP_RAMP_SECONDS * SUBSTEP_SECONDS;
            volts = appliedVolts + MathUtil.clamp(desired - appliedVolts, -maxStep, maxStep);
        } else {
            double targetRps = targetRpm / 60.0;
            volts = CENTRIFUGE_KS * Math.signum(targetRps)
                + CENTRIFUGE_KV * targetRps
                + CENTRIFUGE_KP * (targetRps - rpm / 60.0);
        }
        return MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    private double[] filled(double value) {
        double[] values = new double[motorCount];
        Arrays.fill(values, value);
        return values;
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

    // #endregion
}
