package frc.robot.util;

// #region IMPORTS

import edu.wpi.first.math.system.plant.DCMotor;

// #endregion

/**
 * Braço com gravidade: as MESMAS equações do SingleJointedArmSim da WPILib
 * 2026, mas só com números (double), sem criar objetos a cada passo.
 *
 * Por quê: o modelo em paralelo roda também no robô real, a cada ciclo. O
 * SingleJointedArmSim usa matrizes e um integrador adaptativo e criava
 * ~50 KB de objetos por passo (527 KB por ciclo no modelo do intake). No
 * roboRIO isso enchia a memória e o coletor de lixo pausava o programa
 * inteiro (laço estourado no teste de 29/10/2026).
 *
 * Equações (WPILib): θ' = ω; ω' = A·ω + B·V − 3/2·g·cos(θ)/L, com
 * A = −G²·Kt/(Kv·R·J) e B = G·Kt/(R·J). Ao passar de um limite, para nele
 * com velocidade zero. Integração RK4 de passo fixo. ArmModelTest compara
 * com o SingleJointedArmSim.
 */
public final class ArmModel {

    // #region CONSTANTES DO MODELO

    private static final double GRAVITY = 9.8; // o mesmo valor da WPILib

    private final double velocityGain;
    private final double voltageGain;
    private final double gravityGain;
    private final double gearing;
    private final double kvRadPerSecPerVolt;
    private final double rOhms;
    private final double minAngleRads;
    private final double maxAngleRads;

    // #endregion

    // #region ESTADO

    private double angleRads;
    private double velocityRadPerSec;
    private double inputVolts;

    // #endregion

    // #region CONSTRUÇÃO

    public ArmModel(DCMotor gearbox, double gearing, double moiKgM2, double armLengthMeters,
                    double minAngleRads, double maxAngleRads) {
        this.velocityGain = -gearing * gearing * gearbox.KtNMPerAmp
            / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moiKgM2);
        this.voltageGain = gearing * gearbox.KtNMPerAmp / (gearbox.rOhms * moiKgM2);
        this.gravityGain = 1.5 * GRAVITY / armLengthMeters;
        this.gearing = gearing;
        this.kvRadPerSecPerVolt = gearbox.KvRadPerSecPerVolt;
        this.rOhms = gearbox.rOhms;
        this.minAngleRads = minAngleRads;
        this.maxAngleRads = maxAngleRads;
    }

    // #endregion

    // #region 🧪 PASSO DA FÍSICA

    public void setState(double angleRads, double velocityRadPerSec) {
        this.angleRads = angleRads;
        this.velocityRadPerSec = velocityRadPerSec;
    }

    /** Tensão no motor; quem chama limita à bateria. */
    public void setInputVoltage(double volts) {
        inputVolts = volts;
    }

    public void update(double dtSeconds) {
        double h = dtSeconds;
        double k1a = velocityRadPerSec;
        double k1v = acceleration(angleRads, velocityRadPerSec);
        double k2a = velocityRadPerSec + h / 2 * k1v;
        double k2v = acceleration(angleRads + h / 2 * k1a, velocityRadPerSec + h / 2 * k1v);
        double k3a = velocityRadPerSec + h / 2 * k2v;
        double k3v = acceleration(angleRads + h / 2 * k2a, velocityRadPerSec + h / 2 * k2v);
        double k4a = velocityRadPerSec + h * k3v;
        double k4v = acceleration(angleRads + h * k3a, velocityRadPerSec + h * k3v);
        angleRads += h / 6 * (k1a + 2 * k2a + 2 * k3a + k4a);
        velocityRadPerSec += h / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
        // Como na WPILib: a colisão é conferida depois do passo.
        if (angleRads <= minAngleRads) {
            angleRads = minAngleRads;
            velocityRadPerSec = 0.0;
        } else if (angleRads >= maxAngleRads) {
            angleRads = maxAngleRads;
            velocityRadPerSec = 0.0;
        }
    }

    private double acceleration(double angle, double velocity) {
        return velocityGain * velocity + voltageGain * inputVolts - gravityGain * Math.cos(angle);
    }

    // #endregion

    // #region LEITURAS

    public double getAngleRads() {
        return angleRads;
    }

    public double getVelocityRadPerSec() {
        return velocityRadPerSec;
    }

    /** Mesma conta do SingleJointedArmSim: corrente do conjunto de motores, com o sinal da tensão. */
    public double getCurrentDrawAmps() {
        double motorVelocity = velocityRadPerSec * gearing;
        return (-motorVelocity / kvRadPerSecPerVolt / rOhms + inputVolts / rOhms) * Math.signum(inputVolts);
    }

    // #endregion
}
