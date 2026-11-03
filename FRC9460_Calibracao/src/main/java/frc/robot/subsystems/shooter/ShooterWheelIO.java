package frc.robot.subsystems.shooter;

// #region IMPORTS

import org.littletonrobotics.junction.AutoLog;

// #endregion

/**
 * Contrato das rodas controladas por velocidade: top, tower e centrífuga.
 *
 * O mesmo contrato serve para Spark Flex (top e tower), TalonFX (centrífuga),
 * simulação e replay. O subsistema só pede RPM e lê RPM.
 */
public interface ShooterWheelIO {

    // #region ENTRADAS (GRAVADAS NO LOG E REPRODUZIDAS NO REPLAY)

    @AutoLog
    public static class ShooterWheelIOInputs {

        public boolean connected = false;

        // Um valor por motor: o top tem dois (esquerdo e direito).
        // Positivo = sentido de tiro, já com a inversão de cada motor aplicada.
        public double[] velocityRpm = new double[0];
        public double[] appliedVolts = new double[0];
        public double[] currentAmps = new double[0];
    }

    // #endregion

    // #region COMANDOS

    default void updateInputs(ShooterWheelIOInputs inputs) {}

    /** Velocidade em malha fechada, RPM do motor. */
    default void setVelocityRpm(double rpm) {}

    default void stop() {}

    /**
     * Só a simulação usa: uma bola passou e tirou energia da roda.
     * No robô real a própria bola faz isso; o IO real ignora.
     */
    default void simulateBallPassing() {}

    // #endregion
}
