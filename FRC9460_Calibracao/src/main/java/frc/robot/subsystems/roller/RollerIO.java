package frc.robot.subsystems.roller;

// #region IMPORTS

import org.littletonrobotics.junction.AutoLog;

// #endregion

/** Contrato do motor do rolete: real (Spark Flex), simulação ou replay. */
public interface RollerIO {

    // #region ENTRADAS (GRAVADAS NO LOG E REPRODUZIDAS NO REPLAY)

    @AutoLog
    public static class RollerIOInputs {
        public boolean connected = false;
        public double velocityRpm = 0.0;
        public double appliedVolts = 0.0;
        // A corrente sobe quando uma bola passa: base da contagem de coletas.
        public double currentAmps = 0.0;
    }

    // #endregion

    // #region COMANDOS

    default void updateInputs(RollerIOInputs inputs) {}

    /** Saída direta, -1 a 1. Positivo puxa a bola para dentro. */
    default void setPercentOutput(double percent) {}

    default void stop() {}

    /** Só a simulação usa: uma bola está passando pelo rolo. */
    default void simulateBallIntake() {}

    // #endregion
}
