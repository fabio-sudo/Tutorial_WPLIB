package frc.robot.subsystems.centrifuga;

// #region IMPORTS

import org.littletonrobotics.junction.AutoLog;

// #endregion

/** Contrato do motor da centrífuga: real (TalonFX), simulação ou replay. */
public interface CentrifugaIO {

    // #region ENTRADAS (GRAVADAS NO LOG E REPRODUZIDAS NO REPLAY)

    @AutoLog
    public static class CentrifugaIOInputs {
        public boolean connected = false;
        public double velocityRpm = 0.0;
        // Rotor: feedback permanece 1:1 para preservar os ganhos e a curva de RPM.
        public double rotorPositionRotations = 0.0;
        public boolean positionValid = false;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double referenceRpm = 0.0;
        public boolean configurationApplied = false;
        public boolean controlRequestOK = true;
        public String configurationStatus = "NAO_CONFIGURADO";
        public String controlStatus = "PARADO";
    }

    // #endregion

    // #region COMANDOS

    default void updateInputs(CentrifugaIOInputs inputs) {}

    default void setVelocityRpm(double rpm) {}

    // Teste sem trava: saída direta (0 a 1), sem conferir sensores. Só para a frente.
    default void setPercentOutput(double percent) {}

    // Também é um pedido ao motor: o resultado aparece em controlRequestOK no
    // próximo updateInputs. É assim que a centrífuga sai de uma falha de CAN.
    default void stop() {}

    // #endregion
}
