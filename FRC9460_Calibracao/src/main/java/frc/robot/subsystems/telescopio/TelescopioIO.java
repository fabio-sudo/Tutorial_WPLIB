package frc.robot.subsystems.telescopio;

import org.littletonrobotics.junction.AutoLog;

public interface TelescopioIO {

    // #region DADOS DE ENTRADA

    @AutoLog
    public static class TelescopioIOInputs {

        // Indica se a comunicação com o motor está funcionando.
        public boolean motorConnected = false;

        // Indica se a extensão tem uma leitura válida
        // e uma referência de zero conhecida.
        public boolean positionValid = false;

        // Deslocamento da ponta em relação à posição recolhida.
        // Não inclui a altura do telescópio fechado.
        public double extensionMeters = 0.0;

        // Velocidade de extensão da ponta.
        // Positiva: estendendo. Negativa: recolhendo.
        public double velocityMetersPerSec = 0.0;

        // Tensão efetivamente aplicada ao motor.
        public double appliedVolts = 0.0;

        // Corrente nos enrolamentos do motor.
        // No TalonFX, corresponde à corrente de estator.
        public double motorCurrentAmps = 0.0;

        // O limite de corrente do estator está reduzindo a tensão.
        // No TalonFX, corresponde a Fault_StatorCurrLimit.
        public boolean statorCurrentLimitActive = false;
    }

    // #endregion

    // #region ATUALIZAÇÃO DAS LEITURAS

    // Preenche o objeto inputs com as leituras mais recentes.
    default void updateInputs(TelescopioIOInputs inputs) {}

    // #endregion

    // #region COMANDOS DO MOTOR

    // Solicita uma tensão ao motor.
    // Convenção: positivo para estender; negativo para recolher.
    default void setVoltage(double volts) {}

    // Solicita tensão zero.
    // Isso não garante manter a posição contra a gravidade.
    // O controle de posição e a compensação de gravidade ficam no subsistema.
    default void stop() {
        setVoltage(0.0);
    }

    // #endregion
}
