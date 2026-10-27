package frc.robot.subsystems.telescopio_9458;

import org.littletonrobotics.junction.AutoLog;

public interface TelescopioIO_9458 {

    // #region DADOS DE ENTRADA

    @AutoLog
    public static class TelescopioIOInputs_9458 {

        // Indica se a comunicação com o motor está funcionando.
        public boolean motorConnected = false;

        // Indica se a extensão tem uma leitura válida
        // e uma referência de zero conhecida.
        public boolean positionValid = false;

        // Deslocamento da ponta (tubo 2") em relação à posição recolhida.
        // Na cascata, cada estágio anda metade deste valor.
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

        // Encoder do rotor sem conversão nem clamp (antes da redução 16:1).
        public double rotorPositionRotations = 0.0;
        public double rotorVelocityRotationsPerSec = 0.0;

        // Estado da configuração, do zero e da comunicação, separados
        // para o diagnóstico (Telescopio_9458.getDiagnostic).
        public boolean configurationApplied = false;
        public boolean referenceEstablished = false;
        public boolean referenceLostOnReset = false;
        public boolean controlRequestOK = true;
        public String configurationStatus = "NAO_APLICADA";
        public String zeroStatus = "NAO_REFERENCIADO";
        public String readStatus = "SEM_LEITURA";
        public String controlStatus = "SEM_COMANDO";

        // Referências e saída calculadas DENTRO do Talon (MotionMagicVoltage).
        public boolean motionMagicAvailable = false;
        public boolean motionMagicActive = false;
        public double closedLoopReferenceMeters = 0;
        public double closedLoopReferenceVelocityMetersPerSec = 0;
        public double closedLoopOutputVolts = 0;
        public double closedLoopFeedforwardVolts = 0;
    }

    // #endregion

    // #region ATUALIZAÇÃO DAS LEITURAS

    // Preenche o objeto inputs com as leituras mais recentes.
    default void updateInputs(TelescopioIOInputs_9458 inputs) {}

    /** Somente com robô desabilitado e mecanismo fisicamente recolhido pelo operador. */
    default boolean confirmRetractedZero() { return false; }

    default boolean isRealHardware() { return false; }

    /** O IO sabe executar a meta com Motion Magic (Talon real ou emulado na simulação). */
    default boolean supportsMotionMagic() { return false; }

    /**
     * Motion Magic até {@code meters} (metros da ponta) com velocidade de cruzeiro
     * {@code cruiseMetersPerSec}; aceleração e jerk vêm de configureMotionProfile.
     * A conversão para voltas do rotor pertence ao IO.
     */
    default void setMotionMagicPosition(double meters, double cruiseMetersPerSec) { stop(); }

    /** Ganhos continuam em metros no painel. Somente com o robô desabilitado. */
    default boolean configureGains(TelescopioGains_9458 gains) { return true; }

    /** Velocidade, aceleração e jerk do Motion Magic. Somente com o robô desabilitado. */
    default boolean configureMotionProfile(TelescopioMotionProfile_9458.Limits limits) { return true; }

    default void close() {}

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
