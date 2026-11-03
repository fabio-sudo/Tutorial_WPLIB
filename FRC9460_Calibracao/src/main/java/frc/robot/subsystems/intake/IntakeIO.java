package frc.robot.subsystems.intake;

// #region IMPORTS

import org.littletonrobotics.junction.AutoLog;

// #endregion

/**
 * Contrato dos motores do articulador: real (2 TalonFX), simulação ou replay.
 */
public interface IntakeIO {

    // #region ENTRADAS (GRAVADAS NO LOG E REPRODUZIDAS NO REPLAY)

    @AutoLog
    public static class IntakeIOInputs {
        public boolean connected = false;

        // false se o motor reiniciar: o encoder volta a 0 onde estiver.
        public boolean zeroValid = false;

        // Como o zero veio ao iniciar o código: ZERAR_AQUI (Talon recém-ligado),
        // MANTER_ZERO (só o código reiniciou) ou POSICAO_DESCONHECIDA.
        public String startupZero = "SIMULACAO";

        // O seguidor (direito) reiniciou e voltou a seguir o líder.
        public int followerResets = 0;

        // 📊 O seguidor responde no CAN. Sem ele, o líder empurra o eixo sozinho.
        public boolean followerConnected = false;

        // Voltas do rotor do líder: 0 = fechado; abertura em IntakeConstants.
        // No Tuner X o fechado aparece como 100 voltas (frc.robot.util.TalonZeroMark).
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        // O Talon está bloqueando o movimento no fim do curso.
        public boolean closeLimitActive = false;
        public boolean openLimitActive = false;
    }

    // #endregion

    // #region COMANDOS

    default void updateInputs(IntakeIOInputs inputs) {
    }

    /** Saída direta, -1 a 1. Negativo abre, positivo fecha. */
    default void setPercentOutput(double percent) {
    }

    /** Trava na posição (voltas do motor) com o PID de segurar. */
    default void holdPosition(double rotations) {
    }

    default void stop() {
    }

    /** Robô desabilitado e intake FECHADO à mão: a posição atual vira 0. */
    default void zeroClosed() {
    }

    // #endregion
}
