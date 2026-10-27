package frc.robot.subsystems.shooter;

// #region IMPORTS

import org.littletonrobotics.junction.AutoLog;

// #endregion

/**
 * Contrato do motor que gira o shooter.
 *
 * Três implementações, escolhidas em Shooter.createPivotIO():
 * - ShooterPivotIOTalonFX: robô real;
 * - ShooterPivotIOSim: física no computador;
 * - IO vazio: replay, as entradas vêm do arquivo de log.
 *
 * Unidades: voltas do MOTOR (antes da redução). A conversão para graus do
 * shooter fica no ShotCalculator, igual para os três casos.
 */
public interface ShooterPivotIO {

    // #region ENTRADAS (GRAVADAS NO LOG E REPRODUZIDAS NO REPLAY)

    @AutoLog
    public static class ShooterPivotIOInputs {

        // Comunicação com o motor.
        public boolean connected = false;

        // O zero manual ainda vale. Fica false se o motor reiniciar (queda de
        // bateria): o encoder volta a 0 onde o shooter estiver, e o centro some.
        public boolean zeroValid = false;

        // Posição e velocidade do encoder do motor. 0 = onde o shooter estava
        // quando o robô ligou (ou quando a equipe zerou de novo).
        public double motorPositionRotations = 0.0;
        public double motorVelocityRotationsPerSec = 0.0;

        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;

        // Referência do perfil Motion Magic calculada pelo controlador.
        public double profileReferenceRotations = 0.0;

        // O controlador está bloqueando o movimento no fim do curso.
        public boolean forwardLimitActive = false;
        public boolean reverseLimitActive = false;
    }

    // #endregion

    // #region COMANDOS

    default void updateInputs(ShooterPivotIOInputs inputs) {}

    /** Motion Magic até a posição, em voltas do motor (já limitada a ±6). */
    default void setTargetRotations(double motorRotations) {}

    /** Saída direta, -1 a 1. Usada só no movimento manual de teste. */
    default void setPercentOutput(double percent) {}

    /** Desliga o motor (modo Brake segura pela resistência do motor). */
    default void stop() {}

    /**
     * Define a posição atual como zero. Só faz sentido com o shooter
     * CENTRALIZADO à mão e o robô desabilitado: é o mesmo que ligar o robô.
     */
    default void zeroHere() {}

    // #endregion
}
