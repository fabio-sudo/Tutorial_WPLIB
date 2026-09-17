package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    // ============================================================
    // DADOS DE ENTRADA DO GYRO
    // ============================================================

    @AutoLog
    public static class GyroIOInputs {

        // Indica se o gyro está conectado
        public boolean connected = false;

        // Ângulo atual do robô
        public double yawPositionRad = 0.0;

        // Velocidade de rotação
        public double yawVelocityRadPerSec = 0.0;
    }


    // ============================================================
    // ATUALIZAÇÃO DOS SENSORES
    // ============================================================

    default void updateInputs(GyroIOInputs inputs) {}


    // ============================================================
    // VELOCIDADE DO GYRO NA SIMULAÇÃO
    // ============================================================

    default void setSimYawVelocity(
            double yawVelocityRadPerSec) {}

    
    // ============================================================
    // ZERA O GYRO
    // ============================================================

    default void resetYaw() {}
}