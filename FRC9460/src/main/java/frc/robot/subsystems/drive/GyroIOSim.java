package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;

public class GyroIOSim implements GyroIO {

    // ============================================================
    // ESTADO DO GYRO SIMULADO
    // ============================================================

    // Ângulo atual do robô
    private double yawPositionRad = 0.0;

    // Velocidade atual de rotação
    private double yawVelocityRadPerSec = 0.0;


    // ============================================================
    // ATUALIZAÇÃO DO GYRO
    // ============================================================

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        // O loop do robô roda aproximadamente a cada 20 ms
        double dt = 0.02;

        // Integra a velocidade angular para descobrir o novo ângulo
        yawPositionRad +=
                yawVelocityRadPerSec * dt;

        // Mantém o ângulo entre -PI e +PI
        yawPositionRad =
                MathUtil.angleModulus(yawPositionRad);


        // ========================================================
        // PREENCHE OS DADOS DO SENSOR
        // ========================================================

        inputs.connected = true;

        inputs.yawPositionRad =
                yawPositionRad;

        inputs.yawVelocityRadPerSec =
                yawVelocityRadPerSec;
    }


    // ============================================================
    // VELOCIDADE DE ROTAÇÃO DA SIMULAÇÃO
    // ============================================================

    @Override
    public void setSimYawVelocity(
            double yawVelocityRadPerSec) {

        this.yawVelocityRadPerSec =
                yawVelocityRadPerSec;
    }

    
    // ============================================================
    // ZERA O GYRO
    // ============================================================
        @Override
        public void resetYaw() {

        yawPositionRad = 0.0;
        yawVelocityRadPerSec = 0.0;
        }


}