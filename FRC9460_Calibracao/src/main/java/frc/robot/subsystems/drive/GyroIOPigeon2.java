package frc.robot.subsystems.drive;

// #region IMPORTS

import com.ctre.phoenix6.hardware.Pigeon2;

// #endregion


public class GyroIOPigeon2 implements GyroIO {

    // #region HARDWARE

    // Pigeon 2 real do robô
    private final Pigeon2 pigeon;

    // #endregion

    // #region CONSTRUTOR

    public GyroIOPigeon2(
        int pigeonId
    ) {

        pigeon =
            new Pigeon2(
                pigeonId
            );
    }

    // #endregion



    // #region ATUALIZAÇÃO DOS SENSORES

    @Override
    public void updateInputs(
        GyroIOInputs inputs
    ) {


        // ============================================================
        // 1 - LEITURA DO YAW
        // ============================================================

        // Phoenix retorna o Yaw em graus
        double yawDegrees =
            pigeon
                .getYaw()
                .getValueAsDouble();


        // Converte:
        //
        // graus
        // ↓
        // radianos
        inputs.yawPositionRad =
            Math.toRadians(
                yawDegrees
            );



        // ============================================================
        // 2 - VELOCIDADE ANGULAR
        // ============================================================

        // Velocidade angular ao redor do eixo Z
        //
        // Phoenix retorna em graus por segundo
        double yawVelocityDegreesPerSecond =
            pigeon
                .getAngularVelocityZWorld()
                .getValueAsDouble();


        // Converte:
        //
        // graus/s
        // ↓
        // rad/s
        inputs.yawVelocityRadPerSec =
            Math.toRadians(
                yawVelocityDegreesPerSecond
            );



        // ============================================================
        // 3 - CONEXÃO
        // ============================================================

        // Nesta primeira versão consideramos
        // conectado enquanto o objeto está ativo.
        inputs.connected =
            true;
    }

    // #endregion
}