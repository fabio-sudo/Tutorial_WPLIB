package frc.robot.subsystems.drive;

// #region IMPORTS

import com.ctre.phoenix6.hardware.Pigeon2;

// NEW - Configuração de ajuste fino do giroscópio
import com.ctre.phoenix6.configs.GyroTrimConfigs;

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


        // NEW ============================================================
        // NEW - CALIBRAÇÃO DO GIROSCÓPIO NO EIXO Z
        // NEW ============================================================
        //
        // Resultado dos testes físicos de 10 voltas:
        //
        // Esperado:
        // 3600 graus
        //
        // Média medida:
        // aproximadamente 3542 graus
        //
        // Correção calculada:
        // aproximadamente 5.78 graus por volta
        //
        // O eixo Z corresponde ao Yaw do robô.
        //
        // IMPORTANTE:
        // Este é nosso PRIMEIRO valor de calibração.
        // Depois vamos repetir o teste das 10 voltas
        // para validar se a correção ficou correta.
        //
        // ================================================================

        GyroTrimConfigs gyroTrimConfigs =
            new GyroTrimConfigs()

                .withGyroScalarZ(
                    -6.24
                );


        // NEW ============================================================
        // NEW - APLICA A CALIBRAÇÃO NO PIGEON 2
        // NEW ============================================================

        pigeon
            .getConfigurator()
            .apply(
                gyroTrimConfigs
            );

        // NEW ============================================================
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