package frc.robot.subsystems.drive;

// #region IMPORTS

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

// NEW - Configuração de ajuste fino do giroscópio
import com.ctre.phoenix6.configs.GyroTrimConfigs;

// #endregion


public class GyroIOPigeon2 implements GyroIO {

    // #region HARDWARE

    // Pigeon 2 real do robô
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yawSignal;
    private final StatusSignal<AngularVelocity> yawVelocitySignal;
    private final boolean configurationOk;
    private static final double MAX_SIGNAL_AGE_SECONDS = 0.10;

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
                    -5.48
                );


        // NEW ============================================================
        // NEW - APLICA A CALIBRAÇÃO NO PIGEON 2
        // NEW ============================================================

        var trimStatus = pigeon
            .getConfigurator()
            .apply(
                gyroTrimConfigs
            );

        yawSignal = pigeon.getYaw();
        yawVelocitySignal = pigeon.getAngularVelocityZWorld();
        var frequencyStatus = BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, yawSignal, yawVelocitySignal);
        configurationOk = trimStatus.isOK() && frequencyStatus.isOK();
        if (!configurationOk) {
            DriverStation.reportWarning(
                    "Pigeon: falha de configuração. Trim=" + trimStatus
                            + ", frequência=" + frequencyStatus,
                    false);
        }

        // NEW ============================================================
    }

    // #endregion



    // #region ATUALIZAÇÃO DOS SENSORES

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(yawSignal, yawVelocitySignal);
        inputs.yawPositionRad = Math.toRadians(yawSignal.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Math.toRadians(yawVelocitySignal.getValueAsDouble());
        inputs.yawSignalAgeSeconds = yawSignal.getTimestamp().getLatency();
        inputs.yawVelocitySignalAgeSeconds = yawVelocitySignal.getTimestamp().getLatency();
        inputs.configurationOk = configurationOk;
        inputs.status = status.toString();

        // Um objeto Java existente não comprova que o sensor está respondendo.
        // A idade máxima de 100 ms tolera cinco períodos de publicação a 50 Hz.
        inputs.connected = status.isOK()
                && yawSignal.getTimestamp().isValid()
                && yawVelocitySignal.getTimestamp().isValid()
                && Double.isFinite(inputs.yawSignalAgeSeconds)
                && inputs.yawSignalAgeSeconds >= 0.0
                && inputs.yawSignalAgeSeconds <= MAX_SIGNAL_AGE_SECONDS
                && Double.isFinite(inputs.yawVelocitySignalAgeSeconds)
                && inputs.yawVelocitySignalAgeSeconds >= 0.0
                && inputs.yawVelocitySignalAgeSeconds <= MAX_SIGNAL_AGE_SECONDS;
    }

    // #endregion
}
