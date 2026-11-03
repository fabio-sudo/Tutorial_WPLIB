package frc.robot.util;

import java.util.Locale;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import frc.robot.LoggingConstants;

/** Publica o estado da gravação sem consultar o disco no ciclo de controle. */
public final class RobotHealth extends SubsystemBase {
    private final Alert logMissing = new Alert(
        "Log físico não está gravando. Confira Diagnostico/Log e o pendrive.", AlertType.kError);
    private final Alert logInternal = new Alert(
        "Log no disco interno (máximo 200 MiB por arquivo). Copie os logs e use pendrive.", AlertType.kWarning);
    private final Alert queueFault = new Alert(
        "Fila do AdvantageKit cheia: dados podem estar sendo perdidos.", AlertType.kError);
    private boolean logFailureReported;

    /** Aviso da Driver Station: o que aconteceu e o que fazer para ter replay. */
    static String logFailureMessage(RobotLogWriter.Status status) {
        return String.format(Locale.ROOT, "[FRC9460] Log NÃO está gravando: %s (%.0f MiB livres). "
            + "Esta sessão ficará sem replay: coloque um pendrive ou baixe e apague os logs do roboRIO.",
            status.state(), status.freeMiB());
    }

    @Override public void periodic() {
        Logger.recordOutput("Diagnostico/Modo", Logger.hasReplaySource() ? "REPLAY" : RobotBase.isReal() ? "REAL" : "SIM");
        Logger.recordOutput("Diagnostico/PeriodoAlvoMs", 20.0);
        boolean physical = RobotBase.isReal() && !Logger.hasReplaySource();
        var s = RobotLogWriter.status();
        logMissing.set(physical && !s.writing());
        logInternal.set(physical && s.state().equals("GRAVANDO_INTERNO"));
        queueFault.set(Logger.getReceiverQueueFault());
        // O alerta só aparece no Elastic, e 5 sessões passaram sem log sem ninguém ver.
        if (physical && s.failed() && !logFailureReported) {
            logFailureReported = true;
            DriverStation.reportError(logFailureMessage(s), false);
        }
        Logger.recordOutput("Diagnostico/Log/Estado", physical ? s.state() : "SIM_OU_REPLAY");
        Logger.recordOutput("Diagnostico/Log/Arquivo", physical ? s.path() : LoggingConstants.SIM_LOG_DIRECTORY + " (ou replay)");
        Logger.recordOutput("Diagnostico/Log/GravandoFisico", physical && s.writing());
        Logger.recordOutput("Diagnostico/Log/LivreMiB", physical ? s.freeMiB() : -1.0);
        Logger.recordOutput("Diagnostico/Log/FilaComFalha", Logger.getReceiverQueueFault());
    }
}
