package frc.robot.util;

// #region IMPORTS

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

// #endregion

/**
 * Diagnóstico de um mecanismo, no padrão do telescópio 9458:
 * - Execucao: REPLAY, ROBO_REAL_<hardware> ou SIMULACAO_FISICA;
 * - MotorRespondendo e um alerta de erro quando o motor some do CAN
 *   (aparece no Driver Station, no widget Alerts do Elastic e no log);
 * - Resultado: o motivo principal, em uma palavra, para o pit. Quando ele
 *   deixa de ser OK, a mensagem também vai para a Driver Station (funciona
 *   mesmo sem AdvantageScope conectado).
 *
 * Nomes no log: prefixo/Diagnostico/Execucao, MotorRespondendo, Resultado.
 * O comando atual (prefixo/Comando) fica na telemetria de cada subsistema.
 */
public final class MechanismDiagnostics {

    // #region 📊 RESULTADOS POSSÍVEIS

    public static final String OK = "OK";
    public static final String MOTOR_SEM_RESPOSTA = "MOTOR_SEM_RESPOSTA";
    public static final String ZERO_PERDIDO = "ZERO_PERDIDO";
    public static final String REAL_DIFERENTE_DO_MODELO = "REAL_DIFERENTE_DO_MODELO";

    // #endregion

    // #region ESTADO

    private final String prefix;
    private final String displayName;
    private final String execution;
    private final Alert disconnectedAlert;
    private final boolean reportToDriverStation;
    private String lastResult = OK;

    /**
     * @param prefix      nome do mecanismo no log (ex.: "Centrifuga")
     * @param displayName nome para o alerta (ex.: "Centrífuga")
     * @param hardware    motor e ID, ex.: "TALONFX_ID_36"
     */
    public MechanismDiagnostics(String prefix, String displayName, String hardware) {
        this(prefix, displayName, hardware, new Alert(
            displayName + ": motor sem resposta (" + hardware + "). Conferir CAN e alimentação.",
            AlertType.kError), true);
    }

    /** Mecanismo desligado em CanIds: no robô real o motor não existe e só avisa. */
    public static MechanismDiagnostics disabledInCanIds(String prefix, String displayName) {
        // Esperado enquanto o motor não for instalado: alerta, sem mensagem na DS.
        return new MechanismDiagnostics(prefix, displayName, "DESLIGADO_EM_CANIDS", new Alert(
            displayName + ": desligado em frc.robot.CanIds. O motor não é criado.",
            AlertType.kWarning), false);
    }

    private MechanismDiagnostics(String prefix, String displayName, String hardware, Alert alert,
                                 boolean reportToDriverStation) {
        this.prefix = Objects.requireNonNull(prefix);
        this.displayName = Objects.requireNonNull(displayName);
        this.reportToDriverStation = reportToDriverStation;
        this.execution = Logger.hasReplaySource()
            ? "REPLAY"
            : RobotBase.isReal() ? "ROBO_REAL_" + hardware : "SIMULACAO_FISICA";
        this.disconnectedAlert = alert;
    }

    // #endregion

    // #region 📊 ATUALIZAÇÃO (UMA VEZ POR CICLO, NA TELEMETRIA)

    /** Resultado mais grave primeiro: sem motor, sem zero, modelo divergindo. */
    public static String result(boolean connected, boolean zeroValid, boolean modelDiverging) {
        if (!connected) return MOTOR_SEM_RESPOSTA;
        if (!zeroValid) return ZERO_PERDIDO;
        if (modelDiverging) return REAL_DIFERENTE_DO_MODELO;
        return OK;
    }

    /**
     * O que mostrar sobre o zero. Com o zero inválido, o texto nunca diz "zerado":
     * vale para falha ao zerar, motor reiniciado e posição desconhecida ao iniciar.
     * A recusa continua visível. Depende só de entradas gravadas: o replay mostra o mesmo.
     */
    public static String zeroResult(boolean zeroValid, String status) {
        return !zeroValid && status.startsWith("ZERADO") ? ZERO_PERDIDO : status;
    }

    /** Nome do comando que está usando o subsistema ("NENHUM" sem comando). */
    public static String commandName(Subsystem subsystem) {
        Command command = subsystem.getCurrentCommand();
        return command == null ? "NENHUM" : command.getName();
    }

    /**
     * Só na mudança: uma mensagem por problema, não uma por ciclo. Modelo divergindo
     * fica no log e no Elastic: os modelos ainda não foram calibrados no robô real, e
     * os 100 avisos por sessão escondiam no console os que pedem uma ação.
     */
    static boolean shouldReport(String result, String previous) {
        return !result.equals(previous) && !OK.equals(result) && !REAL_DIFERENTE_DO_MODELO.equals(result);
    }

    public void update(boolean connected, String result) {
        disconnectedAlert.set(!connected);
        if (reportToDriverStation && shouldReport(result, lastResult)) {
            DriverStation.reportWarning("[FRC9460] " + displayName + ": " + result, false);
        }
        lastResult = result;
        Logger.recordOutput(prefix + "/Diagnostico/Execucao", execution);
        Logger.recordOutput(prefix + "/Diagnostico/MotorRespondendo", connected);
        Logger.recordOutput(prefix + "/Diagnostico/Resultado", result);
    }

    // #endregion
}
