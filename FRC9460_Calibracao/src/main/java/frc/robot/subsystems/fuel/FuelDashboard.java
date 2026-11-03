package frc.robot.subsystems.fuel;

// #region IMPORTS

import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// #endregion

/**
 * Botões no AdvantageScope (/Tuning/Fuel/AnotarTiroReal) para a equipe
 * anotar ONDE A BOLA CHEGOU no robô real, onde nenhum sensor vê a bola.
 *
 * Cada clique fica gravado no log junto do número do último disparo detectado:
 * depois, no AdvantageScope, dá para comparar tiro a tiro o planejado
 * (Fuel/Tiro/Planejado) com o que aconteceu (Fuel/Real).
 */
public class FuelDashboard {

    // #region 🎮 BOTÕES

    private final LoggedNetworkBoolean hit = button("Acertou");
    private final LoggedNetworkBoolean shortMiss = button("ErrouCurto");
    private final LoggedNetworkBoolean longMiss = button("ErrouLongo");
    private final LoggedNetworkBoolean sideMiss = button("ErrouDoLado");

    private static LoggedNetworkBoolean button(String name) {
        LoggedNetworkBoolean entry = new LoggedNetworkBoolean("/Tuning/Fuel/AnotarTiroReal/" + name, false);
        entry.set(false);
        return entry;
    }

    // #endregion

    // #region CONTAGEM ANOTADA

    private final IntSupplier lastDetectedShot;
    private int hits;
    private int misses;
    private int annotations;
    // O TIPO do erro mostra o viés: mais LONGO que CURTO = força demais; só
    // DO LADO = mira (heading, offset do shooter). Misturado = dispersão.
    private int shortMisses;
    private int longMisses;
    private int sideMisses;

    /** @param lastDetectedShot número do último disparo contado pelo FuelTracker */
    public FuelDashboard(IntSupplier lastDetectedShot) {
        this.lastDetectedShot = lastDetectedShot;
        log("NENHUMA");
        bind(hit, "ACERTOU");
        bind(shortMiss, "ERROU_CURTO");
        bind(longMiss, "ERROU_LONGO");
        bind(sideMiss, "ERROU_DO_LADO");
    }

    // Funciona também desabilitado: dá para anotar depois de cada tiro.
    private void bind(LoggedNetworkBoolean entry, String result) {
        new Trigger(entry::get).onTrue(Commands.runOnce(() -> {
            annotations++;
            switch (result) {
                case "ACERTOU" -> hits++;
                case "ERROU_CURTO" -> { misses++; shortMisses++; }
                case "ERROU_LONGO" -> { misses++; longMisses++; }
                default -> { misses++; sideMisses++; }
            }
            log(result);
            entry.set(false);
        }).ignoringDisable(true).withName("Fuel - anotar " + result));
    }

    private void log(String result) {
        Logger.recordOutput("Fuel/Real/UltimaAnotacao", result);
        Logger.recordOutput("Fuel/Real/TiroAnotado", lastDetectedShot.getAsInt());
        Logger.recordOutput("Fuel/Real/Acertos", hits);
        Logger.recordOutput("Fuel/Real/Erros", misses);
        Logger.recordOutput("Fuel/Real/Anotacoes", annotations);
        Logger.recordOutput("Fuel/Real/ErrouCurto", shortMisses);
        Logger.recordOutput("Fuel/Real/ErrouLongo", longMisses);
        Logger.recordOutput("Fuel/Real/ErrouDoLado", sideMisses);
        // Sem anotação ainda = NaN (o gráfico mostra falha, não 0% de acerto).
        Logger.recordOutput("Fuel/Real/TaxaAcerto", annotations == 0 ? Double.NaN : hits / (double) annotations);
    }

    // #endregion
}
