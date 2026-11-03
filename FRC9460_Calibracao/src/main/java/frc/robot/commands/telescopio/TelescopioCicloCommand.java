package frc.robot.commands.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.telescopio.Telescopio;

// #endregion

/**
 * Ciclo automático de teste: metade -> aberto -> recolhido, parando 1 s em
 * cada posição, e repete até ser cancelado. Cancelou: corta a tensão.
 *
 * Feito como uma pequena máquina de estados para ficar fácil de ler: a
 * etapa atual diz a meta; ao chegar, conta o tempo de pausa e passa para a
 * próxima.
 */
public class TelescopioCicloCommand extends Command {

    // #region 🔁 ETAPAS DO CICLO

    private static final double[] GOALS_METERS = {
        MAX_EXTENSION_METERS / 2.0, MAX_EXTENSION_METERS, MIN_EXTENSION_METERS
    };

    private final Telescopio telescopio;
    private final DoubleSupplier clock;
    private int step;
    private double arrivedSeconds = Double.NaN;

    public TelescopioCicloCommand(Telescopio telescopio) {
        this.telescopio = telescopio;
        this.clock = Timer::getTimestamp;
        addRequirements(telescopio);
        setName("Telescopio - ciclo automatico");
    }

    // #endregion

    // #region 🔁 CICLO DO COMANDO

    @Override
    public void initialize() {
        step = 0;
        arrivedSeconds = Double.NaN;
        telescopio.clearJam();
    }

    @Override
    public void execute() {
        telescopio.setGoalMeters(GOALS_METERS[step]);
        if (!telescopio.atGoal()) {
            return;
        }
        double now = clock.getAsDouble();
        if (Double.isNaN(arrivedSeconds)) {
            arrivedSeconds = now;
        }
        // Chegou e esperou a pausa: próxima etapa (volta ao início depois da última).
        if (now - arrivedSeconds >= DEMO_DWELL_SECONDS) {
            step = (step + 1) % GOALS_METERS.length;
            arrivedSeconds = Double.NaN;
            telescopio.clearJam();
        }
    }

    @Override
    public void end(boolean interrupted) {
        telescopio.stop();
    }

    // #endregion
}
