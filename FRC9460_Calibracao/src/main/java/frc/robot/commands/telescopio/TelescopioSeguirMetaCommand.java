package frc.robot.commands.telescopio;

// #region IMPORTS

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.telescopio.Telescopio;

// #endregion

/**
 * Segue uma meta que pode mudar a cada ciclo (ex.: o número digitado no
 * painel do AdvantageScope). Ao terminar, corta a tensão.
 */
public class TelescopioSeguirMetaCommand extends Command {

    // #region DEPENDÊNCIAS

    private final Telescopio telescopio;
    private final DoubleSupplier goalMeters;

    public TelescopioSeguirMetaCommand(Telescopio telescopio, DoubleSupplier goalMeters) {
        this.telescopio = telescopio;
        this.goalMeters = goalMeters;
        addRequirements(telescopio);
        setName("Telescopio - acompanhar meta");
    }

    // #endregion

    // #region 🔁 CICLO DO COMANDO

    @Override
    public void initialize() {
        telescopio.clearJam();
    }

    @Override
    public void execute() {
        telescopio.setGoalMeters(goalMeters.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        telescopio.stop();
    }

    // #endregion
}
