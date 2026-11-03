package frc.robot.commands.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.MIN_EXTENSION_METERS;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.telescopio.Telescopio;

// #endregion

/**
 * Segurar: o telescópio 9460 recolhe até o batente, pelo perfil.
 * Soltou: freia pelo perfil e mantém a altura onde parou.
 */
public class TelescopioFecharCommand extends Command {

    // #region DEPENDÊNCIAS

    private final Telescopio telescopio;

    public TelescopioFecharCommand(Telescopio telescopio) {
        this.telescopio = telescopio;
        addRequirements(telescopio);
        setName("Telescopio - fechar (segurar)");
    }

    // #endregion

    // #region 🔁 CICLO DO COMANDO

    @Override
    public void initialize() {
        telescopio.clearJam();
    }

    @Override
    public void execute() {
        telescopio.setGoalMeters(MIN_EXTENSION_METERS);
    }

    @Override
    public void end(boolean interrupted) {
        telescopio.holdAtStoppingPoint();
    }

    // #endregion
}
