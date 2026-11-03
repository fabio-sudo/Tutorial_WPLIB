package frc.robot.commands.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.MAX_EXTENSION_METERS;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.telescopio.Telescopio;

// #endregion

/**
 * Segurar: o telescópio 9460 abre até o fim de curso, pelo perfil de
 * velocidade e aceleração. Soltou: freia pelo perfil e MANTÉM a altura onde
 * conseguiu parar (não volta para trás nem cai pelo peso).
 */
public class TelescopioAbrirCommand extends Command {

    // #region DEPENDÊNCIAS

    private final Telescopio telescopio;

    public TelescopioAbrirCommand(Telescopio telescopio) {
        this.telescopio = telescopio;
        addRequirements(telescopio);
        setName("Telescopio - abrir (segurar)");
    }

    // #endregion

    // #region 🔁 CICLO DO COMANDO

    @Override
    public void initialize() {
        // Cada aperto é uma nova tentativa depois de um travamento.
        telescopio.clearJam();
    }

    @Override
    public void execute() {
        telescopio.setGoalMeters(MAX_EXTENSION_METERS);
    }

    @Override
    public void end(boolean interrupted) {
        telescopio.holdAtStoppingPoint();
    }

    // #endregion
}
