package frc.robot.commands.telescopio;

// #region IMPORTS

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.telescopio.Telescopio;

// #endregion

/**
 * Vai até uma extensão (metros da ponta) e termina quando chega.
 * Chegou: continua segurando a altura. Interrompido: corta a tensão.
 * Útil no autônomo e em sequências (ex.: abrir até a metade e esperar).
 */
public class TelescopioIrParaCommand extends Command {

    // #region DEPENDÊNCIAS

    private final Telescopio telescopio;
    private final double meters;

    public TelescopioIrParaCommand(Telescopio telescopio, double meters) {
        this.telescopio = telescopio;
        this.meters = meters;
        addRequirements(telescopio);
        setName("Telescopio - ir para " + meters + " m");
    }

    // #endregion

    // #region 🔁 CICLO DO COMANDO

    @Override
    public void initialize() {
        telescopio.clearJam();
    }

    @Override
    public void execute() {
        telescopio.setGoalMeters(meters);
    }

    @Override
    public boolean isFinished() {
        return telescopio.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            telescopio.stop();
        }
    }

    // #endregion
}
