package frc.robot.commands.intake;

// #region IMPORTS

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.roller.Roller;

// #endregion

/**
 * Coletar: abre o intake e puxa com o rolete ao mesmo tempo.
 * Soltou: o rolete para e o intake trava onde estiver (não cai pelo peso).
 *
 * Fica em commands/ porque usa DOIS subsistemas. Os comandos de um
 * subsistema só (abrir, fechar, puxar) ficam no próprio subsistema.
 */
public class ColetarCommand extends Command {

    // #region DEPENDÊNCIAS

    private final Intake intake;
    private final Roller roller;

    public ColetarCommand(Intake intake, Roller roller) {
        this.intake = intake;
        this.roller = roller;
        addRequirements(intake, roller);
        setName("Coletar - abrir e puxar");
    }

    // #endregion

    // #region 🔁 CICLO DO COMANDO

    @Override
    public void execute() {
        intake.openSlowly();
        roller.pull();
    }

    @Override
    public void end(boolean interrupted) {
        roller.stop();
        intake.stop();
    }

    // #endregion
}
