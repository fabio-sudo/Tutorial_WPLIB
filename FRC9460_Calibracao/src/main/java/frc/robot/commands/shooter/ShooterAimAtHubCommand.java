package frc.robot.commands.shooter;

// #region IMPORTS

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.Shooter;

// #endregion

/**
 * O pivô segue o hub da aliança o tempo todo; as rodas ficam paradas.
 *
 * Usado de dois jeitos:
 * - comando padrão do shooter, ligado e desligado pelo B do piloto
 *   (enabled = a chave "mira automática");
 * - segurando o LB do operador (enabled sempre verdadeiro).
 *
 * Sem pose confiável (a Limelight ainda não corrigiu), ou com a chave
 * desligada, o pivô fica parado.
 */
public class ShooterAimAtHubCommand extends Command {

    // #region DEPENDÊNCIAS

    private final Shooter shooter;
    private final BooleanSupplier enabled;

    public ShooterAimAtHubCommand(Shooter shooter, BooleanSupplier enabled) {
        this.shooter = shooter;
        this.enabled = enabled;
        addRequirements(shooter);
        setName("Shooter - mirar no hub");
    }

    /** Sempre ligado (enquanto o botão estiver apertado). */
    public ShooterAimAtHubCommand(Shooter shooter) {
        this(shooter, () -> true);
    }

    // #endregion

    // #region 🔁 CICLO DO COMANDO

    @Override
    public void execute() {
        boolean on = enabled.getAsBoolean();
        Logger.recordOutput("Shooter/Modo/MiraAutomaticaLigada", on);
        if (on && shooter.isPoseTrusted()) {
            shooter.aimAt(shooter.getHubTarget());
        } else if (shooter.getPivotMode() != Shooter.PivotMode.PARADO) {
            shooter.stopPivot();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopPivot();
    }

    // #endregion
}
