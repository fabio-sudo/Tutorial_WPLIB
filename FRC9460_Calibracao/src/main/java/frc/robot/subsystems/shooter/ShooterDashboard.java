package frc.robot.subsystems.shooter;

// #region IMPORTS

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// #endregion

/**
 * Botões no AdvantageScope (/Tuning/Shooter), para testar na simulação sem
 * um segundo controle. Funcionam também no robô real.
 *
 * São entradas do AdvantageKit: um clique fica gravado no log e o replay
 * repete o mesmo clique no mesmo instante.
 */
public class ShooterDashboard {

    // #region ENTRADAS DO PAINEL

    private final LoggedNetworkBoolean aim =
        new LoggedNetworkBoolean("/Tuning/Shooter/MirarNoHub", false);
    private final LoggedNetworkBoolean shoot =
        new LoggedNetworkBoolean("/Tuning/Shooter/MirarEAtirar", false);
    private final LoggedNetworkBoolean zero =
        new LoggedNetworkBoolean("/Tuning/Shooter/ZerarPivoNoCentro", false);

    // #endregion

    // #region LIGAÇÃO COM OS COMANDOS

    /**
     * Os comandos vêm prontos do RobotContainer (ficam em frc.robot.commands):
     * shootCommand = mirar e atirar; aimCommand = só mirar.
     */
    public ShooterDashboard(Shooter shooter, Supplier<Command> shootCommand, Supplier<Command> aimCommand) {
        // Um pedido antigo do painel não deve agir na partida.
        aim.set(false);
        shoot.set(false);
        zero.set(false);

        // Ligado + teleoperado habilitado: o comando roda enquanto estiver true.
        new Trigger(() -> shoot.get())
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(shootCommand.get());
        new Trigger(() -> aim.get() && !shoot.get())
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(aimCommand.get());

        // Zero: o Shooter recusa se o robô estiver habilitado. O pedido é
        // consumido na hora para não ser aplicado depois, ao desabilitar.
        new Trigger(() -> zero.get()).onTrue(Commands.runOnce(() -> {
            shooter.zeroPivotHere();
            zero.set(false);
        }).ignoringDisable(true).withName("Shooter - zerar pelo painel"));
    }

    // #endregion
}
