package frc.robot.subsystems.telescopio;

// #region IMPORTS

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.telescopio.TelescopioCicloCommand;
import frc.robot.commands.telescopio.TelescopioSeguirMetaCommand;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// #endregion

/** Controles editáveis no AdvantageScope; todas as entradas participam do replay. */
public class TelescopioDashboard {

    // #region ENTRADAS DO PAINEL (TUNING)

    private final LoggedNetworkBoolean active =
        new LoggedNetworkBoolean("/Tuning/Telescopio/ControleAtivo", false);
    private final LoggedNetworkBoolean demo =
        new LoggedNetworkBoolean("/Tuning/Telescopio/CicloAutomatico", false);
    private final LoggedNetworkNumber goal = new LoggedNetworkNumber(
        "/Tuning/Telescopio/MetaMetros", TelescopioConstants.MAX_EXTENSION_METERS / 2);

    // #endregion

    // #region LIGACAO COM OS COMANDOS

    public TelescopioDashboard(Telescopio telescopio) {
        new Trigger(() -> active.get() && !demo.get())
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(new TelescopioSeguirMetaCommand(telescopio, goal::get));
        new Trigger(() -> active.get() && demo.get())
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(new TelescopioCicloCommand(telescopio));
    }

    // #endregion
}
