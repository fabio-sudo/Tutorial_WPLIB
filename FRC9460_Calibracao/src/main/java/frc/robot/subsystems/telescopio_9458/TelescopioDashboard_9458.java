package frc.robot.subsystems.telescopio_9458;

// #region IMPORTS

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// #endregion

/** Controles editáveis no AdvantageScope; todas as entradas participam do replay. */
public class TelescopioDashboard_9458 {

    // #region ENTRADAS DO PAINEL (TUNING)

    private final LoggedNetworkBoolean active =
        new LoggedNetworkBoolean("/Tuning/Telescopio9458/ControleAtivo", false);
    private final LoggedNetworkBoolean demo =
        new LoggedNetworkBoolean("/Tuning/Telescopio9458/CicloAutomatico", false);
    private final LoggedNetworkNumber goal = new LoggedNetworkNumber(
        "/Tuning/Telescopio9458/MetaMetros", TelescopioConstants_9458.MAX_OPERATING_EXTENSION_METERS / 2);
    private final LoggedNetworkBoolean confirmZero = new LoggedNetworkBoolean(
        "/Tuning/Telescopio9458/ConfirmarRecolhidoEZerar", false);
    private final LoggedNetworkBoolean applyGains = new LoggedNetworkBoolean(
        "/Tuning/Telescopio9458/PID/AplicarGanhos", false);
    private final LoggedNetworkNumber kP, kI, kD, kS, kG, kV, kA;
    // Perfil do movimento (Motion Magic): m/s, m/s² e m/s³ na ponta.
    private final LoggedNetworkBoolean applyProfile = new LoggedNetworkBoolean(
        "/Tuning/Telescopio9458/Perfil/AplicarPerfil", false);
    private final LoggedNetworkNumber profileVelocity, profileAcceleration, profileJerk;

    // #endregion

    // #region LIGACAO COM OS COMANDOS

    public TelescopioDashboard_9458(Telescopio_9458 telescopio) {
        var initial = telescopio.getGains();
        kP = gain("kP", initial.kP());
        kI = gain("kI", initial.kI());
        kD = gain("kD", initial.kD());
        kS = gain("kS", initial.kS());
        kG = gain("kG", initial.kG());
        kV = gain("kV", initial.kV());
        kA = gain("kA", initial.kA());
        var profile = telescopio.getProfileLimits();
        profileVelocity = profileNumber("VelocidadeMaxima", profile.velocity());
        profileAcceleration = profileNumber("Aceleracao", profile.acceleration());
        profileJerk = profileNumber("Jerk", profile.jerk());
        // Uma solicitação antiga do painel não deve ser reaplicada na partida.
        confirmZero.set(false);
        applyGains.set(false);
        applyProfile.set(false);
        new Trigger(applyProfile::get).onTrue(Commands.runOnce(() -> {
            telescopio.applyProfile(profileVelocity.get(), profileAcceleration.get(), profileJerk.get());
            applyProfile.set(false);
        }).ignoringDisable(true).withName("Telescopio 9458 - aplicar perfil desabilitado"));
        // O método recusa pedidos habilitado. Consumir o pedido imediatamente evita
        // que um clique recusado seja aplicado mais tarde ao desabilitar.
        new Trigger(confirmZero::get).onTrue(Commands.runOnce(() -> {
            telescopio.confirmRetractedZero();
            confirmZero.set(false);
        }).ignoringDisable(true).withName("Telescopio 9458 - confirmar zero recolhido"));
        new Trigger(applyGains::get).onTrue(Commands.runOnce(() -> {
            telescopio.applyGains(kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get());
            applyGains.set(false);
        }).ignoringDisable(true).withName("Telescopio 9458 - aplicar ganhos desabilitado"));
        new Trigger(() -> active.get() && !demo.get())
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(telescopio.followGoalCommand(goal::get));
        new Trigger(() -> active.get() && demo.get())
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(telescopio.demoCommand());
    }

    private static LoggedNetworkNumber gain(String name, double initial) {
        return new LoggedNetworkNumber("/Tuning/Telescopio9458/PID/" + name, initial);
    }

    private static LoggedNetworkNumber profileNumber(String name, double initial) {
        return new LoggedNetworkNumber("/Tuning/Telescopio9458/Perfil/" + name, initial);
    }

    // #endregion
}
