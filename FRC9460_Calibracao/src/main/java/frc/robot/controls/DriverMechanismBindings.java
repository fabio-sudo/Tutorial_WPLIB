package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Botões do piloto. Cada comando conserva os requisitos dos seus subsistemas. */
public final class DriverMechanismBindings {
    private DriverMechanismBindings() {}

    public static void bind(CommandXboxController controller, Command lowerIntake,
            Command raiseIntake, Command roller, Command aim, Command shoot) {
        Trigger enabled = controller.back().negate().and(controller.start().negate())
            .and(DriverStation::isTeleopEnabled);
        // R1 e L1 juntos cancelam as duas direções. O runEnd segura a posição.
        controller.rightBumper().and(controller.leftBumper().negate()).and(enabled)
            .whileTrue(lowerIntake);
        controller.leftBumper().and(controller.rightBumper().negate()).and(enabled)
            .whileTrue(raiseIntake);
        controller.rightTrigger().and(enabled).whileTrue(roller);
        // B tem prioridade sobre a mira isolada, pois coordena os mecanismos.
        controller.leftTrigger().and(controller.b().negate()).and(enabled).whileTrue(aim);
        controller.b().and(enabled).whileTrue(shoot);
    }

    /** 🛑 Calibração só no treino: com FMS, nenhum desses botões faz o robô andar. */
    public static Trigger calibrationAllowed() {
        return new Trigger(() -> !DriverStation.isFMSAttached());
    }
}
