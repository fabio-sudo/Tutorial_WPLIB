package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Controle de operador opcional: não consulta botões/eixo ausentes na DS. */
public final class OptionalOperatorController extends CommandXboxController {
    private final int port;

    public OptionalOperatorController(int port) {
        super(port);
        this.port = port;
    }

    @Override public Trigger button(int button, EventLoop loop) {
        return new Trigger(loop, () -> DriverStation.isJoystickConnected(port)
            && button > 0 && DriverStation.getStickButtonCount(port) >= button
            && getHID().getRawButton(button));
    }

    @Override public double getRightX() {
        int axis = XboxController.Axis.kRightX.value;
        return DriverStation.isJoystickConnected(port) && DriverStation.getStickAxisCount(port) > axis
            ? getHID().getRawAxis(axis) : 0.0;
    }
}
