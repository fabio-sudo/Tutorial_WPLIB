package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Escolha inicial do cenário, gravada como entrada para reproduzir a simulação.
 * Não copia a pose fundida e não aplica resets a cada frame.
 */
@AutoLog
public class DriveStartupInputs {
    public boolean initializeSimulationPose = false;
    public Pose2d initialPose = new Pose2d();
}
