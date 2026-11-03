package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import java.util.function.Function;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Andar reto e girar ao mesmo tempo. Os módulos seguem cada estado por 20 ms, mas
 * o robô gira nesse intervalo: sem discretizar, ele escapa para o lado do giro.
 * O caminho é integrado como no robô, com os módulos cumprindo o pedido.
 */
class DriveDiscretizeTest {

    private static final double PERIOD_SECONDS = 0.02;
    private static final double SECONDS = 2.0;
    private static final double SPEED_METERS_PER_SEC = 2.0;
    private static final double OMEGA_RAD_PER_SEC = Math.PI;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3));

    /** Distância lateral final: o pedido era andar reto em X, então o certo é zero. */
    private double lateralDriftMeters(Function<ChassisSpeeds, SwerveModuleState[]> toStates) {
        Pose2d pose = new Pose2d();
        for (int cycle = 0; cycle < SECONDS / PERIOD_SECONDS; cycle++) {
            ChassisSpeeds pedido = ChassisSpeeds.fromFieldRelativeSpeeds(
                SPEED_METERS_PER_SEC, 0.0, OMEGA_RAD_PER_SEC, pose.getRotation());
            ChassisSpeeds feito = kinematics.toChassisSpeeds(toStates.apply(pedido));
            pose = pose.exp(new Twist2d(feito.vxMetersPerSecond * PERIOD_SECONDS,
                feito.vyMetersPerSecond * PERIOD_SECONDS, feito.omegaRadiansPerSecond * PERIOD_SECONDS));
        }
        return Math.abs(pose.getY());
    }

    @Test void translatingWhileRotatingStaysOnTheRequestedLine() {
        double semDiscretizar = lateralDriftMeters(kinematics::toSwerveModuleStates);
        double comDiscretizar = lateralDriftMeters(speeds -> Drive.moduleStatesFor(kinematics, speeds));
        assertTrue(semDiscretizar > 0.05, "sem discretizar o robô escapava: " + semDiscretizar);
        assertTrue(comDiscretizar < 0.001, "ficou na linha: " + comDiscretizar);
    }

    @Test void withoutRotationNothingChanges() {
        ChassisSpeeds reto = new ChassisSpeeds(2.0, 1.0, 0.0);
        SwerveModuleState[] antes = kinematics.toSwerveModuleStates(reto);
        SwerveModuleState[] agora = Drive.moduleStatesFor(kinematics, reto);
        for (int i = 0; i < antes.length; i++) {
            assertEquals(antes[i].speedMetersPerSecond, agora[i].speedMetersPerSecond, 1e-9);
            assertEquals(antes[i].angle.getRadians(), agora[i].angle.getRadians(), 1e-9);
        }
    }

    @Test void onlyRotatingNothingChanges() {
        // Rotate360Command e o giro no lugar não podem mudar.
        ChassisSpeeds giro = new ChassisSpeeds(0.0, 0.0, 2.0);
        SwerveModuleState[] antes = kinematics.toSwerveModuleStates(giro);
        SwerveModuleState[] agora = Drive.moduleStatesFor(kinematics, giro);
        for (int i = 0; i < antes.length; i++) {
            assertEquals(antes[i].speedMetersPerSecond, agora[i].speedMetersPerSecond, 1e-9);
            assertEquals(antes[i].angle.getRadians(), agora[i].angle.getRadians(), 1e-9);
        }
    }
}
