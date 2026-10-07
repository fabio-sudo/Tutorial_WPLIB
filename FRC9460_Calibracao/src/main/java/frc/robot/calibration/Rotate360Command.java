package frc.robot.calibration;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.Drive;

public class Rotate360Command extends Command {

    // ============================================================
    // DRIVE
    // ============================================================

    private final Drive drive;


    // ============================================================
    // CONFIGURAÇÃO DO TESTE
    // ============================================================

    // Quantidade de voltas desejadas
    private static final double TARGET_TURNS =
        10.0;

    // 10 voltas = 20 * PI rad = 3600 graus
    private static final double TARGET_ROTATION_RAD =
        TARGET_TURNS * 2.0 * Math.PI;

    //====================================================================== Velocidade angular máxima do teste
    // 0.50 rad/s ≈ 28.6 graus/s
    private static final double MAX_OMEGA_RAD_PER_SEC =
       1.8;

    // Velocidade mínima para vencer atrito
    private static final double MIN_OMEGA_RAD_PER_SEC =
        0.10;

    // Controle proporcional
    private static final double TURN_KP =
        1.0;

    // Tolerância final
    private static final double TOLERANCE_RAD =
        Math.toRadians(2.0);


    // ============================================================
    // VARIÁVEIS DO TESTE
    // ============================================================

    // Yaw quando o teste começa
    private double startYawRad;

    // Yaw da leitura anterior
    private double previousYawRad;

    // Yaw atual do Pigeon
    private double currentYawRad;

    // Variação de yaw entre dois ciclos
    private double deltaYawRad;

    // Rotação acumulada com sinal
    private double signedRotationRad;

    // Rotação acumulada total
    private double traveledRotationRad;

    // Quantidade acumulada de voltas
    private double accumulatedTurns;

    // Erro restante
    private double errorRad;

    // Omega enviado ao drivetrain
    private double commandedOmega;


    // ============================================================
    // CONSTRUTOR
    // ============================================================

    public Rotate360Command(
        Drive drive
    ) {

        this.drive = drive;

        addRequirements(
            drive
        );
    }


    // ============================================================
    // INITIALIZE
    // ============================================================

    @Override
    public void initialize() {

        // --------------------------------------------------------
        // Guarda a orientação inicial
        // --------------------------------------------------------

        startYawRad =
            drive
                .getRotation()
                .getRadians();

        currentYawRad =
            startYawRad;

        previousYawRad =
            startYawRad;

        deltaYawRad =
            0.0;

        signedRotationRad =
            0.0;

        traveledRotationRad =
            0.0;

        accumulatedTurns =
            0.0;

        errorRad =
            TARGET_ROTATION_RAD;

        commandedOmega =
            0.0;


        // --------------------------------------------------------
        // LOGS INICIAIS
        // --------------------------------------------------------

        Logger.recordOutput(
            "Validation/Rotate10Turns/Active",
            true
        );

        Logger.recordOutput(
            "Validation/Rotate10Turns/StartYawRad",
            startYawRad
        );

        Logger.recordOutput(
            "Validation/Rotate10Turns/StartYawDegrees",
            Math.toDegrees(
                startYawRad
            )
        );

        Logger.recordOutput(
            "Validation/Rotate10Turns/TargetTurns",
            TARGET_TURNS
        );

        Logger.recordOutput(
            "Validation/Rotate10Turns/TargetDegrees",
            Math.toDegrees(
                TARGET_ROTATION_RAD
            )
        );

        Logger.recordOutput(
            "Validation/Rotate10Turns/AccumulatedDegrees",
            0.0
        );

        Logger.recordOutput(
            "Validation/Rotate10Turns/AccumulatedTurns",
            0.0
        );

        Logger.recordOutput(
            "Validation/Rotate10Turns/Finished",
            false
        );
    }


    // ============================================================
    // EXECUTE
    // ============================================================

    @Override
    public void execute() {

        // ========================================================
        // 1 - LÊ O PIGEON
        // ========================================================

        currentYawRad =
            drive
                .getRotation()
                .getRadians();


        // ========================================================
        // 2 - CALCULA A VARIAÇÃO DESDE O ÚLTIMO CICLO
        // ========================================================
        //
        // angleModulus mantém cada pequeno delta entre
        // -PI e +PI.
        //
        // Isso permite acumular várias voltas mesmo quando
        // a leitura passa de +180 para -180.
        //
        // ========================================================

        deltaYawRad =
            MathUtil.angleModulus(
                currentYawRad - previousYawRad
            );

        signedRotationRad +=
            deltaYawRad;

        previousYawRad =
            currentYawRad;


        // ========================================================
        // 3 - ROTAÇÃO TOTAL ACUMULADA
        // ========================================================

        traveledRotationRad =
            Math.abs(
                signedRotationRad
            );

        accumulatedTurns =
            traveledRotationRad
                / (2.0 * Math.PI);


        // ========================================================
        // 4 - ERRO PARA 10 VOLTAS
        // ========================================================

        errorRad =
            TARGET_ROTATION_RAD
                - traveledRotationRad;


        // ========================================================
        // 5 - CONTROLE PROPORCIONAL
        // ========================================================

        commandedOmega =
            TURN_KP
                * errorRad;

        commandedOmega =
            MathUtil.clamp(
                commandedOmega,
                -MAX_OMEGA_RAD_PER_SEC,
                MAX_OMEGA_RAD_PER_SEC
            );


        // ========================================================
        // 6 - VELOCIDADE MÍNIMA
        // ========================================================

        if (
            Math.abs(errorRad)
                > TOLERANCE_RAD
        ) {

            if (
                Math.abs(commandedOmega)
                    < MIN_OMEGA_RAD_PER_SEC
            ) {

                commandedOmega =
                    Math.copySign(
                        MIN_OMEGA_RAD_PER_SEC,
                        commandedOmega
                    );
            }

        } else {

            commandedOmega =
                0.0;
        }


        // ========================================================
        // 7 - ENVIA PARA O SWERVE
        // ========================================================

        drive.setCalibrationChassisSpeeds(
            0.0,
            0.0,
            commandedOmega
        );


        // ========================================================
        // 8 - LOGS - ADVANTAGESCOPE
        // ========================================================

        Logger.recordOutput(
            "Validation/Rotate10Turns/CurrentYawDegrees",
            Math.toDegrees(
                currentYawRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/DeltaYawDegrees",
            Math.toDegrees(
                deltaYawRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/SignedAccumulatedDegrees",
            Math.toDegrees(
                signedRotationRad
            )
        );


        // ========================================================
        // GRAUS ACUMULADOS
        // ========================================================

        Logger.recordOutput(
            "Validation/Rotate10Turns/AccumulatedDegrees",
            Math.toDegrees(
                traveledRotationRad
            )
        );


        // ========================================================
        // VOLTAS ACUMULADAS
        // ========================================================
        //
        // Exemplo:
        //
        // 1.00 = uma volta
        // 2.50 = duas voltas e meia
        // 10.0 = teste completo
        //
        // ========================================================

        Logger.recordOutput(
            "Validation/Rotate10Turns/AccumulatedTurns",
            accumulatedTurns
        );


        // ========================================================
        // VOLTAS COMPLETAS
        // ========================================================

        Logger.recordOutput(
            "Validation/Rotate10Turns/CompletedFullTurns",
            Math.floor(
                accumulatedTurns
            )
        );


        // ========================================================
        // VOLTAS RESTANTES
        // ========================================================

        Logger.recordOutput(
            "Validation/Rotate10Turns/RemainingTurns",
            Math.max(
                0.0,
                TARGET_TURNS - accumulatedTurns
            )
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/ErrorDegrees",
            Math.toDegrees(
                errorRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/CommandedOmega",
            commandedOmega
        );


        // ========================================================
        // POSE
        // ========================================================

        Logger.recordOutput(
            "Validation/Rotate10Turns/PoseDegrees",
            drive
                .getPose()
                .getRotation()
                .getDegrees()
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/PoseX",
            drive
                .getPose()
                .getX()
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/PoseY",
            drive
                .getPose()
                .getY()
        );
    }


    // ============================================================
    // FINALIZAÇÃO AUTOMÁTICA
    // ============================================================

    @Override
    public boolean isFinished() {

        return
            Math.abs(errorRad)
                <= TOLERANCE_RAD;
    }


    // ============================================================
    // END
    // ============================================================

    @Override
    public void end(
        boolean interrupted
    ) {

        // Para imediatamente
        drive.stopCalibration();


        // ========================================================
        // LOGS FINAIS
        // ========================================================

        Logger.recordOutput(
            "Validation/Rotate10Turns/Active",
            false
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/Finished",
            !interrupted
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/Interrupted",
            interrupted
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/FinalYawDegrees",
            Math.toDegrees(
                currentYawRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/FinalSignedAccumulatedDegrees",
            Math.toDegrees(
                signedRotationRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/FinalAccumulatedDegrees",
            Math.toDegrees(
                traveledRotationRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/FinalAccumulatedTurns",
            accumulatedTurns
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/FinalErrorDegrees",
            Math.toDegrees(
                errorRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/FinalPoseDegrees",
            drive
                .getPose()
                .getRotation()
                .getDegrees()
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/FinalPoseX",
            drive
                .getPose()
                .getX()
        );


        Logger.recordOutput(
            "Validation/Rotate10Turns/FinalPoseY",
            drive
                .getPose()
                .getY()
        );
    }
}