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

    // Uma volta completa
    private static final double TARGET_ROTATION_RAD =
        2.0 * Math.PI;


    // Velocidade angular máxima do teste
    //
    // 0.50 rad/s ≈ 28.6 graus/s
    private static final double MAX_OMEGA_RAD_PER_SEC =
        0.50;


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


    // Yaw atual do Pigeon
    private double currentYawRad;


    // Diferença assinada desde o início
    private double signedRotationRad;


    // Quanto o robô realmente girou
    private double traveledRotationRad;


    // Erro restante até 360°
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


        signedRotationRad =
            0.0;


        traveledRotationRad =
            0.0;


        errorRad =
            TARGET_ROTATION_RAD;


        commandedOmega =
            0.0;



        // --------------------------------------------------------
        // LOGS
        // --------------------------------------------------------

        Logger.recordOutput(
            "Validation/Rotate360/Active",
            true
        );


        Logger.recordOutput(
            "Validation/Rotate360/StartYawRad",
            startYawRad
        );


        Logger.recordOutput(
            "Validation/Rotate360/StartYawDegrees",
            Math.toDegrees(
                startYawRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/TargetDegrees",
            360.0
        );


        Logger.recordOutput(
            "Validation/Rotate360/Finished",
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
        // 2 - CALCULA QUANTO GIRAMOS
        // ========================================================

        signedRotationRad =
            currentYawRad
                - startYawRad;


        /*
         * Para o teste de uma volta completa,
         * queremos saber a quantidade física girada.
         *
         * Não importa neste momento se o sentido
         * do Pigeon aparece positivo ou negativo.
         */
        traveledRotationRad =
            Math.abs(
                signedRotationRad
            );



        // ========================================================
        // 3 - ERRO PARA 360°
        // ========================================================

        errorRad =
            TARGET_ROTATION_RAD
                - traveledRotationRad;



        // ========================================================
        // 4 - CONTROLE PROPORCIONAL
        // ========================================================

        commandedOmega =
            TURN_KP
                * errorRad;



        // Limita velocidade máxima
        commandedOmega =
            MathUtil.clamp(
                commandedOmega,
                -MAX_OMEGA_RAD_PER_SEC,
                MAX_OMEGA_RAD_PER_SEC
            );



        // ========================================================
        // 5 - VELOCIDADE MÍNIMA
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
        // 6 - ENVIA PARA O SWERVE
        // ========================================================

        drive.setCalibrationChassisSpeeds(
            0.0,
            0.0,
            commandedOmega
        );



        // ========================================================
        // 7 - LOGS
        // ========================================================

        Logger.recordOutput(
            "Validation/Rotate360/CurrentYawDegrees",
            Math.toDegrees(
                currentYawRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/SignedRotationDegrees",
            Math.toDegrees(
                signedRotationRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/TraveledDegrees",
            Math.toDegrees(
                traveledRotationRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/ErrorDegrees",
            Math.toDegrees(
                errorRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/CommandedOmega",
            commandedOmega
        );


        // Pose da odometria
        Logger.recordOutput(
            "Validation/Rotate360/PoseDegrees",
            drive
                .getPose()
                .getRotation()
                .getDegrees()
        );


        Logger.recordOutput(
            "Validation/Rotate360/PoseX",
            drive
                .getPose()
                .getX()
        );


        Logger.recordOutput(
            "Validation/Rotate360/PoseY",
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
            "Validation/Rotate360/Active",
            false
        );


        Logger.recordOutput(
            "Validation/Rotate360/Finished",
            !interrupted
        );


        Logger.recordOutput(
            "Validation/Rotate360/Interrupted",
            interrupted
        );


        Logger.recordOutput(
            "Validation/Rotate360/FinalYawDegrees",
            Math.toDegrees(
                currentYawRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/FinalSignedRotationDegrees",
            Math.toDegrees(
                signedRotationRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/FinalTraveledDegrees",
            Math.toDegrees(
                traveledRotationRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/FinalErrorDegrees",
            Math.toDegrees(
                errorRad
            )
        );


        Logger.recordOutput(
            "Validation/Rotate360/FinalPoseDegrees",
            drive
                .getPose()
                .getRotation()
                .getDegrees()
        );


        Logger.recordOutput(
            "Validation/Rotate360/FinalPoseX",
            drive
                .getPose()
                .getX()
        );


        Logger.recordOutput(
            "Validation/Rotate360/FinalPoseY",
            drive
                .getPose()
                .getY()
        );
    }
}