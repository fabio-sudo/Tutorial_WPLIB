package frc.robot.calibration;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.Drive;


public class DriveTwoMetersCommand extends Command {

    // ============================================================
    // SUBSYSTEM
    // ============================================================

    private final Drive drive;


    // ============================================================
    // CONFIGURAÇÕES DO TESTE
    // ============================================================

    // Distância que queremos percorrer
    private static final double TARGET_DISTANCE_METERS =
        2.00;


    // Velocidade máxima durante o teste
    //
    // Mantemos baixa para:
    //
    // - reduzir derrapagem
    // - reduzir overshoot
    // - melhorar a precisão da calibração
    //
    private static final double MAX_SPEED_MPS =
        0.50;


    // Velocidade mínima perto do alvo
    //
    // Evita que o robô fique muito lento
    // antes de chegar aos 2 metros.
    //
    private static final double MIN_SPEED_MPS =
        0.12;


    // Ganho proporcional para distância
    private static final double DRIVE_KP =
        1.0;


    // Tolerância final
    //
    // 0.03 m = 3 cm
    //
    private static final double DISTANCE_TOLERANCE_METERS =
        0.03;


    // ============================================================
    // CORREÇÃO DE HEADING
    // ============================================================

    // Ganho proporcional da correção angular
    private static final double HEADING_KP =
        2.0;


    // Limite máximo da correção angular
    private static final double MAX_HEADING_CORRECTION_RAD_PER_SEC =
        0.40;


    // ============================================================
    // DADOS DO INÍCIO DO TESTE
    // ============================================================

    private Pose2d startPose;

    private double startHeadingRad;


    // ============================================================
    // CONSTRUTOR
    // ============================================================

    public DriveTwoMetersCommand(
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
        // Guarda a pose inicial
        // --------------------------------------------------------

        startPose =
            drive.getPose();


        // --------------------------------------------------------
        // Guarda a orientação inicial
        // --------------------------------------------------------

        startHeadingRad =
            drive
                .getRotation()
                .getRadians();


        // --------------------------------------------------------
        // LOGS
        // --------------------------------------------------------

        Logger.recordOutput(
            "Validation/Drive2m/Active",
            true
        );


        Logger.recordOutput(
            "Validation/Drive2m/StartX",
            startPose.getX()
        );


        Logger.recordOutput(
            "Validation/Drive2m/StartY",
            startPose.getY()
        );


        Logger.recordOutput(
            "Validation/Drive2m/StartHeadingRad",
            startHeadingRad
        );


        Logger.recordOutput(
            "Validation/Drive2m/TargetDistanceMeters",
            TARGET_DISTANCE_METERS
        );
    }


    // ============================================================
    // EXECUTE
    // ============================================================

    @Override
    public void execute() {

        // --------------------------------------------------------
        // POSE ATUAL
        // --------------------------------------------------------

        Pose2d currentPose =
            drive.getPose();


        // --------------------------------------------------------
        // DESLOCAMENTO DESDE O INÍCIO
        // --------------------------------------------------------

        double deltaX =
            currentPose.getX()
            -
            startPose.getX();


        double deltaY =
            currentPose.getY()
            -
            startPose.getY();


        // ========================================================
        // DISTÂNCIA PERCORRIDA NA DIREÇÃO INICIAL
        // ========================================================
        //
        // Isso permite realizar o teste mesmo que o robô
        // não esteja apontado exatamente para o eixo X do campo.
        //
        // Exemplo:
        //
        // robô começa apontado 90°
        //
        // continua sendo possível medir corretamente os 2 metros.
        //
        // ========================================================

        double distanceForward =
            deltaX * Math.cos(
                startHeadingRad
            )
            +
            deltaY * Math.sin(
                startHeadingRad
            );


        // ========================================================
        // DESLOCAMENTO LATERAL
        // ========================================================
        //
        // Não é usado para controlar a distância.
        //
        // Serve para verificarmos depois no AdvantageScope
        // se o robô desviou lateralmente.
        //
        // ========================================================

        double lateralDistance =
            -deltaX * Math.sin(
                startHeadingRad
            )
            +
            deltaY * Math.cos(
                startHeadingRad
            );


        // ========================================================
        // ERRO DE DISTÂNCIA
        // ========================================================

        double distanceError =
            TARGET_DISTANCE_METERS
            -
            distanceForward;


        // ========================================================
        // CONTROLE DE VELOCIDADE
        // ========================================================

        double speedCommand =
            distanceError
            *
            DRIVE_KP;


        // Limita a velocidade máxima
        speedCommand =
            MathUtil.clamp(
                speedCommand,
                -MAX_SPEED_MPS,
                MAX_SPEED_MPS
            );


        // --------------------------------------------------------
        // GARANTE UMA VELOCIDADE MÍNIMA
        // --------------------------------------------------------
        //
        // Só fazemos isso se ainda estivermos
        // fora da tolerância.
        //
        // --------------------------------------------------------

        if (
            Math.abs(
                distanceError
            )
            >
            DISTANCE_TOLERANCE_METERS

            &&

            Math.abs(
                speedCommand
            )
            <
            MIN_SPEED_MPS
        ) {

            speedCommand =
                Math.copySign(
                    MIN_SPEED_MPS,
                    distanceError
                );
        }


        // ========================================================
        // CONTROLE DE HEADING
        // ========================================================

        double currentHeadingRad =
            drive
                .getRotation()
                .getRadians();


        // Erro angular sempre entre -PI e +PI
        double headingErrorRad =
            MathUtil.angleModulus(

                startHeadingRad
                -
                currentHeadingRad

            );


        double omegaCommand =
            headingErrorRad
            *
            HEADING_KP;


        omegaCommand =
            MathUtil.clamp(

                omegaCommand,

                -MAX_HEADING_CORRECTION_RAD_PER_SEC,

                MAX_HEADING_CORRECTION_RAD_PER_SEC
            );


        // ========================================================
        // ENVIA O MOVIMENTO PARA O SWERVE
        // ========================================================
        //
        // Robot Relative:
        //
        // vx    = movimento para frente
        // vy    = 0
        // omega = correção de direção
        //
        // ========================================================

        drive.drive(

            speedCommand,

            0.0,

            omegaCommand

        );


        // ========================================================
        // LOGS - ADVANTAGESCOPE
        // ========================================================

        Logger.recordOutput(
            "Validation/Drive2m/DistanceMeters",
            distanceForward
        );


        Logger.recordOutput(
            "Validation/Drive2m/LateralDistanceMeters",
            lateralDistance
        );


        Logger.recordOutput(
            "Validation/Drive2m/ErrorMeters",
            distanceError
        );


        Logger.recordOutput(
            "Validation/Drive2m/SpeedCommand",
            speedCommand
        );


        Logger.recordOutput(
            "Validation/Drive2m/HeadingErrorRad",
            headingErrorRad
        );


        Logger.recordOutput(
            "Validation/Drive2m/HeadingErrorDegrees",
            Math.toDegrees(
                headingErrorRad
            )
        );


        Logger.recordOutput(
            "Validation/Drive2m/CurrentX",
            currentPose.getX()
        );


        Logger.recordOutput(
            "Validation/Drive2m/CurrentY",
            currentPose.getY()
        );
    }


    // ============================================================
    // IS FINISHED
    // ============================================================

    @Override
    public boolean isFinished() {

        Pose2d currentPose =
            drive.getPose();


        double deltaX =
            currentPose.getX()
            -
            startPose.getX();


        double deltaY =
            currentPose.getY()
            -
            startPose.getY();


        // Distância na direção original do robô
        double distanceForward =
            deltaX * Math.cos(
                startHeadingRad
            )
            +
            deltaY * Math.sin(
                startHeadingRad
            );


        double error =
            TARGET_DISTANCE_METERS
            -
            distanceForward;


        // --------------------------------------------------------
        // TERMINA QUANDO CHEGAR EM:
        //
        // 2.00 m ± 0.03 m
        //
        // Portanto aproximadamente:
        //
        // 1.97 m até 2.03 m
        //
        // --------------------------------------------------------

        return Math.abs(
            error
        )
        <=
        DISTANCE_TOLERANCE_METERS;
    }


    // ============================================================
    // END
    // ============================================================

    @Override
    public void end(
        boolean interrupted
    ) {

        // ========================================================
        // PARA IMEDIATAMENTE
        // ========================================================

        drive.stop();


        // ========================================================
        // PEGA A POSE FINAL
        // ========================================================

        Pose2d finalPose =
            drive.getPose();


        double deltaX =
            finalPose.getX()
            -
            startPose.getX();


        double deltaY =
            finalPose.getY()
            -
            startPose.getY();


        // ========================================================
        // DISTÂNCIA FINAL
        // ========================================================

        double finalDistance =
            deltaX * Math.cos(
                startHeadingRad
            )
            +
            deltaY * Math.sin(
                startHeadingRad
            );


        // ========================================================
        // DESLOCAMENTO LATERAL FINAL
        // ========================================================

        double finalLateralDistance =
            -deltaX * Math.sin(
                startHeadingRad
            )
            +
            deltaY * Math.cos(
                startHeadingRad
            );


        // ========================================================
        // HEADING FINAL
        // ========================================================

        double finalHeadingRad =
            drive
                .getRotation()
                .getRadians();


        double finalHeadingErrorRad =
            MathUtil.angleModulus(

                startHeadingRad
                -
                finalHeadingRad

            );


        // ========================================================
        // LOGS FINAIS
        // ========================================================

        Logger.recordOutput(
            "Validation/Drive2m/FinalDistanceMeters",
            finalDistance
        );


        Logger.recordOutput(
            "Validation/Drive2m/FinalLateralDistanceMeters",
            finalLateralDistance
        );


        Logger.recordOutput(
            "Validation/Drive2m/FinalX",
            finalPose.getX()
        );


        Logger.recordOutput(
            "Validation/Drive2m/FinalY",
            finalPose.getY()
        );


        Logger.recordOutput(
            "Validation/Drive2m/FinalHeadingRad",
            finalHeadingRad
        );


        Logger.recordOutput(
            "Validation/Drive2m/FinalHeadingDegrees",
            Math.toDegrees(
                finalHeadingRad
            )
        );


        Logger.recordOutput(
            "Validation/Drive2m/FinalHeadingErrorDegrees",
            Math.toDegrees(
                finalHeadingErrorRad
            )
        );


        Logger.recordOutput(
            "Validation/Drive2m/Interrupted",
            interrupted
        );


        Logger.recordOutput(
            "Validation/Drive2m/Active",
            false
        );
    }
}