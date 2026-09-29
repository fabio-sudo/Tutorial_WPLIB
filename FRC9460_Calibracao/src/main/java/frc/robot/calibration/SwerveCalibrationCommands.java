package frc.robot.calibration;


// #region IMPORTS

// ============================================================
// WPILIB - COMMANDS
// ============================================================

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// ============================================================
// ADVANTAGEKIT
// ============================================================

import org.littletonrobotics.junction.Logger;


// ============================================================
// SUBSYSTEM DRIVE
// ============================================================

import frc.robot.subsystems.drive.Drive;

// #endregion



/**
 * ============================================================
 * SWERVE CALIBRATION COMMANDS
 * ============================================================
 *
 * Classe utilizada SOMENTE no projeto de calibração.
 *
 * Objetivo:
 *
 * criar testes determinísticos para o Swerve sem depender
 * da posição do joystick.
 *
 *
 * Aqui teremos testes de:
 *
 * 1 - Ângulo dos módulos
 *
 *     0°
 *     45°
 *     90°
 *     180°
 *
 *
 * 2 - Velocidade do Drive
 *
 *     +0.25 m/s
 *     -0.25 m/s
 *
 *     +0.50 m/s
 *     -0.50 m/s
 *
 *     +1.00 m/s
 *     -1.00 m/s
 *
 *
 * 3 - Rotação pura do robô
 *
 *     +0.50 rad/s
 *     -0.50 rad/s
 *
 *
 * Todos os testes possuem:
 *
 * - valor conhecido
 * - tempo conhecido
 * - parada automática
 * - logs no AdvantageKit
 *
 * ============================================================
 */
public final class SwerveCalibrationCommands {


    // #region CONSTANTES DOS TESTES


    // ============================================================
    // TEMPO - TESTE DE STEER
    // ============================================================
    //
    // O módulo ficará tentando alcançar o ângulo solicitado
    // durante este período.
    //
    // ============================================================

    private static final double STEER_TEST_TIME_SECONDS =
        2.0;



    // ============================================================
    // TEMPO - TESTE DE VELOCIDADE
    // ============================================================
    //
    // Mantém a velocidade solicitada por tempo suficiente
    // para observar:
    //
    // Setpoint
    // Measured
    // Error
    //
    // ============================================================

    private static final double SPEED_TEST_TIME_SECONDS =
        3.0;



    // ============================================================
    // TEMPO - TESTE DE ROTAÇÃO
    // ============================================================

    private static final double ROTATION_TEST_TIME_SECONDS =
        2.0;



    // ============================================================
    // VELOCIDADES DE TESTE
    // ============================================================

    private static final double SPEED_025 =
        0.25;

    private static final double SPEED_050 =
        0.50;

    private static final double SPEED_100 =
        1.00;



    // ============================================================
    // VELOCIDADE ANGULAR DE TESTE
    // ============================================================
    //
    // 0.50 rad/s
    //
    // aproximadamente:
    //
    // 28.6 graus por segundo
    //
    // ============================================================

    private static final double ROTATION_SPEED_RAD_PER_SEC =
        0.50;


    // #endregion



    // #region CONSTRUTOR


    /**
     * Classe utilitária.
     *
     * Não queremos fazer:
     *
     * new SwerveCalibrationCommands()
     *
     * Todos os métodos serão static.
     */
    private SwerveCalibrationCommands() {
    }


    // #endregion



    // #region TESTES DE STEER


    // ============================================================
    // STEER - 0 GRAUS
    // ============================================================

    public static Command steer0(
        Drive drive
    ) {

        return steerAngleTest(
            drive,
            0.0
        );
    }



    // ============================================================
    // STEER - 45 GRAUS
    // ============================================================

    public static Command steer45(
        Drive drive
    ) {

        return steerAngleTest(
            drive,
            45.0
        );
    }



    // ============================================================
    // STEER - 90 GRAUS
    // ============================================================

    public static Command steer90(
        Drive drive
    ) {

        return steerAngleTest(
            drive,
            90.0
        );
    }



    // ============================================================
    // STEER - 180 GRAUS
    // ============================================================

    public static Command steer180(
        Drive drive
    ) {

        return steerAngleTest(
            drive,
            180.0
        );
    }


    // #endregion



    // #region MÉTODO INTERNO - TESTE DE STEER


    /**
     * Executa um teste genérico de posição do Steer.
     *
     * Os quatro módulos receberão exatamente o mesmo
     * ângulo desejado.
     *
     * @param drive subsystem Drive
     * @param angleDegrees ângulo desejado em graus
     */
    private static Command steerAngleTest(
        Drive drive,
        double angleDegrees
    ) {

        // Converte graus para radianos.
        double angleRadians =
            Math.toRadians(
                angleDegrees
            );


        return Commands.run(

            () -> {

                // ====================================================
                // DRIVE SEM MOVIMENTO
                // ====================================================
                //
                // Durante o teste do Steer,
                // as rodas NÃO devem tracionar.
                //
                // ====================================================

                drive.setCalibrationDriveVelocity(
                    0.0
                );


                // ====================================================
                // POSICIONA OS QUATRO STEERS
                // ====================================================

                drive.setCalibrationSteerAngle(
                    angleRadians
                );


                // ====================================================
                // LOGS
                // ====================================================

                Logger.recordOutput(
                    "Calibration/Swerve/Active",
                    true
                );


                Logger.recordOutput(
                    "Calibration/Swerve/Test",
                    "STEER"
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetAngleDegrees",
                    angleDegrees
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetAngleRad",
                    angleRadians
                );

            },

            drive
        )

        // ============================================================
        // TEMPO FIXO
        // ============================================================

        .withTimeout(
            STEER_TEST_TIME_SECONDS
        )


        // ============================================================
        // FINALIZAÇÃO SEGURA
        // ============================================================

        .finallyDo(

            interrupted -> {

                drive.stopCalibration();


                Logger.recordOutput(
                    "Calibration/Swerve/Active",
                    false
                );


                Logger.recordOutput(
                    "Calibration/Swerve/Test",
                    "IDLE"
                );
            }
        );
    }


    // #endregion



    // #region TESTES DE VELOCIDADE - FORWARD


    // ============================================================
    // +0.25 m/s
    // ============================================================

    public static Command speed025Forward(
        Drive drive
    ) {

        return driveSpeedTest(
            drive,
            SPEED_025
        );
    }



    // ============================================================
    // +0.50 m/s
    // ============================================================

    public static Command speed050Forward(
        Drive drive
    ) {

        return driveSpeedTest(
            drive,
            SPEED_050
        );
    }



    // ============================================================
    // +1.00 m/s
    // ============================================================

    public static Command speed100Forward(
        Drive drive
    ) {

        return driveSpeedTest(
            drive,
            SPEED_100
        );
    }


    // #endregion



    // #region TESTES DE VELOCIDADE - REVERSE


    // ============================================================
    // -0.25 m/s
    // ============================================================

    public static Command speed025Reverse(
        Drive drive
    ) {

        return driveSpeedTest(
            drive,
            -SPEED_025
        );
    }



    // ============================================================
    // -0.50 m/s
    // ============================================================

    public static Command speed050Reverse(
        Drive drive
    ) {

        return driveSpeedTest(
            drive,
            -SPEED_050
        );
    }



    // ============================================================
    // -1.00 m/s
    // ============================================================

    public static Command speed100Reverse(
        Drive drive
    ) {

        return driveSpeedTest(
            drive,
            -SPEED_100
        );
    }


    // #endregion



    // #region MÉTODO INTERNO - TESTE DE VELOCIDADE


    /**
     * Testa uma velocidade exata nos quatro módulos.
     *
     * Antes de aplicar velocidade:
     *
     * Steer = 0°
     *
     * Dessa forma todas as rodas ficam apontadas
     * para frente.
     *
     * @param drive subsystem Drive
     * @param speedMetersPerSecond velocidade desejada
     */
    private static Command driveSpeedTest(
        Drive drive,
        double speedMetersPerSecond
    ) {

        return Commands.run(

            () -> {

                // ====================================================
                // ALINHA STEER EM 0°
                // ====================================================

                drive.setCalibrationSteerAngle(
                    0.0
                );


                // ====================================================
                // VELOCIDADE EXATA
                // ====================================================

                drive.setCalibrationDriveVelocity(
                    speedMetersPerSecond
                );


                // ====================================================
                // LOGS
                // ====================================================

                Logger.recordOutput(
                    "Calibration/Swerve/Active",
                    true
                );


                Logger.recordOutput(
                    "Calibration/Swerve/Test",
                    "DRIVE_SPEED"
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetSpeedMps",
                    speedMetersPerSecond
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetAngleDegrees",
                    0.0
                );
            },

            drive
        )

        // ============================================================
        // TEMPO FIXO
        // ============================================================

        .withTimeout(
            SPEED_TEST_TIME_SECONDS
        )


        // ============================================================
        // PARADA AUTOMÁTICA
        // ============================================================

        .finallyDo(

            interrupted -> {

                drive.stopCalibration();


                Logger.recordOutput(
                    "Calibration/Swerve/Active",
                    false
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetSpeedMps",
                    0.0
                );


                Logger.recordOutput(
                    "Calibration/Swerve/Test",
                    "IDLE"
                );
            }
        );
    }


    // #endregion



    // #region TESTES DE ROTAÇÃO


    // ============================================================
    // ROTAÇÃO POSITIVA
    // ============================================================
    //
    // vx = 0
    // vy = 0
    // omega = +0.50 rad/s
    //
    // ============================================================

    public static Command rotatePositive(
        Drive drive
    ) {

        return rotationTest(
            drive,
            ROTATION_SPEED_RAD_PER_SEC
        );
    }



    // ============================================================
    // ROTAÇÃO NEGATIVA
    // ============================================================
    //
    // vx = 0
    // vy = 0
    // omega = -0.50 rad/s
    //
    // ============================================================

    public static Command rotateNegative(
        Drive drive
    ) {

        return rotationTest(
            drive,
            -ROTATION_SPEED_RAD_PER_SEC
        );
    }


    // #endregion



    // #region MÉTODO INTERNO - TESTE DE ROTAÇÃO


    /**
     * Teste de rotação pura.
     *
     * Nenhuma velocidade linear é solicitada:
     *
     * vx = 0
     * vy = 0
     *
     * Apenas:
     *
     * omega != 0
     *
     * Esse teste será importante para validar:
     *
     * - WheelBase
     * - TrackWidth
     * - SwerveDriveKinematics
     * - direção dos módulos
     * - velocidade dos módulos
     * - Pigeon
     * - odometria
     *
     */
    private static Command rotationTest(
        Drive drive,
        double omegaRadiansPerSecond
    ) {

        return Commands.run(

            () -> {

                // ====================================================
                // MOVIMENTO ROBOT-RELATIVE
                // ====================================================
                //
                // vx = 0
                // vy = 0
                //
                // somente omega.
                //
                // ====================================================

                drive.setCalibrationChassisSpeeds(
                    0.0,
                    0.0,
                    omegaRadiansPerSecond
                );


                // ====================================================
                // LOGS
                // ====================================================

                Logger.recordOutput(
                    "Calibration/Swerve/Active",
                    true
                );


                Logger.recordOutput(
                    "Calibration/Swerve/Test",
                    "ROTATION"
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetVxMps",
                    0.0
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetVyMps",
                    0.0
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetOmegaRadPerSec",
                    omegaRadiansPerSecond
                );
            },

            drive
        )

        // ============================================================
        // TEMPO FIXO
        // ============================================================

        .withTimeout(
            ROTATION_TEST_TIME_SECONDS
        )


        // ============================================================
        // PARADA AUTOMÁTICA
        // ============================================================

        .finallyDo(

            interrupted -> {

                drive.stopCalibration();


                Logger.recordOutput(
                    "Calibration/Swerve/Active",
                    false
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetOmegaRadPerSec",
                    0.0
                );


                Logger.recordOutput(
                    "Calibration/Swerve/Test",
                    "IDLE"
                );
            }
        );
    }


    // #endregion



    // #region TESTE DE PARADA


    /**
     * Comando utilizado para garantir que nenhum movimento
     * de calibração permaneça ativo.
     */
    public static Command stop(
        Drive drive
    ) {

        return Commands.runOnce(

            () -> {

                drive.stopCalibration();


                Logger.recordOutput(
                    "Calibration/Swerve/Active",
                    false
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetSpeedMps",
                    0.0
                );


                Logger.recordOutput(
                    "Calibration/Swerve/TargetOmegaRadPerSec",
                    0.0
                );


                Logger.recordOutput(
                    "Calibration/Swerve/Test",
                    "STOPPED"
                );
            },

            drive
        );
    }


    // #endregion
}