package frc.robot.calibration;


// #region IMPORTS

import static frc.robot.calibration.CalibrationConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.Drive;

// #endregion



public final class CalibrationCommands {


    // #region CONSTRUTOR

    private CalibrationCommands() {}

    // #endregion



    // #region FUNÇÕES AUXILIARES


    // ============================================================
    // INICIAR TESTE
    // ============================================================
    //
    // Registra no AdvantageKit:
    //
    // Calibration/Test
    // Calibration/Step
    // Calibration/Active
    // Calibration/CommandVelocity
    //
    // ============================================================

    private static void startTest(
        String testName
    ) {

        Logger.recordOutput(
            "Calibration/Test",
            testName
        );


        Logger.recordOutput(
            "Calibration/Step",
            "START"
        );


        Logger.recordOutput(
            "Calibration/Active",
            true
        );


        Logger.recordOutput(
            "Calibration/CommandVelocity",
            0.0
        );
    }



    // ============================================================
    // FINALIZAR TESTE
    // ============================================================
    //
    // Este método é chamado pelo finallyDo().
    //
    // interrupted = true
    //
    // significa:
    //
    // o operador soltou o botão
    // ou o Command foi cancelado.
    //
    //
    // interrupted = false
    //
    // significa:
    //
    // o teste chegou normalmente ao final.
    //
    // ============================================================

    private static void finishTest(
        Drive drive,
        boolean interrupted
    ) {

        // --------------------------------------------------------
        // GARANTE QUE O ROBÔ PARE
        // --------------------------------------------------------

        drive.stop();


        // --------------------------------------------------------
        // VELOCIDADE COMANDADA VOLTA PARA ZERO
        // --------------------------------------------------------

        Logger.recordOutput(
            "Calibration/CommandVelocity",
            0.0
        );


        // --------------------------------------------------------
        // TESTE NÃO ESTÁ MAIS ATIVO
        // --------------------------------------------------------

        Logger.recordOutput(
            "Calibration/Active",
            false
        );


        // --------------------------------------------------------
        // INFORMA COMO O TESTE TERMINOU
        // --------------------------------------------------------

        Logger.recordOutput(

            "Calibration/Step",

            interrupted
                ? "CANCELLED"
                : "FINISHED"
        );
    }



    // ============================================================
    // PARAR ROBÔ
    // ============================================================
    //
    // Mantém o Swerve parado pelo tempo definido.
    //
    // ============================================================

    private static Command stop(
        Drive drive,
        double seconds,
        String stepName
    ) {

        return Commands.run(

            () -> {

                Logger.recordOutput(
                    "Calibration/Step",
                    stepName
                );


                Logger.recordOutput(
                    "Calibration/CommandVelocity",
                    0.0
                );


                drive.stop();
            },

            drive

        )

        .withTimeout(seconds);
    }



    // ============================================================
    // DRIVE RETO
    // ============================================================
    //
    // velocity > 0
    //     → frente
    //
    // velocity < 0
    //     → ré
    //
    //
    // drive.drive(
    //
    //     vx,
    //     vy,
    //     omega
    //
    // );
    //
    //
    // Neste teste:
    //
    // vx    = velocidade informada
    // vy    = 0
    // omega = 0
    //
    // ============================================================

    private static Command driveStraight(
        Drive drive,
        double velocity,
        double seconds,
        String stepName
    ) {

        return Commands.runEnd(


            // ====================================================
            // EXECUTA DURANTE A ETAPA
            // ====================================================

            () -> {

                Logger.recordOutput(
                    "Calibration/Step",
                    stepName
                );


                Logger.recordOutput(
                    "Calibration/CommandVelocity",
                    velocity
                );


                drive.drive(

                    velocity,   // vx
                    0.0,        // vy
                    0.0         // omega

                );
            },


            // ====================================================
            // EXECUTA QUANDO ESTA ETAPA TERMINAR
            // ====================================================

            drive::stop,


            // ====================================================
            // SUBSYSTEM UTILIZADO
            // ====================================================

            drive
        )

        .withTimeout(seconds);
    }

    // #endregion





    // #region TESTE DRIVE LOW


    // ============================================================
    // TESTE DRIVE LOW
    // ============================================================
    //
    // Objetivo:
    //
    // testar uma velocidade baixa nos dois sentidos.
    //
    //
    // SEQUÊNCIA:
    //
    // PARADO
    //    ↓
    // +0.25 m/s
    //    ↓
    // PARADO
    //    ↓
    // -0.25 m/s
    //    ↓
    // PARADO
    //
    //
    // Utilizar com:
    //
    // .whileTrue(...)
    //
    //
    // Soltou o botão:
    //
    // Command cancelado
    //      ↓
    // finallyDo()
    //      ↓
    // drive.stop()
    //
    // ============================================================

    public static Command driveLowTest(
        Drive drive
    ) {

        return Commands.sequence(


            // #region INÍCIO

            Commands.runOnce(

                () -> startTest(
                    "DRIVE LOW"
                )
            ),

            // #endregion



            // #region PARADO INICIAL

            stop(
                drive,
                STOP_TIME,
                "INITIAL STOP"
            ),

            // #endregion



            // #region FRENTE LOW

            driveStraight(

                drive,

                DRIVE_LOW,

                STEP_TIME,

                "FORWARD LOW"
            ),

            // #endregion



            // #region PARADO

            stop(

                drive,

                STOP_TIME,

                "STOP AFTER FORWARD"
            ),

            // #endregion



            // #region RÉ LOW

            driveStraight(

                drive,

                -DRIVE_LOW,

                STEP_TIME,

                "REVERSE LOW"
            ),

            // #endregion



            // #region PARADA FINAL

            stop(

                drive,

                STOP_TIME,

                "FINAL STOP"
            )

            // #endregion

        )


        // #region SEGURANÇA FINAL

        .finallyDo(

            interrupted ->

                finishTest(
                    drive,
                    interrupted
                )
        );

        // #endregion
    }

    // #endregion





    // #region TESTE DRIVE SWEEP


    // ============================================================
    // TESTE DRIVE SWEEP
    // ============================================================
    //
    // Objetivo:
    //
    // verificar o comportamento do Drive em diferentes
    // velocidades.
    //
    //
    // VELOCIDADES:
    //
    // 0.25 m/s
    // 0.50 m/s
    // 1.00 m/s
    //
    //
    // Cada velocidade é testada:
    //
    // FRENTE
    //    ↓
    // PARADO
    //    ↓
    // RÉ
    //    ↓
    // PARADO
    //
    //
    // Isso permite analisar:
    //
    // kS
    // kV
    // kP
    //
    // usando:
    //
    // DriveVelocitySetpoint
    // DriveVelocityMeasured
    // DriveVelocityError
    //
    //
    // SEGURANÇA:
    //
    // Este Command deve ser utilizado com:
    //
    // .whileTrue(...)
    //
    //
    // Soltou o POV:
    //
    // CANCELLED
    //     ↓
    // drive.stop()
    //
    // ============================================================

    public static Command driveSweepTest(
        Drive drive
    ) {

        return Commands.sequence(


            // #region INÍCIO

            Commands.runOnce(

                () -> startTest(
                    "DRIVE SWEEP"
                )
            ),

            // #endregion



            // #region PARADO INICIAL

            stop(

                drive,

                STOP_TIME,

                "INITIAL STOP"
            ),

            // #endregion





            // #region TESTE 0.25 m/s


            // ====================================================
            // FRENTE 0.25
            // ====================================================

            driveStraight(

                drive,

                DRIVE_LOW,

                STEP_TIME,

                "FORWARD 0.25"
            ),


            // ====================================================
            // PARADO
            // ====================================================

            stop(

                drive,

                STOP_TIME,

                "STOP AFTER FORWARD 0.25"
            ),


            // ====================================================
            // RÉ 0.25
            // ====================================================

            driveStraight(

                drive,

                -DRIVE_LOW,

                STEP_TIME,

                "REVERSE 0.25"
            ),


            // ====================================================
            // PARADO
            // ====================================================

            stop(

                drive,

                STOP_TIME,

                "STOP AFTER REVERSE 0.25"
            ),

            // #endregion





            // #region TESTE 0.50 m/s


            // ====================================================
            // FRENTE 0.50
            // ====================================================

            driveStraight(

                drive,

                DRIVE_MEDIUM,

                STEP_TIME,

                "FORWARD 0.50"
            ),


            // ====================================================
            // PARADO
            // ====================================================

            stop(

                drive,

                STOP_TIME,

                "STOP AFTER FORWARD 0.50"
            ),


            // ====================================================
            // RÉ 0.50
            // ====================================================

            driveStraight(

                drive,

                -DRIVE_MEDIUM,

                STEP_TIME,

                "REVERSE 0.50"
            ),


            // ====================================================
            // PARADO
            // ====================================================

            stop(

                drive,

                STOP_TIME,

                "STOP AFTER REVERSE 0.50"
            ),

            // #endregion





            // #region TESTE 1.00 m/s


            // ====================================================
            // FRENTE 1.00
            // ====================================================

            driveStraight(

                drive,

                DRIVE_HIGH,

                STEP_TIME,

                "FORWARD 1.00"
            ),


            // ====================================================
            // PARADO
            // ====================================================

            stop(

                drive,

                STOP_TIME,

                "STOP AFTER FORWARD 1.00"
            ),


            // ====================================================
            // RÉ 1.00
            // ====================================================

            driveStraight(

                drive,

                -DRIVE_HIGH,

                STEP_TIME,

                "REVERSE 1.00"
            ),


            // ====================================================
            // PARADA FINAL
            // ====================================================

            stop(

                drive,

                STOP_TIME,

                "FINAL STOP"
            )

            // #endregion

        )


        // #region SEGURANÇA FINAL

        .finallyDo(

            interrupted ->

                finishTest(
                    drive,
                    interrupted
                )
        );

        // #endregion
    }

    // #endregion
}