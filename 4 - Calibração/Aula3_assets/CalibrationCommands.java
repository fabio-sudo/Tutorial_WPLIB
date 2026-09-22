package frc.robot.calibration;


// #region IMPORTS

import static frc.robot.calibration.CalibrationConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.Drive;

// #endregion



public final class CalibrationCommands {


    // #region LIMITES DE SEGURANÇA

    // ============================================================
    // LIMITES ADICIONAIS DE SEGURANÇA
    // ============================================================
    //
    // Mesmo que uma constante seja alterada acidentalmente,
    // estes valores limitam os testes.
    //
    // ============================================================


    // Velocidade máxima permitida pelos nossos testes automáticos.
    private static final double MAX_TEST_SPEED_MPS = 1.00;


    // Maior tensão permitida no teste experimental de kS.
    //
    // IMPORTANTE:
    // Isso NÃO é o limite físico do TalonFX.
    //
    // É apenas um limite propositalmente baixo para este
    // experimento de atrito.
    private static final double MAX_KS_TEST_VOLTS = 0.60;


    // Tempo máximo que um único degrau de tensão pode permanecer ativo.
    private static final double MAX_KS_STEP_SECONDS = 2.0;

    // #endregion





    // #region CONSTRUTOR

    private CalibrationCommands() {}

    // #endregion





    // #region FUNÇÕES AUXILIARES - LOGS


    // ============================================================
    // INICIAR TESTE DE VELOCIDADE
    // ============================================================

    private static void startSpeedTest(
        Drive drive,
        String testName
    ) {

        // Garante que não ficou nenhum modo de tensão ativo.
        drive.stopCalibrationDriveVoltage();

        // Garante chassi parado.
        drive.stop();


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
    // FINALIZAR TESTE DE VELOCIDADE
    // ============================================================

    private static void finishSpeedTest(
        Drive drive,
        boolean interrupted
    ) {

        // --------------------------------------------------------
        // PARA O ROBÔ
        // --------------------------------------------------------

        drive.stop();


        // --------------------------------------------------------
        // GARANTE QUE MODO DE TENSÃO ESTEJA DESLIGADO
        // --------------------------------------------------------

        drive.stopCalibrationDriveVoltage();


        // --------------------------------------------------------
        // LOGS
        // --------------------------------------------------------

        Logger.recordOutput(
            "Calibration/CommandVelocity",
            0.0
        );


        Logger.recordOutput(
            "Calibration/Active",
            false
        );


        Logger.recordOutput(

            "Calibration/Step",

            interrupted
                ? "CANCELLED"
                : "FINISHED"
        );
    }



    // ============================================================
    // INICIAR TESTE DE TENSÃO
    // ============================================================

    private static void startVoltageTest(
        Drive drive,
        String testName
    ) {

        // Para qualquer movimento de velocidade anterior.
        drive.stop();


        // Entra no modo de calibração,
        // mas começa obrigatoriamente com ZERO volts.
        drive.setCalibrationDriveVoltage(
            0.0
        );


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
            "Calibration/KsVoltage",
            0.0
        );
    }



    // ============================================================
    // FINALIZAR TESTE DE TENSÃO
    // ============================================================

    private static void finishVoltageTest(
        Drive drive,
        boolean interrupted
    ) {

        // --------------------------------------------------------
        // PRIMEIRO:
        // remove imediatamente a tensão dos Drives
        // --------------------------------------------------------

        drive.stopCalibrationDriveVoltage();


        // --------------------------------------------------------
        // GARANTE CHASSI PARADO
        // --------------------------------------------------------

        drive.stop();


        // --------------------------------------------------------
        // LOGS
        // --------------------------------------------------------

        Logger.recordOutput(
            "Calibration/KsVoltage",
            0.0
        );


        Logger.recordOutput(
            "Calibration/Active",
            false
        );


        Logger.recordOutput(

            "Calibration/Step",

            interrupted
                ? "CANCELLED"
                : "FINISHED"
        );
    }

    // #endregion





    // #region FUNÇÕES AUXILIARES - DRIVE


    // ============================================================
    // PARAR ROBÔ
    // ============================================================
    //
    // Usado nos testes normais de velocidade.
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
    // ============================================================

    private static Command driveStraight(
        Drive drive,
        double velocity,
        double seconds,
        String stepName
    ) {

        // --------------------------------------------------------
        // LIMITA A VELOCIDADE
        // --------------------------------------------------------

        double safeVelocity =
            Math.max(
                -MAX_TEST_SPEED_MPS,
                Math.min(
                    MAX_TEST_SPEED_MPS,
                    velocity
                )
            );


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
                    safeVelocity
                );


                drive.drive(

                    safeVelocity,  // vx
                    0.0,           // vy
                    0.0            // omega
                );
            },


            // ====================================================
            // QUANDO A ETAPA TERMINAR
            // ====================================================

            drive::stop,


            // ====================================================
            // REQUIREMENT
            // ====================================================

            drive
        )

        .withTimeout(seconds);
    }

    // #endregion





    // #region FUNÇÕES AUXILIARES - TENSÃO


    // ============================================================
    // DEGRAU DE TENSÃO
    // ============================================================
    //
    // Aplica uma tensão muito pequena diretamente nos Drives.
    //
    // O valor também possui um clamp de segurança aqui,
    // além da proteção existente dentro do Drive.
    //
    // ============================================================

    private static Command driveVoltageStep(
        Drive drive,
        double volts,
        double seconds,
        String stepName
    ) {

        // --------------------------------------------------------
        // LIMITA TENSÃO
        // --------------------------------------------------------

        double safeVolts =
            Math.max(
                -MAX_KS_TEST_VOLTS,
                Math.min(
                    MAX_KS_TEST_VOLTS,
                    volts
                )
            );


        // --------------------------------------------------------
        // LIMITA TEMPO DO DEGRAU
        // --------------------------------------------------------

        double safeSeconds =
            Math.max(
                0.05,
                Math.min(
                    MAX_KS_STEP_SECONDS,
                    seconds
                )
            );


        return Commands.run(

            () -> {

                // ------------------------------------------------
                // LOG DA ETAPA
                // ------------------------------------------------

                Logger.recordOutput(
                    "Calibration/Step",
                    stepName
                );


                // ------------------------------------------------
                // LOG DA TENSÃO
                // ------------------------------------------------

                Logger.recordOutput(
                    "Calibration/KsVoltage",
                    safeVolts
                );


                // ------------------------------------------------
                // APLICA TENSÃO
                // ------------------------------------------------

                drive.setCalibrationDriveVoltage(
                    safeVolts
                );
            },

            drive
        )

        .withTimeout(safeSeconds);
    }



    // ============================================================
    // PARADA ENTRE DEGRAUS DE TENSÃO
    // ============================================================
    //
    // Mantém o modo de calibração ativo,
    // mas aplica exatamente ZERO volts.
    //
    // ============================================================

    private static Command voltageStop(
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
                    "Calibration/KsVoltage",
                    0.0
                );


                drive.setCalibrationDriveVoltage(
                    0.0
                );
            },

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
    // Sequência:
    //
    // parado
    //   ↓
    // +0.25 m/s
    //   ↓
    // parado
    //   ↓
    // -0.25 m/s
    //   ↓
    // parado
    //
    // ============================================================

    public static Command driveLowTest(
        Drive drive
    ) {

        return Commands.sequence(


            // #region INÍCIO

            Commands.runOnce(

                () -> startSpeedTest(
                    drive,
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



            // #region FRENTE

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



            // #region RÉ

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


        // ========================================================
        // SEGURANÇA FINAL
        // ========================================================

        .finallyDo(

            interrupted ->

                finishSpeedTest(
                    drive,
                    interrupted
                )
        );
    }

    // #endregion





    // #region TESTE DRIVE SWEEP


    // ============================================================
    // TESTE DRIVE SWEEP
    // ============================================================
    //
    // Testa:
    //
    // +0.25
    // -0.25
    //
    // +0.50
    // -0.50
    //
    // +1.00
    // -1.00
    //
    // Sempre com parada entre os movimentos.
    //
    // ============================================================

    public static Command driveSweepTest(
        Drive drive
    ) {

        return Commands.sequence(


            // #region INÍCIO

            Commands.runOnce(

                () -> startSpeedTest(
                    drive,
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





            // #region 0.25 m/s

            driveStraight(
                drive,
                DRIVE_LOW,
                STEP_TIME,
                "FORWARD 0.25"
            ),

            stop(
                drive,
                STOP_TIME,
                "STOP AFTER FORWARD 0.25"
            ),

            driveStraight(
                drive,
                -DRIVE_LOW,
                STEP_TIME,
                "REVERSE 0.25"
            ),

            stop(
                drive,
                STOP_TIME,
                "STOP AFTER REVERSE 0.25"
            ),

            // #endregion





            // #region 0.50 m/s

            driveStraight(
                drive,
                DRIVE_MEDIUM,
                STEP_TIME,
                "FORWARD 0.50"
            ),

            stop(
                drive,
                STOP_TIME,
                "STOP AFTER FORWARD 0.50"
            ),

            driveStraight(
                drive,
                -DRIVE_MEDIUM,
                STEP_TIME,
                "REVERSE 0.50"
            ),

            stop(
                drive,
                STOP_TIME,
                "STOP AFTER REVERSE 0.50"
            ),

            // #endregion





            // #region 1.00 m/s

            driveStraight(
                drive,
                DRIVE_HIGH,
                STEP_TIME,
                "FORWARD 1.00"
            ),

            stop(
                drive,
                STOP_TIME,
                "STOP AFTER FORWARD 1.00"
            ),

            driveStraight(
                drive,
                -DRIVE_HIGH,
                STEP_TIME,
                "REVERSE 1.00"
            ),

            stop(
                drive,
                STOP_TIME,
                "FINAL STOP"
            )

            // #endregion
        )


        // ========================================================
        // SEGURANÇA FINAL
        // ========================================================

        .finallyDo(

            interrupted ->

                finishSpeedTest(
                    drive,
                    interrupted
                )
        );
    }

    // #endregion





    // #region TESTE KS DO DRIVE


    // ============================================================
    // TESTE EXPERIMENTAL DE kS
    // ============================================================
    //
    // OBJETIVO:
    //
    // Encontrar aproximadamente a tensão necessária
    // para o drivetrain começar a se movimentar.
    //
    //
    // IMPORTANTE:
    //
    // Este teste NÃO define o valor final de kS.
    //
    // Depois vamos comparar com SysId.
    //
    //
    // TENSÕES:
    //
    // 0.10 V
    // 0.20 V
    // 0.30 V
    // 0.40 V
    // 0.50 V
    // 0.60 V
    //
    //
    // SEGURANÇA:
    //
    // - tensão máxima deste teste = 0.60 V
    //
    // - cada etapa possui timeout
    //
    // - botão deve permanecer pressionado
    //
    // - soltou:
    //
    //      Command cancelado
    //             ↓
    //      finallyDo()
    //             ↓
    //      stopCalibrationDriveVoltage()
    //             ↓
    //             0 V
    //
    // ============================================================

    public static Command driveKsTest(
        Drive drive
    ) {

        return Commands.sequence(


            // #region INÍCIO

            Commands.runOnce(

                () -> startVoltageTest(
                    drive,
                    "DRIVE KS"
                )
            ),

            // #endregion



            // #region PARADO INICIAL

            voltageStop(
                drive,
                KS_STOP_TIME,
                "INITIAL STOP"
            ),

            // #endregion





            // #region 0.10 V

            driveVoltageStep(
                drive,
                KS_VOLTAGE_1,
                KS_STEP_TIME,
                "KS 0.10 V"
            ),

            voltageStop(
                drive,
                KS_STOP_TIME,
                "STOP AFTER 0.10 V"
            ),

            // #endregion





            // #region 0.20 V

            driveVoltageStep(
                drive,
                KS_VOLTAGE_2,
                KS_STEP_TIME,
                "KS 0.20 V"
            ),

            voltageStop(
                drive,
                KS_STOP_TIME,
                "STOP AFTER 0.20 V"
            ),

            // #endregion





            // #region 0.30 V

            driveVoltageStep(
                drive,
                KS_VOLTAGE_3,
                KS_STEP_TIME,
                "KS 0.30 V"
            ),

            voltageStop(
                drive,
                KS_STOP_TIME,
                "STOP AFTER 0.30 V"
            ),

            // #endregion





            // #region 0.40 V

            driveVoltageStep(
                drive,
                KS_VOLTAGE_4,
                KS_STEP_TIME,
                "KS 0.40 V"
            ),

            voltageStop(
                drive,
                KS_STOP_TIME,
                "STOP AFTER 0.40 V"
            ),

            // #endregion





            // #region 0.50 V

            driveVoltageStep(
                drive,
                KS_VOLTAGE_5,
                KS_STEP_TIME,
                "KS 0.50 V"
            ),

            voltageStop(
                drive,
                KS_STOP_TIME,
                "STOP AFTER 0.50 V"
            ),

            // #endregion





            // #region 0.60 V

            driveVoltageStep(
                drive,
                KS_VOLTAGE_6,
                KS_STEP_TIME,
                "KS 0.60 V"
            ),

            voltageStop(
                drive,
                KS_STOP_TIME,
                "FINAL STOP"
            )

            // #endregion
        )


        // ========================================================
        // SEGURANÇA FINAL
        // ========================================================

        .finallyDo(

            interrupted ->

                finishVoltageTest(
                    drive,
                    interrupted
                )
        );
    }

    // #endregion
}