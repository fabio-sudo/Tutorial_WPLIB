package frc.robot.calibration;


// #region IMPORTS

// ============================================================
// UNIDADES WPILIB
// ============================================================

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


// ============================================================
// COMMANDS
// ============================================================

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// ============================================================
// SYSID
// ============================================================

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


// ============================================================
// ADVANTAGEKIT
// ============================================================

import org.littletonrobotics.junction.Logger;


// ============================================================
// DRIVE
// ============================================================

import frc.robot.subsystems.drive.Drive;


//LOG
import com.ctre.phoenix6.SignalLogger;

// #endregion



public final class DriveSysId {


    // #region ATRIBUTOS

    // Drivetrain que será caracterizado
    private final Drive drive;


    // Rotina oficial de SysId da WPILib
    private final SysIdRoutine sysIdRoutine;

    // #endregion



    // #region CONSTRUTOR

    public DriveSysId(
        Drive drive
    ) {

        this.drive = drive;


        // ============================================================
        // CRIA A ROTINA DO SYSID
        // ============================================================

        sysIdRoutine =
            new SysIdRoutine(


                // ====================================================
                // CONFIGURAÇÃO
                // ====================================================

                new SysIdRoutine.Config(


                    // ------------------------------------------------
                    // QUASISTATIC
                    // ------------------------------------------------
                    //
                    // A tensão aumenta lentamente:
                    //
                    // 0.0 V
                    // 0.5 V
                    // 1.0 V
                    // 1.5 V
                    // 2.0 V
                    //
                    // Taxa:
                    //
                    // 0.5 V por segundo
                    //
                    // ------------------------------------------------

                    Volts.of(0.5)
                        .per(Second),



                    // ------------------------------------------------
                    // DYNAMIC
                    // ------------------------------------------------
                    //
                    // O teste dinâmico aplica um degrau.
                    //
                    // Começaremos conservadoramente em:
                    //
                    // 2.0 V
                    //
                    // ------------------------------------------------

                    Volts.of(2.0),



                    // ------------------------------------------------
                    // TIMEOUT DE SEGURANÇA
                    // ------------------------------------------------
                    //
                    // Depois de 4 segundos o comando termina
                    // automaticamente.
                    //
                    // ------------------------------------------------

                    Seconds.of(4.0),



                    // ------------------------------------------------
                    // ESTADO DO SYSID
                    // ------------------------------------------------
                    //
                    // AdvantageKit recomenda registrar o estado
                    // dessa forma.
                    //
                    // Exemplos:
                    //
                    // quasistatic-forward
                    // quasistatic-reverse
                    // dynamic-forward
                    // dynamic-reverse
                    //
                    // ------------------------------------------------

                    state -> {

                            // AdvantageKit
                            Logger.recordOutput(
                                "SysIdTestState",
                                state.toString()
                            );


                            // CTRE .hoot
                            SignalLogger.writeString(
                                "state",
                                state.toString()
                            );
                        }
                    ),



                // ====================================================
                // MECANISMO
                // ====================================================

                new SysIdRoutine.Mechanism(


                    // ------------------------------------------------
                    // ENVIA A TENSÃO PARA O DRIVE
                    // ------------------------------------------------

                    voltage -> {

                        double volts =
                            voltage.in(
                                Volts
                            );


                        // Utilizamos exatamente o modo de
                        // calibração que acabamos de validar.
                        //
                        // Ele:
                        //
                        // 1. alinha os Steers
                        // 2. verifica o alinhamento
                        // 3. somente depois libera tensão
                        drive.setCalibrationDriveVoltage(
                            volts
                        );


                        // Log extra para AdvantageScope
                        Logger.recordOutput(
                            "SysId/RequestedVolts",
                            volts
                        );
                    },



                    // ------------------------------------------------
                    // LOG CALLBACK
                    // ------------------------------------------------
                    //
                    // NULL propositalmente.
                    //
                    // Estamos usando AdvantageKit para registrar:
                    //
                    // drivePositionRad
                    // driveVelocityRadPerSec
                    // driveAppliedVolts
                    //
                    // ------------------------------------------------

                    null,



                    // ------------------------------------------------
                    // SUBSYSTEM
                    // ------------------------------------------------

                    drive,



                    // ------------------------------------------------
                    // NOME DO MECANISMO
                    // ------------------------------------------------

                    "Drive"
                )
            );
    }

    // #endregion



    // #region PREPARAÇÃO

    // ============================================================
    // PREPARA O SWERVE ANTES DO SYSID
    // ============================================================
    //
    // Antes da rampa começar:
    //
    // tensão = 0 V
    //
    // durante 1.5 segundos.
    //
    // Nesse período o Drive continua:
    //
    // alinhando FL
    // alinhando FR
    // alinhando BL
    // alinhando BR
    //
    // Isso evita começar o SysId com módulos atravessados.
    //
    // ============================================================

    private Command prepare(
        String testName
    ) {

        return Commands.sequence(


            // ========================================================
            // INFORMA QUAL TESTE VAI COMEÇAR
            // ========================================================

            Commands.runOnce(

                () -> {

                    Logger.recordOutput(
                        "SysId/Test",
                        testName
                    );


                    Logger.recordOutput(
                        "SysId/Active",
                        true
                    );


                    Logger.recordOutput(
                        "SysId/RequestedVolts",
                        0.0
                    );
                }
            ),



            // ========================================================
            // 1.5 SEGUNDOS EM 0 V
            // ========================================================

            Commands.run(

                () -> {

                    drive.setCalibrationDriveVoltage(
                        0.0
                    );


                    Logger.recordOutput(
                        "SysId/RequestedVolts",
                        0.0
                    );
                },

                drive

            )

            .withTimeout(
                1.5
            )
        );
    }

    // #endregion



    // #region SEGURANÇA

    // ============================================================
    // ENVOLVE CADA TESTE COM AS PROTEÇÕES
    // ============================================================

    private Command safeTest(
        Command test,
        String testName
    ) {

        return Commands.sequence(

            // Primeiro alinha o Swerve
            prepare(
                testName
            ),

            // Depois executa o SysId
            test

        )

        .finallyDo(

            interrupted -> {

                // ====================================================
                // SEMPRE ZERA O DRIVE
                // ====================================================

                drive.stopCalibrationDriveVoltage();


                // ====================================================
                // LOGS
                // ====================================================

                Logger.recordOutput(
                    "SysId/RequestedVolts",
                    0.0
                );


                Logger.recordOutput(
                    "SysId/Active",
                    false
                );


                Logger.recordOutput(

                    "SysId/Result",

                    interrupted
                        ? "CANCELLED"
                        : "FINISHED"
                );
            }
        );
    }

    // #endregion



    // #region QUASISTATIC FORWARD

    public Command quasistaticForward() {

        return safeTest(

            sysIdRoutine.quasistatic(
                SysIdRoutine.Direction.kForward
            ),

            "QUASISTATIC FORWARD"
        );
    }

    // #endregion



    // #region QUASISTATIC REVERSE

    public Command quasistaticReverse() {

        return safeTest(

            sysIdRoutine.quasistatic(
                SysIdRoutine.Direction.kReverse
            ),

            "QUASISTATIC REVERSE"
        );
    }

    // #endregion



    // #region DYNAMIC FORWARD

    public Command dynamicForward() {

        return safeTest(

            sysIdRoutine.dynamic(
                SysIdRoutine.Direction.kForward
            ),

            "DYNAMIC FORWARD"
        );
    }

    // #endregion



    // #region DYNAMIC REVERSE

    public Command dynamicReverse() {

        return safeTest(

            sysIdRoutine.dynamic(
                SysIdRoutine.Direction.kReverse
            ),

            "DYNAMIC REVERSE"
        );
    }

    // #endregion
}