// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// #region IMPORTS

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.Drive;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.DriveConstants.kMaxAngularSpeed;
import static frc.robot.Constants.DriveConstants.kMaxLinearSpeed;

import frc.robot.calibration.DriveSysId;


//CALIBRAÇÃO
import frc.robot.calibration.CalibrationCommands;

// #endregion



public class RobotContainer {

    // #region SUBSYSTEMS

    // Subsistema criado pelo template do WPILib
    private final ExampleSubsystem m_exampleSubsystem =
        new ExampleSubsystem();


    // Drivetrain Swerve
    private final Drive drive =
        new Drive();

    // #endregion

    // ============================================================
    // SYSID DO DRIVE
    // ============================================================

    private final DriveSysId driveSysId =
        new DriveSysId(
            drive
        );


    // #region CONTROLE DO MOTORISTA

    // Controle Xbox conectado na porta definida
    // em OperatorConstants
    private final CommandXboxController m_driverController =
        new CommandXboxController(
            OperatorConstants.kDriverControllerPort
        );

    // #endregion


    // #region MODO DE DIREÇÃO

        // true  = Field Relative
        // false = Robot Relative
        private boolean fieldRelativeEnabled = true;

    // #endregion
    

    // #region CONSTRUTOR

    /**
     * O RobotContainer contém:
     *
     * - Subsistemas
     * - Controle
     * - Comandos
     * - Bindings
     */
    public RobotContainer() {


        // ============================================================
        // COMANDO PADRÃO DO SWERVE
        // ============================================================

        drive.setDefaultCommand(

            drive.runEnd(

                () -> {


                    // ====================================================
                    // MOVIMENTO PARA FRENTE / TRÁS
                    // ====================================================

                    double vx =
                        -MathUtil.applyDeadband(

                            m_driverController.getLeftY(),

                            0.10

                        ) * kMaxLinearSpeed*0.50;



                    // ====================================================
                    // MOVIMENTO LATERAL
                    // ====================================================

                    double vy =
                        -MathUtil.applyDeadband(

                            m_driverController.getLeftX(),

                            0.10

                        ) * kMaxLinearSpeed*0.50;



                    // ====================================================
                    // ROTAÇÃO DO ROBÔ
                    // ====================================================

                    double omega =
                        -MathUtil.applyDeadband(

                            m_driverController.getRightX(),

                            0.10

                        ) * kMaxAngularSpeed*0.50;


                    // ====================================================
                    // ESCOLHE O MODO DE DIREÇÃO
                    // ====================================================

                    if (fieldRelativeEnabled) {

                        // -----------------------------------------------
                        // FIELD RELATIVE
                        // -----------------------------------------------

                        drive.driveFieldRelative(
                            vx,
                            vy,
                            omega
                        );

                    } else {

                        // -----------------------------------------------
                        // ROBOT RELATIVE
                        // -----------------------------------------------

                        drive.drive(
                            vx,
                            vy,
                            omega
                        );
                    }

                },
                // Quando o comando terminar
                // ou for interrompido
                drive::stop
            )
        );


        // Configura os botões do controle
        configureBindings();
    }

    // #endregion


    // #region BINDINGS DOS CONTROLES

    /**
     * Configuração dos botões
     * e triggers do controle Xbox.
     */
    private void configureBindings() {


        // ============================================================
        // BOTÃO Y - ZERO HEADING
        // ============================================================

        // Define a orientação atual do robô
        // como a nova referência de 0 graus
        //
        // Muito útil para Field Relative
        m_driverController
            .y()
            .onTrue(

                drive.runOnce(
                    drive::zeroHeading
                )
            );

            // ============================================================
            // BOTÃO X - ALTERNA FIELD / ROBOT RELATIVE
            // ============================================================

            m_driverController
                .x()
                .onTrue(

                    drive.runOnce(
                        () -> {

                            fieldRelativeEnabled =
                                !fieldRelativeEnabled;

                        }
                    )
                );


        // ============================================================
        // TRIGGER DE EXEMPLO DO WPILIB
        // ============================================================

        new Trigger(
            m_exampleSubsystem::exampleCondition
        )
        .onTrue(

            new ExampleCommand(
                m_exampleSubsystem
            )
        );



        // ============================================================
        // BOTÃO B - EXEMPLO DO TEMPLATE
        // ============================================================

        m_driverController
            .b()
            .whileTrue(

                m_exampleSubsystem.exampleMethodCommand()
            );


        // ============================================================
        // BOTÃO A - Segurar Calibração
        // ============================================================
        m_driverController
            .a()
            .whileTrue(
                CalibrationCommands.driveLowTest(drive)
            );
        // ============================================================
        // POV ↑ - TESTE DRIVE SWEEP
        // ============================================================

        m_driverController
            .povUp()
            .whileTrue(
                CalibrationCommands.driveSweepTest(drive)
            );
        // ============================================================
        // POV → - TESTE KS
        // ============================================================

        m_driverController
            .povRight()
            .whileTrue(
                CalibrationCommands.driveKsTest(drive)
            );

        // ============================================================
        // POV ⬅️ - TESTE KS
        // ============================================================

            m_driverController
            .povLeft()
            .whileTrue(
                CalibrationCommands.driveKsReverseTest(
                    drive
                )
            );

        // ============================================================
        // SYSID - QUASISTATIC FORWARD
        // ============================================================
        //
        // BACK + LB
        //
        // A tensão sobe lentamente no sentido positivo.
        //
        // Soltou qualquer um dos botões:
        // o Command é cancelado
        // e o DriveSysId zera a tensão.
        //
        // ============================================================

        m_driverController
            .back()
            .and(
                m_driverController.leftBumper()
            )
            .whileTrue(
                driveSysId.quasistaticForward()
            );



        // ============================================================
        // SYSID - QUASISTATIC REVERSE
        // ============================================================
        //
        // BACK + RB
        //
        // A tensão sobe lentamente no sentido negativo.
        //
        // ============================================================

        m_driverController
            .back()
            .and(
                m_driverController.rightBumper()
            )
            .whileTrue(
                driveSysId.quasistaticReverse()
            );



        // ============================================================
        // SYSID - DYNAMIC FORWARD
        // ============================================================
        //
        // BACK + LT
        //
        // Aplica o degrau positivo configurado:
        //
        // +2.0 V
        //
        // ============================================================

        m_driverController
            .back()
            .and(
                m_driverController.leftTrigger()
            )
            .whileTrue(
                driveSysId.dynamicForward()
            );



        // ============================================================
        // SYSID - DYNAMIC REVERSE
        // ============================================================
        //
        // BACK + RT
        //
        // Aplica:
        //
        // -2.0 V
        //
        // ============================================================

        m_driverController
            .back()
            .and(
                m_driverController.rightTrigger()
            )
            .whileTrue(
                driveSysId.dynamicReverse()
            );


     }
    // #endregion


    // #region AUTÔNOMO

    /**
     * Retorna o comando utilizado
     * durante o período autônomo.
     */
    public Command getAutonomousCommand() {

        return Autos.exampleAuto(
            m_exampleSubsystem
        );
    }

    // #endregion
}