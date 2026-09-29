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

// NEW ==========================================================
// NEW - SUAVIZAÇÃO DA ACELERAÇÃO DO DRIVE
// NEW ==========================================================
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.DriveConstants.kMaxAngularSpeed;
import static frc.robot.Constants.DriveConstants.kMaxLinearSpeed;

// #endregion



public class RobotContainer {

    // #region SUBSYSTEMS

    private final ExampleSubsystem m_exampleSubsystem =
        new ExampleSubsystem();


    private final Drive drive =
        new Drive();

    // #endregion



    // #region CONTROLE DO MOTORISTA

    private final CommandXboxController m_driverController =
        new CommandXboxController(
            OperatorConstants.kDriverControllerPort
        );

    // #endregion



    // NEW ============================================================
    // NEW - SUAVIZAÇÃO DO DRIVE
    // NEW ============================================================
    //
    // Valores já testados no robô:
    //
    // vx    = 3.0
    // vy    = 3.0
    // omega = 4.0
    //

    // NEW - Frente / trás
    private final SlewRateLimiter vxLimiter =
        new SlewRateLimiter(3.0);


    // NEW - Movimento lateral
    private final SlewRateLimiter vyLimiter =
        new SlewRateLimiter(3.0);


    // NEW - Rotação
    private final SlewRateLimiter omegaLimiter =
        new SlewRateLimiter(4.0);



    // #region MODO DE DIREÇÃO

    // true  = Field Relative
    // false = Robot Relative
    private boolean fieldRelativeEnabled = true;

    // #endregion



    // #region CONSTRUTOR

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

                    double vxInput =
                        -MathUtil.applyDeadband(

                            m_driverController.getLeftY(),

                            0.10

                        );


                    // NEW =================================================
                    // NEW - CURVA QUADRÁTICA DO JOYSTICK
                    // NEW =================================================

                    vxInput =
                        Math.copySign(

                            vxInput * vxInput,

                            vxInput
                        );


                    // NEW =================================================
                    // NEW - LIMITADOR DE ACELERAÇÃO
                    // NEW =================================================

                    double vx =
                        vxLimiter.calculate(
                            vxInput
                        )
                        * kMaxLinearSpeed
                        * 0.50;



                    // ====================================================
                    // MOVIMENTO LATERAL
                    // ====================================================

                    double vyInput =
                        -MathUtil.applyDeadband(

                            m_driverController.getLeftX(),

                            0.10

                        );


                    // NEW =================================================
                    // NEW - CURVA QUADRÁTICA DO JOYSTICK
                    // NEW =================================================

                    vyInput =
                        Math.copySign(

                            vyInput * vyInput,

                            vyInput
                        );


                    // NEW =================================================
                    // NEW - LIMITADOR DE ACELERAÇÃO
                    // NEW =================================================

                    double vy =
                        vyLimiter.calculate(
                            vyInput
                        )
                        * kMaxLinearSpeed
                        * 0.50;



                    // ====================================================
                    // ROTAÇÃO DO ROBÔ
                    // ====================================================

                    double omegaInput =
                        -MathUtil.applyDeadband(

                            m_driverController.getRightX(),

                            0.10

                        );


                    // NEW =================================================
                    // NEW - CURVA QUADRÁTICA DO JOYSTICK
                    // NEW =================================================

                    omegaInput =
                        Math.copySign(

                            omegaInput * omegaInput,

                            omegaInput
                        );


                    // NEW =================================================
                    // NEW - LIMITADOR DE ACELERAÇÃO DA ROTAÇÃO
                    // NEW =================================================

                    double omega =
                        omegaLimiter.calculate(
                            omegaInput
                        )
                        * kMaxAngularSpeed
                        * 0.50;



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


        configureBindings();
    }

    // #endregion



    // #region BINDINGS DOS CONTROLES

    private void configureBindings() {


        // ============================================================
        // BOTÃO Y - ZERO HEADING
        // ============================================================

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
    }

    // #endregion



    // #region AUTÔNOMO

    public Command getAutonomousCommand() {

        return Autos.exampleAuto(
            m_exampleSubsystem
        );
    }

    // #endregion
}