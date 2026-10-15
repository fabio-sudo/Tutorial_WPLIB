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
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.DriveConstants.kMaxAngularSpeed;
import static frc.robot.Constants.DriveConstants.kMaxLinearSpeed;


// Testes automáticos de validação
import frc.robot.calibration.DriveTwoMetersCommand;
import frc.robot.calibration.Rotate360Command;

import frc.robot.calibration.DriveSysId;

// 🆕 Comandos de calibração do Swerve
import frc.robot.calibration.SwerveCalibrationCommands;

//CALIBRAÇÃO
import frc.robot.calibration.CalibrationCommands;

//Curva de Aceleração e Suavisação
import edu.wpi.first.math.filter.SlewRateLimiter;

//Limelight Vision
import frc.robot.subsystems.vision.Vision;


//Virtualização limelight
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.subsystems.vision.VisionSimulationScenario;

// #endregion



public class RobotContainer {

    // #region SUBSYSTEMS

    // Subsistema criado pelo template do WPILib
    private final ExampleSubsystem m_exampleSubsystem =
        new ExampleSubsystem();


    // Drivetrain Swerve
    private final Drive drive =
        new Drive();

    
        // ============================================================
        // CENÁRIO DE VISÃO PARA SIMULAÇÃO
        // ============================================================

        // Cria o cenário somente na simulação normal.
        // No replay, as entradas virão do arquivo gravado.
        private final VisionSimulationScenario visionSimulation =
            RobotBase.isSimulation() && !Logger.hasReplaySource()
                ? new VisionSimulationScenario()
                : null;

        // Na simulação, usamos explicitamente as duas câmeras virtuais.
        // Nas demais execuções, usamos a configuração existente do Vision.
        private final Vision vision =
            visionSimulation != null
                ? new Vision(drive, visionSimulation.getCameras())
                : new Vision(drive);

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
    private final CommandXboxController m_driverController =
        new CommandXboxController(
            OperatorConstants.kDriverControllerPort
        );


    // ============================================================
    // SUAVIZAÇÃO DO SWERVE
    // ============================================================

    // Frente / trás
    private final SlewRateLimiter vxLimiter =
        new SlewRateLimiter(2.0);

    // Movimento lateral
    private final SlewRateLimiter vyLimiter =
        new SlewRateLimiter(2.0);

    // Rotação
    private final SlewRateLimiter omegaLimiter =
        new SlewRateLimiter(2.0);


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
        if (!DriverStation.isTeleopEnabled()) {
            drive.stop();
            resetDriverLimiters();
            return;
        }
        // ====================================================
        // MOVIMENTO PARA FRENTE / TRÁS
        // ====================================================

        double vxInput =
            -MathUtil.applyDeadband(

                m_driverController.getLeftY(),

                0.10

            );

        // Curva quadrática para deixar o centro mais suave
        vxInput =
            Math.copySign(
                vxInput * vxInput,
                vxInput
            );

        double vx =
            vxLimiter.calculate(vxInput)
                * kMaxLinearSpeed;



        // ====================================================
        // MOVIMENTO LATERAL
        // ====================================================

        double vyInput =
            -MathUtil.applyDeadband(

                m_driverController.getLeftX(),

                0.10

            );

        // Curva quadrática
        vyInput =
            Math.copySign(
                vyInput * vyInput,
                vyInput
            );

        double vy =
            vyLimiter.calculate(vyInput)
                * kMaxLinearSpeed;



        // ====================================================
        // ROTAÇÃO DO ROBÔ
        // ====================================================

        double omegaInput =
            -MathUtil.applyDeadband(

                m_driverController.getRightX(),

                0.10

            );

        // Curva quadrática
        omegaInput =
            Math.copySign(
                omegaInput * omegaInput,
                omegaInput
            );

        double omega =
            omegaLimiter.calculate(omegaInput)
                * kMaxAngularSpeed;



        // ====================================================
        // ESCOLHE O MODO DE DIREÇÃO
        // ====================================================

        if (fieldRelativeEnabled) {

            // -----------------------------------------------
            // FIELD RELATIVE
            // -----------------------------------------------

            drive.driveDriverRelative(
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

        // Seta para baixo sem Start: manter o freio em X no teleoperado.
        // Start + seta para baixo continua reservado ao seed da visão.
        m_driverController.povDown()
            .and(m_driverController.start().negate())
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(
                drive.xLockCommand()
                    .finallyDo(interrupted -> resetDriverLimiters())
            );


        // ============================================================
        // BOTÃO Y - ZERO DA REFERÊNCIA DO PILOTO
        // ============================================================

        // Preserva a pose de campo enviada à Limelight. Exige robô parado.
        m_driverController
            .y()
            .and(m_driverController.start().negate())
            .and(() -> DriverStation.isDisabled() || DriverStation.isTeleopEnabled())
            .onTrue(
                drive.runOnce(
                    () -> {
                        drive.zeroDriverHeading();
                        resetDriverLimiters();
                    }
                ).ignoringDisable(true)
            );

        // Start + Y: definir 0 graus de CAMPO, desabilitado e parado.
        // Antes de usar, apontar a frente do robô para +X do campo azul.
        m_driverController.y()
            .and(m_driverController.start())
            .and(DriverStation::isDisabled)
            .onTrue(drive.runOnce(drive::zeroHeading).ignoringDisable(true));

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


        // #region CALIBRAÇÃO DO SWERVE - ÂNGULOS


        // ============================================================
        // START + L1
        // STEER = 0°
        // ============================================================

        m_driverController
            .start()
            .and(
                m_driverController.leftBumper()
            )
            .whileTrue(

                SwerveCalibrationCommands.steer0(
                    drive
                )
            );



        // ============================================================
        // START + R1
        // STEER = 45°
        // ============================================================

        m_driverController
            .start()
            .and(
                m_driverController.rightBumper()
            )
            .whileTrue(

                SwerveCalibrationCommands.steer45(
                    drive
                )
            );



        // ============================================================
        // START + L2
        // STEER = 90°
        // ============================================================

        m_driverController
            .start()
            .and(
                m_driverController.leftTrigger()
            )
            .whileTrue(

                SwerveCalibrationCommands.steer90(
                    drive
                )
            );



        // ============================================================
        // START + R2
        // STEER = 180°
        // ============================================================

        m_driverController
            .start()
            .and(
                m_driverController.rightTrigger()
            )
            .whileTrue(

                SwerveCalibrationCommands.steer180(
                    drive
                )
                
            );




                // ============================================================
                // CALIBRAÇÃO DO SWERVE - ROTAÇÃO PURA
                // ============================================================

                // ============================================================
                // START + LEFT STICK
                // ROTAÇÃO POSITIVA
                //
                // vx    = 0
                // vy    = 0
                // omega = +0.50 rad/s
                //
                // O robô deve girar no próprio centro.
                // ============================================================
                m_driverController
                    .start()
                    .and(
                        m_driverController.leftStick()
                    )
                    .whileTrue(
                        SwerveCalibrationCommands.rotatePositive(
                            drive
                        )
                    );


                // ============================================================
                // START + RIGHT STICK
                // ROTAÇÃO NEGATIVA
                //
                // vx    = 0
                // vy    = 0
                // omega = -0.50 rad/s
                //
                // O robô deve girar no próprio centro
                // no sentido contrário.
                // ============================================================
                m_driverController
                    .start()
                    .and(
                        m_driverController.rightStick()
                    )
                    .whileTrue(
                        SwerveCalibrationCommands.rotateNegative(
                            drive
                        )
                    );
                     
                    // ============================================================
                    // TESTE AUTOMÁTICO - ANDAR 2 METROS
                    // ============================================================
                    //
                    // BACK + A
                    //
                    // SEGURAR OS DOIS BOTÕES.
                    //
                    // Enquanto estiver segurando:
                    //
                    //     robô anda para frente
                    //          ↓
                    //     odometria mede a distância
                    //          ↓
                    //     chegou em 2,00 metros
                    //          ↓
                    //     para automaticamente
                    //
                    // SEGURANÇA:
                    //
                    // Soltou BACK ou A
                    //        ↓
                    // comando é cancelado
                    //        ↓
                    // DriveTwoMetersCommand.end()
                    //        ↓
                    // drive.stop()
                    //
                    // Timeout máximo = 6 segundos
                    //
                    // ============================================================

                    m_driverController
                        .back()
                        .and(
                            m_driverController.a()
                        )
                        .whileTrue(

                            new DriveTwoMetersCommand(
                                drive
                            )
                            .withTimeout(
                                6.0
                            )
                        );



                    // ============================================================
                    // BACK + B
                    // TESTE AUTOMÁTICO - ROTAÇÃO 360°
                    //
                    // PRECISA SEGURAR.
                    //
                    // Soltou BACK ou B:
                    // cancela imediatamente.
                    //
                    // Segurança adicional:
                    // timeout de 25 segundos.
                    // ============================================================

                    m_driverController
                        .back()
                        .and(
                            m_driverController.b()
                        )
                        .whileTrue(

                            new Rotate360Command(
                                drive
                            )
                            .withTimeout(
                                150.0//Quantidade tempo que metodo roda deposi destroy
                            )
                        );



                    // ============================================================
                    // LIMELIGHT - SINCRONIZAÇÃO MANUAL DA POSE
                    // ============================================================
                    //
                    // START + SETA PARA BAIXO (D-PAD ↓)
                    //
                    // Executa uma única tentativa por acionamento.
                    //
                    // ============================================================

                    m_driverController
                        .start()
                        .and(
                            m_driverController.povDown()
                        )
                        .onTrue(

                            drive.runOnce(
                                () -> vision.trySeedDrivePose()
                            )

                        );

                        }
    
    //#endregion
    
    // #endregion


    // #region AUTÔNOMO

                         private void resetDriverLimiters() {
                             vxLimiter.reset(0.0);
                             vyLimiter.reset(0.0);
                             omegaLimiter.reset(0.0);
                         }

                         /** Retorna o comando utilizado durante o período autônomo. */
                         public Command getAutonomousCommand() {

                            return Autos.exampleAuto(
                                m_exampleSubsystem
                            );

                        }

    // #endregion


// ============================================================
// ATUALIZAÇÃO DA VISÃO VIRTUAL
// ============================================================

public void updateVisionSimulation() {
    if (visionSimulation != null) {
        visionSimulation.update(drive.getPose());
    }
}


}
