package frc.robot.subsystems.drive;


// #region IMPORTS

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// #endregion



public class ModuleIOSim implements ModuleIO {


    // #region MOTOR DRIVE SIMULADO

    // Motor que faz a roda andar
    private final DCMotor driveMotor =
        DCMotor.getKrakenX60Foc(1);


    // Cria um motor virtual para o Drive
    private final DCMotorSim driveSim =
        new DCMotorSim(

            LinearSystemId.createDCMotorSystem(

                driveMotor,

                0.01,   // momento de inércia

                6.746   // redução do Drive
            ),

            driveMotor
        );

    // #endregion



    // #region MOTOR STEER SIMULADO

    // Motor que gira/orienta o módulo
    private final DCMotor turnMotor =
        DCMotor.getKrakenX60Foc(1);


    // Cria um motor virtual para o Steer
    private final DCMotorSim turnSim =
        new DCMotorSim(

            LinearSystemId.createDCMotorSystem(

                turnMotor,

                0.01,    // momento de inércia

                21.428   // redução do Steer
            ),

            turnMotor
        );

    // #endregion



    // #region CONTROLE FECHADO - DRIVE

    // PID responsável por corrigir
    // a velocidade da roda
    private final PIDController drivePID =
        new PIDController(

            kDriveKp,

            kDriveKi,

            kDriveKd
        );


    // Feedforward responsável por fornecer
    // uma tensão inicial aproximada
    private final SimpleMotorFeedforward driveFeedforward =
        new SimpleMotorFeedforward(

            kDriveKs,

            kDriveKv
        );

    // #endregion



    // #region CONTROLE FECHADO - STEER

    // PID responsável por controlar
    // o ângulo do módulo
    private final PIDController turnPID =
        new PIDController(

            4.0,

            0.0,

            0.0
        );

    // #endregion



    // #region TENSÕES APLICADAS

    private double driveAppliedVolts =
        0.0;


    private double turnAppliedVolts =
        0.0;

    // #endregion



    // #region CONSTRUTOR

    public ModuleIOSim() {


        // ============================================================
        // DRIVE PID
        // ============================================================

        // Considera a velocidade correta
        // quando estiver dentro de 0.05 m/s
        drivePID.setTolerance(
            0.05
        );



        // ============================================================
        // STEER PID
        // ============================================================

        // Permite trabalhar corretamente
        // com ângulos circulares
        //
        // Exemplo:
        //
        // +179° e -179°
        //
        // estão muito próximos
        turnPID.enableContinuousInput(

            -Math.PI,

            Math.PI
        );


        // Considera o módulo alinhado
        // quando o erro for menor que 1 grau
        turnPID.setTolerance(

            Math.toRadians(1.0)
        );
    }

    // #endregion



    // #region ATUALIZAÇÃO DOS SENSORES SIMULADOS

    @Override
    public void updateInputs(
        ModuleIOInputs inputs
    ) {


        // ============================================================
        // ATUALIZAÇÃO DA FÍSICA
        // ============================================================

        // Atualiza os motores virtuais
        // a cada 20 ms
        driveSim.update(
            0.02
        );


        turnSim.update(
            0.02
        );



        // ============================================================
        // SENSORES DO DRIVE
        // ============================================================

        inputs.drivePositionRad =
            driveSim.getAngularPositionRad();


        inputs.driveVelocityRadPerSec =
            driveSim.getAngularVelocityRadPerSec();


        inputs.driveAppliedVolts =
            driveAppliedVolts;


        inputs.driveCurrentAmps =
            driveSim.getCurrentDrawAmps();



        // ============================================================
        // SENSORES DO STEER
        // ============================================================

        inputs.turnPositionRad =
            turnSim.getAngularPositionRad();


        inputs.turnVelocityRadPerSec =
            turnSim.getAngularVelocityRadPerSec();


        inputs.turnAppliedVolts =
            turnAppliedVolts;


        inputs.turnCurrentAmps =
            turnSim.getCurrentDrawAmps();
    }

    // #endregion



    // #region CONTROLE ABERTO - DRIVE

    @Override
    public void setDriveVoltage(
        double volts
    ) {


        // Limita a tensão entre -12V e +12V
        driveAppliedVolts =
            MathUtil.clamp(

                volts,

                -12.0,

                12.0
            );


        // Aplica a tensão no motor virtual
        driveSim.setInputVoltage(
            driveAppliedVolts
        );
    }

    // #endregion



    // #region CONTROLE ABERTO - STEER

    @Override
    public void setTurnVoltage(
        double volts
    ) {


        // Limita a tensão entre -12V e +12V
        turnAppliedVolts =
            MathUtil.clamp(

                volts,

                -12.0,

                12.0
            );


        // Aplica a tensão no motor virtual
        turnSim.setInputVoltage(
            turnAppliedVolts
        );
    }

    // #endregion



    // #region MALHA FECHADA - DRIVE

    @Override
    public void setDriveVelocity(
        double velocityMetersPerSecond
    ) {


        // ============================================================
        // 1 - VELOCIDADE ATUAL DA RODA
        // ============================================================

        // Converte:
        //
        // rad/s
        //
        // para:
        //
        // metros por segundo
        double measuredVelocityMetersPerSecond =

            driveSim.getAngularVelocityRadPerSec()

            *

            kWheelRadiusMeters;



        // ============================================================
        // 2 - PID
        // ============================================================

        // Corrige a diferença entre:
        //
        // velocidade desejada
        //
        // e
        //
        // velocidade medida
        double pidVolts =
            drivePID.calculate(

                measuredVelocityMetersPerSecond,

                velocityMetersPerSecond
            );



        // ============================================================
        // 3 - FEEDFORWARD
        // ============================================================

        // Calcula aproximadamente
        // quanta tensão o motor precisa
        // para atingir a velocidade desejada
        double feedforwardVolts =
            driveFeedforward.calculate(

                velocityMetersPerSecond
            );



        // ============================================================
        // 4 - TENSÃO FINAL
        // ============================================================

        double totalVolts =

            pidVolts

            +

            feedforwardVolts;



        // ============================================================
        // 5 - ENVIA PARA O MOTOR
        // ============================================================

        setDriveVoltage(
            totalVolts
        );
    }

    // #endregion



    // #region MALHA FECHADA - STEER

    @Override
    public void setTurnPosition(
        double angleRadians
    ) {


        // ============================================================
        // 1 - ÂNGULO ATUAL
        // ============================================================

        double currentAngle =
            MathUtil.angleModulus(

                turnSim.getAngularPositionRad()
            );



        // ============================================================
        // 2 - ÂNGULO DESEJADO
        // ============================================================

        double desiredAngle =
            MathUtil.angleModulus(

                angleRadians
            );



        // ============================================================
        // 3 - PID DO STEER
        // ============================================================

        double turnVolts =
            turnPID.calculate(

                currentAngle,

                desiredAngle
            );



        // ============================================================
        // 4 - ENVIA PARA O MOTOR
        // ============================================================

        setTurnVoltage(
            turnVolts
        );
    }

    // #endregion
}