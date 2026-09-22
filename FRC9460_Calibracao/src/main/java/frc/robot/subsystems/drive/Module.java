package frc.robot.subsystems.drive;


// #region IMPORTS

import static frc.robot.Constants.DriveConstants.kWheelRadiusMeters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// #endregion



public class Module {


    // #region IO DO MÓDULO

    // Pode ser:
    //
    // ModuleIOSim
    // ou futuramente
    // ModuleIOTalonFX
    private final ModuleIO io;

    // #endregion



    // #region DADOS DO MÓDULO

    // Classe gerada automaticamente pelo @AutoLog
    private final ModuleIOInputsAutoLogged inputs =
        new ModuleIOInputsAutoLogged();


    // Número do módulo
    //
    // 0 = Front Left
    // 1 = Front Right
    // 2 = Back Left
    // 3 = Back Right
    private final int index;

    // #endregion



    // #region CONSTRUTOR

    public Module(
        ModuleIO io,
        int index
    ) {

        this.io = io;
        this.index = index;
    }

    // #endregion



    // #region PERIODIC

    public void periodic() {

        // Pede para a implementação IO
        // atualizar os sensores
        io.updateInputs(
            inputs
        );


        // Envia os dados para o AdvantageKit
        Logger.processInputs(
            "Drive/Module" + index,
            inputs
        );
    }

    // #endregion



    // #region COMANDOS DE TENSÃO

    // ============================================================
    // DRIVE MOTOR
    // ============================================================

    // Controle aberto.
    //
    // Continua existindo para:
    //
    // - testes
    // - diagnóstico
    // - futuramente SysId
    public void setDriveVoltage(
        double volts
    ) {

        io.setDriveVoltage(
            volts
        );
    }


    // ============================================================
    // STEER MOTOR
    // ============================================================

    // Controle aberto.
    public void setTurnVoltage(
        double volts
    ) {

        io.setTurnVoltage(
            volts
        );
    }

    // #endregion



    // #region ESTADO DESEJADO DO MÓDULO

        public void setDesiredState(
            SwerveModuleState state
        ) {


            // ==========================================================
            // 1 - ÂNGULO ATUAL DO MÓDULO
            // ==========================================================

            Rotation2d currentAngle =
                Rotation2d.fromRadians(
                    inputs.turnPositionRad
                );



            // ==========================================================
            // 2 - OTIMIZAÇÃO DO ESTADO
            // ==========================================================

            // Escolhe a menor rotação possível para o módulo.
            //
            // Se necessário, o WPILib inverte a velocidade
            // da roda para evitar que o Steer gire demais.
            SwerveModuleState optimizedState =
                SwerveModuleState.optimize(
                    state,
                    currentAngle
                );



            // ==========================================================
            // 3 - COMPENSAÇÃO POR COSSENO
            // ==========================================================

            // Calcula o erro entre o ângulo desejado
            // e o ângulo atual do módulo
            double angleError =
                optimizedState.angle
                    .minus(currentAngle)
                    .getRadians();


            // Reduz a velocidade da roda enquanto
            // ela ainda não está apontando corretamente
            double cosineScale =
                Math.cos(
                    angleError
                );


            optimizedState.speedMetersPerSecond *=
                cosineScale;



            // ==========================================================
            // 4 - VELOCIDADE DESEJADA DO DRIVE
            // ==========================================================

            // Velocidade que queremos que a roda alcance
            double desiredVelocity =
                optimizedState.speedMetersPerSecond;


            // Velocidade REAL medida
            double measuredVelocity =
                inputs.driveVelocityRadPerSec
                    * kWheelRadiusMeters;



            // ==========================================================
            // 5 - ÂNGULO DESEJADO DO STEER
            // ==========================================================

            // Ângulo que queremos que o módulo alcance
            double desiredAngle =
                optimizedState.angle.getRadians();



            // ==========================================================
            // 6 - MALHA FECHADA - DRIVE
            // >>> MALHA FECHADA <<<
            // ==========================================================

            // O Module NÃO calcula mais:
            //
            // PID
            // Feedforward
            // tensão
            //
            // Ele simplesmente informa:
            //
            // "Quero que a roda tenha esta velocidade."
            //
            // Quem fará o controle será:
            //
            // ModuleIOSim
            // ou futuramente
            // ModuleIOTalonFX / Phoenix 6
            io.setDriveVelocity(
                desiredVelocity
            );



            // ==========================================================
            // 7 - MALHA FECHADA - STEER
            // >>> MALHA FECHADA <<<
            // ==========================================================

            // O Module NÃO calcula mais
            // a tensão necessária para girar o Steer.
            //
            // Ele simplesmente informa:
            //
            // "Quero que o módulo chegue neste ângulo."
            //
            // ModuleIOSim:
            // PID calcula a correção.
            //
            // Hardware real:
            // Phoenix 6 fará o Closed Loop.
            io.setTurnPosition(
                desiredAngle
            );



            // ==========================================================
            // 8 - ADVANTAGEKIT
            // ==========================================================

            // Estado originalmente solicitado
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/DesiredState",
                state
            );


            // Estado depois da otimização
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/OptimizedState",
                optimizedState
            );


            // Compensação por cosseno
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/CosineScale",
                cosineScale
            );



            // ==========================================================
            // LOGS DO DRIVE
            // ==========================================================

            // Velocidade desejada
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/DriveVelocitySetpoint",
                desiredVelocity
            );


            // Velocidade medida
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/DriveVelocityMeasured",
                measuredVelocity
            );


            // Erro de velocidade
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/DriveVelocityError",
                desiredVelocity
                    - measuredVelocity
            );



            // ==========================================================
            // LOGS DO STEER
            // ==========================================================

            // Ângulo desejado
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/TurnPositionSetpointRad",
                desiredAngle
            );


            // Ângulo medido
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/TurnPositionMeasuredRad",
                currentAngle.getRadians()
            );


            // Erro angular
            Logger.recordOutput(
                "Drive/Module"
                    + index
                    + "/TurnErrorRad",
                angleError
            );
        }

        // #endregion



    // #region CALIBRAÇÃO - ALINHAMENTO DO STEER

    // ============================================================
    // POSICIONAR STEER DIRETAMENTE PARA CALIBRAÇÃO
    // ============================================================
    //
    // Diferente de setDesiredState(), este método NÃO utiliza:
    //
    // SwerveModuleState.optimize()
    //
    // Portanto:
    //
    // 0 rad = realmente pedir 0 rad ao controlador do Steer.
    //
    // Isso é importante nos testes de kS,
    // pois queremos que todas as rodas estejam orientadas
    // fisicamente para a mesma direção.
    //
    // ============================================================

    public void setCalibrationTurnPosition(
        double angleRadians
    ) {

        // Normaliza o ângulo entre -PI e +PI
        double normalizedAngle =
            MathUtil.angleModulus(
                angleRadians
            );


        // Envia diretamente para o IO.
        //
        // No robô real:
        //
        // ModuleIOTalonFX
        //      ↓
        // PositionVoltage
        //      ↓
        // Kraken do Steer
        io.setTurnPosition(
            normalizedAngle
        );


        // --------------------------------------------------------
        // LOG
        // --------------------------------------------------------

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/CalibrationTurnSetpointRad",
            normalizedAngle
        );
    }



    // ============================================================
    // ERRO DO STEER DURANTE CALIBRAÇÃO
    // ============================================================

    public double getCalibrationTurnErrorRad(
        double targetAngleRadians
    ) {

        return MathUtil.angleModulus(

            targetAngleRadians
                - inputs.turnPositionRad
        );
    }



    // ============================================================
    // VERIFICA SE O STEER ESTÁ ALINHADO
    // ============================================================

    public boolean isCalibrationTurnAligned(
        double targetAngleRadians,
        double toleranceRadians
    ) {

        double error =
            getCalibrationTurnErrorRad(
                targetAngleRadians
            );


        boolean aligned =
            Math.abs(error)
                <= toleranceRadians;


        // --------------------------------------------------------
        // LOGS
        // --------------------------------------------------------

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/CalibrationTurnErrorRad",
            error
        );


        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/CalibrationTurnAligned",
            aligned
        );

        Logger.recordOutput(
        "Drive/Module"
            + index
            + "/CalibrationTurnMeasuredRad",
        MathUtil.angleModulus(
            inputs.turnPositionRad
        )
    );


        return aligned;
    }

    // #endregion



    // #region SYSID - DADOS DO DRIVE

    // ============================================================
    // TENSÃO REAL APLICADA AO DRIVE
    // ============================================================
    //
    // O SysId precisa saber qual tensão realmente chegou
    // ao motor.
    //
    // Não usamos apenas a tensão solicitada porque nossa
    // proteção pode manter o motor em 0 V enquanto o
    // Steer ainda não estiver alinhado.
    //
    public double getDriveAppliedVolts() {

        return inputs.driveAppliedVolts;
    }

    // #endregion
    


    // #region ESTADO ATUAL DO MÓDULO

    public SwerveModuleState getState() {


        // Converte:
        //
        // rad/s
        // ↓
        // m/s
        double velocityMetersPerSecond =
            inputs.driveVelocityRadPerSec
                * kWheelRadiusMeters;


        // Ângulo atual do módulo
        Rotation2d angle =
            Rotation2d.fromRadians(
                inputs.turnPositionRad
            );


        return new SwerveModuleState(
            velocityMetersPerSecond,
            angle
        );
    }

    // #endregion



    // #region POSIÇÃO ATUAL DO MÓDULO

    public SwerveModulePosition getPosition() {


        // Converte:
        //
        // rad
        // ↓
        // metros
        double distanceMeters =
            inputs.drivePositionRad
                * kWheelRadiusMeters;


        // Ângulo atual do módulo
        Rotation2d angle =
            Rotation2d.fromRadians(
                inputs.turnPositionRad
            );


        return new SwerveModulePosition(
            distanceMeters,
            angle
        );
    }

    // #endregion
}