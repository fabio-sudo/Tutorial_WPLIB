package frc.robot.subsystems.drive;


// #region IMPORTS

// ❌ REMOVIDO
// Não precisamos mais do kMaxLinearSpeed dentro do Module.
// O Module não calcula mais tensão diretamente.
// import static frc.robot.Constants.DriveConstants.kMaxLinearSpeed;


// 🔄 MANTIDO
// Continua necessário para converter rad/s → m/s
import static frc.robot.Constants.DriveConstants.kWheelRadiusMeters;


import org.littletonrobotics.junction.Logger;


// ❌ REMOVIDO
// MathUtil era utilizado para limitar as tensões em ±12V.
// Agora quem controla isso é o IO.
// import edu.wpi.first.math.MathUtil;


// 🔄 MANTIDOS
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// #endregion



public class Module {


    // #region IO DO MÓDULO

    // 🔄 MANTIDO
    //
    // O Module continua sem saber
    // se está trabalhando com:
    //
    // ModuleIOSim
    //
    // ou futuramente:
    //
    // ModuleIOTalonFX
    private final ModuleIO io;

    // #endregion



    // #region DADOS DO MÓDULO

    // 🔄 MANTIDO
    private final ModuleIOInputsAutoLogged inputs =
        new ModuleIOInputsAutoLogged();


    // 🔄 MANTIDO
    //
    // 0 = Front Left
    // 1 = Front Right
    // 2 = Back Left
    // 3 = Back Right
    private final int index;

    // #endregion



    // #region PID ANTIGO - REMOVIDO


    // ============================================================
    // ❌ REMOVIDO - PID DO STEER
    // ============================================================

    /*
    private final PIDController turnPID =
        new PIDController(
            4.0,
            0.0,
            0.0
        );
    */


    // ============================================================
    // ❌ REMOVIDO - PID DO DRIVE
    // ============================================================

    /*
    private final PIDController drivePID =
        new PIDController(
            kDriveKp,
            kDriveKi,
            kDriveKd
        );
    */


    // ============================================================
    // ❌ REMOVIDO - FEEDFORWARD DO DRIVE
    // ============================================================

    /*
    private final SimpleMotorFeedforward driveFeedforward =
        new SimpleMotorFeedforward(
            kDriveKs,
            kDriveKv
        );
    */

    // #endregion



    // #region CONSTRUTOR

    public Module(
        ModuleIO io,
        int index
    ) {

        // 🔄 MANTIDO
        this.io = io;

        // 🔄 MANTIDO
        this.index = index;



        // ========================================================
        // ❌ REMOVIDO - CONFIGURAÇÃO DO PID DO STEER
        // ========================================================

        /*
        turnPID.enableContinuousInput(
            -Math.PI,
            Math.PI
        );
        */


        /*
        turnPID.setTolerance(
            Math.toRadians(1.0)
        );
        */



        // ========================================================
        // ❌ REMOVIDO - CONFIGURAÇÃO DO PID DO DRIVE
        // ========================================================

        /*
        drivePID.setTolerance(
            0.05
        );
        */


        // Agora essas configurações ficam
        // dentro da implementação do IO.
        //
        // Na simulação:
        // ModuleIOSim
        //
        // No robô real:
        // ModuleIOTalonFX / Phoenix 6
    }

    // #endregion



    // #region PERIODIC

    public void periodic() {


        // 🔄 MANTIDO
        //
        // Atualiza:
        //
        // posição
        // velocidade
        // tensão
        // corrente
        io.updateInputs(
            inputs
        );


        // 🔄 MANTIDO
        Logger.processInputs(
            "Drive/Module" + index,
            inputs
        );
    }

    // #endregion



    // #region CONTROLE ABERTO - TENSÃO

    // Estes métodos continuam existindo.
    //
    // 🔄 MANTIDOS
    //
    // Podem ser usados para:
    //
    // testes
    // diagnóstico
    // SysId


    // ============================================================
    // DRIVE
    // ============================================================

    public void setDriveVoltage(
        double volts
    ) {

        io.setDriveVoltage(
            volts
        );
    }


    // ============================================================
    // STEER
    // ============================================================

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
        // 1 - ÂNGULO ATUAL
        // 🔄 MANTIDO
        // ==========================================================

        Rotation2d currentAngle =
            Rotation2d.fromRadians(
                inputs.turnPositionRad
            );



        // ==========================================================
        // 2 - OPTIMIZE
        // 🔄 MANTIDO
        // ==========================================================

        // Escolhe a menor rotação possível.
        //
        // Se for melhor:
        //
        // gira menos
        // +
        // inverte a velocidade da roda
        SwerveModuleState optimizedState =
            SwerveModuleState.optimize(
                state,
                currentAngle
            );



        // ==========================================================
        // 3 - COMPENSAÇÃO POR COSSENO
        // 🔄 MANTIDO
        // ==========================================================

        double angleError =
            optimizedState.angle
                .minus(currentAngle)
                .getRadians();


        double cosineScale =
            Math.cos(
                angleError
            );


        optimizedState.speedMetersPerSecond *=
            cosineScale;



        // ==========================================================
        // 4 - VELOCIDADE DESEJADA
        // 🔄 MANTIDO
        // ==========================================================

        double desiredVelocity =
            optimizedState.speedMetersPerSecond;



        // ==========================================================
        // 5 - VELOCIDADE MEDIDA
        // 🔄 MANTIDO PARA LOGS
        // ==========================================================

        double measuredVelocity =
            inputs.driveVelocityRadPerSec
                * kWheelRadiusMeters;



        // ==========================================================
        // 6 - ÂNGULO DESEJADO
        // 🔄 MANTIDO
        // ==========================================================

        double desiredAngle =
            optimizedState.angle.getRadians();



        // #region CONTROLE ANTIGO - REMOVIDO


        // ==========================================================
        // ❌ REMOVIDO - PID DO DRIVE
        // ==========================================================

        /*
        double drivePIDVolts =
            drivePID.calculate(
                measuredVelocity,
                desiredVelocity
            );
        */



        // ==========================================================
        // ❌ REMOVIDO - FEEDFORWARD DO DRIVE
        // ==========================================================

        /*
        double driveFFVolts =
            driveFeedforward.calculate(
                desiredVelocity
            );
        */



        // ==========================================================
        // ❌ REMOVIDO - SOMA PID + FEEDFORWARD
        // ==========================================================

        /*
        double driveVolts =
            driveFFVolts
            +
            drivePIDVolts;
        */



        // ==========================================================
        // ❌ REMOVIDO - LIMITAÇÃO DA TENSÃO DO DRIVE
        // ==========================================================

        /*
        driveVolts =
            MathUtil.clamp(
                driveVolts,
                -12.0,
                12.0
            );
        */



        // ==========================================================
        // ❌ REMOVIDO - PID DO STEER
        // ==========================================================

        /*
        double turnVolts =
            turnPID.calculate(
                currentAngle.getRadians(),
                desiredAngle
            );
        */



        // ==========================================================
        // ❌ REMOVIDO - LIMITAÇÃO DA TENSÃO DO STEER
        // ==========================================================

        /*
        turnVolts =
            MathUtil.clamp(
                turnVolts,
                -12.0,
                12.0
            );
        */



        // ==========================================================
        // ❌ REMOVIDO - ENVIO DIRETO DE TENSÃO
        // ==========================================================

        /*
        io.setDriveVoltage(
            driveVolts
        );


        io.setTurnVoltage(
            turnVolts
        );
        */

        // #endregion



        // #region NOVA MALHA FECHADA


        // ==========================================================
        // ✅ NOVO - MALHA FECHADA DO DRIVE
        // ==========================================================

        // ANTES:
        //
        // Module calculava:
        //
        // PID
        // +
        // Feedforward
        // +
        // tensão
        //
        //
        // AGORA:
        //
        // Module apenas diz:
        //
        // "Quero esta velocidade."
        io.setDriveVelocity(
            desiredVelocity
        );



        // ==========================================================
        // ✅ NOVO - MALHA FECHADA DO STEER
        // ==========================================================

        // ANTES:
        //
        // Module calculava o PID
        // e mandava tensão.
        //
        //
        // AGORA:
        //
        // Module apenas diz:
        //
        // "Quero este ângulo."
        io.setTurnPosition(
            desiredAngle
        );

        // #endregion



        // #region LOGS ADVANTAGEKIT


        // ==========================================================
        // ESTADO DESEJADO
        // 🔄 MANTIDO
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/DesiredState",
            state
        );



        // ==========================================================
        // ESTADO OTIMIZADO
        // 🔄 MANTIDO
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/OptimizedState",
            optimizedState
        );



        // ==========================================================
        // COSINE SCALE
        // 🔄 MANTIDO
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/CosineScale",
            cosineScale
        );



        // ==========================================================
        // DRIVE - SETPOINT
        // 🔄 MANTIDO
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/DriveVelocitySetpoint",
            desiredVelocity
        );



        // ==========================================================
        // DRIVE - MEDIDO
        // 🔄 MANTIDO
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/DriveVelocityMeasured",
            measuredVelocity
        );



        // ==========================================================
        // DRIVE - ERRO
        // 🔄 MANTIDO
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/DriveVelocityError",
            desiredVelocity
                - measuredVelocity
        );



        // ==========================================================
        // ✅ NOVO LOG - STEER SETPOINT
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/TurnPositionSetpointRad",
            desiredAngle
        );



        // ==========================================================
        // ✅ NOVO LOG - STEER MEDIDO
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/TurnPositionMeasuredRad",
            currentAngle.getRadians()
        );



        // ==========================================================
        // ✅ NOVO LOG - ERRO DO STEER
        // ==========================================================

        Logger.recordOutput(
            "Drive/Module"
                + index
                + "/TurnErrorRad",
            angleError
        );



        // ==========================================================
        // ❌ LOGS ANTIGOS REMOVIDOS
        // ==========================================================

        /*
        Logger.recordOutput(
            "Drive/Module" + index + "/DrivePIDVolts",
            drivePIDVolts
        );


        Logger.recordOutput(
            "Drive/Module" + index + "/DriveFFVolts",
            driveFFVolts
        );


        Logger.recordOutput(
            "Drive/Module" + index + "/DriveTotalVolts",
            driveVolts
        );


        Logger.recordOutput(
            "Drive/Module" + index + "/DriveAtSetpoint",
            drivePID.atSetpoint()
        );


        Logger.recordOutput(
            "Drive/Module" + index + "/TurnPIDOutputVolts",
            turnVolts
        );


        Logger.recordOutput(
            "Drive/Module" + index + "/TurnAtSetpoint",
            turnPID.atSetpoint()
        );
        */

        // #endregion
    }

    // #endregion



    // #region ESTADO ATUAL DO MÓDULO

    // 🔄 MANTIDO

    public SwerveModuleState getState() {


        double velocityMetersPerSecond =
            inputs.driveVelocityRadPerSec
                * kWheelRadiusMeters;


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

    // 🔄 MANTIDO

    public SwerveModulePosition getPosition() {


        double distanceMeters =
            inputs.drivePositionRad
                * kWheelRadiusMeters;


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