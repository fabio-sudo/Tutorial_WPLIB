package frc.robot.subsystems.drive;


// #region IMPORTS

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


// ============================================================
// NEW - MOTION MAGIC EXPO
// ============================================================

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;


import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

// #endregion



public class ModuleIOTalonFX implements ModuleIO {


    // #region CONSTANTES MECÂNICAS


    // ============================================================
    // DRIVE GEAR RATIO
    // ============================================================
    //
    // 6.746 rotações do motor
    // =
    // 1 rotação da roda
    //

    private static final double DRIVE_GEAR_RATIO =
        6.746031746031747;



    // ============================================================
    // STEER GEAR RATIO
    // ============================================================
    //
    // 21.428 rotações do motor
    // =
    // 1 rotação completa do módulo
    //

    private static final double STEER_GEAR_RATIO =
        21.428571428571427;



    // ============================================================
    // RAIO DA RODA
    // ============================================================
    //
    // 2 polegadas
    // =
    // 0.0508 metros
    //

    private static final double WHEEL_RADIUS_METERS =
        0.0508;


    // #endregion



    // #region HARDWARE


    // Kraken responsável pela tração
    private final TalonFX driveMotor;


    // Kraken responsável pela orientação
    private final TalonFX steerMotor;


    // CANcoder absoluto do módulo
    private final CANcoder steerEncoder;


    // #endregion



    // #region CONTROLES PHOENIX


    // ============================================================
    // DRIVE - VELOCITY VOLTAGE
    // ============================================================

    private final VelocityVoltage driveVelocityRequest =
        new VelocityVoltage(0.0)

            .withSlot(0)

            .withEnableFOC(true);



    // ============================================================
    // NEW - STEER - MOTION MAGIC EXPO
    // ============================================================
    //
    // ANTES:
    //
    // PositionVoltage
    //
    // AGORA:
    //
    // MotionMagicExpoVoltage
    //
    // Isso permite controlar melhor a velocidade
    // e a aceleração da mudança de orientação.
    //

    private final MotionMagicExpoVoltage steerPositionRequest =
        new MotionMagicExpoVoltage(0.0)

            .withSlot(0)

            .withEnableFOC(true);



    // ============================================================
    // CONTROLE ABERTO
    // ============================================================
    //
    // Mantidos para testes e SysId.
    //

    private final VoltageOut driveVoltageRequest =
        new VoltageOut(0.0)

            .withEnableFOC(true);


    private final VoltageOut steerVoltageRequest =
        new VoltageOut(0.0)

            .withEnableFOC(true);


    // #endregion



    // #region CONSTRUTOR


    public ModuleIOTalonFX(

        int driveMotorId,

        int steerMotorId,

        int encoderId,

        double encoderOffsetRotations,

        boolean driveInverted,

        boolean steerInverted,

        boolean encoderInverted

    ) {


        // ============================================================
        // 1 - CRIAÇÃO DOS DISPOSITIVOS
        // ============================================================


        driveMotor =
            new TalonFX(
                driveMotorId
            );


        steerMotor =
            new TalonFX(
                steerMotorId
            );


        steerEncoder =
            new CANcoder(
                encoderId
            );



        // ============================================================
        // 2 - CONFIGURAÇÃO DO CANCODER
        // ============================================================


        MagnetSensorConfigs magnetConfig =
            new MagnetSensorConfigs()


                // Offset obtido durante
                // a calibração no Phoenix Tuner
                .withMagnetOffset(
                    encoderOffsetRotations
                )


                // Define o sentido positivo do encoder
                .withSensorDirection(

                    encoderInverted

                        ? SensorDirectionValue.Clockwise_Positive

                        : SensorDirectionValue.CounterClockwise_Positive
                );



        // ============================================================
        // 3 - CONFIGURAÇÃO COMPLETA DO CANCODER
        // ============================================================


        CANcoderConfiguration encoderConfig =
            new CANcoderConfiguration()

                .withMagnetSensor(
                    magnetConfig
                );



        // ============================================================
        // 4 - ENVIA CONFIGURAÇÃO PARA O CANCODER
        // ============================================================


        steerEncoder

            .getConfigurator()

            .apply(
                encoderConfig
            );



        // #region TALONFX DRIVE


        // ============================================================
        // 1 - GANHOS DO DRIVE
        // ============================================================


        Slot0Configs driveGains =
            new Slot0Configs()


                .withKP(
                    0.10
                )


                .withKI(
                    0.0
                )


                .withKD(
                    0.0
                )


                .withKS(
                    0.26
                )


                .withKV(
                    0.108
                )


                .withKA(
                    0.0
                );



        // ============================================================
        // 2 - LIMITES DE CORRENTE DO DRIVE
        // ============================================================
        //
        // Supply = 70 A
        //
        // Supply Lower = 40 A
        //
        // Supply Lower Time = 1 segundo
        //
        // Stator = 120 A
        //


        CurrentLimitsConfigs driveCurrentLimits =
            new CurrentLimitsConfigs()


                .withSupplyCurrentLimit(
                    Amps.of(70)
                )


                .withSupplyCurrentLimitEnable(
                    true
                )


                .withSupplyCurrentLowerLimit(
                    Amps.of(40)
                )


                .withSupplyCurrentLowerTime(
                    Seconds.of(1.0)
                )


                .withStatorCurrentLimit(
                    Amps.of(120)
                )


                .withStatorCurrentLimitEnable(
                    true
                );



        // ============================================================
        // 3 - INVERSÃO DO MOTOR DE DRIVE
        // ============================================================


        MotorOutputConfigs driveMotorOutput =
            new MotorOutputConfigs()


                .withInverted(

                    driveInverted

                        ? InvertedValue.Clockwise_Positive

                        : InvertedValue.CounterClockwise_Positive
                );



        // ============================================================
        // 4 - CONFIGURAÇÃO COMPLETA DO DRIVE
        // ============================================================


        TalonFXConfiguration driveConfig =
            new TalonFXConfiguration()


                // PID + Feedforward
                .withSlot0(
                    driveGains
                )


                // Limite de corrente
                .withCurrentLimits(
                    driveCurrentLimits
                )


                // Inversão
                .withMotorOutput(
                    driveMotorOutput
                );



        // ============================================================
        // 5 - ENVIA CONFIGURAÇÃO PARA O KRAKEN DRIVE
        // ============================================================


        driveMotor

            .getConfigurator()

            .apply(
                driveConfig
            );


        // #endregion



        // #region TALONFX STEER


        // ============================================================
        // 1 - GANHOS DO STEER
        // ============================================================
        //
        // Esses ganhos permanecem os mesmos.
        //
        // NEW:
        //
        // Agora são usados junto com
        // MotionMagicExpoVoltage.
        //


        Slot0Configs steerGains =
            new Slot0Configs()


                .withKP(
                    100.0
                )


                .withKI(
                    0.0
                )


                .withKD(
                    0.5
                )


                .withKS(
                    0.1
                )


                .withKV(
                    2.66
                )


                .withKA(
                    0.0
                )


                .withStaticFeedforwardSign(
                    StaticFeedforwardSignValue.UseClosedLoopSign
                );



        // ============================================================
        // 2 - SENSOR DO STEER
        // ============================================================
        //
        // Kraken utiliza o CANcoder
        // como referência absoluta.
        //
        // Como estamos usando Phoenix Pro:
        //
        // FusedCANcoder
        //


        FeedbackConfigs steerFeedback =
            new FeedbackConfigs()


                // CANcoder deste módulo
                .withFeedbackRemoteSensorID(
                    encoderId
                )


                // Sensor utilizado pelo TalonFX
                .withFeedbackSensorSource(
                    FeedbackSensorSourceValue.FusedCANcoder
                )


                // ====================================================
                // ROTOR → SENSOR
                // ====================================================
                //
                // Kraken gira 21.428 vezes
                // para o módulo girar uma volta.
                //

                .withRotorToSensorRatio(
                    STEER_GEAR_RATIO
                )


                // ====================================================
                // SENSOR → MECANISMO
                // ====================================================
                //
                // CANcoder mede diretamente
                // a rotação do módulo.
                //

                .withSensorToMechanismRatio(
                    1.0
                );



        // ============================================================
        // 3 - CONTINUOUS WRAP
        // ============================================================
        //
        // Permite escolher o menor caminho angular.
        //
        // Exemplo:
        //
        // Atual = +179°
        //
        // Alvo = -179°
        //
        // O módulo gira aproximadamente 2°
        // em vez de 358°.
        //


        ClosedLoopGeneralConfigs steerClosedLoop =
            new ClosedLoopGeneralConfigs()


                .withContinuousWrap(
                    true
                );



        // ============================================================
        // 4 - LIMITE DE CORRENTE DO STEER
        // ============================================================


        CurrentLimitsConfigs steerCurrentLimits =
            new CurrentLimitsConfigs()


                .withStatorCurrentLimit(
                    Amps.of(60)
                )


                .withStatorCurrentLimitEnable(
                    true
                );



        // ============================================================
        // NEW - 5 - MOTION MAGIC EXPO DO STEER
        // ============================================================
        //
        // ESTES SÃO OS VALORES QUE FICARAM BONS
        // NO TESTE FÍSICO DO ROBÔ.
        //
        // CruiseVelocity:
        //
        // 1.5 rotações do módulo por segundo
        //
        // 1.5 rps × 360°
        // =
        // 540 graus/s
        //
        //
        // Expo kV:
        //
        // 0.12 × relação do Steer
        //
        //
        // Expo kA:
        //
        // 0.15
        //
        // Esse valor foi aumentado para suavizar
        // a aceleração da troca de orientação.
        //


        MotionMagicConfigs steerMotionMagic =
            new MotionMagicConfigs()


                // NEW --------------------------------------------
                // Limite de velocidade do Steer
                // ------------------------------------------------

                .withMotionMagicCruiseVelocity(
                    1.5
                )


                // NEW --------------------------------------------
                // Motion Magic Expo - velocidade
                // ------------------------------------------------

                .withMotionMagicExpo_kV(
                    0.12 * STEER_GEAR_RATIO
                )


                // NEW --------------------------------------------
                // Motion Magic Expo - aceleração
                // ------------------------------------------------

                .withMotionMagicExpo_kA(
                    0.15
                );



        // ============================================================
        // 6 - INVERSÃO DO MOTOR DO STEER
        // ============================================================


        MotorOutputConfigs steerMotorOutput =
            new MotorOutputConfigs()


                .withInverted(

                    steerInverted

                        ? InvertedValue.Clockwise_Positive

                        : InvertedValue.CounterClockwise_Positive
                );



        // ============================================================
        // 7 - CONFIGURAÇÃO COMPLETA DO STEER
        // ============================================================


        TalonFXConfiguration steerConfig =
            new TalonFXConfiguration()


                // PID + Feedforward
                .withSlot0(
                    steerGains
                )


                // Fused CANcoder
                .withFeedback(
                    steerFeedback
                )


                // Continuous Wrap
                .withClosedLoopGeneral(
                    steerClosedLoop
                )


                // ====================================================
                // NEW - MOTION MAGIC EXPO
                // ====================================================

                .withMotionMagic(
                    steerMotionMagic
                )


                // Limite de corrente
                .withCurrentLimits(
                    steerCurrentLimits
                )


                // Inversão
                .withMotorOutput(
                    steerMotorOutput
                );



        // ============================================================
        // 8 - ENVIA CONFIGURAÇÃO PARA O KRAKEN STEER
        // ============================================================


        steerMotor

            .getConfigurator()

            .apply(
                steerConfig
            );


        // #endregion
    }


    // #endregion



    // #region ATUALIZAÇÃO DOS SENSORES


    @Override
    public void updateInputs(
        ModuleIOInputs inputs
    ) {


        // ============================================================
        // 1 - DRIVE - POSIÇÃO
        // ============================================================


        double driveRotorRotations =
            driveMotor

                .getRotorPosition()

                .getValueAsDouble();



        // Rotor
        // ↓
        // redução
        // ↓
        // roda
        // ↓
        // radianos

        inputs.drivePositionRad =

            (
                driveRotorRotations
                /
                DRIVE_GEAR_RATIO
            )

            *

            (
                2.0 * Math.PI
            );



        // ============================================================
        // 2 - DRIVE - VELOCIDADE
        // ============================================================


        double driveRotorRps =
            driveMotor

                .getRotorVelocity()

                .getValueAsDouble();



        inputs.driveVelocityRadPerSec =

            (
                driveRotorRps
                /
                DRIVE_GEAR_RATIO
            )

            *

            (
                2.0 * Math.PI
            );



        // ============================================================
        // 3 - DRIVE - TENSÃO
        // ============================================================


        inputs.driveAppliedVolts =

            driveMotor

                .getMotorVoltage()

                .getValueAsDouble();



        // ============================================================
        // 4 - DRIVE - CORRENTE
        // ============================================================


        inputs.driveCurrentAmps =

            driveMotor

                .getStatorCurrent()

                .getValueAsDouble();



        // ============================================================
        // 5 - STEER - POSIÇÃO
        // ============================================================
        //
        // Como temos:
        //
        // FusedCANcoder
        // RotorToSensorRatio
        // SensorToMechanismRatio
        //
        // getPosition() representa a posição
        // do mecanismo do Steer.
        //


        double steerRotations =
            steerMotor

                .getPosition()

                .getValueAsDouble();



        inputs.turnPositionRad =

            steerRotations

            *

            (
                2.0 * Math.PI
            );



        // ============================================================
        // 6 - STEER - VELOCIDADE
        // ============================================================


        double steerRotationsPerSecond =
            steerMotor

                .getVelocity()

                .getValueAsDouble();



        inputs.turnVelocityRadPerSec =

            steerRotationsPerSecond

            *

            (
                2.0 * Math.PI
            );



        // ============================================================
        // 7 - STEER - TENSÃO
        // ============================================================


        inputs.turnAppliedVolts =

            steerMotor

                .getMotorVoltage()

                .getValueAsDouble();



        // ============================================================
        // 8 - STEER - CORRENTE
        // ============================================================


        inputs.turnCurrentAmps =

            steerMotor

                .getStatorCurrent()

                .getValueAsDouble();
    }


    // #endregion



    // #region CONTROLE ABERTO - DRIVE


    @Override
    public void setDriveVoltage(
        double volts
    ) {


        driveMotor.setControl(

            driveVoltageRequest.withOutput(
                volts
            )

        );
    }


    // #endregion



    // #region CONTROLE ABERTO - STEER


    @Override
    public void setTurnVoltage(
        double volts
    ) {


        steerMotor.setControl(

            steerVoltageRequest.withOutput(
                volts
            )

        );
    }


    // #endregion



    // #region MALHA FECHADA - DRIVE


    @Override
    public void setDriveVelocity(
        double velocityMetersPerSecond
    ) {


        // ============================================================
        // 1 - m/s → ROTAÇÕES DA RODA POR SEGUNDO
        // ============================================================


        double wheelRotationsPerSecond =

            velocityMetersPerSecond

            /

            (
                2.0
                *
                Math.PI
                *
                WHEEL_RADIUS_METERS
            );



        // ============================================================
        // 2 - ROTAÇÕES DA RODA → ROTAÇÕES DO KRAKEN
        // ============================================================


        double motorRotationsPerSecond =

            wheelRotationsPerSecond

            *

            DRIVE_GEAR_RATIO;



        // ============================================================
        // 3 - PHOENIX 6 - CLOSED LOOP DO DRIVE
        // ============================================================


        driveMotor.setControl(

            driveVelocityRequest.withVelocity(
                motorRotationsPerSecond
            )

        );
    }


    // #endregion



    // #region MALHA FECHADA - STEER


    @Override
    public void setTurnPosition(
        double angleRadians
    ) {


        // ============================================================
        // 1 - NORMALIZA O ÂNGULO
        // ============================================================
        //
        // Mantém o ângulo entre:
        //
        // -PI e +PI
        //


        double normalizedAngle =

            MathUtil.angleModulus(
                angleRadians
            );



        // ============================================================
        // 2 - RADIANOS → ROTAÇÕES
        // ============================================================
        //
        // 2 PI rad
        // =
        // 1 rotação do módulo
        //


        double desiredRotations =

            normalizedAngle

            /

            (
                2.0 * Math.PI
            );



        // ============================================================
        // NEW - 3 - MOTION MAGIC EXPO DO STEER
        // ============================================================
        //
        // Antes:
        //
        // PositionVoltage
        //
        // Agora:
        //
        // MotionMagicExpoVoltage
        //
        // O Module continua pedindo apenas o ângulo desejado.
        //
        // O TalonFX controla:
        //
        // PID
        // FusedCANcoder
        // ContinuousWrap
        // Motion Magic Expo
        //


        steerMotor.setControl(

            steerPositionRequest.withPosition(
                desiredRotations
            )

        );
    }


    // #endregion
}