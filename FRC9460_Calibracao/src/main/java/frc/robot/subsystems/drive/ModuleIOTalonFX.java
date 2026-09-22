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

import com.ctre.phoenix6.controls.PositionVoltage;
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

    // Relação do Drive:
    //
    // 6.746 rotações do motor
    // =
    // 1 rotação da roda
    private static final double DRIVE_GEAR_RATIO = 6.746031746031747;

    // Relação do Steer:
    //
    // 21.428 rotações do motor
    // =
    // 1 rotação completa do módulo
    private static final double STEER_GEAR_RATIO = 21.428571428571427;

    // Raio da roda:
    //
    // 2 polegadas
    // =
    // 0.0508 metros
    private static final double WHEEL_RADIUS_METERS = 0.0508;

    // #endregion

    // #region HARDWARE

    // Kraken responsável pela tração
    private final TalonFX driveMotor;

    // Kraken responsável pela orientação
    private final TalonFX steerMotor;

    // Encoder absoluto da orientação do módulo
    private final CANcoder steerEncoder;

    // #endregion

    // #region CONTROLES PHOENIX - MALHA FECHADA
    // ============================================================
    // DRIVE - VELOCIDADE
    // ============================================================

    // O TalonFX recebe uma velocidade desejada
    // e executa o PID internamente.
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0)
            .withSlot(0)
            .withEnableFOC(true);

    // ============================================================
    // STEER - POSIÇÃO
    // ============================================================

    // O TalonFX recebe uma posição angular desejada
    // e executa o PID internamente.
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0.0)
            .withSlot(0)
            .withEnableFOC(true);

    // #endregion

    // #region CONTROLE ABERTO

    // Mantidos para testes / SysId
    private final VoltageOut driveVoltageRequest = new VoltageOut(0.0)
            .withEnableFOC(true);

    private final VoltageOut steerVoltageRequest = new VoltageOut(0.0)
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
            boolean encoderInverted) {

        // ============================================================
        // 1 - CRIAÇÃO DOS DISPOSITIVOS
        // ============================================================

        // Kraken responsável pela tração
        driveMotor = new TalonFX(
                driveMotorId);

        // Kraken responsável por girar o módulo
        steerMotor = new TalonFX(
                steerMotorId);

        // CANcoder responsável por medir
        // o ângulo absoluto do módulo
        steerEncoder = new CANcoder(
                encoderId);

        // ============================================================
        // 2 - CONFIGURAÇÃO DO CANCODER
        // ============================================================

        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()

                // Offset obtido durante
                // a calibração no Phoenix Tuner
                .withMagnetOffset(
                        encoderOffsetRotations)

                // Define qual sentido será considerado positivo
                .withSensorDirection(

                        encoderInverted

                                ? SensorDirectionValue.Clockwise_Positive

                                : SensorDirectionValue.CounterClockwise_Positive);

        // ============================================================
        // 3 - MONTA CONFIGURAÇÃO DO CANCODER
        // ============================================================

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration()

                .withMagnetSensor(
                        magnetConfig);

        // ============================================================
        // 4 - ENVIA CONFIGURAÇÃO PARA O CANCODER REAL
        // ============================================================

        steerEncoder
                .getConfigurator()
                .apply(
                        encoderConfig);

        // #region CONFIGURAÇÃO DO TALONFX - DRIVE

        // ============================================================
        // 1 - GANHOS DA MALHA FECHADA DO DRIVE
        // ============================================================

        // Estes valores vieram do projeto
        // gerado pelo Phoenix Tuner.
        //
        // O TalonFX executará este controle
        // internamente quando usarmos:
        //
        // VelocityVoltage
        Slot0Configs driveGains = new Slot0Configs()

                .withKP(
                        0.1)

                .withKI(
                        0.0)

                .withKD(
                        0.0)

                .withKS(
                        0.0)

                .withKV(
                        0.124);

        // ============================================================
        // 2 - LIMITE DE CORRENTE DO DRIVE
        // ============================================================

        // O arquivo gerado pelo Phoenix Tuner
        // utiliza limite de corrente de alimentação
        // de 70 A.
        //
        // Isso ajuda a controlar o consumo
        // elétrico do Kraken.
        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()

                .withSupplyCurrentLimit(
                        Amps.of(70))

                .withSupplyCurrentLimitEnable(
                        true);

        // ============================================================
        // 3 - INVERSÃO DO MOTOR DE DRIVE
        // ============================================================

        // Cada lado do Swerve pode precisar
        // girar o motor em sentido diferente.
        //
        // No arquivo do Tuner:
        //
        // lado esquerdo = false
        // lado direito = true
        MotorOutputConfigs driveMotorOutput = new MotorOutputConfigs()

                .withInverted(

                        driveInverted

                                ? InvertedValue.Clockwise_Positive

                                : InvertedValue.CounterClockwise_Positive);

        // ============================================================
        // 4 - MONTA A CONFIGURAÇÃO DO TALONFX
        // ============================================================

        TalonFXConfiguration driveConfig = new TalonFXConfiguration()

                // PID + Feedforward
                .withSlot0(
                        driveGains)

                // Limite de corrente
                .withCurrentLimits(
                        driveCurrentLimits)

                // Sentido do motor
                .withMotorOutput(
                        driveMotorOutput);

        // ============================================================
        // 5 - ENVIA A CONFIGURAÇÃO PARA O KRAKEN
        // ============================================================

        driveMotor
                .getConfigurator()
                .apply(
                        driveConfig);

        // #endregion

        // #region CONFIGURAÇÃO DO TALONFX - STEER

        // ============================================================
        // 1 - GANHOS DA MALHA FECHADA DO STEER
        // ============================================================

        // Esses valores vieram do Phoenix Tuner.
        //
        // O TalonFX irá utilizar esses ganhos
        // quando recebermos um comando:
        //
        // setTurnPosition(...)
        // ↓
        // PositionVoltage
        // ↓
        // TalonFX Closed Loop
        Slot0Configs steerGains = new Slot0Configs()

                .withKP(
                        100.0)

                .withKI(
                        0.0)

                .withKD(
                        0.5)

                .withKS(
                        0.1)

                .withKV(
                        2.66)

                .withKA(
                        0.0)

                .withStaticFeedforwardSign(
                        StaticFeedforwardSignValue.UseClosedLoopSign);

        // ============================================================
        // 2 - CONFIGURAÇÃO DO SENSOR DO STEER
        // ============================================================

        // Aqui dizemos ao TalonFX:
        //
        // "Use o CANcoder deste módulo
        // como referência de posição."
        //
        // Como temos Phoenix Pro,
        // usamos FusedCANcoder.
        FeedbackConfigs steerFeedback = new FeedbackConfigs()

                // ID do CANcoder pertencente
                // a este módulo
                .withFeedbackRemoteSensorID(
                        encoderId)

                // CANcoder + encoder interno do Kraken
                .withFeedbackSensorSource(
                        FeedbackSensorSourceValue.FusedCANcoder)

                // ========================================================
                // RELAÇÃO ROTOR → SENSOR
                // ========================================================

                // O Kraken precisa girar aproximadamente
                // 21.428 vezes para o módulo girar 1 volta.
                .withRotorToSensorRatio(
                        STEER_GEAR_RATIO)

                // ========================================================
                // RELAÇÃO SENSOR → MECANISMO
                // ========================================================

                // O CANcoder mede diretamente
                // a rotação do módulo.
                //
                // 1 rotação CANcoder
                // =
                // 1 rotação do módulo
                .withSensorToMechanismRatio(
                        1.0);

        // ============================================================
        // 3 - CONTINUOUS WRAP
        // ============================================================

        // O Steer pode girar continuamente.
        //
        // Isso permite ao TalonFX escolher
        // o menor caminho angular.
        //
        // Exemplo:
        //
        // posição atual = +179°
        // posição desejada = -179°
        //
        // O correto é girar aproximadamente 2°,
        // e não 358°.
        ClosedLoopGeneralConfigs steerClosedLoop = new ClosedLoopGeneralConfigs()

                .withContinuousWrap(
                        true);

        // ============================================================
        // 4 - LIMITE DE CORRENTE DO STEER
        // ============================================================

        // O projeto gerado pelo Phoenix Tuner
        // usa 60 A de limite de corrente
        // no estator do Steer.
        CurrentLimitsConfigs steerCurrentLimits = new CurrentLimitsConfigs()

                .withStatorCurrentLimit(
                        Amps.of(60))

                .withStatorCurrentLimitEnable(
                        true);

        // ============================================================
        // 5 - INVERSÃO DO MOTOR DO STEER
        // ============================================================

        // Cada módulo recebe no construtor
        // se o motor precisa estar invertido.
        MotorOutputConfigs steerMotorOutput = new MotorOutputConfigs()

                .withInverted(

                        steerInverted

                                ? InvertedValue.Clockwise_Positive

                                : InvertedValue.CounterClockwise_Positive);

        // ============================================================
        // 6 - MONTA A CONFIGURAÇÃO DO TALONFX
        // ============================================================

        TalonFXConfiguration steerConfig = new TalonFXConfiguration()

                // PID + Feedforward
                .withSlot0(
                        steerGains)

                // FusedCANcoder
                .withFeedback(
                        steerFeedback)

                // Movimento angular contínuo
                .withClosedLoopGeneral(
                        steerClosedLoop)

                // Limite de corrente
                .withCurrentLimits(
                        steerCurrentLimits)

                // Inversão do motor
                .withMotorOutput(
                        steerMotorOutput);

        // ============================================================
        // 7 - ENVIA CONFIGURAÇÃO PARA O KRAKEN DO STEER
        // ============================================================

        steerMotor
                .getConfigurator()
                .apply(
                        steerConfig);

        // #endregion

    }
    // #endregion

    // #region ATUALIZAÇÃO DOS SENSORES

    @Override
    public void updateInputs(
            ModuleIOInputs inputs) {

        // ============================================================
        // 1 - DRIVE - POSIÇÃO
        // ============================================================

        // O TalonFX informa a posição do rotor
        // em rotações.
        double driveRotorRotations = driveMotor
                .getRotorPosition()
                .getValueAsDouble();

        // Converte:
        //
        // rotações do Kraken
        // ↓ divide pela redução
        // rotações da roda
        // ↓ multiplica por 2π
        // radianos da roda
        inputs.drivePositionRad = (driveRotorRotations
                /
                DRIVE_GEAR_RATIO)
                *
                (2.0 * Math.PI);

        // ============================================================
        // 2 - DRIVE - VELOCIDADE
        // ============================================================

        // Velocidade do rotor em rotações por segundo
        double driveRotorRps = driveMotor
                .getRotorVelocity()
                .getValueAsDouble();

        // Converte:
        //
        // rotações/s do Kraken
        // ↓
        // rotações/s da roda
        // ↓
        // rad/s da roda
        inputs.driveVelocityRadPerSec = (driveRotorRps
                /
                DRIVE_GEAR_RATIO)
                *
                (2.0 * Math.PI);

        // ============================================================
        // 3 - DRIVE - TENSÃO
        // ============================================================

        inputs.driveAppliedVolts = driveMotor
                .getMotorVoltage()
                .getValueAsDouble();

        // ============================================================
        // 4 - DRIVE - CORRENTE
        // ============================================================

        inputs.driveCurrentAmps = driveMotor
                .getStatorCurrent()
                .getValueAsDouble();

        // ============================================================
        // 5 - STEER - POSIÇÃO
        // ============================================================

        // Como configuramos:
        //
        // FusedCANcoder
        // RotorToSensorRatio
        // SensorToMechanismRatio
        //
        // o TalonFX já fornece a posição
        // relativa ao mecanismo do Steer.
        double steerRotations = steerMotor
                .getPosition()
                .getValueAsDouble();

        // Converte rotações do módulo
        // para radianos
        inputs.turnPositionRad = steerRotations
                *
                (2.0 * Math.PI);

        // ============================================================
        // 6 - STEER - VELOCIDADE
        // ============================================================

        double steerRotationsPerSecond = steerMotor
                .getVelocity()
                .getValueAsDouble();

        inputs.turnVelocityRadPerSec = steerRotationsPerSecond
                *
                (2.0 * Math.PI);

        // ============================================================
        // 7 - STEER - TENSÃO
        // ============================================================

        inputs.turnAppliedVolts = steerMotor
                .getMotorVoltage()
                .getValueAsDouble();

        // ============================================================
        // 8 - STEER - CORRENTE
        // ============================================================

        inputs.turnCurrentAmps = steerMotor
                .getStatorCurrent()
                .getValueAsDouble();
    }

    // #endregion

    // #region CONTROLE ABERTO - DRIVE

    @Override
    public void setDriveVoltage(
        double volts
    ) {

        // Envia diretamente uma tensão
        // para o Kraken responsável pelo Drive.
        //
        // Não existe PID aqui.
        //
        // Exemplo:
        //
        // setDriveVoltage(6.0)
        //
        // significa:
        //
        // "aplique aproximadamente 6 V no motor"
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

        // Envia diretamente uma tensão
        // para o Kraken responsável pelo Steer.
        //
        // Também não utiliza PID.
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

        // Circunferência da roda:
        //
        // 2 × PI × raio
        //
        // Exemplo:
        //
        // queremos 2 m/s
        // ↓
        // descobrimos quantas rotações por segundo
        // a roda precisa realizar.
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

        // Como existe uma redução de:
        //
        // 6.746 : 1
        //
        // o motor precisa girar mais rápido
        // que a roda.
        double motorRotationsPerSecond =

            wheelRotationsPerSecond

            *

            DRIVE_GEAR_RATIO;

        // ============================================================
        // 3 - PHOENIX 6 - MALHA FECHADA
        // ============================================================

        // Agora NÃO estamos dizendo:
        //
        // "mande 6 volts"
        //
        // Estamos dizendo:
        //
        // "mantenha esta velocidade"
        //
        // O TalonFX usa:
        //
        // PID
        // Feedforward
        // sensor interno do Kraken
        //
        // para calcular automaticamente
        // a saída necessária.
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

        // Mantém o ângulo entre:
        //
        // -PI e +PI
        //
        // Exemplo:
        //
        // 270° vira -90°
        //
        // Isso ajuda o módulo a trabalhar
        // sempre com o menor caminho angular.
        double normalizedAngle =
            MathUtil.angleModulus(
                angleRadians
            );



        // ============================================================
        // 2 - RADIANOS → ROTAÇÕES
        // ============================================================

        // Phoenix trabalha aqui com rotações
        // do mecanismo.
        //
        // 2 * PI radianos
        // =
        // 1 rotação completa
        double desiredRotations =

            normalizedAngle

            /

            (2.0 * Math.PI);



        // ============================================================
        // 3 - PHOENIX 6 - MALHA FECHADA DO STEER
        // ============================================================

        // Agora estamos dizendo:
        //
        // "Quero que o módulo chegue
        // nesta posição angular."
        //
        // O TalonFX usa:
        //
        // PositionVoltage
        // Slot0
        // FusedCANcoder
        // ContinuousWrap
        //
        // para corrigir automaticamente
        // o ângulo.
        steerMotor.setControl(

            steerPositionRequest.withPosition(
                desiredRotations
            )
        );
    }

    // #endregion

}
