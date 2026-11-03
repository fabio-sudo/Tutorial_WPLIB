package frc.robot.subsystems.drive;

// #region IMPORTS
import static frc.robot.subsystems.drive.DriveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.drive.DriveConstants.STEER_GEAR_RATIO;
import static frc.robot.subsystems.drive.DriveConstants.kWheelRadiusMeters;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

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


import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.PhoenixUtil;
// #endregion

public class ModuleIOTalonFX implements ModuleIO {

    // #region 📏 CONSTANTES MECÂNICAS

    // Reduções (DRIVE_GEAR_RATIO, STEER_GEAR_RATIO) e raio da roda
    // (kWheelRadiusMeters) ficam em DriveConstants, na mesma pasta.

    // #endregion

    // #region FREQUÊNCIA DOS SINAIS

    // Um pacote por ciclo de 20 ms: odometria e tensão.
    private static final double CONTROL_SIGNAL_HZ = 50.0;

    // Corrente é só log nos módulos: 10 Hz bastam para ver um pico.
    private static final double CURRENT_SIGNAL_HZ = 10.0;

    // 📊 Falha de leitura mais curta que isto não vira alerta: com o CAN cheio,
    // um pacote atrasado geraria um aviso a cada oscilação.
    private static final double CONNECTION_DEBOUNCE_SECONDS = 0.5;

    // #endregion

    // #region HARDWARE

    // Kraken responsável pela tração
    private final TalonFX driveMotor;

    // Kraken responsável pela orientação
    private final TalonFX steerMotor;

    // Encoder absoluto da orientação do módulo
    private final CANcoder steerEncoder;
    private final StatusSignal<Angle> drivePosition, turnPosition;
    private final StatusSignal<AngularVelocity> driveVelocity, turnVelocity;
    private final StatusSignal<Voltage> driveVoltage, turnVoltage;
    private final StatusSignal<Current> driveCurrent, turnCurrent;
    private final BaseStatusSignal[] driveSignals, turnSignals;
    private final Debouncer driveConnected =
        new Debouncer(CONNECTION_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);
    private final Debouncer turnConnected =
        new Debouncer(CONNECTION_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);
    private final Debouncer encoderConnected =
        new Debouncer(CONNECTION_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);

    // #endregion

    // #region 🎯 CONTROLES PHOENIX - MALHA FECHADA
    // ============================================================
    // DRIVE - VELOCIDADE
    // ============================================================

    // O TalonFX recebe uma velocidade desejada
    // e executa o PID internamente.
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0)
            .withSlot(0)
            .withEnableFOC(true);

        // ============================================================
        // STEER - MOTION MAGIC EXPO
        // ============================================================

        private final MotionMagicExpoVoltage steerPositionRequest =
        new MotionMagicExpoVoltage(0.0)
                .withSlot(0)
                .withEnableFOC(true);

    // #endregion

    // #region 🚀 CONTROLE ABERTO

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
        // 🛑 Nos três envios (CANcoder, drive e steer): com o CAN cheio na partida uma
        // configuração pode expirar. Tenta de novo e, se falhar, avisa na Driver Station.
        // ============================================================

        PhoenixUtil.reportKeptRunning(
                PhoenixUtil.tryUntilOk(() -> steerEncoder.getConfigurator().apply(encoderConfig)),
                "Swerve: configuração do CANcoder " + encoderId);

        // #region CONFIGURAÇÃO DO TALONFX - DRIVE

        // ============================================================
        // 1 - GANHOS DA MALHA FECHADA DO DRIVE
        // 🎯 PID DO DRIVE REAL: kP corrige o erro de velocidade, kS vence o atrito,
        // kV dá a tensão para a velocidade pedida. Unidades: volta do motor por segundo.
        // ============================================================

        // Estes valores vieram do projeto
        // gerado pelo Phoenix Tuner.
        //
        // O TalonFX executará este controle
        // internamente quando usarmos:
        //
        // VelocityVoltage
        // Slot0Configs driveGains = new Slot0Configs()

        //         .withKP(
        //                 0.1)

        //         .withKI(
        //                 0.0)

        //         .withKD(
        //                 0.0)

        //         .withKS(
        //                 0.0)

        //         .withKV(
        //                 0.124);

        Slot0Configs driveGains = new Slot0Configs()

                .withKP(
                0.10)

                .withKI(
                0.0)

                .withKD(
                0.0)

                .withKS(
                0.26)

                .withKV(
                0.108)

                .withKA(
                0.0);

// ============================================================
// 2 - LIMITES DE CORRENTE DO DRIVE
// ⚡🛑 CORRENTE DO DRIVE
// ============================================================
//
// Baseado no TunerConstants gerado pelo Phoenix Tuner:
//
// Supply Current = 70 A
// Stator / Slip Current = 120 A
//
// Supply:
// controla principalmente a corrente puxada da bateria.
//
// Stator:
// limita o torque produzido pelo Kraken.
// No Swerve da CTRE, esse valor corresponde ao conceito
// de kSlipCurrent.
//
// IMPORTANTE:
// 120 A é nosso valor inicial vindo do Tuner.
// Futuramente podemos medir o ponto real de patinagem
// e ajustar esse valor.
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

        PhoenixUtil.reportKeptRunning(
                PhoenixUtil.tryUntilOk(() -> driveMotor.getConfigurator().apply(driveConfig)),
                "Swerve: configuração do drive " + driveMotorId);

        // #endregion

        // #region CONFIGURAÇÃO DO TALONFX - STEER

        // ============================================================
        // 1 - GANHOS DA MALHA FECHADA DO STEER
        // 🎯 PID DO STEER REAL: kP puxa para o ângulo, kD amortece, kS vence o atrito
        // e kV acompanha o perfil. Unidades: volta do módulo.
        // ============================================================

        // Esses valores vieram do Phoenix Tuner.
        //
        // O TalonFX irá utilizar esses ganhos
        // quando recebermos um comando:
        //
        // setTurnPosition(...)
        // ↓
        // MotionMagicExpoVoltage
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
        ClosedLoopGeneralConfigs steerClosedLoop =
        new ClosedLoopGeneralConfigs()

                .withContinuousWrap(
                true
                );

        // ============================================================
        // 4 - LIMITE DE CORRENTE DO STEER
        // ⚡🛑 CORRENTE DO STEER
        // ============================================================

        // O projeto gerado pelo Phoenix Tuner
        // usa 60 A de limite de corrente
        // no estator do Steer. karaquem X60
        CurrentLimitsConfigs steerCurrentLimits = new CurrentLimitsConfigs()

        .withStatorCurrentLimit(
                Amps.of(60))

        .withStatorCurrentLimitEnable(
                true)

        .withSupplyCurrentLimit(
                Amps.of(70))

        .withSupplyCurrentLimitEnable(
                true)

        // Recomendação da CTRE (frc.robot.MotorLimits): 70 A por até 1 s,
        // depois 40 A, o valor do disjuntor. Igual ao motor de drive.
        .withSupplyCurrentLowerLimit(
                Amps.of(40))

        .withSupplyCurrentLowerTime(
                Seconds.of(1.0));

        // ============================================================
        // 5 - MOTION MAGIC EXPO DO STEER
        // 🚀 VELOCIDADE DO GIRO DAS RODAS
        // ============================================================
        //
        // NEW - AJUSTE PARA DEIXAR A TROCA DE DIREÇÃO MAIS SUAVE
        //

        MotionMagicConfigs steerMotionMagic =
        new MotionMagicConfigs()

                // Velocidade máxima do módulo. 
                //
                // 1.8 rotações/s (valor atual)
                // =
                // 648 graus/s
                //
                // Uma mudança de 90 graus não será mais instantânea.
                .withMotionMagicCruiseVelocity(
                1.8
                //era 1.5
                //=========================Testando 1.8 para nao perder a suavidade
                //Depoois vamos testar 2.0 rps
                )

                // Mantemos o kV inicial
                .withMotionMagicExpo_kV(
                0.12 * STEER_GEAR_RATIO
                )

                // NEW
                //
                // Antes:
                // 1.2 / 21.428 ≈ 0.056
                //
                // Agora aumentamos para 0.15
                // para reduzir a aceleração do Steer.
                .withMotionMagicExpo_kA(
                0.15
                );


        // ============================================================
        // 6 - INVERSÃO DO MOTOR DO STEER
        // ============================================================

        // Cada módulo recebe no construtor
        // se o motor precisa estar invertido.
        MotorOutputConfigs steerMotorOutput = new MotorOutputConfigs()

                .withInverted(

                        steerInverted

                                ? InvertedValue.Clockwise_Positive

                                : InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast);//======================COAST

        // ============================================================
        // 7 - MONTA A CONFIGURAÇÃO DO TALONFX
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
                // NEW - ENVIA A CONFIGURAÇÃO PARA O KRAKEN DO STEER
                // ============================================================

                PhoenixUtil.reportKeptRunning(
                        PhoenixUtil.tryUntilOk(() -> steerMotor.getConfigurator().apply(steerConfig)),
                        "Swerve: configuração do steer " + steerMotorId);

        // #endregion

        // Reutiliza sinais: uma atualização em lote por motor, em vez de oito chamadas
        // por módulo. Em lotes separados, o resultado diz qual motor parou de responder.
        //
        // Tração em getPosition/getVelocity, e não nos sinais do rotor: sem sensor
        // externo os valores são os mesmos, mas no CAN 2.0 o rotor sai a 4 Hz por
        // padrão e estes a 50 Hz. Se o Talon reiniciar no meio da partida, as
        // frequências pedidas voltam ao padrão e a odometria continua a 50 Hz.
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveVoltage = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getStatorCurrent();
        turnPosition = steerMotor.getPosition();
        turnVelocity = steerMotor.getVelocity();
        turnVoltage = steerMotor.getMotorVoltage();
        turnCurrent = steerMotor.getStatorCurrent();
        driveSignals = new BaseStatusSignal[] { drivePosition, driveVelocity, driveVoltage, driveCurrent };
        turnSignals = new BaseStatusSignal[] { turnPosition, turnVelocity, turnVoltage, turnCurrent };

        // O CANcoder fica de fora de propósito: o steer usa FusedCANcoder e recebe
        // a posição dele pelo CAN. Desacelerar o CANcoder pioraria o steer.
        PhoenixUtil.applySignalRates("Módulo do drive " + driveMotorId, () -> {
            StatusCode fast = BaseStatusSignal.setUpdateFrequencyForAll(CONTROL_SIGNAL_HZ,
                drivePosition, driveVelocity, driveVoltage, turnPosition, turnVelocity, turnVoltage);
            StatusCode currents = BaseStatusSignal.setUpdateFrequencyForAll(CURRENT_SIGNAL_HZ,
                driveCurrent, turnCurrent);
            return fast.isOK() ? currents : fast;
        }, driveMotor, steerMotor);
    }
    // #endregion

    // #region ATUALIZAÇÃO DOS SENSORES

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Não espera pacote novo no ciclo de 20 ms.
        StatusCode driveStatus = BaseStatusSignal.refreshAll(driveSignals);
        StatusCode turnStatus = BaseStatusSignal.refreshAll(turnSignals);
        // 📊 O Module transforma isto em alerta e aviso na Driver Station.
        inputs.driveConnected = driveConnected.calculate(driveStatus.isOK());
        inputs.turnConnected = turnConnected.calculate(turnStatus.isOK());
        inputs.encoderConnected = encoderConnected.calculate(steerEncoder.isConnected());
        inputs.drivePositionRad = drivePosition.getValueAsDouble() / DRIVE_GEAR_RATIO * 2.0 * Math.PI;
        inputs.driveVelocityRadPerSec = driveVelocity.getValueAsDouble() / DRIVE_GEAR_RATIO * 2.0 * Math.PI;
        inputs.driveAppliedVolts = driveVoltage.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
        inputs.turnPositionRad = turnPosition.getValueAsDouble() * 2.0 * Math.PI;
        inputs.turnVelocityRadPerSec = turnVelocity.getValueAsDouble() * 2.0 * Math.PI;
        inputs.turnAppliedVolts = turnVoltage.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    }

    // #endregion

    // #region 🚀 CONTROLE ABERTO - DRIVE

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

    // #region 🚀 CONTROLE ABERTO - STEER

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

    // #region 🎯🚀 MALHA FECHADA - DRIVE

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
                kWheelRadiusMeters
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

    // #region 🎯 MALHA FECHADA - STEER

                @Override
                public void setTurnPosition(
                double angleRadians
                ) {

                // ============================================================
                // 1 - NORMALIZA O ÂNGULO
                // ============================================================

                double normalizedAngle =
                        MathUtil.angleModulus(
                        angleRadians
                        );


                // ============================================================
                // 2 - RADIANOS → ROTAÇÕES
                // ============================================================

                double desiredRotations =
                        normalizedAngle
                        /
                        (2.0 * Math.PI);


                // ============================================================
                // 3 - PHOENIX 6 - MALHA FECHADA DO STEER
                // ============================================================

                // NEW
                //
                // Antes:
                // PositionVoltage
                //
                // Agora:
                // MotionMagicExpoVoltage
                //
                // Continua utilizando:
                //
                // Slot0
                // FusedCANcoder
                // ContinuousWrap
                //
                // mas agora o TalonFX gera
                // o perfil Motion Magic Expo.

                steerMotor.setControl(

                        steerPositionRequest.withPosition(
                        desiredRotations
                        )

                );
                }

                // #endregion

}
