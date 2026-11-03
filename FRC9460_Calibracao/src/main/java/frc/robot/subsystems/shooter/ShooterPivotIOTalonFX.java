package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.util.TalonZeroMark;

// #endregion

/**
 * Pivô real: TalonFX no CAN 32, com a mesma configuração da competição.
 *
 * O Motion Magic roda DENTRO do TalonFX (1 kHz). O código só envia a posição
 * final; o Talon gera o perfil e segura a posição com o kP.
 */
public class ShooterPivotIOTalonFX implements ShooterPivotIO {

    // #region MOTOR E SINAIS

    private static final double SIGNAL_UPDATE_HZ = 50.0;
    private static final double STARTUP_READ_TIMEOUT_SECONDS = 0.1;

    private final TalonFX motor = new TalonFX(PIVOT_MOTOR_ID);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);

    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<Current> current = motor.getStatorCurrent();
    private final StatusSignal<Double> reference = motor.getClosedLoopReference();
    private final StatusSignal<Boolean> forwardLimit = motor.getFault_ForwardSoftLimit();
    private final StatusSignal<Boolean> reverseLimit = motor.getFault_ReverseSoftLimit();

    private boolean configured;
    private boolean zeroValid;
    private boolean positionControlActive;
    private String startupZero = "NAO_AVALIADO";

    // #endregion

    // #region 🧭 CONFIGURAÇÃO E ZERO AO LIGAR

    public ShooterPivotIOTalonFX() {
        TalonFXConfiguration config = createConfiguration();
        StatusCode configResult = tryUntilOk(() -> motor.getConfigurator().apply(config));
        configured = configResult.isOK();
        reportIfFailed(configResult, "Pivô do shooter: configuração do TalonFX " + PIVOT_MOTOR_ID);
        applySignalRates("Pivô do shooter", () -> BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ,
            position, velocity, voltage, current, reference, forwardLimit, reverseLimit), motor);

        // Descarta o aviso de reset da própria ligação; resets futuros contam.
        motor.hasResetOccurred();

        // Talon recém-ligado: o shooter foi centralizado à mão, o centro é aqui.
        // Só o código reiniciou (deploy): mantém o centro que o Talon guarda.
        // Antes zerava sempre, inclusive com o shooter virado (frc.robot.util.TalonZeroMark).
        zeroOnStartup();
    }

    private void zeroOnStartup() {
        StatusCode read = position.waitForUpdate(STARTUP_READ_TIMEOUT_SECONDS).getStatus();
        TalonZeroMark.Decision decision = configured && read.isOK()
            ? TalonZeroMark.onStartup(position.getValueAsDouble(),
                -PIVOT_MAX_MOTOR_ROTATIONS, PIVOT_MAX_MOTOR_ROTATIONS)
            : TalonZeroMark.Decision.POSICAO_DESCONHECIDA;
        startupZero = decision.name();
        switch (decision) {
            case ZERAR_AQUI -> zeroHere();
            case MANTER_ZERO -> zeroValid = true;
            case POSICAO_DESCONHECIDA -> {
                zeroValid = false;
                motor.stopMotor();
                if (configured) {
                    reportIfFailed(StatusCode.GeneralError, "Pivô do shooter: posição ao iniciar "
                        + "(centralize e zere com Back + Start do operador, desabilitado)");
                }
            }
        }
    }

    @Override
    public void zeroHere() {
        positionControlActive = false;
        zeroValid = configured
            && tryUntilOk(() -> motor.setPosition(TalonZeroMark.toTalon(0.0))).isOK();
        if (!zeroValid) {
            motor.stopMotor();
            if (configured) {
                reportIfFailed(StatusCode.GeneralError, "Pivô do shooter: zerar o encoder");
            }
        }
    }

    // #endregion

    // #region CONFIGURAÇÃO DO TALONFX (PHOENIX 6)

    /** Configuração da competição, separada por assunto. */
    public static TalonFXConfiguration createConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // #region MOTOR E SENTIDO

        // Brake: parado, o motor resiste a ser girado.
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.DutyCycleNeutralDeadband = PIVOT_NEUTRAL_DEADBAND;
        config.MotorOutput.Inverted = PIVOT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        // #endregion

        // #region 🛑⚡ SEGURANÇA - LIMITES DE CORRENTE

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = PIVOT_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_CURRENT_LIMIT_AMPS;

        // #endregion

        // #region 🛑📏 SEGURANÇA - SOFT LIMITS (CURSO DA FIAÇÃO)

        // Segunda proteção do curso: mesmo que algum código peça mais, o Talon
        // corta a saída no sentido do limite calibrado. Só vale com zero no centro.
        // Feedback permanece em voltas do rotor; a conversão angular fica no código.
        // No Talon o centro é a marca (100 voltas), então os limites também.
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TalonZeroMark.toTalon(PIVOT_MAX_MOTOR_ROTATIONS);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TalonZeroMark.toTalon(-PIVOT_MAX_MOTOR_ROTATIONS);

        // #endregion

        // #region 🎯🚀 CONTROLE - PID (SLOT0) E PERFIL DO MOTION MAGIC

        config.Slot0.kP = PIVOT_KP;
        config.Slot0.kI = PIVOT_KI;
        config.Slot0.kD = PIVOT_KD;
        config.Slot0.kS = PIVOT_KS_VOLTS;
        // No fim do perfil, referência de velocidade = 0. A compensação ainda
        // precisa vencer atrito no sentido do erro residual de posição.
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        // Tensão proporcional à velocidade do perfil: o pivô acompanha o
        // Motion Magic em vez de esperar o erro crescer para o kP empurrar.
        config.Slot0.kV = PIVOT_KV_VOLTS_PER_ROTATION_PER_SEC;

        config.MotionMagic.MotionMagicCruiseVelocity = PIVOT_CRUISE_ROTATIONS_PER_SEC;
        config.MotionMagic.MotionMagicAcceleration = PIVOT_ACCELERATION_ROTATIONS_PER_SEC2;
        config.MotionMagic.MotionMagicJerk = PIVOT_JERK_ROTATIONS_PER_SEC3;

        // #endregion

        return config;
    }

    // #endregion

    // #region LEITURAS

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        StatusCode read = BaseStatusSignal.refreshAll(
            position, velocity, voltage, current, reference, forwardLimit, reverseLimit);

        // 🛑🧭 Um reset do Talon (queda de bateria, por exemplo) zera o encoder onde
        // o shooter estiver. O centro deixa de ser conhecido: para o motor até
        // a equipe centralizar e zerar de novo.
        if (motor.hasResetOccurred()) {
            zeroValid = false;
            motor.stopMotor();
            reportReset("Pivô do shooter (TalonFX " + PIVOT_MOTOR_ID + ")");
        }

        inputs.connected = read.isOK();
        inputs.zeroValid = configured && zeroValid;
        inputs.startupZero = startupZero;
        inputs.motorPositionRotations = TalonZeroMark.toMechanism(position.getValueAsDouble());
        inputs.motorVelocityRotationsPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.statorCurrentAmps = current.getValueAsDouble();
        // Fora do Motion Magic não há perfil: como na simulação, vale a posição.
        inputs.profileReferenceRotations = positionControlActive
            ? TalonZeroMark.toMechanism(reference.getValueAsDouble())
            : inputs.motorPositionRotations;
        inputs.forwardLimitActive = forwardLimit.getValue();
        inputs.reverseLimitActive = reverseLimit.getValue();
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setTargetRotations(double motorRotations) {
        if (!configured || !zeroValid || !Double.isFinite(motorRotations)) {
            stop();
            return;
        }
        positionControlActive = true;
        // 🛑📏 O clamp é a última barreira no código; depois dele ainda há o soft limit do Talon.
        motor.setControl(positionRequest.withPosition(TalonZeroMark.toTalon(
            MathUtil.clamp(motorRotations, -PIVOT_MAX_MOTOR_ROTATIONS, PIVOT_MAX_MOTOR_ROTATIONS))));
    }

    @Override
    public void setPercentOutput(double percent) {
        if (!configured || !zeroValid || !Double.isFinite(percent)) {
            stop();
            return;
        }
        positionControlActive = false;
        motor.setControl(percentRequest.withOutput(MathUtil.clamp(percent, -1.0, 1.0)));
    }

    @Override
    public void stop() {
        positionControlActive = false;
        motor.stopMotor();
    }

    // #endregion
}
