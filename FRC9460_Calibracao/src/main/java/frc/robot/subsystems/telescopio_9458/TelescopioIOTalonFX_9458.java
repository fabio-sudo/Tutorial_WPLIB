package frc.robot.subsystems.telescopio_9458;

// #region IMPORTS

import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

// #endregion

/**
 * Motor do telescópio 9458 físico: TalonFX (Kraken X60) no ID 5.
 * Ao ligar o robô o telescópio está fechado, então a posição é zerada no construtor.
 * Sem a configuração ou o zero confirmados, a posição fica inválida e o subsistema não move.
 */
public class TelescopioIOTalonFX_9458 implements TelescopioIO_9458 {

    // #region MOTOR E SINAIS

    private final Hardware hardware;
    private boolean configured;
    private boolean referenced;
    private boolean connected;
    private boolean referenceLostOnReset;
    private boolean controlRequestOK = true;
    private String configurationStatus = "NAO_APLICADA";
    private String zeroStatus = "NAO_REFERENCIADO";
    private String controlStatus = "SEM_COMANDO";
    private TelescopioGains_9458 activeGains = REAL_GAINS;
    private TelescopioMotionProfile_9458.Limits activeProfile = MOTION_MAGIC_LIMITS;
    // Velocidade de cruzeiro gravada no Talon; NaN = desconhecida (reescrever).
    private double appliedCruise = Double.NaN;
    // Cruzeiro cuja escrita falhou: só tenta de novo quando o pedido mudar.
    private double failedCruise = Double.NaN;
    private boolean motionMagicActive;
    private double extensionMeters;

    // #endregion

    // #region 🧭 CONFIGURACAO E ZERO AO LIGAR

    public TelescopioIOTalonFX_9458() {
        this(new TalonHardware());
    }

    // Permite testar falhas e resets sem criar um dispositivo CAN.
    TelescopioIOTalonFX_9458(Hardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
        confirmRetractedZero(); // Procedimento da equipe: partida sempre recolhido.
    }

    private static StatusCode retry(Supplier<StatusCode> action) {
        StatusCode result = action.get();
        for (int attempt = 1; attempt < CONFIG_ATTEMPTS && !result.isOK(); attempt++) {
            result = action.get();
        }
        return result;
    }

    @Override
    public boolean confirmRetractedZero() {
        if (!DriverStation.isDisabled()) return false;
        configured = false;
        referenced = false;
        connected = false;
        setVoltage(0);
        hardware.hasResetOccurred(); // Consome o reset anterior; novos resets serão detectados.
        StatusCode configuration = retry(() -> hardware.configure(activeGains, activeProfile));
        configured = configuration.isOK();
        appliedCruise = configured ? activeProfile.velocity() : Double.NaN;
        configurationStatus = configuration.toString();
        zeroStatus = "CONFIGURACAO_PENDENTE";
        if (configured) {
            StatusCode zero = retry(hardware::zeroPosition);
            referenced = zero.isOK();
            zeroStatus = zero.toString();
        }
        if (referenced) referenceLostOnReset = false;
        return referenced;
    }

    @Override
    public boolean isRealHardware() {
        return true;
    }

    @Override public boolean supportsMotionMagic() { return true; }

    @Override public boolean configureGains(TelescopioGains_9458 gains) {
        // Phoenix não oferece o mesmo clamp de integral do controlador WPILib.
        // Não habilitar integral sem uma estratégia explícita contra windup.
        if (!DriverStation.isDisabled() || gains.kI() != 0
                || gains.kS() > MAX_CONTROL_VOLTS || gains.kG() > MAX_CONTROL_VOLTS) return false;
        StatusCode result = retry(() -> hardware.configureGains(gains));
        if (!result.isOK()) {
            configured = false;
            configurationStatus = result.toString();
            setVoltage(0);
            return false;
        }
        activeGains = gains;
        return true;
    }

    @Override public boolean configureMotionProfile(TelescopioMotionProfile_9458.Limits limits) {
        if (!DriverStation.isDisabled()) return false;
        StatusCode result = retry(() -> hardware.configureMotionMagic(
                limits.velocity(), limits.acceleration(), limits.jerk(), CONFIG_TIMEOUT_SECONDS));
        if (!result.isOK()) {
            // O Talon aplica a configuração inteira ou nada: continua com o perfil anterior.
            configurationStatus = "PERFIL_RECUSADO: " + result;
            return false;
        }
        activeProfile = limits;
        appliedCruise = limits.velocity();
        failedCruise = Double.NaN;
        configurationStatus = "OK";
        return true;
    }

    // #endregion

    // #region LEITURAS DO MOTOR

    @Override
    public void updateInputs(TelescopioIOInputs_9458 inputs) {
        if (hardware.hasResetOccurred()) {
            referenced = false;
            configured = false;
            referenceLostOnReset = true;
            zeroStatus = "ZERO_PERDIDO_APOS_RESET";
            configurationStatus = "RECONFIGURACAO_NECESSARIA";
            setVoltage(0);
        }
        StatusCode read = hardware.read(inputs);
        connected = read.isOK();
        inputs.motorConnected = connected;
        inputs.positionValid = connected && configured && referenced;
        inputs.configurationApplied = configured;
        inputs.referenceEstablished = referenced;
        inputs.referenceLostOnReset = referenceLostOnReset;
        inputs.controlRequestOK = controlRequestOK;
        inputs.configurationStatus = configurationStatus;
        inputs.zeroStatus = zeroStatus;
        inputs.readStatus = read.toString();
        inputs.controlStatus = controlStatus;
        inputs.extensionMeters = inputs.rotorPositionRotations / MOTOR_ROTATIONS_PER_METER;
        inputs.velocityMetersPerSec = inputs.rotorVelocityRotationsPerSec / MOTOR_ROTATIONS_PER_METER;
        extensionMeters = inputs.extensionMeters;
        connected &= Double.isFinite(extensionMeters) && Double.isFinite(inputs.velocityMetersPerSec)
                && extensionMeters >= MIN_EXTENSION_METERS - POSITION_VALID_MARGIN_METERS
                && extensionMeters <= MAX_EXTENSION_METERS + POSITION_VALID_MARGIN_METERS;
        inputs.positionValid &= connected;
        if (!connected) setVoltage(0); // Interrompe um perfil que continuaria no Talon.
        inputs.motionMagicAvailable = true;
        inputs.motionMagicActive = motionMagicActive;
    }

    // #endregion

    // #region COMANDO DO MOTOR

    @Override
    public void setVoltage(double volts) {
        motionMagicActive = false;
        double output = configured && referenced && connected && !DriverStation.isDisabled()
                && Double.isFinite(volts)
                ? MathUtil.clamp(volts, -MAX_CONTROL_VOLTS, MAX_CONTROL_VOLTS) : 0.0;
        StatusCode result = hardware.setVoltage(output);
        controlRequestOK = result.isOK();
        controlStatus = result.toString();
    }

    @Override public void setMotionMagicPosition(double meters, double cruiseMetersPerSec) {
        if (!configured || !referenced || !connected || DriverStation.isDisabled()
                || !Double.isFinite(meters) || !(cruiseMetersPerSec > 0)) {
            setVoltage(0);
            return;
        }
        double target = MathUtil.clamp(meters, MIN_EXTENSION_METERS, MAX_OPERATING_EXTENSION_METERS);
        // Recolhido: apoia no batente sem tensão.
        if (target <= MIN_EXTENSION_METERS + GOAL_EPSILON_METERS
                && extensionMeters <= RETRACTED_REST_TOLERANCE_METERS) {
            setVoltage(0);
            return;
        }
        // Só escreve configuração quando o trecho muda (normal <-> lento perto dos batentes),
        // no máximo duas vezes por movimento. Falha aqui não é de segurança: o Talon segue
        // com o cruzeiro anterior e a curva em S para na meta. Fica no diagnóstico e só
        // tenta de novo quando o cruzeiro pedido mudar (sem insistir a cada ciclo).
        double cruise = Math.min(cruiseMetersPerSec, activeProfile.velocity());
        if (!Double.isNaN(failedCruise) && Math.abs(cruise - failedCruise) > 1e-9) {
            failedCruise = Double.NaN; // o pedido mudou: a próxima troca pode tentar de novo
        }
        boolean changed = Double.isNaN(appliedCruise) || Math.abs(cruise - appliedCruise) > 1e-9;
        boolean alreadyFailed = !Double.isNaN(failedCruise) && Math.abs(cruise - failedCruise) <= 1e-9;
        if (changed && !alreadyFailed) {
            StatusCode result = hardware.configureMotionMagic(
                    cruise, activeProfile.acceleration(), activeProfile.jerk(), MOTION_CONFIG_TIMEOUT_SECONDS);
            if (result.isOK()) {
                appliedCruise = cruise;
                failedCruise = Double.NaN;
            } else {
                failedCruise = cruise;
                configurationStatus = "TROCA_DE_TRECHO_RECUSADA: " + result;
            }
        }
        StatusCode result = hardware.setMotionMagic(target * MOTOR_ROTATIONS_PER_METER,
                extensionMeters >= UPPER_SOFT_LIMIT_METERS,
                extensionMeters <= MIN_EXTENSION_METERS);
        controlRequestOK = result.isOK();
        controlStatus = result.toString();
        motionMagicActive = result.isOK();
        if (!result.isOK()) {
            hardware.setVoltage(0); // Preserva o erro para o diagnóstico do próximo ciclo.
        }
    }

    @Override
    public void close() {
        stop();
        hardware.close();
    }

    // #endregion

    // #region FRONTEIRA CAN (SUBSTITUIDA NOS TESTES)

    /** Tudo que fala com o TalonFX passa por aqui; os testes injetam uma versão falsa. */
    interface Hardware {
        StatusCode configure(TelescopioGains_9458 gains, TelescopioMotionProfile_9458.Limits profile);
        StatusCode configureGains(TelescopioGains_9458 gains);
        /** Metros da ponta. timeoutSeconds > 0: a CTRE não aceita configuração com timeout 0. */
        StatusCode configureMotionMagic(double velocity, double acceleration, double jerk, double timeoutSeconds);
        StatusCode zeroPosition();
        boolean hasResetOccurred();
        StatusCode read(TelescopioIOInputs_9458 inputs);
        StatusCode setVoltage(double volts);
        StatusCode setMotionMagic(double rotorRotations, boolean limitForward, boolean limitReverse);
        void close();
    }

    static class TalonHardware implements Hardware {
        private final TalonFX motor = new TalonFX(MOTOR_CAN_ID);
        private final StatusSignal<Angle> position = motor.getPosition();
        private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
        private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
        private final StatusSignal<Current> current = motor.getStatorCurrent();
        private final StatusSignal<Boolean> currentLimit = motor.getFault_StatorCurrLimit();
        final TalonFX getMotor() { return motor; }
        private final StatusSignal<Double> profilePosition = motor.getClosedLoopReference();
        private final StatusSignal<Double> profileVelocity = motor.getClosedLoopReferenceSlope();
        private final StatusSignal<Double> loopOutput = motor.getClosedLoopOutput();
        private final StatusSignal<Double> loopFeedforward = motor.getClosedLoopFeedForward();
        private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false).withUpdateFreqHz(0);
        private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0)
                .withEnableFOC(false).withSlot(0).withUpdateFreqHz(0)
                .withIgnoreHardwareLimits(false).withIgnoreSoftwareLimits(false);

        @Override public StatusCode configure(TelescopioGains_9458 gains, TelescopioMotionProfile_9458.Limits profile) {
            motor.setExpiration(MOTOR_SAFETY_EXPIRATION_SECONDS);
            motor.setSafetyEnabled(true);
            var config = createConfiguration();
            config.Slot0 = createSlot(gains);
            config.MotionMagic = createMotionMagic(profile.velocity(), profile.acceleration(), profile.jerk());
            StatusCode result = motor.getConfigurator().apply(config);
            return result.isOK() ? BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ,
                    position, velocity, voltage, current, currentLimit,
                    profilePosition, profileVelocity, loopOutput, loopFeedforward) : result;
        }

        @Override public StatusCode configureGains(TelescopioGains_9458 gains) {
            return motor.getConfigurator().apply(createSlot(gains));
        }

        @Override public StatusCode configureMotionMagic(double velocity, double acceleration, double jerk,
                double timeoutSeconds) {
            return motor.getConfigurator().apply(createMotionMagic(velocity, acceleration, jerk), timeoutSeconds);
        }

        @Override public StatusCode zeroPosition() {
            return motor.setPosition(POWER_ON_EXTENSION_METERS * MOTOR_ROTATIONS_PER_METER);
        }

        @Override public boolean hasResetOccurred() { return motor.hasResetOccurred(); }

        @Override public StatusCode read(TelescopioIOInputs_9458 inputs) {
            StatusCode result = BaseStatusSignal.refreshAll(position, velocity, voltage, current, currentLimit,
                    profilePosition, profileVelocity, loopOutput, loopFeedforward);
            inputs.rotorPositionRotations = position.getValueAsDouble();
            inputs.rotorVelocityRotationsPerSec = velocity.getValueAsDouble();
            inputs.appliedVolts = voltage.getValueAsDouble();
            inputs.motorCurrentAmps = current.getValueAsDouble();
            inputs.statorCurrentLimitActive = currentLimit.getValue();
            inputs.closedLoopReferenceMeters = profilePosition.getValueAsDouble() / MOTOR_ROTATIONS_PER_METER;
            inputs.closedLoopReferenceVelocityMetersPerSec = profileVelocity.getValueAsDouble() / MOTOR_ROTATIONS_PER_METER;
            inputs.closedLoopOutputVolts = loopOutput.getValueAsDouble();
            inputs.closedLoopFeedforwardVolts = loopFeedforward.getValueAsDouble();
            if (result.isOK() && (position.getTimestamp().getLatency() > SIGNAL_MAX_AGE_SECONDS
                    || velocity.getTimestamp().getLatency() > SIGNAL_MAX_AGE_SECONDS
                    || profilePosition.getTimestamp().getLatency() > SIGNAL_MAX_AGE_SECONDS
                    || profileVelocity.getTimestamp().getLatency() > SIGNAL_MAX_AGE_SECONDS)) {
                return StatusCode.RxTimeout;
            }
            return result;
        }

        @Override public StatusCode setVoltage(double volts) {
            return motor.setControl(voltageRequest.withOutput(volts));
        }

        @Override public StatusCode setMotionMagic(double rotorRotations, boolean limitForward, boolean limitReverse) {
            return motor.setControl(positionRequest.withPosition(rotorRotations)
                    .withLimitForwardMotion(limitForward).withLimitReverseMotion(limitReverse));
        }

        @Override public void close() { motor.close(); }
    }

    // #endregion

    // #region CONFIGURACAO DO TALONFX (PHOENIX 6)

    /** Proteções do Kraken X60. Os soft limits contam a partir do zero feito ao ligar. */
    public static TalonFXConfiguration createConfiguration() {
        var config = new TalonFXConfiguration();
        // A conversão para metros é feita no roboRIO, uma única vez.
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Feedback.RotorToSensorRatio = 1.0;

        // #region SENTIDO E MODO NEUTRO

        config.MotorOutput.Inverted = MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        // Brake: ao desabilitar, o motor freia em vez de deixar o telescópio cair livre.
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // #endregion

        // #region 🛑⚡ LIMITES DE CORRENTE

        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME_SECONDS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // O teto agora precisa existir no dispositivo: o PID roda dentro dele.
        config.Voltage.PeakForwardVoltage = MAX_CONTROL_VOLTS;
        config.Voltage.PeakReverseVoltage = -MAX_CONTROL_VOLTS;
        config.Slot0 = createSlot(REAL_GAINS);
        config.MotionMagic = createMotionMagic(MOTION_MAGIC_VELOCITY_METERS_PER_SEC,
                MOTION_MAGIC_ACCELERATION_METERS_PER_SEC_SQ, MOTION_MAGIC_JERK_METERS_PER_SEC_CUBED);
        // A equipe ainda não instalou os sensores; não declarar homing automático.
        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;

        // #endregion

        // #region 🛑📏 LIMITES DE CURSO (SOFT LIMITS)

        // Em voltas do rotor (SensorToMechanismRatio = 1, padrão): 78,4 no fim de curso.
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            UPPER_SOFT_LIMIT_METERS * MOTOR_ROTATIONS_PER_METER;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            MIN_EXTENSION_METERS * MOTOR_ROTATIONS_PER_METER;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // #endregion

        return config;
    }

    /** SI na ponta -> voltas do rotor. kG e kS são tensões e não mudam. */
    static Slot0Configs createSlot(TelescopioGains_9458 gains) {
        var slot = new Slot0Configs();
        slot.kP = gains.kP() / MOTOR_ROTATIONS_PER_METER;
        slot.kI = gains.kI() / MOTOR_ROTATIONS_PER_METER;
        slot.kD = gains.kD() / MOTOR_ROTATIONS_PER_METER;
        slot.kS = gains.kS();
        slot.kG = gains.kG();
        slot.kV = gains.kV() / MOTOR_ROTATIONS_PER_METER;
        slot.kA = gains.kA() / MOTOR_ROTATIONS_PER_METER;
        slot.GravityType = GravityTypeValue.Elevator_Static;
        slot.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        return slot;
    }

    /** Metros da ponta -> voltas do rotor por segundo (e por s², s³). */
    static MotionMagicConfigs createMotionMagic(double velocity, double acceleration, double jerk) {
        var profile = new MotionMagicConfigs();
        profile.MotionMagicCruiseVelocity = velocity * MOTOR_ROTATIONS_PER_METER;
        profile.MotionMagicAcceleration = acceleration * MOTOR_ROTATIONS_PER_METER;
        profile.MotionMagicJerk = jerk * MOTOR_ROTATIONS_PER_METER;
        return profile;
    }

    // #endregion
}
