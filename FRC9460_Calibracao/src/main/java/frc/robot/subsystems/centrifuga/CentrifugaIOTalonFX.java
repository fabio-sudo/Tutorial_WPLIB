package frc.robot.subsystems.centrifuga;

// #region IMPORTS

import static frc.robot.subsystems.centrifuga.CentrifugaConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

// #endregion

/** Centrífuga CAN 36: malha de velocidade com aceleração perfilada no TalonFX. */
public class CentrifugaIOTalonFX implements CentrifugaIO {

    // #region MOTOR E SINAIS

    private static final double SIGNAL_UPDATE_HZ = 50.0;

    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0.0).withEnableFOC(false).withUpdateFreqHz(50.0);
    private final MotionMagicVelocityVoltage profiledRequest =
        new MotionMagicVelocityVoltage(0.0).withEnableFOC(false).withUpdateFreqHz(50.0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);

    private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    private final StatusSignal<Angle> position = motor.getPosition();
    private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
    private final StatusSignal<Current> current = motor.getStatorCurrent();
    private final StatusSignal<Double> reference = motor.getClosedLoopReference();
    private boolean configured;
    private int resets;
    private boolean measurementValid;
    private String configurationStatus = "NAO_CONFIGURADO";
    private StatusCode controlStatus = StatusCode.OK;
    // Teste sem trava ativo: a leitura inválida não corta a saída.
    private boolean openLoopActive;

    // #endregion

    // #region CONFIGURAÇÃO

    public CentrifugaIOTalonFX() {
        TalonFXConfiguration config = createConfiguration();
        StatusCode configResult = tryUntilOk(() -> motor.getConfigurator().apply(config));
        configured = configResult.isOK();
        configurationStatus = configResult.toString();
        reportIfFailed(configResult, "Centrífuga: configuração do TalonFX " + MOTOR_ID);
        applySignalRates("Centrífuga", () -> BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ,
            velocity, position, voltage, current, reference), motor);
        motor.hasResetOccurred();
        motor.setExpiration(MOTOR_SAFETY_SECONDS);
        motor.setSafetyEnabled(false);
    }

    /** Configuração da competição, separada por assunto. */
    public static TalonFXConfiguration createConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // #region MOTOR E SENTIDO

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        // #endregion

        // #region 🛑⚡ SEGURANÇA - LIMITES DE CORRENTE

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;

        // #endregion

        // #region 🎯🚀 CONTROLE - PID DE VELOCIDADE (SLOT0: kP, kS, kV)

        config.Slot0.kP = KP;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION_ROTATIONS_PER_SEC2;
        config.MotionMagic.MotionMagicJerk = 0.0;

        // #endregion

        return config;
    }

    // #endregion

    // #region LEITURAS E COMANDOS

    @Override
    public void updateInputs(CentrifugaIOInputs inputs) {
        StatusCode read = BaseStatusSignal.refreshAll(velocity, position, voltage, current, reference);
        // Reinício do Talon (queda de bateria, por exemplo). Antes a centrífuga
        // ficava recusando todo pedido até reiniciar o código, sem aviso: o B não
        // alimentava e só o teste sem trava (Start) girava. O Phoenix 6 guarda a
        // configuração no próprio Talon; só as frequências dos sinais voltam ao
        // padrão. A centrífuga não depende de zero: para neste ciclo e segue.
        if (motor.hasResetOccurred()) {
            cutOutput();
            BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ, velocity, position, voltage, current, reference);
            resets++;
            configurationStatus = "REINICIOU_E_RETOMOU_" + resets;
            DriverStation.reportWarning("[FRC9460] Centrífuga: TalonFX " + MOTOR_ID
                + " reiniciou. Retomando com a configuração gravada nele.", false);
        }
        measurementValid = configured && read.isOK()
            && velocity.getTimestamp().getLatency() <= MAX_SIGNAL_AGE_SECONDS
            && Double.isFinite(velocity.getValueAsDouble())
            && Double.isFinite(voltage.getValueAsDouble())
            && Double.isFinite(current.getValueAsDouble());
        if (!measurementValid && !openLoopActive) cutOutput();
        inputs.connected = measurementValid;
        inputs.velocityRpm = velocity.getValueAsDouble() * 60.0;
        inputs.rotorPositionRotations = position.getValueAsDouble();
        inputs.positionValid = measurementValid && position.getStatus().isOK()
            && position.getTimestamp().getLatency() <= MAX_SIGNAL_AGE_SECONDS
            && Double.isFinite(inputs.rotorPositionRotations);
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.referenceRpm = reference.getValueAsDouble() * 60.0;
        inputs.configurationApplied = configured;
        inputs.configurationStatus = configurationStatus;
        inputs.controlRequestOK = controlStatus.isOK();
        inputs.controlStatus = controlStatus.toString();
    }

    @Override
    public void setVelocityRpm(double rpm) {
        if (!configured || !measurementValid || DriverStation.isDisabled() || !Double.isFinite(rpm)) {
            stop();
            return;
        }
        openLoopActive = false;
        double limitedRpm = MathUtil.clamp(rpm, -UNJAM_ROTOR_RPM, MAX_COMMAND_RPM);
        motor.setSafetyEnabled(true);
        // Para frear ao soltar, 0 RPM usa VelocityVoltage sem esperar uma
        // rampa de descida. A partida e os novos alvos usam perfil no Talon.
        controlStatus = limitedRpm == 0.0
            ? motor.setControl(velocityRequest.withVelocity(0.0))
            : motor.setControl(profiledRequest.withVelocity(limitedRpm / 60.0));
        if (!controlStatus.isOK()) cutOutput();
    }

    /**
     * Parar também é um pedido ao Talon, e o resultado fica guardado.
     * Depois de uma falha, o subsistema só chama stop(): o primeiro stop
     * aceito libera o RPM no ciclo seguinte. Antes o stop não guardava o
     * resultado e uma única falha de CAN deixava a centrífuga parada até
     * reiniciar o código (sem aviso).
     */
    @Override
    public void stop() {
        openLoopActive = false;
        motor.setSafetyEnabled(false);
        controlStatus = motor.setControl(neutralRequest);
    }

    /**
     * Teste sem trava: saída direta, como no Tuner X, sem
     * conferir configuração nem sensores. Só não gira com o robô desabilitado
     * (o próprio Talon bloqueia) e só para a frente. O MotorSafety continua:
     * se o código parar de mandar, o motor para em MOTOR_SAFETY_SECONDS.
     */
    @Override
    public void setPercentOutput(double percent) {
        if (DriverStation.isDisabled() || !Double.isFinite(percent)) {
            stop();
            return;
        }
        openLoopActive = true;
        motor.setSafetyEnabled(true);
        controlStatus = motor.setControl(percentRequest.withOutput(MathUtil.clamp(percent, 0.0, 1.0)));
    }

    /** Corta a saída sem trocar o resultado guardado: a falha aparece no próximo ciclo. */
    private void cutOutput() {
        motor.setSafetyEnabled(false);
        motor.stopMotor();
    }

    // #endregion
}
