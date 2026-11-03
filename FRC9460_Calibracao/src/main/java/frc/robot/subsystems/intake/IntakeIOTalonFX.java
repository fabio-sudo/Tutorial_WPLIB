package frc.robot.subsystems.intake;

// #region IMPORTS

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.util.TalonZeroMark;

// #endregion

/**
 * Articulador real: TalonFX 34 (líder) e 35 (segue invertido).
 * Mesma configuração da competição: Brake, limite de corrente, soft limits
 * no fechado e no aberto, e PID só para segurar a posição.
 */
public class IntakeIOTalonFX implements IntakeIO {

    // #region MOTORES E SINAIS

    private static final double SIGNAL_UPDATE_HZ = 50.0;
    private static final double STARTUP_READ_TIMEOUT_SECONDS = 0.1;
    // 📊 Falha de leitura mais curta que isto não vira alerta: com o CAN cheio,
    // um pacote atrasado geraria um aviso a cada oscilação.
    private static final double CONNECTION_DEBOUNCE_SECONDS = 0.5;

    private final TalonFX leader = new TalonFX(LEFT_MOTOR_ID);
    private final TalonFX follower = new TalonFX(RIGHT_MOTOR_ID);
    private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);
    private final PositionVoltage holdRequest = new PositionVoltage(0.0);
    // O direito espelha o esquerdo (montado do outro lado do eixo).
    private final Follower followRequest = new Follower(LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);

    private final StatusSignal<Angle> position = leader.getPosition();
    private final StatusSignal<AngularVelocity> velocity = leader.getVelocity();
    private final StatusSignal<Voltage> voltage = leader.getMotorVoltage();
    private final StatusSignal<Double> dutyCycle = leader.getDutyCycle();
    private final StatusSignal<Current> current = leader.getStatorCurrent();
    private final StatusSignal<Boolean> closeLimit = leader.getFault_ForwardSoftLimit();
    private final StatusSignal<Boolean> openLimit = leader.getFault_ReverseSoftLimit();

    private final boolean configured;
    private boolean zeroValid;
    private String startupZero = "NAO_AVALIADO";
    private int followerResets;
    private final Debouncer followerConnected =
            new Debouncer(CONNECTION_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);

    // #endregion

    // #region 🧭 CONFIGURAÇÃO E ZERO AO LIGAR

    public IntakeIOTalonFX() {
        // Antes era uma tentativa só: se expirasse na partida, o zero ficava
        // inválido e o intake recusava mexer (só descia pelo peso).
        TalonFXConfiguration leaderConfig = createLeaderConfiguration();
        TalonFXConfiguration followerConfig = createFollowerConfiguration();
        StatusCode leaderResult = tryUntilOk(() -> leader.getConfigurator().apply(leaderConfig));
        StatusCode followerResult = tryUntilOk(() -> follower.getConfigurator().apply(followerConfig));
        configured = leaderResult.isOK() && followerResult.isOK();
        reportIfFailed(leaderResult, "Intake: configuração do TalonFX " + LEFT_MOTOR_ID);
        reportIfFailed(followerResult, "Intake: configuração do TalonFX " + RIGHT_MOTOR_ID);

        follower.setControl(followRequest);
        // voltage e dutyCycle não são só leitura: o seguidor copia do líder o sinal
        // do modo em uso (PositionVoltage e DutyCycleOut), e pede pelo menos 20 Hz.
        applySignalRates("Intake", () -> BaseStatusSignal.setUpdateFrequencyForAll(SIGNAL_UPDATE_HZ,
                position, velocity, voltage, dutyCycle, current, closeLimit, openLimit), leader, follower);

        leader.hasResetOccurred();
        follower.hasResetOccurred();
        // Talon recém-ligado: o intake foi fechado à mão, o fechado é aqui.
        // Só o código reiniciou (deploy): mantém o zero, mesmo com o intake aberto.
        // Antes zerava sempre, e um deploy com o intake aberto fazia o "fechado"
        // ficar no meio do curso (frc.robot.util.TalonZeroMark).
        zeroOnStartup();
        stop(); // O subsistema captura o alvo ao habilitar, não aqui.
    }

    private void zeroOnStartup() {
        StatusCode read = position.waitForUpdate(STARTUP_READ_TIMEOUT_SECONDS).getStatus();
        TalonZeroMark.Decision decision = configured && read.isOK()
                ? TalonZeroMark.onStartup(position.getValueAsDouble(), MECHANICAL_OPEN_ROTATIONS, CLOSED_ROTATIONS)
                : TalonZeroMark.Decision.POSICAO_DESCONHECIDA;
        startupZero = decision.name();
        switch (decision) {
            case ZERAR_AQUI -> zeroClosed();
            case MANTER_ZERO -> zeroValid = true;
            case POSICAO_DESCONHECIDA -> {
                zeroValid = false;
                if (configured) {
                    reportIfFailed(StatusCode.GeneralError, "Intake: posição ao iniciar "
                            + "(feche à mão e zere com Back + Y do operador, desabilitado)");
                }
            }
        }
    }

    @Override
    public void zeroClosed() {
        double closed = TalonZeroMark.toTalon(CLOSED_ROTATIONS);
        zeroValid = configured
                && tryUntilOk(() -> leader.setPosition(closed)).isOK()
                && tryUntilOk(() -> follower.setPosition(closed)).isOK();
        if (configured && !zeroValid) {
            reportIfFailed(StatusCode.GeneralError, "Intake: zerar os encoders (fechado)");
        }
    }

    // #endregion

    // #region CONFIGURAÇÃO DOS TALONFX (PHOENIX 6)

    /** Líder: motor, segurança (corrente e soft limits) e o PID de segurar. */
    public static TalonFXConfiguration createLeaderConfiguration() {
        return new TalonFXConfiguration()
                .withMotorOutput(motorOutput())
                .withCurrentLimits(currentLimits())
                .withSoftwareLimitSwitch(softLimits())
                .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(HOLD_MAX_VOLTS)
                        .withPeakReverseVoltage(-HOLD_MAX_VOLTS))
                .withSlot0(holdGains());
    }

    /** Seguidor: só motor e corrente; os limites e o PID ficam no líder. */
    public static TalonFXConfiguration createFollowerConfiguration() {
        return new TalonFXConfiguration()
                .withMotorOutput(motorOutput())
                .withCurrentLimits(currentLimits());
    }

    // #region MOTOR

    private static MotorOutputConfigs motorOutput() {
        return new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withDutyCycleNeutralDeadband(HOLD_NEUTRAL_DEADBAND);
    }

    // #endregion

    // #region 🛑⚡ SEGURANÇA - LIMITES DE CORRENTE E SOFT LIMITS

    private static CurrentLimitsConfigs currentLimits() {
        return new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS).withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(CURRENT_LIMIT_AMPS).withStatorCurrentLimitEnable(true);
    }

    /**
     * Fechado e aberto são também os soft limits do líder (somados à marca do
     * zero).
     */
    private static SoftwareLimitSwitchConfigs softLimits() {
        return new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(CLOSE_LIMIT_ENABLED)
                .withForwardSoftLimitThreshold(TalonZeroMark.toTalon(CLOSED_ROTATIONS))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(TalonZeroMark.toTalon(OPEN_ROTATIONS));
    }

    // #endregion

    // #region 🎯 CONTROLE - PID DE SEGURAR A POSIÇÃO (SLOT0 COM kG)

    /**
     * Ganhos usados só para segurar parado. A gravidade puxa sempre
     * para o mesmo lado (abrir), então um kG constante basta.
     */
    private static Slot0Configs holdGains() {
        return new Slot0Configs()
                .withKP(HOLD_KP).withKI(HOLD_KI).withKD(HOLD_KD).withKG(HOLD_KG)
                .withGravityType(GravityTypeValue.Elevator_Static);
    }

    // #endregion

    // #endregion

    // #region LEITURAS

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        StatusCode read = BaseStatusSignal.refreshAll(position, velocity, voltage, current, closeLimit, openLimit);
        if (leader.hasResetOccurred()) {
            zeroValid = false;
            leader.stopMotor();
            reportReset("Intake (TalonFX " + LEFT_MOTOR_ID + ")");
        }
        // O seguidor reiniciado fica parado (Brake) e o líder empurraria o eixo
        // sozinho contra ele. Para neste ciclo e volta a mandar seguir; o zero
        // é do líder, então continua valendo.
        if (follower.hasResetOccurred()) {
            leader.stopMotor();
            follower.setControl(followRequest);
            followerResets++;
            DriverStation.reportWarning("[FRC9460] Intake: TalonFX " + RIGHT_MOTOR_ID
                    + " (seguidor) reiniciou e voltou a seguir o " + LEFT_MOTOR_ID + ".", false);
        }
        inputs.connected = read.isOK();
        // 📊 Só avisa: o intake segue recebendo pedidos, com o líder empurrando sozinho.
        inputs.followerConnected = followerConnected.calculate(follower.isConnected());
        inputs.zeroValid = zeroValid;
        inputs.startupZero = startupZero;
        inputs.followerResets = followerResets;
        inputs.positionRotations = TalonZeroMark.toMechanism(position.getValueAsDouble());
        inputs.velocityRotationsPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.closeLimitActive = closeLimit.getValue();
        inputs.openLimitActive = openLimit.getValue();
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setPercentOutput(double percent) {
        if (!zeroValid || !Double.isFinite(percent)) {
            stop();
            return;
        }
        leader.setControl(percentRequest.withOutput(MathUtil.clamp(percent, -1.0, 1.0)));
    }

    @Override
    public void holdPosition(double rotations) {
        if (!zeroValid || !Double.isFinite(rotations)) {
            stop();
            return;
        }
        leader.setControl(holdRequest.withPosition(TalonZeroMark.toTalon(rotations)));
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }

    // #endregion
}
