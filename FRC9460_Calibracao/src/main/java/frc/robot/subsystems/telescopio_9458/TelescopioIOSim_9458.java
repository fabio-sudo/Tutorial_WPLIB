package frc.robot.subsystems.telescopio_9458;

// #region IMPORTS

import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import frc.robot.subsystems.telescopio_9458.TelescopioMotionProfile_9458.Limits;
import frc.robot.subsystems.telescopio_9458.TelescopioMotionProfile_9458.State;

// #endregion

/**
 * Massa concentrada na ponta; a cascata entra no raio equivalente. Sensores ideais.
 * Emula o Motion Magic do TalonFX: referência em curva em S e PID/feedforward a 1 kHz.
 */
public class TelescopioIOSim_9458 implements TelescopioIO_9458 {

    // #region MODELO FISICO

    // Passo interno da integração; menor que a constante de tempo do motor (~2 ms).
    private static final double SUBSTEP_SECONDS = 0.001;
    private static final double STOPPED_METERS_PER_SEC = 1e-6;
    private static final double GRAVITY = 9.8;

    // ElevatorSim com integração própria (exata) e atrito. O rkdp da WPILib 2026 soma
    // ao tempo decorrido o tamanho do PRÓXIMO passo: neste sistema rápido perdia ~9% de
    // cada ciclo, com a velocidade certa e a posição andando menos que o real.
    private final ElevatorSim sim = new ElevatorSim(
            SIM_MOTOR, GEAR_REDUCTION, SIM_INITIAL_MOVING_MASS_KG,
            SIM_EFFECTIVE_DRUM_RADIUS_METERS, MIN_EXTENSION_METERS,
            MAX_EXTENSION_METERS, true, MIN_EXTENSION_METERS) {
        @Override
        protected Matrix<N2, N1> updateX(Matrix<N2, N1> x, Matrix<N1, N1> u, double dtSeconds) {
            double[] next = integrate(x.get(0, 0), x.get(1, 0), u.get(0, 0), dtSeconds,
                    m_plant.getA().get(1, 1), m_plant.getB().get(1, 0));
            return VecBuilder.fill(next[0], next[1]);
        }
    };
    private double requestedVolts;
    private boolean measuredVoltage;

    // #endregion

    // #region MOTION MAGIC EMULADO

    private boolean motionMagicActive;
    private double motionMagicTarget;
    private double motionMagicCruise;
    private final TelescopioMotionProfile_9458.Generator generator =
            new TelescopioMotionProfile_9458.Generator(SUBSTEP_SECONDS);
    private Limits motionProfile = MOTION_MAGIC_LIMITS;
    private TelescopioGains_9458 gains = SIM_GAINS;
    private double closedLoopOutputVolts;
    private double closedLoopFeedforwardVolts;
    private boolean currentLimited;

    // #endregion

    // #region COMANDO DO MOTOR

    @Override
    public void setVoltage(double volts) {
        requestedVolts = Double.isFinite(volts) ? volts : 0.0;
        measuredVoltage = false;
        motionMagicActive = false;
    }

    /** O Talon já limitou esta tensão (corrente e soft limits); o modelo não deve limitá-la novamente. */
    public void setMeasuredVoltage(double volts) {
        requestedVolts = Double.isFinite(volts) ? volts : 0.0;
        measuredVoltage = true;
        motionMagicActive = false;
    }

    @Override
    public boolean supportsMotionMagic() {
        return true;
    }

    /** Mesmas regras do TalonFX real: meta limitada ao curso e apoio sem tensão no fundo. */
    @Override
    public void setMotionMagicPosition(double meters, double cruiseMetersPerSec) {
        if (!Double.isFinite(meters) || !(cruiseMetersPerSec > 0) || DriverStation.isDisabled()) {
            setVoltage(0);
            return;
        }
        double target = MathUtil.clamp(meters, MIN_EXTENSION_METERS, MAX_OPERATING_EXTENSION_METERS);
        if (target <= MIN_EXTENSION_METERS + GOAL_EPSILON_METERS
                && sim.getPositionMeters() <= RETRACTED_REST_TOLERANCE_METERS) {
            setVoltage(0);
            return;
        }
        if (!motionMagicActive) {
            // Como o Talon: o perfil parte da posição e da velocidade medidas.
            generator.reset(sim.getPositionMeters(), sim.getVelocityMetersPerSecond(), motionProfile);
        }
        motionMagicActive = true;
        measuredVoltage = false;
        motionMagicTarget = target;
        motionMagicCruise = Math.min(cruiseMetersPerSec, motionProfile.velocity());
    }

    @Override
    public boolean configureGains(TelescopioGains_9458 newGains) {
        gains = newGains;
        return true;
    }

    @Override
    public boolean configureMotionProfile(Limits limits) {
        motionProfile = limits;
        return true;
    }

    @Override
    public boolean confirmRetractedZero() {
        if (!DriverStation.isDisabled()) return false;
        setVoltage(0);
        setState(MIN_EXTENSION_METERS, 0);
        return true;
    }

    // #endregion

    // #region SINCRONIZACAO COM O ROBO REAL

    /** Coloca o modelo na posição medida; usado pelo modelo em paralelo do Telescopio_9458. */
    public void setState(double extensionMeters, double velocityMetersPerSec) {
        sim.setState(extensionMeters, velocityMetersPerSec);
    }

    // #endregion

    // #region ATUALIZACAO DAS LEITURAS

    @Override
    public void updateInputs(TelescopioIOInputs_9458 inputs) {

        // Disabled corta a tensão, mas continua calculando a queda pela gravidade.
        if (DriverStation.isDisabled()) {
            requestedVolts = 0.0;
            motionMagicActive = false;
        }

        double appliedVolts = motionMagicActive ? stepMotionMagic() : stepVoltage();

        // #region SENSORES IDEAIS

        inputs.motorConnected = true;
        inputs.positionValid = true; // O zero virtual é conhecido desde o início.
        inputs.configurationApplied = true;
        inputs.referenceEstablished = true;
        inputs.configurationStatus = "SIMULACAO";
        inputs.zeroStatus = "SIMULACAO";
        inputs.readStatus = "OK";
        inputs.controlStatus = "OK";
        inputs.extensionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
        inputs.rotorPositionRotations = inputs.extensionMeters * MOTOR_ROTATIONS_PER_METER;
        inputs.rotorVelocityRotationsPerSec = inputs.velocityMetersPerSec * MOTOR_ROTATIONS_PER_METER;
        inputs.appliedVolts = appliedVolts;
        double motorVelocity = inputs.velocityMetersPerSec * GEAR_REDUCTION
                / SIM_EFFECTIVE_DRUM_RADIUS_METERS;
        inputs.motorCurrentAmps = Math.abs(SIM_MOTOR.getCurrent(motorVelocity, appliedVolts));
        inputs.statorCurrentLimitActive = currentLimited;

        inputs.motionMagicAvailable = supportsMotionMagic();
        inputs.motionMagicActive = motionMagicActive;
        inputs.closedLoopReferenceMeters = motionMagicActive ? generator.reference().position() : 0;
        inputs.closedLoopReferenceVelocityMetersPerSec = motionMagicActive ? generator.reference().velocity() : 0;
        inputs.closedLoopOutputVolts = closedLoopOutputVolts;
        inputs.closedLoopFeedforwardVolts = closedLoopFeedforwardVolts;

        // #endregion
    }

    /** Tensão fixa por 20 ms (VoltageOut, SysId ou modelo em paralelo). */
    private double stepVoltage() {
        closedLoopOutputVolts = 0;
        closedLoopFeedforwardVolts = 0;
        double appliedVolts = measuredVoltage ? requestedVolts : talonOutput(requestedVolts);
        if (measuredVoltage) currentLimited = false;
        sim.setInputVoltage(appliedVolts);
        sim.update(LOOP_PERIOD_SECONDS); // Atualizar apenas aqui, uma vez por ciclo.
        return appliedVolts;
    }

    /** Motion Magic: referência em curva em S e PID/feedforward a cada 1 ms, como no Talon. */
    private double stepMotionMagic() {
        var limits = new Limits(motionMagicCruise, motionProfile.acceleration(), motionProfile.jerk());
        int steps = (int) Math.round(LOOP_PERIOD_SECONDS / SUBSTEP_SECONDS);
        double sum = 0;
        for (int i = 0; i < steps; i++) {
            State reference = generator.step(motionMagicTarget, limits);
            closedLoopFeedforwardVolts = gains.feedforward(reference.velocity(), reference.acceleration());
            double output = closedLoopFeedforwardVolts
                    + gains.kP() * (reference.position() - sim.getPositionMeters())
                    + gains.kD() * (reference.velocity() - sim.getVelocityMetersPerSecond());
            closedLoopOutputVolts = MathUtil.clamp(output, -MAX_CONTROL_VOLTS, MAX_CONTROL_VOLTS);
            double applied = talonOutput(closedLoopOutputVolts);
            sim.setInputVoltage(applied);
            sim.update(SUBSTEP_SECONDS);
            sum += applied;
        }
        // Média do ciclo: é o que a leitura de 20 ms mostraria.
        return sum / steps;
    }

    /** Bateria, soft limits e limite de corrente do estator, como no TalonFX. */
    private double talonOutput(double volts) {
        double batteryVolts = Math.max(0.0, RobotController.getBatteryVoltage());
        double batteryLimitedVolts = MathUtil.clamp(volts, -batteryVolts, batteryVolts);
        // Soft limits do TalonFX (createConfiguration): abrir é cortado a partir de
        // UPPER_SOFT_LIMIT_METERS (0,956 m) e fechar a partir de MIN_EXTENSION_METERS.
        double outputVolts = softLimit(batteryLimitedVolts, sim.getPositionMeters());
        // Limite de corrente do estator: V = R·I + ω/Kv.
        // 0 V é saída neutra: não há tensão para o limite reduzir.
        double backEmfVolts = sim.getVelocityMetersPerSecond() * GEAR_REDUCTION
                / SIM_EFFECTIVE_DRUM_RADIUS_METERS / SIM_MOTOR.KvRadPerSecPerVolt;
        double currentLimitVolts = STATOR_CURRENT_LIMIT_AMPS * SIM_MOTOR.rOhms;
        double appliedVolts = outputVolts == 0 ? 0
                : MathUtil.clamp(outputVolts,
                        backEmfVolts - currentLimitVolts, backEmfVolts + currentLimitVolts);
        currentLimited = appliedVolts != outputVolts;
        return appliedVolts;
    }

    // #endregion

    // #region INTEGRACAO DA FISICA

    /**
     * Passo exato de v' = a·v + b·(V − atrito) − g em subpassos de 1 ms, com atrito de
     * Coulomb (SIM_KS, medido no SysId) e os batentes. Parado, o atrito segura enquanto
     * |V − kG| ≤ kS. a e b vêm do modelo do motor (ElevatorSim).
     */
    static double[] integrate(double position, double velocity, double volts,
            double dtSeconds, double a, double b) {

        double gravityVolts = GRAVITY / b;
        int steps = Math.max(1, (int) Math.round(dtSeconds / SUBSTEP_SECONDS));
        double h = dtSeconds / steps;
        double decay = Math.exp(a * h);

        for (int i = 0; i < steps; i++) {
            boolean stopped = Math.abs(velocity) < STOPPED_METERS_PER_SEC;
            if (stopped && Math.abs(volts - gravityVolts) <= SIM_KS) {
                velocity = 0;
                continue;
            }
            double direction = Math.signum(stopped ? volts - gravityVolts : velocity);
            double c = b * (volts - SIM_KS * direction) - GRAVITY; // v' = a·v + c
            double steady = -c / a;
            double next = steady + (velocity - steady) * decay;
            position += steady * h + (velocity - steady) * (decay - 1) / a;
            // O atrito freia, mas não inverte o movimento: ao cruzar zero, para.
            velocity = !stopped && Math.signum(next) == -direction ? 0 : next;

            if (position <= MIN_EXTENSION_METERS && velocity <= 0) {
                position = MIN_EXTENSION_METERS;
                velocity = 0;
            } else if (position >= MAX_EXTENSION_METERS && velocity >= 0) {
                position = MAX_EXTENSION_METERS;
                velocity = 0;
            }
        }
        return new double[] {position, velocity};
    }

    // #endregion

    // #region SOFT LIMITS DO TALONFX

    /**
     * Como o TalonFX: além do limite, a saída naquele sentido vira neutra (0 V);
     * no sentido contrário continua liberada.
     */
    static double softLimit(double volts, double extensionMeters) {
        if ((volts > 0 && extensionMeters >= UPPER_SOFT_LIMIT_METERS)
                || (volts < 0 && extensionMeters <= MIN_EXTENSION_METERS)) {
            return 0;
        }
        return volts;
    }

    // #endregion
}
