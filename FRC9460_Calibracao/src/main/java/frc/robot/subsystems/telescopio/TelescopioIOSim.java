package frc.robot.subsystems.telescopio;

// #region IMPORTS

import static frc.robot.subsystems.telescopio.TelescopioConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import frc.robot.util.MechanismModel;

// #endregion

/**
 * Massa concentrada na ponta e transmissão equivalente; sensores ideais.
 * Também é o modelo em paralelo do telescópio (frc.robot.util.ParallelModel).
 */
public class TelescopioIOSim implements TelescopioIO, MechanismModel {

    // #region 🧪 MODELO FISICO

    private final ElevatorSim sim = new ElevatorSim(
            SIM_MOTOR, GEAR_REDUCTION, SIM_INITIAL_MOVING_MASS_KG,
            SIM_EFFECTIVE_DRUM_RADIUS_METERS, MIN_EXTENSION_METERS,
            MAX_EXTENSION_METERS, true, MIN_EXTENSION_METERS);
    private double requestedVolts;
    private double modelVolts;

    // #endregion

    // #region COMANDO DO MOTOR

    @Override
    public void setVoltage(double volts) {
        requestedVolts = Double.isFinite(volts) ? volts : 0.0;
    }

    // #endregion

    // #region ATUALIZACAO DAS LEITURAS

    @Override
    public void updateInputs(TelescopioIOInputs inputs) {

        // #region 🛑⚡ SEGURANCA - DESABILITADO, BATERIA E LIMITE DE CORRENTE

        // Sem PID aqui: o controle roda no roboRIO (Telescopio). O IO só
        // aplica as proteções que o TalonFX real também aplica.

        // Disabled corta a tensão, mas continua calculando a queda pela gravidade.
        if (DriverStation.isDisabled())
            requestedVolts = 0.0;
        double batteryVolts = Math.max(0.0, RobotController.getBatteryVoltage());
        double batteryLimitedVolts = MathUtil.clamp(requestedVolts, -batteryVolts, batteryVolts);
        // Limite de corrente do estator, como no TalonFX: V = R·I + ω/Kv.
        // 0 V é saída neutra: não há tensão para o limite reduzir.
        double backEmfVolts = sim.getVelocityMetersPerSecond() * GEAR_REDUCTION
                / SIM_EFFECTIVE_DRUM_RADIUS_METERS / SIM_MOTOR.KvRadPerSecPerVolt;
        double currentLimitVolts = STATOR_CURRENT_LIMIT_AMPS * SIM_MOTOR.rOhms;
        double appliedVolts = batteryLimitedVolts == 0 ? 0
                : MathUtil.clamp(batteryLimitedVolts,
                        backEmfVolts - currentLimitVolts, backEmfVolts + currentLimitVolts);

        // #endregion

        // #region 🧪 PASSO DA SIMULACAO

        sim.setInputVoltage(appliedVolts);
        sim.update(LOOP_PERIOD_SECONDS); // Atualizar apenas aqui, uma vez por ciclo.

        // #endregion

        // #region SENSORES IDEAIS

        inputs.motorConnected = true;
        inputs.positionValid = true; // O zero virtual é conhecido desde o início.
        inputs.extensionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
        inputs.appliedVolts = appliedVolts;
        double motorVelocity = inputs.velocityMetersPerSec * GEAR_REDUCTION
                / SIM_EFFECTIVE_DRUM_RADIUS_METERS;
        inputs.motorCurrentAmps = Math.abs(SIM_MOTOR.getCurrent(motorVelocity, appliedVolts));
        inputs.statorCurrentLimitActive = appliedVolts != batteryLimitedVolts;

        // #endregion
    }

    // #endregion

    // #region 🧪 MODELO EM PARALELO (frc.robot.util.ParallelModel)

    // A mesma física, recebendo a tensão MEDIDA no TalonFX. O limite de
    // corrente não entra: a tensão medida já vem limitada pelo Talon.

    @Override
    public void setModelState(double extensionMeters, double velocityMetersPerSec) {
        sim.setState(extensionMeters, velocityMetersPerSec);
    }

    @Override
    public void stepModel(double volts) {
        modelVolts = volts;
        sim.setInputVoltage(volts);
        sim.update(LOOP_PERIOD_SECONDS);
    }

    @Override
    public double modelPosition() {
        return sim.getPositionMeters();
    }

    @Override
    public double modelVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public double modelCurrentAmps() {
        double motorVelocity = sim.getVelocityMetersPerSecond() * GEAR_REDUCTION
                / SIM_EFFECTIVE_DRUM_RADIUS_METERS;
        return Math.abs(SIM_MOTOR.getCurrent(motorVelocity, modelVolts));
    }

    // #endregion
}
