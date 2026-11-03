package frc.robot.subsystems.roller;

// #region IMPORTS

import static frc.robot.subsystems.roller.RollerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import frc.robot.util.MechanismModel;
import frc.robot.util.ParallelModel;

// #endregion

/**
 * Rolete no computador: volante em malha aberta (tensão = saída × 12 V).
 * Quando a simulação das bolas avisa que uma bola entrou, um torque de carga
 * freia o rolo por um instante: a corrente sobe como no robô real.
 * Também é o modelo em paralelo do rolete: o modelo não tem bolas, então a
 * diferença de corrente mostra cada bola entrando.
 */
public class RollerIOSim implements RollerIO, MechanismModel {

    // #region 🧪 FÍSICA

    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.001;
    private static final double BATTERY_VOLTS = 12.0;
    private static final double COAST_TIME_CONSTANT_SECONDS = 0.5;

    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(SIM_MOTOR, SIM_MOI_KG_M2, 1.0), SIM_MOTOR);

    private double percent;
    private double loadSecondsLeft;
    private double modelVolts;

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        for (int i = 0; i < steps; i++) {
            if (percent == 0.0) {
                // Coast (padrão do Spark Flex): motor solto, sem corrente.
                // Antes era 0 V (curto), que gerava um pico falso ao parar.
                coast(SUBSTEP_SECONDS);
                continue;
            }
            // Controle: malha aberta, saída × 12 V (sem PID).
            sim.setInputVoltage(percent * BATTERY_VOLTS);
            sim.update(SUBSTEP_SECONDS);
            if (loadSecondsLeft > 0.0) {
                // Torque da bola contra o rolo: tira velocidade a cada passo.
                double slowdown = SIM_BALL_LOAD_TORQUE_NM / SIM_MOI_KG_M2 * SUBSTEP_SECONDS;
                double speed = sim.getAngularVelocityRadPerSec();
                sim.setAngularVelocity(Math.copySign(Math.max(0.0, Math.abs(speed) - slowdown), speed));
                loadSecondsLeft -= SUBSTEP_SECONDS;
            }
        }
        inputs.connected = true;
        inputs.velocityRpm = sim.getAngularVelocityRPM();
        inputs.appliedVolts = percent * BATTERY_VOLTS;
        inputs.currentAmps = percent == 0.0 ? 0.0 : limitedCurrentAmps();
    }

    // #endregion

    // #region 🛑⚡ SEGURANÇA - LIMITE DE CORRENTE DO SPARK FLEX

    /** O limite de corrente do Spark corta picos acima de 40 A. */
    private double limitedCurrentAmps() {
        return Math.min(Math.abs(sim.getCurrentDrawAmps()), CURRENT_LIMIT_AMPS);
    }

    // #endregion

    // #region 🧪 FÍSICA AUXILIAR (COAST)

    /** Motor solto: o rolo perde velocidade só por atrito. */
    private void coast(double seconds) {
        sim.setAngularVelocity(sim.getAngularVelocityRadPerSec()
            * Math.exp(-seconds / COAST_TIME_CONSTANT_SECONDS));
    }

    // #endregion

    // #region 🧪 MODELO EM PARALELO (frc.robot.util.ParallelModel)

    // A mesma física, recebendo a tensão MEDIDA no Spark Flex.
    // Quase 0 V = saída neutra: Coast, como o rolete real parado.
    // Um passo de 20 ms: com a tensão constante no ciclo, a física linear é
    // exata sem subpassos (e fica leve para rodar no roboRIO).

    @Override
    public void setModelState(double position, double velocityRpm) {
        sim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm));
    }

    @Override
    public void stepModel(double volts) {
        modelVolts = Math.abs(volts) < ParallelModel.NEUTRAL_VOLTS
            ? 0.0
            : MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS);
        if (modelVolts == 0.0) {
            coast(LOOP_SECONDS);
            return;
        }
        sim.setInputVoltage(modelVolts);
        sim.update(LOOP_SECONDS);
    }

    @Override
    public double modelPosition() {
        return 0.0;
    }

    @Override
    public double modelVelocity() {
        return sim.getAngularVelocityRPM();
    }

    @Override
    public double modelCurrentAmps() {
        return modelVolts == 0.0 ? 0.0 : limitedCurrentAmps();
    }

    // #endregion

    // #region COMANDOS

    @Override
    public void setPercentOutput(double percent) {
        this.percent = Double.isFinite(percent) ? MathUtil.clamp(percent, -1.0, 1.0) : 0.0;
    }

    @Override
    public void stop() {
        percent = 0.0;
    }

    @Override
    public void simulateBallIntake() {
        loadSecondsLeft = SIM_BALL_LOAD_SECONDS;
    }

    // #endregion
}
