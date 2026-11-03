package frc.robot.subsystems.roller;

// #region IMPORTS

import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.CanIds;

// #endregion

/**
 * Números do rolete do intake, que puxa a bola para dentro.
 * Valores do código de competição (IntakeRollerSubsystem).
 */
public final class RollerConstants {

    // #region MOTOR (SPARK FLEX)

    public static final int MOTOR_ID = CanIds.INTAKE_ROLLER;
    public static final boolean MOTOR_INVERTED = false;

    // #endregion

    // #region 🛑⚡ SEGURANÇA - LIMITE DE CORRENTE

    // Protege motor e PDP se uma bola travar no rolete. Dentro da faixa
    // recomendada pela REV (frc.robot.MotorLimits).
    public static final int CURRENT_LIMIT_AMPS = 40;

    // #endregion

    // #region 🚀 CONTROLE - MALHA ABERTA (SEM PID)

    // 🚀 VELOCIDADE DO ROLETE.
    // Malha aberta: 30% da saída para puxar.
    public static final double PULL_PERCENT = 0.30;

    // #endregion

    // #region 🧪 SIMULAÇÃO (ESTIMATIVAS)

    // Spark Flex com NEO Vortex. Inércia do rolo estimada.
    public static final DCMotor SIM_MOTOR = DCMotor.getNeoVortex(1);
    public static final double SIM_MOI_KG_M2 = 0.0005;

    // Uma bola sendo engolida freia o rolo por 0,06 s: a corrente sobe.
    // É isso que permite contar coletas pela corrente, no robô e aqui.
    public static final double SIM_BALL_LOAD_TORQUE_NM = 0.60;
    public static final double SIM_BALL_LOAD_SECONDS = 0.06;

    // #endregion

    // #region 🧪 MODELO EM PARALELO (REAL X SIMULADO)

    // Diferença aceitável entre o mecanismo e a física recebendo a mesma
    // tensão (frc.robot.util.ParallelModel). Acima disso por 0,5 s, o
    // diagnóstico mostra REAL_DIFERENTE_DO_MODELO. Calibrar no robô real.
    // O modelo não tem bolas: cada coleta deixa o rolo até ~600 RPM abaixo por
    // um instante (medido na simulação). Bola presa = diferença por mais de 0,5 s.
    public static final double MODEL_TOLERANCE_RPM = 500.0;

    // #endregion

    // #region SEM INSTÂNCIAS

    private RollerConstants() {}

    // #endregion
}
