package frc.robot.subsystems.centrifuga;

// #region IMPORTS

import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.CanIds;

// #endregion

/**
 * Números da centrífuga, a peça que empurra a bola do depósito para a tower.
 * Valores do código de competição (ShooterCentrifugeSubsystem).
 * O RPM de cada tiro vem da curva do shooter (ShotCalculator).
 */
public final class CentrifugaConstants {

    // #region MOTOR (TALONFX)

    public static final int MOTOR_ID = CanIds.CENTRIFUGE;
    public static final boolean MOTOR_INVERTED = true;
    // Redução total informada pela equipe: RPM e PID existentes são do ROTOR.
    public static final double GEAR_REDUCTION = 5.0;
    // 📦 Reversão de destravar: o botão foi arquivado em 29/10/2026 (RobotContainer).
    public static final double UNJAM_OUTPUT_ROTATIONS = 0.25;
    public static final double UNJAM_ROTOR_RPM = 150.0; // 30 RPM na saída
    public static final double UNJAM_TIMEOUT_SECONDS = 1.5;
    public static final double UNJAM_BRAKE_TIMEOUT_SECONDS = 0.75;
    public static final double UNJAM_STOPPED_ROTOR_RPM = 15.0;
    public static final double UNJAM_STOPPED_SECONDS = 0.10;
    public static final double UNJAM_CURRENT_AMPS = 15.0;
    public static final double UNJAM_OVERCURRENT_SECONDS = 0.10;

    // #endregion

    // #region 🛑⚡ SEGURANÇA - LIMITES DE CORRENTE

    // Supply no valor do disjuntor; estator abaixo do ponto de partida da
    // CTRE (frc.robot.MotorLimits). Valores da competição.
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;

    // #endregion

    // #region 🎯🚀 CONTROLE - PID DE VELOCIDADE (VELOCITYVOLTAGE NO TALONFX)

    // O Phoenix 6 trabalha em rotações por SEGUNDO.
    // Tensão = kS + kV × alvo + kP × erro.
    public static final double KRAKEN_FREE_SPEED_RPM = 6380.0;
    // 🎯 PID DA CENTRÍFUGA (roda no TalonFX): só kP, kS e kV.
    public static final double KP = 0.12;               // V por rot/s de erro
    // Medição da equipe: começa a sair da inércia com DutyCycleOut 0,028.
    // Conversão inicial a 12 V; validar depois sob carga e tensão medida.
    public static final double BREAKAWAY_DUTY_CYCLE = 0.028;
    public static final double CHARACTERIZATION_NOMINAL_VOLTS = 12.0;
    public static final double KS = BREAKAWAY_DUTY_CYCLE * CHARACTERIZATION_NOMINAL_VOLTS;
    public static final double KV = 12.0 / (KRAKEN_FREE_SPEED_RPM / 60.0); // V por rot/s
    // 🚀 Maior RPM aceito e rampa de aceleração da partida.
    public static final double MAX_COMMAND_RPM = 4000.0;
    public static final double ACCELERATION_ROTATIONS_PER_SEC2 = 60.0;
    // 🛑 Leitura mais velha que isto: corta a saída.
    public static final double MAX_SIGNAL_AGE_SECONDS = 0.10;
    // 🛑 MotorSafety: o código parou de mandar pedido, o motor para.
    // Com 0,10 s, os picos do ciclo do roboRIO (até 358 ms em 30/10/2026) paravam a
    // centrífuga no meio do tiro: 5 vezes em duas sessões. Acima do ciclo normal e
    // ainda curto: se o código travar de verdade, o motor para em 0,25 s.
    public static final double MOTOR_SAFETY_SECONDS = 0.25;

    public static final double VELOCITY_TOLERANCE_RPM = 75.0;

    // Teste isolado do código antigo (R1): 2300 RPM.
    public static final double TEST_RPM = 2300.0;
    // Teste SEM TRAVA (sem botão desde 31/10/2026): saída direta, como no Phoenix Tuner X,
    // onde DutyCycleOut 0,308 girou a centrífuga (29/10/2026). Só para a frente.
    public static final double OPEN_LOOP_TEST_PERCENT = 0.30;

    // #endregion

    // #region 🧪 SIMULAÇÃO (ESTIMATIVAS)

    // CAD: Kraken X44. Inércia da peça giratória estimada, não medida.
    public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX44(1);
    public static final double SIM_MOI_KG_M2 = 0.005;
    // A física virtual não caracteriza o atrito medido no robô; conserva o
    // feedforward anterior do simulador. O perfil de aceleração é compartilhado.
    public static final double SIM_KS = 0.20;

    // #endregion

    // #region 🧪 MODELO EM PARALELO (REAL X SIMULADO)

    // Diferença aceitável entre o mecanismo e a física recebendo a mesma
    // tensão (frc.robot.util.ParallelModel). Acima disso por 0,5 s, o
    // diagnóstico mostra REAL_DIFERENTE_DO_MODELO. Calibrar no robô real.
    public static final double MODEL_TOLERANCE_RPM = 300.0;

    // #endregion

    // #region SEM INSTÂNCIAS

    private CentrifugaConstants() {}

    // #endregion
}
