package frc.robot.subsystems.intake;

// #region IMPORTS

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.CanIds;

// #endregion

/**
 * Números do articulador do intake (abre e fecha o intake).
 * Valores do código de competição (IntakeArticulatorSubsystem).
 * O rolete que puxa a bola tem pasta própria (subsystems/roller).
 */
public final class IntakeConstants {

    // #region MOTORES (DOIS TALONFX)

    // O esquerdo é o líder; o direito segue o esquerdo em sentido oposto.
    // Números no mapa único frc.robot.CanIds.
    public static final int LEFT_MOTOR_ID = CanIds.INTAKE_LEFT;
    public static final int RIGHT_MOTOR_ID = CanIds.INTAKE_RIGHT;

    // #endregion

    // #region 🛑⚡ SEGURANÇA - LIMITES DE CORRENTE

    // Supply e estator: 40 A. Abaixo do ponto de partida da CTRE porque o
    // mecanismo tem batente nos dois lados (veja frc.robot.MotorLimits).
    public static final double CURRENT_LIMIT_AMPS = 40.0;

    // #endregion

    // #region 📏🧭 POSIÇÕES E ZERO MANUAL

    // 🚧 Robô ligado com o intake aberto: o zero nasce errado e o aberto passa do batente.
    // Sem sensor: ao ligar, o encoder vale 0 onde o intake estiver.
    // Por isso o robô deve ser ligado com o intake FECHADO (recolhido).
    // Unidade: voltas do motor. Os mesmos valores são os soft limits.
    public static final double CLOSED_ROTATIONS = 0.0;
    // tuner-plot.csv: extremo manual no rotor direito, CAN 35. Usa-se a
    // magnitude do curso no líder, cujo sentido de abertura já é negativo.
    // Não é um alvo para o motor: o alvo fica ANTES dele, com margem.
    public static final double MAX_ROTOR_TRAVEL_ROTATIONS = 22.205078;
    public static final double MECHANICAL_OPEN_ROTATIONS = -MAX_ROTOR_TRAVEL_ROTATIONS;
    public static final double OPEN_SAFETY_MARGIN_ROTATIONS = 0.50;
    public static final double OPEN_ROTATIONS = MECHANICAL_OPEN_ROTATIONS + OPEN_SAFETY_MARGIN_ROTATIONS;
    public static final double TOLERANCE_ROTATIONS = 0.40;

    // #endregion

    // #region 🛑📏 SEGURANÇA - FIM DE CURSO

    // Para um pouco antes do fim para não forçar o mecanismo.
    public static final boolean CLOSE_LIMIT_ENABLED = true;
    public static final double CLOSE_BUFFER_ROTATIONS = 0.10;
    public static final double OPEN_BUFFER_ROTATIONS = 0.10;

    // #endregion

    // #region 🚀 CONTROLE - ABRIR E FECHAR (MALHA ABERTA)

    // 🚀 VELOCIDADE DO INTAKE.
    // Movimento lento de abrir e fechar (15% da saída).
    public static final double OPEN_PERCENT = -0.15;
    public static final double CLOSE_PERCENT = 0.15;

    // #endregion

    // #region 🎯 CONTROLE - SEGURAR A POSIÇÃO (PID NO TALONFX)

    // Sem comando, o articulador TRAVA na posição atual. O Brake só freia,
    // não segura peso: sem isso o intake desce sozinho (simulação, só Brake:
    // fechado 7° em 30 s, meio do curso 47° em 30 s).
    public static final boolean HOLD_ENABLED = true;
    // 🎯 PID DE SEGURAR: só corrige a posição parada.
    public static final double HOLD_KP = 1.0; // V por volta de erro
    public static final double HOLD_KI = 0.0;
    public static final double HOLD_KD = 0.0;
    public static final double HOLD_KG = 0.0; // V contra a gravidade (começar em 0)
    // Perto do batente ABERTO a gravidade já encosta o braço no batente: só
    // Brake, sem PID (sem tensão, ruído nem calor). Em voltas do rotor.
    // Antes (até 29/10/2026) o PID relaxava e retomava por histerese em
    // QUALQUER posição. Como o P puro precisa de um erro estável de 0,03 a
    // 0,08 volta para segurar o peso, e esse erro cai dentro da faixa de
    // relaxar, o Talon alternava entre neutro e PID 20 a 40 vezes por segundo:
    // o "jitter" que se ouvia parado. Fora do batente o PID agora é contínuo.
    public static final double BRAKE_ONLY_NEAR_OPEN_ROTATIONS = 0.25;
    // ⚡ Tensão máxima do PID de segurar.
    // Limite inicial de retenção, a validar com a carga real. Não é ganho SysId.
    public static final double HOLD_MAX_VOLTS = 2.0;
    // Observação no Tuner: +/-3% iniciou movimento. Não aplicar como kG/kS.
    public static final double OBSERVED_BREAKAWAY_DUTY_CYCLE = 0.03;
    // Banda neutra do Talon: pedido do PID menor que isto vira neutro (Brake).
    // Tensão pequena demais para mover o braço só faz o motor zumbir parado. Com
    // kP 1 V/volta, o braço fica em Brake a até 0,36 volta (~1,6°) do alvo, dentro
    // da tolerância de 0,40 volta, e o PID só volta com tensão que já o move.
    // Vale para os dois motores: o seguidor copia a saída do líder.
    public static final double HOLD_NEUTRAL_DEADBAND = OBSERVED_BREAKAWAY_DUTY_CYCLE;

    // #endregion

    // #region 📏 GEOMETRIA (ESTIMATIVA PARA O DESENHO E A SIMULAÇÃO)

    // Cada motor tem dois estágios 9:1 em série (informado pela equipe).
    // Os dois motores trabalham em paralelo: não se somam as reduções.
    public static final double FIRST_STAGE_REDUCTION = 9.0;
    public static final double SECOND_STAGE_REDUCTION = 9.0;
    public static final double GEAR_REDUCTION = FIRST_STAGE_REDUCTION * SECOND_STAGE_REDUCTION;
    public static final double DEGREES_PER_MOTOR_ROTATION = 360.0 / GEAR_REDUCTION;
    public static final double CLOSED_RAISE_DEGREES = (CLOSED_ROTATIONS - OPEN_ROTATIONS) * DEGREES_PER_MOTOR_ROTATION;

    // Pivô medido no CAD (model_0 do modelo 3D do robô).
    public static final Translation3d PIVOT_IN_ROBOT = new Translation3d(0.2785, 0.0, 0.2389);

    // #endregion

    // #region 🧪 SIMULAÇÃO (ESTIMATIVAS)

    // CAD: dois Kraken X44. Braço do pivô até o rolo: 0,31 m; massa estimada.
    public static final int SIM_MOTOR_COUNT = 2;
    public static final DCMotor SIM_MOTORS = DCMotor.getKrakenX44(SIM_MOTOR_COUNT);
    public static final double SIM_ARM_LENGTH_METERS = 0.31;
    public static final double SIM_MASS_KG = 3.0;
    // Aberto, o braço fica 10° abaixo da horizontal (medido no CAD).
    public static final double SIM_OPEN_ARM_ANGLE_DEGREES = -10.0;

    // #endregion

    // #region 🧪 MODELO EM PARALELO (REAL X SIMULADO)

    // Diferença aceitável entre o mecanismo e a física recebendo a mesma
    // tensão (frc.robot.util.ParallelModel). Acima disso por 0,5 s, o
    // diagnóstico mostra REAL_DIFERENTE_DO_MODELO. Calibrar no robô real.
    public static final double MODEL_TOLERANCE_DEGREES = 5.0;
    // Abaixo disto o braço está parado: real e modelo parados = realinha.
    public static final double MODEL_REST_DEGREES_PER_SEC = 2.0;

    // #endregion

    // #region SEM INSTÂNCIAS

    private IntakeConstants() {
    }

    // #endregion
}
