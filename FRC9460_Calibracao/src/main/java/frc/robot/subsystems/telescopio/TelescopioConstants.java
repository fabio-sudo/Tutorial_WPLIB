package frc.robot.subsystems.telescopio;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Constantes do telescópio 9460.
 *
 * Valores baseados nos dados registrados na documentação da etapa 1.
 * Distâncias em metros e massas em quilogramas.
 */
public final class TelescopioConstants {

    // #region 📏 GEOMETRIA DO TELESCÓPIO 9460

    // Altura total medida no modelo CAD na posição exportada.
    public static final double OPEN_HEIGHT_METERS = 2.1145;

    // Cursos utilizados na montagem visual existente.
    // Referências provisórias até validar os limites mecânicos.

    // Deslocamento de N2 em relação à base N1.
    public static final double N2_TRAVEL_METERS = 0.624;

    // Deslocamento de N3 em relação a N2.
    public static final double N3_RELATIVE_TRAVEL_METERS = 0.628;

    // Extensão zero corresponde ao mecanismo recolhido.
    public static final double MIN_EXTENSION_METERS = 0.0;

    // Deslocamento total da ponta: 1,252 m.
    public static final double MAX_EXTENSION_METERS = N2_TRAVEL_METERS + N3_RELATIVE_TRAVEL_METERS;

    // Altura recolhida reconstruída para a visualização: 0,8625 m.
    // Calculada; não é uma nova medição do mecanismo real.
    public static final double CLOSED_HEIGHT_METERS = OPEN_HEIGHT_METERS - MAX_EXTENSION_METERS;

    // #endregion

    // #region 📏 MOTOR E TRANSMISSÃO

    // Motor informado: Kraken X44. O ID CAN fica em frc.robot.CanIds.
    public static final int MOTOR_COUNT = 1;

    // Sentido do motor: positivo deve ESTENDER. Não confirmado no mecanismo.
    // Conferir com o telescópio sem carga antes do primeiro teste com perfil.
    public static final boolean MOTOR_INVERTED = false;

    // Redução 5:1:
    // 5 voltas na entrada do redutor para 1 volta na saída.
    // Não inclui a vantagem mecânica dos cabos e polias.
    public static final double GEAR_REDUCTION = 5.0;

    // Diâmetro do tambor: 35 mm.
    public static final double DRUM_DIAMETER_METERS = 0.035;

    // Raio do tambor: 17,5 mm.
    public static final double DRUM_RADIUS_METERS = DRUM_DIAMETER_METERS / 2.0;

    // #endregion

    // #region 🧪 FÍSICA DA SIMULAÇÃO E CONVERSÃO PARA O MOTOR

    // Massa inicial informada para os ensaios.
    // A distribuição entre os estágios e a carga ainda será definida.
    public static final double SIM_INITIAL_MOVING_MASS_KG = 5.0;

    // Hipótese provisória: 1 m de cabo corresponde a 1 m da ponta.
    // O CAD não informa o percurso completo do cabo. Não é uma medida real.
    public static final double SIM_CABLE_TO_TIP_RATIO = 1.0;
    public static final double SIM_EFFECTIVE_DRUM_RADIUS_METERS =
        DRUM_RADIUS_METERS * SIM_CABLE_TO_TIP_RATIO;
    public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX44(MOTOR_COUNT);

    // 🚧 Medir antes de ligar o hardware: todos os limites em metros dependem desta conversão.
    // Voltas do rotor por metro da ponta, com a relação provisória cabo/ponta.
    public static final double MOTOR_ROTATIONS_PER_METER =
        GEAR_REDUCTION / (2 * Math.PI * SIM_EFFECTIVE_DRUM_RADIUS_METERS);

    // #endregion

    // #region 🎯🚀 CONTROLE - PERFIL E PID DE POSIÇÃO (NO ROBORIO)

    public static final double LOOP_PERIOD_SECONDS = 0.02;

    // Controle inicial para o modelo ideal. Recalibrar antes do hardware real.
    // 🎯 PID (roda no roboRIO).
    public static final double POSITION_KP = 18.0;
    // 🚀 Velocidade e aceleração do perfil.
    public static final double MAX_VELOCITY_METERS_PER_SEC = 0.4;
    public static final double MAX_ACCELERATION_METERS_PER_SEC_SQ = 0.6;
    public static final double POSITION_TOLERANCE_METERS = 0.005;
    public static final double VELOCITY_TOLERANCE_METERS_PER_SEC = 0.025;

    // #endregion

    // #region 🎯 CONTROLE - FEEDFORWARD (kG, kV, kA)

    // Feedforward derivado da mesma massa e transmissão equivalentes da física.
    // São valores de simulação, sem atrito, e não uma caracterização do robô.
    public static final double SIM_KA = SIM_INITIAL_MOVING_MASS_KG
        * SIM_EFFECTIVE_DRUM_RADIUS_METERS * SIM_MOTOR.rOhms
        / (GEAR_REDUCTION * SIM_MOTOR.KtNMPerAmp);
    public static final double SIM_KG = SIM_KA * 9.8;
    public static final double SIM_KV = GEAR_REDUCTION
        / (SIM_EFFECTIVE_DRUM_RADIUS_METERS * SIM_MOTOR.KvRadPerSecPerVolt);

    // #endregion

    // #region 🛑⚡ SEGURANÇA - TENSÃO MÁXIMA DO CONTROLE

    // O PID + feedforward nunca pede mais que isto (Telescopio, passo 6).
    public static final double MAX_CONTROL_VOLTS = 6.0;

    // #endregion

    // #region 🛑⚡ SEGURANÇA - LIMITES DE CORRENTE

    // Limite de corrente do estator (Phoenix 6, CurrentLimitsConfigs).
    // CTRE: 120 A é ponto de partida comum (80-160 A), mas mecanismos com batente
    // podem precisar de um limite menor para reduzir o torque contra o batente.
    // Na simulação, segurar 5 kg usa ~12 A e mover ~13 A: 40 A deixa ~3x de folga
    // e limita a força do motor na ponta a ~170 N. Validar no mecanismo real.
    public static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;

    // Corrente da bateria: 30 A segue o exemplo da CTRE para elevador.
    // Depois de 1 s no limite, reduz para 20 A.
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 20.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME_SECONDS = 1.0;

    // #endregion

    // #region 🛑 SEGURANÇA - TRAVAMENTO

    // Travamento: corrente perto do limite, sem movimento, por 0,25 s.
    public static final double JAM_CURRENT_AMPS = 30.0;
    public static final double JAM_VELOCITY_METERS_PER_SEC = 0.02;
    public static final double JAM_TIME_SECONDS = 0.25;

    // #endregion

    // #region 🛑📏 SEGURANÇA - SOFT LIMITS DO TALONFX

    // Soft limits do TalonFX 1 cm além dos limites do subsistema: são reserva.
    public static final double SOFT_LIMIT_MARGIN_METERS = 0.01;

    // #endregion

    // #region 🔧 CICLO AUTOMÁTICO DE TESTE

    // Pausa em cada posição do ciclo automático: metade, aberto e recolhido.
    public static final double DEMO_DWELL_SECONDS = 1.0;

    // #endregion

    // #region 🧪 MODELO EM PARALELO (REAL X SIMULADO)

    // Diferença aceitável real x simulado (frc.robot.util.ParallelModel). Calibrar.
    public static final double MODEL_TOLERANCE_CENTIMETERS = 3.0;
    // Abaixo disto o telescópio está parado: real e modelo parados = realinha.
    public static final double MODEL_REST_CENTIMETERS_PER_SEC = 1.0;

    // #endregion

    // #region VISUALIZAÇÃO (ASSETS DO ADVANTAGESCOPE)

    // Geometria da montagem de bancada já usada pelos assets do AdvantageScope.
    // Posições dos pivôs recolhidos, relativas à base do modelo (não ao robô).
    public static final double N2_PIVOT_X_METERS = -0.016200607661327762;
    public static final double N2_PIVOT_Y_METERS = -0.04515224914169346;
    public static final double N2_CLOSED_PIVOT_Z_METERS = 0.4612325230899238;
    public static final double N3_PIVOT_X_METERS = -0.008905774943281507;
    public static final double N3_PIVOT_Y_METERS = -0.04515224914169342;
    public static final double N3_CLOSED_PIVOT_Z_METERS = 0.5029012160752977;
    public static final double N1_VISIBLE_LENGTH_METERS = 0.804;
    public static final double N2_CLOSED_VISIBLE_LENGTH_METERS = 0.04;
    public static final double N3_CLOSED_VISIBLE_LENGTH_METERS = 0.0185;

    // #endregion

    // #region SEM INSTÂNCIAS

    // Impede criar objetos desta classe de constantes.
    private TelescopioConstants() {
    }

    // #endregion
}
