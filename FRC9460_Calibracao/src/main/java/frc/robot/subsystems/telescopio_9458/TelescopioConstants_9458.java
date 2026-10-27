package frc.robot.subsystems.telescopio_9458;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Constantes do telescópio 9458: cascata de 2 estágios.
 *
 * Redução 16:1 e fim de curso (78,4 voltas do motor) do robô físico; coroa, corrente e
 * dimensões do CAD (config/advantagescope/docs/telescopio_9458/telescopio_9458_cad_x_medidas.png).
 * Distâncias em metros e massas em quilogramas.
 * Os nomes seguem TelescopioConstants (9460) para reaproveitar as classes.
 */
public final class TelescopioConstants_9458 {

    // #region MOTOR E TRANSMISSÃO

    // Motor: 1 Kraken X60 (no CAD: WCP-0940).
    public static final int MOTOR_COUNT = 1;

    // Duas reduções de 4:1 = 16:1, como no robô físico.
    // O redutor do CAD está 4:1 × 3:1 = 12:1 e não foi usado.
    public static final double GEAR_REDUCTION = 4.0 * 4.0;

    // Coroa do CAD: REV-21-2016, 16 dentes, corrente #25 (passo de 6,35 mm).
    public static final int SPROCKET_TEETH = 16;
    public static final double CHAIN_PITCH_METERS = 0.00635;

    // Diâmetro primitivo: 6,35 ÷ sen(180°/16) = 32,55 mm (externo no CAD: 35,9 mm).
    public static final double SPROCKET_PITCH_DIAMETER_METERS =
        CHAIN_PITCH_METERS / Math.sin(Math.PI / SPROCKET_TEETH);

    // A corrente anda 16 × 6,35 = 101,6 mm por volta da coroa.
    public static final double CHAIN_METERS_PER_SPROCKET_ROTATION =
        SPROCKET_TEETH * CHAIN_PITCH_METERS;

    // Cada volta do motor move cada estágio 101,6 mm ÷ 16 = 6,35 mm.
    public static final double STAGE_METERS_PER_MOTOR_ROTATION =
        CHAIN_METERS_PER_SPROCKET_ROTATION / GEAR_REDUCTION;

    // Fim de curso medido no robô: 78,4 voltas do rotor (antes da redução).
    public static final double MOTOR_ROTATIONS_AT_LIMIT = 78.4;

    // Cascata: a ponta anda 2x a corrente da coroa (10 cm + 10 cm, informado).
    // O prefixo SIM_ vem do 9460, mas estas duas constantes também convertem
    // as leituras do robô real (MOTOR_ROTATIONS_PER_METER).
    public static final double SIM_CABLE_TO_TIP_RATIO = 2.0;

    // Raio equivalente para a ponta: (101,6 mm ÷ 2π) × 2 = 32,34 mm.
    // A ponta anda 12,7 mm por volta do motor: 78,4 voltas = 0,996 m.
    public static final double SIM_EFFECTIVE_DRUM_RADIUS_METERS =
        CHAIN_METERS_PER_SPROCKET_ROTATION / (2 * Math.PI) * SIM_CABLE_TO_TIP_RATIO;

    // Voltas do rotor por metro da ponta: 78,7. Converte o encoder em extensão.
    public static final double MOTOR_ROTATIONS_PER_METER =
        GEAR_REDUCTION / (2 * Math.PI * SIM_EFFECTIVE_DRUM_RADIUS_METERS);

    // #endregion

    // #region GEOMETRIA DO TELESCÓPIO 9458

    // N1 = tubo 4" fixo, N2 = tubo 3", N3 = tubo 2".
    // Cascata: N3 sobe em relação a N2 o mesmo que N2 sobe em relação a N1.

    // Curso de N2 em relação a N1: 78,4 × 6,35 mm = 0,498 m.
    // O CAD tem ~0,57 m de folga mecânica por estágio.
    public static final double N2_TRAVEL_METERS =
        MOTOR_ROTATIONS_AT_LIMIT * STAGE_METERS_PER_MOTOR_ROTATION;

    // Curso de N3 em relação a N2: igual ao de N2.
    public static final double N3_RELATIVE_TRAVEL_METERS = N2_TRAVEL_METERS;

    // Extensão zero corresponde ao mecanismo recolhido.
    public static final double MIN_EXTENSION_METERS = 0.0;

    // Deslocamento total da ponta: 2 × 0,498 = 0,996 m (batente físico).
    public static final double MAX_EXTENSION_METERS = N2_TRAVEL_METERS + N3_RELATIVE_TRAVEL_METERS;

    // Altura fechado no CAD (topo do tubo 2"): 0,780 m. A equipe mediu 0,80 m.
    public static final double CLOSED_HEIGHT_METERS = 0.7802;

    // Altura aberta: 0,780 + 0,996 = 1,776 m. A equipe mediu 1,75 m.
    public static final double OPEN_HEIGHT_METERS = CLOSED_HEIGHT_METERS + MAX_EXTENSION_METERS;

    // #endregion

    // #region LIMITES DE OPERAÇÃO (EM METROS DA PONTA)

    // Margem escolhida pela equipe: 5 cm NA PONTA antes do batente físico.
    // MAX_EXTENSION_METERS continua representando a geometria e o batente reais.
    // Segurar RT para em 0,946 m.
    public static final double UPPER_SAFETY_MARGIN_METERS = 0.05;
    public static final double MAX_OPERATING_EXTENSION_METERS =
        MAX_EXTENSION_METERS - UPPER_SAFETY_MARGIN_METERS;

    // Reserva de 1 cm após a meta normal; o Talon corta abertura ainda 4 cm antes
    // do batente. Não é distância de frenagem garantida: depende de velocidade e carga.
    public static final double SOFT_LIMIT_OVERRUN_METERS = 0.01;
    public static final double UPPER_SOFT_LIMIT_METERS =
        MAX_OPERATING_EXTENSION_METERS + SOFT_LIMIT_OVERRUN_METERS;

    // Leitura ainda válida até 5 cm além dos batentes (3,9 voltas do rotor).
    // No robô real o telescópio pode passar alguns mm do topo; com só 5 mm de margem
    // a posição ficava inválida e o L2 não conseguia descer. Além de 5 cm é erro de zero.
    public static final double POSITION_VALID_MARGIN_METERS = 0.05;

    // Com meta zero e até 2 mm do fundo, o telescópio apoia no batente sem tensão.
    public static final double RETRACTED_REST_TOLERANCE_METERS = 0.002;
    // Tolerância numérica de meta, distinta da tolerância física de posição.
    public static final double GOAL_EPSILON_METERS = 1e-6;

    // Modo Test: segurar R1 sobe e L1 desce este passo, e o telescópio mantém a altura.
    public static final double TEST_STEP_METERS = 0.10;

    // Aproximação lenta dos fins de curso: indo para o fundo ou para o topo operacional,
    // o perfil entra nos últimos 2 cm a 6 cm/s. Encurta o trecho lento sem aumentar
    // a velocidade final já aprovada pela equipe. Sem aproximação, o telescópio
    // real chegava a ~0,18 m/s a 1 cm do batente (segue o perfil ~1 cm atrasado) e batia.

    //==========================================================================================Aqui parada
    public static final double END_APPROACH_DISTANCE_METERS = 0.02;
    public static final double END_APPROACH_VELOCITY_METERS_PER_SEC = 0.06;

    // Motion Magic: a troca para o trecho lento é decidida a cada 20 ms e enviada ao Talon
    // como configuração (sem esperar resposta); antecipa a troca pelo que a referência anda
    // em 1 ciclo, o pior atraso da decisão. Cada ciclo a mais deixaria ~1,5 cm a 6 cm/s.
    public static final double END_APPROACH_LOOKAHEAD_SECONDS = 0.02;

    // #endregion

    // #region CONTROLE: PERFIL, TOLERÂNCIAS E TENSÃO

    public static final double LOOP_PERIOD_SECONDS = 0.02;

    // EXECUTOR DO CONTROLE DE POSIÇÃO
    // true:  Motion Magic dentro do TalonFX. A curva em S e o PID rodam a 1 kHz com o
    //        encoder sem atraso; na simulação o mesmo perfil é emulado (TelescopioIOSim_9458).
    // false: perfil trapezoidal + PID no roboRIO a cada 20 ms (versão usada até 22/10).
    //        No roboRIO o encoder chega com 20–40 ms de atraso, o que limitava kP/kD.
    public static final boolean USE_TALON_MOTION_MAGIC = true;

    // Motion Magic: 0,75 m/s, 2,5 m/s² e jerk de 25 m/s³. Com o jerk, a aceleração leva
    // 0,1 s para chegar ao máximo, por isso não fica seco como o trapezoide a 3 m/s².
    // Ao soltar em velocidade máxima, para em ~15 cm (o mesmo que antes a 0,55 m/s), e o
    // curso completo cai de ~2,6 s para ~2,0 s. Feedforward a 0,75 m/s ≈ 7,3 V.
    // Ajustáveis pelo painel (/Tuning/Telescopio9458/Perfil) com o robô desabilitado.

    //============================================================================================Velocidade
    public static final double MOTION_MAGIC_VELOCITY_METERS_PER_SEC = 0.85;
    public static final double MOTION_MAGIC_ACCELERATION_METERS_PER_SEC_SQ = 3.5;
    public static final double MOTION_MAGIC_JERK_METERS_PER_SEC_CUBED = 35.0;



    public static final TelescopioMotionProfile_9458.Limits MOTION_MAGIC_LIMITS =
        new TelescopioMotionProfile_9458.Limits(MOTION_MAGIC_VELOCITY_METERS_PER_SEC,
            MOTION_MAGIC_ACCELERATION_METERS_PER_SEC_SQ, MOTION_MAGIC_JERK_METERS_PER_SEC_CUBED);

    // Faixa aceita no ajuste pelo painel. A 0,85 m/s o feedforward pede ~8,2 V,
    // ainda abaixo do teto de 9 V, com folga para o PID.
    public static final double PROFILE_MAX_VELOCITY_METERS_PER_SEC = 0.85;
    public static final double PROFILE_MAX_ACCELERATION_METERS_PER_SEC_SQ = 5.0;
    public static final double PROFILE_MIN_JERK_METERS_PER_SEC_CUBED = 5.0;
    public static final double PROFILE_MAX_JERK_METERS_PER_SEC_CUBED = 100.0;

    // Perfil no roboRIO (USE_TALON_MOTION_MAGIC = false), validado no robô em 22/10:
    // 0,55 m/s e 1 m/s², sem jerk. Ao soltar em velocidade máxima, freia em v²/2a ≈ 15 cm.
    // 3 m/s² sem jerk deixou partidas e paradas secas e o telescópio batia no fim.
    // kP: volts por metro de erro na ponta.
    public static final double POSITION_KP = 18.0;
    public static final double MAX_VELOCITY_METERS_PER_SEC = 0.55;
    public static final double MAX_ACCELERATION_METERS_PER_SEC_SQ = 1.0;
    // O trapezoide não limita jerk; o valor alto só satisfaz o tipo Limits.
    public static final TelescopioMotionProfile_9458.Limits ROBORIO_LIMITS =
        new TelescopioMotionProfile_9458.Limits(MAX_VELOCITY_METERS_PER_SEC,
            MAX_ACCELERATION_METERS_PER_SEC_SQ, 1e6);

    // O Talon neutraliza o motor se o roboRIO ficar 100 ms sem mandar comando.
    public static final double MOTOR_SAFETY_EXPIRATION_SECONDS = 0.10;
    // Leitura mais velha que isto é tratada como perda de comunicação.
    public static final double SIGNAL_MAX_AGE_SECONDS = 0.10;
    public static final double POSITION_TOLERANCE_METERS = 0.005;
    public static final double VELOCITY_TOLERANCE_METERS_PER_SEC = 0.025;

    // Teto de tensão do controle (também PeakVoltage no Talon). 9 V cobrem o
    // feedforward de 7,3 V a 0,75 m/s; o limite de corrente continua em 30 A.
    public static final double MAX_CONTROL_VOLTS = 9.0;

    // Contribuição máxima do termo integral (kI), em volts.
    public static final double INTEGRAL_MAX_VOLTS = 1.0;

    // Pausa em cada posição do ciclo automático: metade, aberto e recolhido.
    public static final double DEMO_DWELL_SECONDS = 1.0;

    // #endregion

    // #region FÍSICA DA SIMULAÇÃO

    // Massa equivalente na ponta medida no SysId de 22/10/2026: kG real 0,201–0,203 V
    // corresponde a ~6,0 kg (antes: 5 kg informados).
    public static final double SIM_INITIAL_MOVING_MASS_KG = 6.0;

    // Atrito de Coulomb medido no SysId (kS real 0,042–0,058 V), em volts.
    public static final double SIM_KS = 0.05;

    public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60(MOTOR_COUNT);

    // #endregion

    // #region GANHOS: kP, kI, kD, kS, kG, kV, kA (TelescopioGains_9458)

    // SIMULAÇÃO: feedforward derivado da mesma massa, atrito e transmissão da física:
    // kS = 0,05 V, kG ≈ 0,20 V, kV ≈ 9,40 V/(m/s), kA ≈ 0,021 V/(m/s²).
    public static final double SIM_KA = SIM_INITIAL_MOVING_MASS_KG
        * SIM_EFFECTIVE_DRUM_RADIUS_METERS * SIM_MOTOR.rOhms
        / (GEAR_REDUCTION * SIM_MOTOR.KtNMPerAmp);
    public static final double SIM_KG = SIM_KA * 9.8;
    public static final double SIM_KV = GEAR_REDUCTION
        / (SIM_EFFECTIVE_DRUM_RADIUS_METERS * SIM_MOTOR.KvRadPerSecPerVolt);

    public static final TelescopioGains_9458 SIM_GAINS =
        new TelescopioGains_9458(POSITION_KP, 0, 0, SIM_KS, SIM_KG, SIM_KV, SIM_KA);

    // MOTOR REAL: feedforward medido no SysId de 22/10/2026 (bancada 9458, R² = 0,9995;
    // config/advantagescope/docs/telescopio_9458/SYSID.md). kG ~20% acima do teórico.
    // kA = 0: a aceleração não aparece nas amostras de 20 ms.
    // kP/kD: LQR de posição (erro máx 5 cm, 6 V, 20 ms) COM o atraso de leitura de
    // 20–40 ms que o SysId estimou: kP 34–38, kD < 0,6. EM TESTE NO ROBÔ.
    // kP 55 / kD 3,7 (LQR sem atraso) fez o telescópio pular: ~2,6% das leituras do
    // encoder chegam repetidas e o kD virava solavancos de ±0,4 V (picos de 2,4 V).
    // Com kP 18 (antes): atraso de 1–1,5 cm no perfil, passava ~1 cm ao soltar e
    // parava ~6 mm abaixo da meta depois de descer.
    // Com Motion Magic o PID roda no Talon a 1 kHz, sem esse atraso: o kP pode subir aos
    // poucos pelo painel. O IO converte para o Slot0 (voltas do rotor).
    // Independentes da massa/física simulada. Em metros da ponta.
    // Ajustáveis pelo painel (/Tuning/Telescopio9458/PID), somente com o robô desabilitado.
    public static final double REAL_KP = 35.0;  // V/m
    public static final double REAL_KI = 0.0;   // V/(m*s)
    public static final double REAL_KD = 0.0;   // V/(m/s)
    public static final double REAL_KS = 0.042; // V, atrito
    public static final double REAL_KG = 0.201; // V, gravidade
    public static final double REAL_KV = 9.36;  // V/(m/s)
    public static final double REAL_KA = 0.0;   // V/(m/s²)
    public static final TelescopioGains_9458 REAL_GAINS =
        new TelescopioGains_9458(REAL_KP, REAL_KI, REAL_KD, REAL_KS, REAL_KG, REAL_KV, REAL_KA);

    // #endregion

    // #region SYSID (calibration/Telescopio9458SysId)

    // Quasistático: rampa de 0,5 V/s. Com o kV teórico, sobe até a parada em ~6 s.
    public static final double SYSID_RAMP_VOLTS_PER_SEC = 0.5;

    // Dinâmico: degrau de 2 V, ~0,2 m/s na ponta.
    public static final double SYSID_STEP_VOLTS = 2.0;

    // Cada teste termina sozinho depois deste tempo (a rampa chega a 5 V).
    public static final double SYSID_TIMEOUT_SECONDS = 10.0;

    // Paradas do SysId: 10 cm antes do limite operacional (0,846 m) e 5 cm acima do fundo.
    // A tensão é cortada nesses pontos para sobrar espaço de frenagem.
    public static final double SYSID_UPPER_STOP_METERS = MAX_OPERATING_EXTENSION_METERS - 0.10;
    public static final double SYSID_LOWER_STOP_METERS = 0.05;

    // Pausa em 0 V no alto, entre a subida e a descida do mesmo botão.
    public static final double SYSID_PAUSE_SECONDS = 1.0;

    // Travamento no SysId: corrente e velocidade do travamento normal (JAM_*),
    // por 1 s. Mais longo que JAM_TIME_SECONDS porque a rampa começa parada.
    public static final double SYSID_STALL_TIME_SECONDS = 1.0;

    // #endregion

    // #region PROTEÇÕES

    // Limite de corrente do estator (Phoenix 6, CurrentLimitsConfigs).
    // Segurar a carga (~6 kg) usa ~6 A e o pico simulado é ~12 A: 30 A deixa ~2,5x
    // de folga e limita a força do motor na ponta a ~290 N. Validar no real.
    public static final double STATOR_CURRENT_LIMIT_AMPS = 30.0;

    // Corrente da bateria: 30 A segue o exemplo da CTRE para elevador.
    // Depois de 1 s no limite, reduz para 20 A.
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 20.0;
    public static final double SUPPLY_CURRENT_LOWER_TIME_SECONDS = 1.0;

    // Travamento: 75% do limite do estator, sem movimento, por 0,25 s.
    public static final double JAM_CURRENT_AMPS = 22.0;
    public static final double JAM_VELOCITY_METERS_PER_SEC = 0.02;
    public static final double JAM_TIME_SECONDS = 0.25;

    // #endregion

    // #region HARDWARE DO TELESCÓPIO 9458 FÍSICO

    // TalonFX (Kraken X60) no barramento do roboRIO.
    // Livre no CAN: o swerve usa 10 a 21 e o Pigeon usa 3.
    public static final int MOTOR_CAN_ID = 5;

    // Tensão positiva deve abrir. Confirmar no robô com um toque curto no R2;
    // se o telescópio fechar, trocar para true.
    public static final boolean MOTOR_INVERTED = false;

    // Ao ligar, o telescópio está sempre fechado: a posição do motor é zerada ali.
    public static final double POWER_ON_EXTENSION_METERS = 0.0;

    // Tentativas para aplicar a configuração e o zero no TalonFX.
    public static final int CONFIG_ATTEMPTS = 5;

    // Espera pela confirmação de uma configuração. A CTRE não aceita 0
    // (devolve TimeoutCannotBeZero sem aplicar). Em movimento, 25 ms: o Motion Magic
    // continua rodando no Talon enquanto o roboRIO espera.
    public static final double CONFIG_TIMEOUT_SECONDS = 0.1;
    public static final double MOTION_CONFIG_TIMEOUT_SECONDS = 0.025;

    // Leituras a 100 Hz: com 50 Hz (igual ao ciclo) ~2,6% das leituras chegavam
    // repetidas e a posição atrasava até 20 ms a mais.
    public static final double SIGNAL_UPDATE_HZ = 100.0;

    // Ao soltar o gatilho, a meta de parada chega ao Talon só no próximo ciclo (20 ms), e a
    // referência lida pode ter até um período de status (10 ms). Sem contar esse atraso, a
    // 0,75 m/s a referência passaria ~2 cm da meta e voltaria. Na simulação (sem idade de
    // leitura) a sobra só prolonga a parada em ~7 mm.
    public static final double RELEASE_DELAY_SECONDS = LOOP_PERIOD_SECONDS + 1.0 / SIGNAL_UPDATE_HZ;
    // Ao soltar com a referência abaixo de 1 mm/s, o sentido vem da meta anterior: o
    // perfil está partindo, e o sinal de uma velocidade quase zero é só ruído.
    public static final double VELOCITY_DIRECTION_THRESHOLD_METERS_PER_SEC = 0.001;

    // #endregion

    // #region VISUALIZAÇÃO (asset Robot_Telescopio_9458)

    // Tubos pelas caixas do CAD (GLB), em Z, com o telescópio fechado.
    // O topo do tubo 2" fica em 0,780 m (CLOSED_HEIGHT_METERS).
    public static final double N1_LENGTH_METERS = 0.7354;
    public static final double N2_CLOSED_BOTTOM_METERS = 0.0283;
    public static final double N2_LENGTH_METERS = 0.7089;
    public static final double N3_CLOSED_BOTTOM_METERS = 0.0450;
    public static final double N3_LENGTH_METERS = 0.7352;

    // Modelo em paralelo (real x simulado) desenhado 0,6 m ao lado do real no 3D.
    public static final double MODEL_VIEW_OFFSET_Y_METERS = 0.6;

    // No asset "FRC9458 - Telescopio", os componentes têm o zero na posição da base.
    // Poses 3D, relativas à base: tubo 3" em (0, 0, N2) e tubo 2" em (0, 0, N2 + N3).

    // #endregion

    // Impede criar objetos desta classe de constantes.
    private TelescopioConstants_9458() {
    }
}
