package frc.robot.subsystems.fuel;

/**
 * Números das bolas (Fuel 2026): física do voo, coleta, depósito, hub e a
 * detecção de coletas e disparos pelos sensores.
 *
 * Quase tudo aqui é ESTIMATIVA para a simulação. Os valores de detecção
 * (corrente e queda de RPM) precisam ser calibrados com o robô real.
 */
public final class FuelConstants {

    // #region BOLA

    // Diâmetro de 15 cm (raio medido no modelo do campo do AdvantageScope).
    public static final double BALL_RADIUS_METERS = 0.075;
    // Estimativas: espuma de alta densidade, esfera lisa.
    public static final double BALL_MASS_KG = 0.21;
    public static final double DRAG_COEFFICIENT = 0.47;
    public static final double AIR_DENSITY_KG_M3 = 1.2;
    public static final double GRAVITY = 9.81;

    // Desaceleração do arrasto por (velocidade²): 0,5 × ρ × Cd × área / massa.
    public static final double DRAG_PER_METER = 0.5 * AIR_DENSITY_KG_M3 * DRAG_COEFFICIENT
        * Math.PI * BALL_RADIUS_METERS * BALL_RADIUS_METERS / BALL_MASS_KG;

    // #endregion

    // #region LANÇAMENTO

    // Ângulo de saída e eficiência roda -> bola. NENHUM dos dois foi medido.
    // Escolhidos juntos para a curva de RPM da competição cair dentro do hub
    // de 1,5 a 5 m (erro máximo 0,29 m, com arrasto) num arco de ~3,6 m de
    // altura. Com 80° e a eficiência da competição (0,80) a bola subia ~6 m.
    // Medir no robô (ângulo do capô e alcance real) e corrigir aqui.
    // 29/10/2026: o teste físico pediu +8% no top (TOP_FORCE_SCALE 0,88 ->
    // 0,95): a bola real sai mais devagar do que este modelo supunha. Com
    // 0,50 todo tiro simulado passava do hub (0,2 a 1,2 m além do centro).
    // 0,465 devolve o ajuste: de 1,1 a 5,2 m, no máximo 0,37 m do centro.
    public static final double LAUNCH_ANGLE_DEGREES = 62.0;
    public static final double BALL_SPEED_EFFICIENCY = 0.465;

    // #endregion

    // #region HUB

    // Raio interno da boca hexagonal do hub, medido no modelo oficial do campo
    // (1,07 m entre lados opostos). A bola conta como acerto quando desce pela
    // altura da boca (ShooterConstants.TARGET_HEIGHT_METERS) dentro deste raio.
    public static final double HUB_OPENING_RADIUS_METERS = 0.535;

    // #endregion

    // #region DEPÓSITO E ALIMENTAÇÃO

    // Estimativas: quantas bolas cabem e quantas saem por segundo.
    public static final int HOPPER_CAPACITY = 40;
    // Bolas já no robô ao começar a simulação (pré-carga; conferir no manual).
    public static final int PRELOADED_BALLS = 8;
    public static final double FEED_PERIOD_SECONDS = 0.20;

    // A bola só sobe com a centrífuga comandada e girando E a tower girando.
    public static final double FEED_MIN_CENTRIFUGE_RPM = 1500.0;
    public static final double FEED_MIN_TOWER_RPM = 1000.0;

    // #endregion

    // #region COLETA

    // O intake precisa estar quase aberto (até 20° de subida) e o rolo perto
    // da velocidade de trabalho: ~88% dos ~2035 RPM do NEO Vortex a 30% sem
    // carga. No robô real, conferir Roller/Rpm puxando sem bola e ajustar.
    public static final double INTAKE_COLLECT_MAX_RAISE_DEGREES = 20.0;
    public static final double ROLLER_COLLECT_MIN_RPM = 1800.0;

    // Boca do intake no referencial do robô (X frente, Y esquerda), medida no
    // CAD pelo rolo: centro da bola entre 0,45 e 0,70 m à frente, ±0,335 m.
    public static final double CAPTURE_MIN_X_METERS = 0.45;
    public static final double CAPTURE_MAX_X_METERS = 0.70;
    public static final double CAPTURE_HALF_WIDTH_METERS = 0.335;
    public static final double CAPTURE_MAX_HEIGHT_METERS = 0.20;

    // No máximo uma bola a cada 0,2 s (5 por segundo, estimativa do rolo).
    // Com bolas mais juntas a corrente não cai entre uma e outra e a contagem
    // pela corrente junta duas bolas em uma (no robô real também).
    public static final double COLLECT_PERIOD_SECONDS = 0.20;

    // Posições de largada extraídas do campo 2026 do AdvantageScope.
    public static final String INITIAL_POSITIONS_FILE = "simulacao/fuel_2026_inicial.json";

    // #endregion

    // #region DETECÇÃO PELOS SENSORES (ROBÔ REAL E SIMULAÇÃO)

    // Coleta: o rolo puxando e a corrente acima disto em duas leituras
    // seguidas (20 ms). Rearma quando a corrente cai abaixo de 70% do limite.
    // Enquanto o rolo acelera (abaixo de ROLLER_COLLECT_MIN_RPM desde que
    // ligou) nada conta: a corrente de partida do motor parece uma bola.
    public static final double COLLECT_CURRENT_AMPS = 12.0;
    public static final int COLLECT_MIN_READINGS = 2;
    public static final double COLLECT_REARM_FRACTION = 0.7;

    // Disparo: com a centrífuga GIRANDO (RPM medido, não o comando: ela
    // empurra bola enquanto gira) e o top comandado, o RPM do top cai pelo
    // menos isto em 60 ms (a bola rouba energia da roda). Uma contagem a
    // cada 0,1 s. Top sem comando desacelera sozinho e não conta.
    public static final double SHOT_RPM_DROP = 100.0;
    public static final double SHOT_DROP_WINDOW_SECONDS = 0.06;
    public static final double SHOT_REARM_SECONDS = 0.10;

    // #endregion

    // #region CONSTÂNCIA DOS TIROS (ESTATÍSTICA ENTRE OS DISPAROS)

    // Janela móvel: média e desvio dos últimos N disparos detectados. Poucos
    // tiros dão média instável; muitos misturam ajustes antigos com os atuais.
    public static final int CONSISTENCY_WINDOW_SHOTS = 10;
    // Abaixo disto o diagnóstico só diz que faltam tiros (desvio com 2 amostras engana).
    public static final int CONSISTENCY_MIN_SHOTS = 4;

    // Limites do diagnóstico em texto (Fuel/Consistencia/Diagnostico). Valores
    // INICIAIS, sem medição no robô: ajustar depois de algumas sessões de
    // tiros anotados. Viés = média; dispersão = desvio entre os tiros.
    // Velocidade de saída: desvio relativo (desvio / média) acima disto = inconstante.
    public static final double CONSISTENCY_SPEED_MAX_CV = 0.03;
    // Mira no disparo (direção do alvo menos direção do shooter), em graus.
    public static final double CONSISTENCY_AIM_MAX_SIGMA_DEGREES = 1.0;
    public static final double CONSISTENCY_AIM_MAX_BIAS_DEGREES = 1.0;
    // Erro de queda previsto pelo modelo (distância ao centro do hub): média
    // acima de 60% do raio da boca já deixa pouca margem.
    public static final double CONSISTENCY_MISS_MAX_MEAN_FRACTION_OF_RADIUS = 0.60;
    // RPM do top no disparo diferente do pedido por mais que isto, em média:
    // abaixo = a roda não chega ao alvo (bateria, ganho do PID, ritmo de
    // alimentação); acima = o kV/ganho não bate com o motor (visto no simulador).
    public static final double CONSISTENCY_TOP_RPM_MAX_MEAN_ERROR = 150.0;

    // #endregion

    // #region TRAJETÓRIA PREVISTA

    // Desenhada sempre que o top gira acima disto: onde a bola iria agora.
    // Calculada a cada 10 ms de voo, mas gravada com um ponto a cada 0,1 s
    // e a 10 Hz: era a maior parte do log (40 MB em 8 minutos).
    public static final double PREDICTION_MIN_TOP_RPM = 500.0;
    public static final double PREDICTION_OUTPUT_STEP_SECONDS = 0.10;
    public static final int PREDICTION_LOG_EVERY_CYCLES = 5;
    public static final double PREDICTION_MAX_SECONDS = 4.0;
    public static final double PREDICTION_STEP_SECONDS = 0.01;

    // #endregion

    // #region SEM INSTÂNCIAS

    private FuelConstants() {}

    // #endregion
}
