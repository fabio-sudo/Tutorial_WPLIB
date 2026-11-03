package frc.robot.subsystems.shooter;

// #region IMPORTS

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.CanIds;
import frc.robot.subsystems.vision.FieldTags;

// #endregion

/**
 * Todos os números do shooter ficam aqui.
 *
 * Origem: código usado na competição (ProjetoFrc, agosto de 2026). Quando um
 * valor é estimativa (CAD ou simulação), o comentário avisa. Mudou algo no
 * robô? Mude aqui: os outros arquivos do shooter não guardam calibração.
 *
 * Nomes mantidos do código de competição:
 * - Pivô: gira o shooter inteiro (no modelo 3D é a "torre que gira", model_1).
 * - Top: as duas rodas de disparo (flywheel).
 * - Tower: alimentador que leva a bola até o top. NÃO gira o shooter.
 * - Centrífuga: empurra a bola do depósito para a tower. Tem pasta própria
 *   (subsystems/centrifuga); aqui fica só o RPM dela na curva de tiro.
 */
public final class ShooterConstants {

    // #region IDS CAN E SENTIDOS (iguais ao robô de competição)

    // Os números ficam no mapa único frc.robot.CanIds.

    // TalonFX: motor que gira o shooter.
    public static final int PIVOT_MOTOR_ID = CanIds.SHOOTER_PIVOT;
    public static final boolean PIVOT_MOTOR_INVERTED = true;

    // Spark Flex: as duas rodas de disparo. Cada uma tem seu sentido.
    public static final int TOP_LEFT_MOTOR_ID = CanIds.SHOOTER_TOP_LEFT;
    public static final boolean TOP_LEFT_INVERTED = true;
    public static final int TOP_RIGHT_MOTOR_ID = CanIds.SHOOTER_TOP_RIGHT;
    public static final boolean TOP_RIGHT_INVERTED = false;

    // Spark Flex: alimentador (tower).
    public static final int TOWER_MOTOR_ID = CanIds.SHOOTER_TOWER;
    public static final boolean TOWER_INVERTED = true;


    // #endregion

    // #region 📏🧭 PIVÔ - CURSO E ZERO MANUAL

    // Não existe sensor absoluto no pivô. Ao ligar o robô, o encoder do motor
    // vale 0 onde o shooter estiver. Por isso a equipe CENTRALIZA o shooter à
    // mão antes de ligar: esse é o zero de todas as contas abaixo.

    // CAN 32: redutor informado 9:1, mas a conversão FINAL até a mira vem
    // da medição do conjunto. Não confundir redução do redutor com a total.
    public static final double PIVOT_MOTOR_GEARBOX_REDUCTION = 9.0;
    // Desktop/logs/tuner-plot.csv e print: patamar inicial = centro físico;
    // patamar +8,895020 = 132° medidos. Descontar o offset inicial do encoder.
    public static final double PIVOT_CALIBRATION_START_ROTOR_ROTATIONS = -0.084961;
    public static final double PIVOT_CALIBRATION_END_ROTOR_ROTATIONS = 8.895020;
    public static final double PIVOT_CALIBRATION_ANGLE_DEGREES = 132.0;
    public static final double PIVOT_CALIBRATION_TRAVEL_ROTATIONS =
        PIVOT_CALIBRATION_END_ROTOR_ROTATIONS - PIVOT_CALIBRATION_START_ROTOR_ROTATIONS;
    // Limite simétrico solicitado: 130°, antes do ponto medido de 132°.
    // É margem geométrica de 2°, não garantia de frenagem sob qualquer carga.
    // 🛑📏 LIMITE DE CURSO DO PIVÔ: a mira, o IO e os soft limits do Talon usam este número.
    public static final double PIVOT_MAX_ANGLE_DEGREES = 130.0;
    public static final double PIVOT_DEGREES_PER_MOTOR_ROTATION =
        PIVOT_CALIBRATION_ANGLE_DEGREES / PIVOT_CALIBRATION_TRAVEL_ROTATIONS;
    // Redução efetiva calibrada (~24,49:1), não contagem de dentes confirmada.
    public static final double PIVOT_GEAR_REDUCTION = 360.0 / PIVOT_DEGREES_PER_MOTOR_ROTATION;
    public static final double PIVOT_MAX_MOTOR_ROTATIONS =
        PIVOT_MAX_ANGLE_DEGREES / PIVOT_DEGREES_PER_MOTOR_ROTATION;

    // Para onde o shooter aponta no zero, no referencial do robô.
    // 0° = frente do robô, 90° = esquerda, 180° = para trás.
    public static final double PIVOT_ZERO_FACING_DEGREES = 180.0;

    // Sinal entre "ângulo positivo" (anti-horário, visto de cima) e voltas
    // positivas do motor. Era ajustável no SmartDashboard; o padrão era 1.
    public static final double PIVOT_DIRECTION_SIGN = 1.0;

    // #endregion

    // #region 🛑⚡ PIVÔ - SEGURANÇA (LIMITES DE CORRENTE)

    // O curso convertido para voltas também vira soft limit no TalonFX.
    // ⚡ Supply = corrente tirada da bateria. Estator = corrente no motor (força).
    public static final double PIVOT_SUPPLY_CURRENT_LIMIT_AMPS = 20.0;
    public static final double PIVOT_STATOR_CURRENT_LIMIT_AMPS = 40.0;

    // #endregion

    // #region 🎯🚀 PIVÔ - CONTROLE (MOTION MAGIC NO TALONFX)

    // Mantém a tolerância angular de 0,75° com a redução correta.
    public static final double PIVOT_TOLERANCE_MOTOR_ROTATIONS =
        0.75 / PIVOT_DEGREES_PER_MOTOR_ROTATION;

    // O alvo só é reenviado ao Motion Magic quando muda mais que isto.
    // Filtra só o ruído da pose. Precisa ficar ABAIXO da tolerância: com
    // 1,2° (> 0,75°) o alvo enviado podia ficar até 1,2° atrás da mira
    // certa e o pivô "chegava" no lugar errado. Trocar o alvo no meio do
    // movimento não reinicia o perfil: o Talon parte do estado atual.
    public static final double PIVOT_RETARGET_MIN_MOTOR_ROTATIONS =
        0.3 / PIVOT_DEGREES_PER_MOTOR_ROTATION;

    // 🎯 PID DO PIVÔ (roda dentro do TalonFX): kP corrige o erro, kD amortece,
    // kS vence o atrito e kV acompanha a velocidade do perfil.
    // Ganho proporcional: volts por volta de erro do motor. Com o kV abaixo,
    // o kP só corrige o erro que sobra; antes (2,0, sem kV) ele empurrava o
    // movimento inteiro e o pivô ficava ~5° atrás do perfil.
    public static final double PIVOT_KP = 6.0;
    public static final double PIVOT_KI = 0.0;
    // Amortece o kP maior (volts por volta/s de erro de velocidade).
    public static final double PIVOT_KD = 0.07;
    // Ensaio: ~3% para mover (nominalmente 0,36 V a 12 V). Começa abaixo
    // disso para reduzir oscilação perto do alvo; validar no conjunto real.
    // Log de 29/10: erro residual 1,7–2,9° impedia alimentar (tolerância 0,75°).
    public static final double PIVOT_KS_VOLTS = 0.30;
    // Banda neutra do Talon (fração de 12 V): pedido menor que isto vira neutro.
    // O kS usa o sinal do erro, e a CTRE avisa que isso faz a saída tremer com o
    // erro perto de zero: parado no alvo, o Talon alternava +0,30 V e -0,30 V, o
    // barulho de segurar o botão de mirar com o shooter já na posição. Com 0,48 V,
    // acima do kS, o pivô fica em Brake a até ~0,44° do alvo (dentro da tolerância
    // de 0,75°) e só empurra com tensão que já vence o atrito (~0,36 V).
    public static final double PIVOT_NEUTRAL_DEADBAND = 0.04;
    // Tensão para girar a 1 volta/s do motor, pela velocidade livre do
    // Kraken X44 no modelo da WPILib: ~0,092 V. Faltava: sem ele o perfil
    // do Motion Magic não era seguido. Medir no robô: tensão / velocidade
    // no trecho de velocidade constante (AdvantageScope, Shooter/Pivo/Inputs).
    public static final double PIVOT_KV_VOLTS_PER_ROTATION_PER_SEC =
        2.0 * Math.PI / DCMotor.getKrakenX44(1).KvRadPerSecPerVolt;

    // 🚀 VELOCIDADE DA MIRA: cruzeiro, aceleração e jerk do pivô.
    // Perfil do Motion Magic, em voltas do motor (rot/s, rot/s², rot/s³).
    // Pedido da equipe: mira mais rápida, com desaceleração antes do alvo.
    // 29/10/2026: 110°/s e 220°/s² levavam 1,3 s para 90° (simulação).
    // Ainda é ~15% da velocidade livre do pivô (~1800°/s) e ~8% da
    // aceleração que 40 A dão com a inércia estimada. Se o pivô passar do
    // alvo ou a fiação sofrer, reduza primeiro a aceleração.
    public static final double PIVOT_CRUISE_DEGREES_PER_SEC = 270.0;
    public static final double PIVOT_ACCELERATION_DEGREES_PER_SEC2 = 1080.0;
    // Jerk: a aceleração sobe em 0,1 s (suaviza a partida e a frenagem).
    public static final double PIVOT_JERK_DEGREES_PER_SEC3 = 10800.0;
    public static final double PIVOT_CRUISE_ROTATIONS_PER_SEC =
        PIVOT_CRUISE_DEGREES_PER_SEC / PIVOT_DEGREES_PER_MOTOR_ROTATION;
    public static final double PIVOT_ACCELERATION_ROTATIONS_PER_SEC2 =
        PIVOT_ACCELERATION_DEGREES_PER_SEC2 / PIVOT_DEGREES_PER_MOTOR_ROTATION;
    public static final double PIVOT_JERK_ROTATIONS_PER_SEC3 =
        PIVOT_JERK_DEGREES_PER_SEC3 / PIVOT_DEGREES_PER_MOTOR_ROTATION;

    // 🚀 VELOCIDADE DO MODO MANUAL (botão A do operador).
    // Equipe mediu movimento perto de 3% com o conjunto montado.
    // Não é kS nem kG; mantém o ajuste manual lento perto da fiação.
    public static final double PIVOT_MANUAL_PERCENT = 0.03;

    // #endregion

    // #region 🛑⚡ RODAS - SEGURANÇA (LIMITE DE CORRENTE)

    // Por motor, dentro da faixa da REV para o Spark Flex (frc.robot.MotorLimits).
    public static final int SPARK_CURRENT_LIMIT_AMPS = 60;

    // #endregion

    // #region 🎯🚀 RODAS - CONTROLE (PID DE VELOCIDADE NO SPARK FLEX)

    // 🚀 RPM DAS RODAS: tolerância que libera o tiro e velocidade livre usada nas contas.
    // Tolerância para considerar a roda "no RPM".
    public static final double VELOCITY_TOLERANCE_RPM = 75.0;

    // Velocidade livre usada pelo código de competição no limite e no kV.
    // Atenção: o Spark Flex usa o valor do NEO (5676), não o do NEO Vortex (6784).
    public static final double NEO_FREE_SPEED_RPM = 5676.0;

    // 🎯 PID DAS RODAS (roda dentro do Spark Flex): kP corrige o erro de RPM, kV dá a tensão base.
    // 🚧 ATENÇÃO (pente fino 31/10/2026): o kV usa o NEO (5676), mas o top é NEO Vortex (6784). Pela conta,
    // a 3000 RPM o top estabiliza ~7% acima do alvo, longe da tolerância de 75 RPM: o tiro
    // tende a sair sempre "pronto por tempo". Conferir Shooter/Sequencia/ProntoPorTempo.
    // Mudar o kV exige recalibrar TOP_FORCE_SCALE.
    // Spark Flex (top e tower). Na API REV 2026:
    // kP em fração da saída por RPM de erro; kV em volts por RPM.
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double SPARK_KP = 0.00025;
    public static final double SPARK_KV_VOLTS_PER_RPM = NOMINAL_VOLTAGE / NEO_FREE_SPEED_RPM;
    // Tempo mínimo para a saída ir de 0 a 100% (suaviza a partida).
    public static final double SPARK_CLOSED_LOOP_RAMP_SECONDS = 0.25;

    // #endregion

    // #region 🧭 POSIÇÃO DO SHOOTER NO ROBÔ

    // 🚧 ATENÇÃO (pente fino 31/10/2026): competição e CAD diferem 29 cm na lateral, ou ~5° de mira a 3 m.
    // Medir o eixo do pivô no robô: é o maior erro geométrico que sobrou na mira.
    // De onde a bola sai, em relação ao centro do robô (X frente, Y esquerda).
    // Valor da competição: 5 cm para trás e 14 cm para a DIREITA.
    // O CAD mede o eixo do pivô em 14,4 cm para trás e 14,5 cm para a ESQUERDA
    // (veja VISUALIZAÇÃO). A mira usa o valor da competição até medirem no robô.
    public static final double SHOOTER_OFFSET_FORWARD_METERS = -0.05;
    public static final double SHOOTER_OFFSET_LEFT_METERS = -0.14;

    // #endregion

    // #region 🧭 ALVO: CENTRO DOS HUBS

    // Sem FMS (treino), o shooter mira SEMPRE neste hub: o mesmo alvo fixo do
    // código de competição (4,64; 4,04), que é o hub AZUL. Fora do FMS a
    // Driver Station vem como Red 1; seguir a aliança no treino fazia o
    // shooter mirar no hub do outro lado do campo (teste de 28/10/2026).
    // Com FMS (competição), vale o hub da aliança.
    public static final Alliance PRACTICE_HUB_ALLIANCE = Alliance.Blue;

    // Calculado a partir das tags do mapa oficial carregado pela visão.
    // O código antigo usava (4,64; 4,04) fixo, mesmo jogando na aliança vermelha.
    // Hub azul: faces 26/20 dão o X; faces 18/21 dão o Y. Hub vermelho: 10/4 e 5/2.
    public static final Translation2d BLUE_HUB_CENTER = FieldTags.centerBetween(26, 20, 18, 21);
    public static final Translation2d RED_HUB_CENTER = FieldTags.centerBetween(10, 4, 5, 2);

    // Alturas usadas no desenho 3D e na física das bolas (não mudam o RPM).
    // Saída: roda de cima do shooter no CAD (0,535 m) + raio da bola.
    // Alvo: borda da boca do hub no modelo oficial do campo 2026 = 1,829 m
    // (72"). O código antigo usava 0,80 e 2,49 m, sem origem conhecida.
    public static final double RELEASE_HEIGHT_METERS = 0.61;
    public static final double TARGET_HEIGHT_METERS = 1.829;

    // #endregion

    // #region 🚀 CURVA DE TIRO (igual ao código de competição)

    // 📏 Alcance da curva de tiro.
    // A distância é limitada a esta faixa antes de entrar na curva.
    public static final double SHOT_MIN_DISTANCE_METERS = 1.10;
    public static final double SHOT_MAX_DISTANCE_METERS = 5.20;

    // 🛑 Trava de alcance do tiro.
    // Acima de 5,2 m a curva só repete o RPM máximo: o código antigo atirava
    // assim mesmo e errava (12 a 20 m nos testes). Com folga de 0,5 m, a
    // sequência mira e acelera, mas não alimenta. false = como na competição.
    public static final boolean BLOCK_SHOTS_BEYOND_CURVE = true;
    public static final double SHOT_DISTANCE_MARGIN_METERS = 0.50;

    // 🚀 RPM DO TIRO: top, tower e centrífuga.
    // Top: faixa de RPM e fator de força.
    public static final double TOP_SHOT_MIN_RPM = 2350.0;
    public static final double TOP_SHOT_MAX_RPM = 3900.0;
    // Incremento de ~8% após o teste físico; recalibrar os acertos por distância.
    public static final double TOP_FORCE_SCALE = 0.95;

    // Tower: faixa, fator e quanto ela gira acima do top.
    public static final double TOWER_SHOT_MIN_RPM = 1600.0;
    public static final double TOWER_SHOT_MAX_RPM = 2800.0;
    public static final double TOWER_FORCE_SCALE = 1.10;
    public static final double TOWER_LEAD_OVER_TOP_RPM = 180.0;

    // Centrífuga: 15% acima da tower, dentro da faixa do mecanismo.
    public static final double CENTRIFUGE_SHOT_MIN_RPM = 2200.0;
    public static final double CENTRIFUGE_SHOT_MAX_RPM = 4000.0;
    public static final double CENTRIFUGE_FORCE_SCALE = 1.15;

    // Quanto cada mecanismo contribui para a velocidade de saída estimada.
    public static final double TOP_EXIT_CONTRIBUTION = 0.68;
    public static final double TOWER_EXIT_CONTRIBUTION = 0.27;
    public static final double CENTRIFUGE_EXIT_CONTRIBUTION = 0.05;

    // Roda de 4" e perda entre roda e bola (1,0 seria perfeito).
    public static final double FLYWHEEL_RADIUS_METERS = 0.0508;
    public static final double VELOCITY_TRANSFER_EFFICIENCY = 0.80;

    // #endregion

    // #region 🛑🚀 SEQUÊNCIA DE TIRO

    // 🔁 Abaixo, os tempos da sequência e as travas que liberam a bola.
    // O pivô começa a mirar este tempo antes de ligar as rodas.
    public static final double PIVOT_AIM_HEADSTART_SECONDS = 0.25;

    // Código de competição: depois deste tempo, top e tower contam como prontos
    // mesmo fora da tolerância (o kV do NEO no Vortex deixa o RPM ~7% acima).
    // O log marca quando aconteceu (Shooter/Sequencia/ProntoPorTempo).
    // false = exige o RPM na tolerância.
    public static final boolean USE_SPINUP_TIME_FALLBACK = true;
    public static final double SPINUP_FALLBACK_SECONDS = 0.75;

    // 🛑 Segurança do "pronto por tempo": a roda precisa estar girando de verdade,
    // com pelo menos esta fração do RPM pedido. Roda parada ou travada nunca
    // libera a bola. Antes o tempo liberava com qualquer RPM.
    public static final double SPINUP_FALLBACK_MIN_RPM_FRACTION = 0.85;

    // Mantém a centrífuga ligada por oscilações de um ciclo.
    public static final double CENTRIFUGE_READY_GRACE_SECONDS = 0.08;
    // Exige prontidão contínua antes da primeira alimentação ou de retomar.
    public static final double FEED_READY_STABLE_SECONDS = 0.10;
    // Botão de tiro segurado por mais que isto sem ligar a centrífuga: a Driver
    // Station mostra qual trava segurou (estado e erro do pivô).
    public static final double NO_FEED_WARNING_SECONDS = 1.0;

    // 🛑📷 Só atira depois que a Limelight corrigiu a pose ao menos uma vez.
    // Sem isso, a pose é só o chute inicial da odometria.
    public static final boolean REQUIRE_VISION_CORRECTION = true;

    // #endregion

    // #region 🧪 SIMULAÇÃO (ESTIMATIVAS)

    // Motores pelo CAD: 4 Kraken X44 (intake x2, centrífuga e pivô) e 2 NEO
    // Vortex (top). Tower como NEO, o motor usado no kV da competição.
    // Nada disso foi confirmado no robô. As inércias são estimativas.
    // Sem atrito no modelo, um kV calculado para outro motor aparece como RPM
    // acima do alvo (top e centrífuga): conferir o RPM real no primeiro teste.
    public static final DCMotor PIVOT_SIM_MOTOR = DCMotor.getKrakenX44(1);
    public static final double PIVOT_SIM_MOI_KG_M2 = 0.06;

    // Envelope de referência da simulação: ±132° informados pela equipe.
    // Não caracteriza rigidez/elasticidade da fiação física.
    public static final double PIVOT_SIM_WIRING_LIMIT_DEGREES = PIVOT_CALIBRATION_ANGLE_DEGREES;

    // Erro ao centralizar o shooter à mão antes de ligar o robô.
    // 0 = colocado no centro. Ex.: 10 = ligado com o shooter 10° para a esquerda.
    // O código não sabe disso: só a simulação mostra o efeito.
    public static final double PIVOT_SIM_MANUAL_PLACEMENT_ERROR_DEGREES = 0.0;

    public static final int TOP_MOTOR_COUNT = 2;
    public static final DCMotor TOP_SIM_MOTOR = DCMotor.getNeoVortex(TOP_MOTOR_COUNT);
    public static final double TOP_SIM_MOI_KG_M2 = 0.0025;
    public static final DCMotor TOWER_SIM_MOTOR = DCMotor.getNEO(1);
    public static final double TOWER_SIM_MOI_KG_M2 = 0.0008;

    // Cada bola que passa tira esta fração da velocidade do top (estimativa).
    // É o que permite contar disparos pela queda de RPM, no robô e aqui.
    public static final double SIM_RPM_LOSS_PER_BALL_FRACTION = 0.06;

    // O código de competição lê o RPM do motor; sem redução conhecida, 1:1.
    public static final double WHEEL_SIM_GEARING = 1.0;

    // #endregion

    // #region 🧪 MODELO EM PARALELO (REAL X SIMULADO)

    // Diferença aceitável entre o mecanismo e a física recebendo a mesma
    // tensão (frc.robot.util.ParallelModel). Acima disso por 0,5 s, o
    // diagnóstico mostra REAL_DIFERENTE_DO_MODELO. Calibrar no robô real.
    public static final double PIVOT_MODEL_TOLERANCE_DEGREES = 3.0;
    // Abaixo disto o pivô está parado: real e modelo parados = realinha.
    public static final double PIVOT_MODEL_REST_DEGREES_PER_SEC = 2.0;
    // O modelo das rodas não tem bolas: tiro contínuo deixa o top ~300 RPM
    // abaixo do modelo (medido na simulação). Acima de 500 por 0,5 s é outra coisa.
    public static final double TOP_MODEL_TOLERANCE_RPM = 500.0;
    public static final double TOWER_MODEL_TOLERANCE_RPM = 300.0;

    // #endregion

    // #region VISUALIZAÇÃO (MODELO 3D DO ADVANTAGESCOPE)

    // Pivô do shooter medido no CAD (config/advantagescope/docs/MODELO_3D.md).
    // O desenho das peças móveis fica em frc.robot.RobotComponentsVisualizer.
    public static final Translation3d TURRET_PIVOT_IN_ROBOT = new Translation3d(-0.1439, 0.1449, 0.3475);

    // No CAD, a marca "FrenteShooter" aponta para 175,5° (não exatamente 180°).
    // O desenho compensa a diferença para mostrar o mesmo ângulo que o código usa.
    public static final double CAD_ZERO_FACING_DEGREES = 175.5;

    // #endregion

    // #region SEM INSTÂNCIAS

    private ShooterConstants() {}

    // #endregion
}
