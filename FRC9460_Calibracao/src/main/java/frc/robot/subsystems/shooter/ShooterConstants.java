package frc.robot.subsystems.shooter;

// #region IMPORTS

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.subsystems.vision.VisionFieldConstants;

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
 * - Centrífuga: empurra a bola do depósito para a tower.
 */
public final class ShooterConstants {

    // #region IDS CAN E SENTIDOS (iguais ao robô de competição)

    // TalonFX: motor que gira o shooter.
    public static final int PIVOT_MOTOR_ID = 32;
    public static final boolean PIVOT_MOTOR_INVERTED = true;

    // Spark Flex: as duas rodas de disparo. Cada uma tem seu sentido.
    public static final int TOP_LEFT_MOTOR_ID = 37;
    public static final boolean TOP_LEFT_INVERTED = true;
    public static final int TOP_RIGHT_MOTOR_ID = 33;
    public static final boolean TOP_RIGHT_INVERTED = false;

    // Spark Flex: alimentador (tower).
    public static final int TOWER_MOTOR_ID = 30;
    public static final boolean TOWER_INVERTED = true;

    // TalonFX: centrífuga.
    public static final int CENTRIFUGE_MOTOR_ID = 36;
    public static final boolean CENTRIFUGE_INVERTED = true;

    // #endregion

    // #region PIVÔ - CURSO E ZERO MANUAL

    // Não existe sensor absoluto no pivô. Ao ligar o robô, o encoder do motor
    // vale 0 onde o shooter estiver. Por isso a equipe CENTRALIZA o shooter à
    // mão antes de ligar: esse é o zero de todas as contas abaixo.

    // Curso do código de competição: ±6 voltas do motor = ±90° a partir do zero.
    // Os ±90° são o limite da fiação; passar disso pode arrancar os cabos.
    public static final double PIVOT_MAX_ANGLE_DEGREES = 90.0;
    public static final double PIVOT_MAX_MOTOR_ROTATIONS = 6.0;

    // Consequência dos dois números acima: 15° por volta do motor, ou seja,
    // redução de 24:1. Não foi medido; confirmar contando voltas no robô.
    public static final double PIVOT_DEGREES_PER_MOTOR_ROTATION =
        PIVOT_MAX_ANGLE_DEGREES / PIVOT_MAX_MOTOR_ROTATIONS;
    public static final double PIVOT_GEAR_REDUCTION =
        360.0 / PIVOT_DEGREES_PER_MOTOR_ROTATION;

    // Para onde o shooter aponta no zero, no referencial do robô.
    // 0° = frente do robô, 90° = esquerda, 180° = para trás.
    public static final double PIVOT_ZERO_FACING_DEGREES = 180.0;

    // Sinal entre "ângulo positivo" (anti-horário, visto de cima) e voltas
    // positivas do motor. Era ajustável no SmartDashboard; o padrão era 1.
    public static final double PIVOT_DIRECTION_SIGN = 1.0;

    // "No alvo": erro menor que 0,05 volta do motor (0,75° no shooter).
    public static final double PIVOT_TOLERANCE_MOTOR_ROTATIONS = 0.05;

    // O alvo só é reenviado ao Motion Magic quando muda mais que isto.
    // Evita reiniciar o perfil a cada 20 ms por ruído da pose.
    public static final double PIVOT_RETARGET_MIN_MOTOR_ROTATIONS = 0.08;

    // #endregion

    // #region PIVÔ - CONTROLE (MOTION MAGIC DO TALONFX)

    // Ganho proporcional: volts por volta de erro do motor.
    public static final double PIVOT_KP = 2.0;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KD = 0.0;

    // Perfil do Motion Magic, em voltas do motor (rot/s, rot/s², rot/s³).
    // 5 rot/s no motor = 75°/s no shooter.
    public static final double PIVOT_CRUISE_ROTATIONS_PER_SEC = 5.0;
    public static final double PIVOT_ACCELERATION_ROTATIONS_PER_SEC2 = 10.0;
    public static final double PIVOT_JERK_ROTATIONS_PER_SEC3 = 60.0;

    public static final double PIVOT_SUPPLY_CURRENT_LIMIT_AMPS = 20.0;
    public static final double PIVOT_STATOR_CURRENT_LIMIT_AMPS = 40.0;

    // Movimento manual lento (10% da tensão), para teste de sentido.
    public static final double PIVOT_MANUAL_PERCENT = 0.10;

    // #endregion

    // #region RODAS - CONTROLE DE VELOCIDADE

    // Tolerância para considerar a roda "no RPM".
    public static final double VELOCITY_TOLERANCE_RPM = 75.0;

    // Velocidades livres usadas pelo código de competição nos limites e no kV.
    // Atenção: o Spark Flex usa o valor do NEO (5676), não o do NEO Vortex (6784).
    public static final double NEO_FREE_SPEED_RPM = 5676.0;
    public static final double KRAKEN_FREE_SPEED_RPM = 6380.0;

    // Spark Flex (top e tower). Na API REV 2026:
    // kP em fração da saída por RPM de erro; kV em volts por RPM.
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double SPARK_KP = 0.00025;
    public static final double SPARK_KV_VOLTS_PER_RPM = NOMINAL_VOLTAGE / NEO_FREE_SPEED_RPM;
    // Tempo mínimo para a saída ir de 0 a 100% (suaviza a partida).
    public static final double SPARK_CLOSED_LOOP_RAMP_SECONDS = 0.25;
    public static final int SPARK_CURRENT_LIMIT_AMPS = 60;

    // TalonFX da centrífuga (VelocityVoltage), em rotações por segundo:
    // kP em volts por rot/s de erro; kS em volts; kV em volts por rot/s.
    public static final double CENTRIFUGE_KP = 0.12;
    public static final double CENTRIFUGE_KS = 0.20;
    public static final double CENTRIFUGE_KV = 12.0 / (KRAKEN_FREE_SPEED_RPM / 60.0);
    public static final double CENTRIFUGE_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double CENTRIFUGE_STATOR_CURRENT_LIMIT_AMPS = 60.0;

    // #endregion

    // #region POSIÇÃO DO SHOOTER NO ROBÔ

    // De onde a bola sai, em relação ao centro do robô (X frente, Y esquerda).
    // Valor da competição: 5 cm para trás e 14 cm para a DIREITA.
    // O CAD mede o eixo do pivô em 14,4 cm para trás e 14,5 cm para a ESQUERDA
    // (veja VISUALIZAÇÃO). A mira usa o valor da competição até medirem no robô.
    public static final double SHOOTER_OFFSET_FORWARD_METERS = -0.05;
    public static final double SHOOTER_OFFSET_LEFT_METERS = -0.14;

    // #endregion

    // #region ALVO: CENTRO DOS HUBS

    // Calculado a partir das tags do mapa oficial carregado pela visão.
    // O código antigo usava (4,64; 4,04) fixo, mesmo jogando na aliança vermelha.
    // Hub azul: faces 26/20 dão o X; faces 18/21 dão o Y. Hub vermelho: 10/4 e 5/2.
    public static final Translation2d BLUE_HUB_CENTER = hubCenter(26, 20, 18, 21);
    public static final Translation2d RED_HUB_CENTER = hubCenter(10, 4, 5, 2);

    // Usado pelo cálculo do tempo de voo e pelo desenho 3D.
    // Valores do código antigo; conferir a altura do alvo no manual do jogo.
    public static final double RELEASE_HEIGHT_METERS = 0.80;
    public static final double TARGET_HEIGHT_METERS = 2.49;

    // #endregion

    // #region CURVA DE TIRO (igual ao código de competição)

    // A distância é limitada a esta faixa antes de entrar na curva.
    public static final double SHOT_MIN_DISTANCE_METERS = 1.10;
    public static final double SHOT_MAX_DISTANCE_METERS = 5.20;

    // Top: faixa de RPM e fator de força.
    public static final double TOP_SHOT_MIN_RPM = 2350.0;
    public static final double TOP_SHOT_MAX_RPM = 3900.0;
    public static final double TOP_FORCE_SCALE = 0.88;

    // Tower: faixa, fator e quanto ela gira acima do top.
    public static final double TOWER_SHOT_MIN_RPM = 1600.0;
    public static final double TOWER_SHOT_MAX_RPM = 2800.0;
    public static final double TOWER_FORCE_SCALE = 1.10;
    public static final double TOWER_LEAD_OVER_TOP_RPM = 180.0;

    // Centrífuga: 15% acima da tower, dentro da faixa do mecanismo.
    public static final double CENTRIFUGE_SHOT_MIN_RPM = 2200.0;
    public static final double CENTRIFUGE_SHOT_MAX_RPM = 3800.0;
    public static final double CENTRIFUGE_FORCE_SCALE = 1.15;

    // Quanto cada mecanismo contribui para a velocidade de saída estimada.
    public static final double TOP_EXIT_CONTRIBUTION = 0.68;
    public static final double TOWER_EXIT_CONTRIBUTION = 0.27;
    public static final double CENTRIFUGE_EXIT_CONTRIBUTION = 0.05;

    // Roda de 4" e perda entre roda e bola (1,0 seria perfeito).
    public static final double FLYWHEEL_RADIUS_METERS = 0.0508;
    public static final double VELOCITY_TRANSFER_EFFICIENCY = 0.80;

    // #endregion

    // #region SEQUÊNCIA DE TIRO

    // O pivô começa a mirar este tempo antes de ligar as rodas.
    public static final double PIVOT_AIM_HEADSTART_SECONDS = 0.25;

    // Código de competição: depois deste tempo, top e tower contam como prontos
    // MESMO SEM chegar ao RPM. Mantido para igualar o robô; o log marca quando
    // aconteceu (Shooter/Sequencia/ProntoPorTempo). false = exige o RPM.
    public static final boolean USE_SPINUP_TIME_FALLBACK = true;
    public static final double SPINUP_FALLBACK_SECONDS = 0.75;

    // Mantém a centrífuga ligada por oscilações de um ciclo.
    public static final double CENTRIFUGE_READY_GRACE_SECONDS = 0.08;

    // Só atira depois que a Limelight corrigiu a pose ao menos uma vez.
    // Sem isso, a pose é só o chute inicial da odometria.
    public static final boolean REQUIRE_VISION_CORRECTION = true;

    // #endregion

    // #region SIMULAÇÃO (ESTIMATIVAS)

    // Motores pelo CAD: 4 Kraken X44 (intake x2, centrífuga e pivô) e 2 NEO
    // Vortex (top). Tower como NEO, o motor usado no kV da competição.
    // Nada disso foi confirmado no robô. As inércias são estimativas.
    // Sem atrito no modelo, um kV calculado para outro motor aparece como RPM
    // acima do alvo (top e centrífuga): conferir o RPM real no primeiro teste.
    public static final DCMotor PIVOT_SIM_MOTOR = DCMotor.getKrakenX44(1);
    public static final double PIVOT_SIM_MOI_KG_M2 = 0.06;

    // Batente físico da simulação: a fiação limita a ±90° do centro real.
    public static final double PIVOT_SIM_WIRING_LIMIT_DEGREES = 90.0;

    // Erro ao centralizar o shooter à mão antes de ligar o robô.
    // 0 = colocado no centro. Ex.: 10 = ligado com o shooter 10° para a esquerda.
    // O código não sabe disso: só a simulação mostra o efeito.
    public static final double PIVOT_SIM_MANUAL_PLACEMENT_ERROR_DEGREES = 0.0;

    public static final int TOP_MOTOR_COUNT = 2;
    public static final DCMotor TOP_SIM_MOTOR = DCMotor.getNeoVortex(TOP_MOTOR_COUNT);
    public static final double TOP_SIM_MOI_KG_M2 = 0.0025;
    public static final DCMotor TOWER_SIM_MOTOR = DCMotor.getNEO(1);
    public static final double TOWER_SIM_MOI_KG_M2 = 0.0008;
    public static final DCMotor CENTRIFUGE_SIM_MOTOR = DCMotor.getKrakenX44(1);
    public static final double CENTRIFUGE_SIM_MOI_KG_M2 = 0.005;

    // O código de competição lê o RPM do motor; sem redução conhecida, 1:1.
    public static final double WHEEL_SIM_GEARING = 1.0;

    // #endregion

    // #region VISUALIZAÇÃO (MODELO 3D DO ADVANTAGESCOPE)

    // Pivôs medidos no CAD (config/advantagescope/docs/MODELO_3D.md),
    // na ordem dos arquivos do modelo: model_0 = intake, model_1 = shooter.
    public static final Translation3d INTAKE_PIVOT_IN_ROBOT = new Translation3d(0.2785, 0.0, 0.2389);
    public static final Translation3d TURRET_PIVOT_IN_ROBOT = new Translation3d(-0.1439, 0.1449, 0.3475);

    // No CAD, a marca "FrenteShooter" aponta para 175,5° (não exatamente 180°).
    // O desenho compensa a diferença para mostrar o mesmo ângulo que o código usa.
    public static final double CAD_ZERO_FACING_DEGREES = 175.5;

    // #endregion

    // #region CENTRO DO HUB PELAS TAGS

    private static Translation2d hubCenter(int frontTagX, int backTagX, int sideTagY, int otherSideTagY) {
        return new Translation2d(
            (tagXY(frontTagX).getX() + tagXY(backTagX).getX()) / 2.0,
            (tagXY(sideTagY).getY() + tagXY(otherSideTagY).getY()) / 2.0);
    }

    private static Translation2d tagXY(int id) {
        return VisionFieldConstants.FIELD_LAYOUT.getTagPose(id)
            .orElseThrow(() -> new IllegalStateException("Tag " + id + " ausente no mapa do campo"))
            .toPose2d().getTranslation();
    }

    // #endregion

    private ShooterConstants() {}
}
