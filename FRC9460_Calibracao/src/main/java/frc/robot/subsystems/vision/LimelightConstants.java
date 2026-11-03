package frc.robot.subsystems.vision;

import java.util.stream.IntStream;

/**
 * As Limelights físicas: nome na rede, IDs de tags aceitos e montagem no robô.
 * Os limites de aceitação das poses (idade, giro, distância) ficam em
 * VisionConstants; a óptica das câmeras virtuais, em VisionSimulationConstants.
 */
public final class LimelightConstants {

    // #region 📷 LIMELIGHT TRASEIRA

    public static final String REAR_NAME = "limelight-rear";

    // Todas as tags do campo. Antes eram só 25, 26, 9 e 10, e as 25 e 26 ficam na
    // mesma face do hub azul, virada para a parede. Girando o robô ou saindo da
    // frente dessa face, a câmera ficava sem tag aceita e a pose parava de ser
    // corrigida. Pelas posições oficiais, havia tag em 24% das poses de tiro (0%
    // com o pivô acima de 60°); com todas, 75%. A simulação não aplica este filtro,
    // por isso o problema não aparecia nela.
    // Treino com tag fora da posição oficial: restrinja aqui, senão a pose salta.
    public static final int[] VALID_TAG_IDS = IntStream.rangeClosed(1, 32).toArray();

    public static final int IMU_MODE = 0;

    // 📏🔧 Posição da câmera no robô: erro aqui desloca a pose inteira.
    public static final double REAR_FORWARD = -0.29;
    public static final double REAR_RIGHT = 0.16;
    public static final double REAR_UP = 0.46;

    public static final double REAR_ROLL = 0.0;
    public static final double REAR_PITCH = 4.0;
    public static final double REAR_YAW = 180.0;

    // #endregion

    // #region 📷 LIMELIGHT FRONTAL PROVISÓRIA

    public static final String FRONT_NAME = "limelight-front";

    public static final int[] FRONT_VALID_TAG_IDS = VALID_TAG_IDS;

    // Valores provisórios baseados na câmera traseira.
    // Substituir pelas medidas reais da câmera frontal.
    public static final double FRONT_FORWARD = 0.29;
    public static final double FRONT_RIGHT = 0.16;
    public static final double FRONT_UP = 0.46;

    public static final double FRONT_ROLL = 0.0;
    public static final double FRONT_PITCH = 4.0;
    public static final double FRONT_YAW = 0.0;

    // #endregion

    // #region SEM INSTÂNCIAS

    private LimelightConstants() {}

    // #endregion
}
