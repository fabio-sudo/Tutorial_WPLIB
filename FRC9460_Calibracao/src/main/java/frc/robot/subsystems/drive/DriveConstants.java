package frc.robot.subsystems.drive;

/**
 * Números do swerve (Drive), todos na pasta do subsistema.
 * Os IDs CAN dos motores, CANcoders e do Pigeon ficam no mapa único
 * frc.robot.CanIds. Os nomes com prefixo "k" vêm do código antigo e foram
 * mantidos para não mudar quem já os conhece.
 */
public final class DriveConstants {

    // #region 📏 GEOMETRIA DO ROBÔ

    // Distância entre os módulos dianteiros e traseiros
    // Medida de centro a centro dos módulos
    public static final double kWheelBaseMeters =
        0.5461;

    // Distância entre os módulos da esquerda e direita
    // Medida de centro a centro dos módulos
    public static final double kTrackWidthMeters =
        0.5461;

    // Raio da roda do módulo Swerve: 2 polegadas = 0,0508 m.
    // Único valor usado pela simulação, pelo Module e pelo ModuleIOTalonFX.
    public static final double kWheelRadiusMeters = 0.0508;

    // #endregion

    // #region 📏 MÓDULOS - REDUÇÕES

    // Relação do Drive:
    //
    // 6.746 rotações do motor
    // =
    // 1 rotação da roda
    public static final double DRIVE_GEAR_RATIO = 6.746031746031747;

    // Relação do Steer:
    //
    // 21.428 rotações do motor
    // =
    // 1 rotação completa do módulo
    public static final double STEER_GEAR_RATIO = 21.428571428571427;

    // #endregion

    // #region 📏 MÓDULOS - OFFSETS DOS CANCODERS E SENTIDOS

    // 🔧 Offset (rotações) medido com as rodas alinhadas para frente.
    // Refazer se um CANcoder ou um módulo for trocado.

    public static final double FRONT_LEFT_ENCODER_OFFSET_ROTATIONS = -0.067626953125;
    public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
    public static final boolean FRONT_LEFT_STEER_INVERTED = true;
    public static final boolean FRONT_LEFT_ENCODER_INVERTED = false;

    public static final double FRONT_RIGHT_ENCODER_OFFSET_ROTATIONS = -0.204833984375;
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;
    public static final boolean FRONT_RIGHT_STEER_INVERTED = true;
    public static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;

    public static final double BACK_LEFT_ENCODER_OFFSET_ROTATIONS = -0.291259765625;
    public static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    public static final boolean BACK_LEFT_STEER_INVERTED = true;
    public static final boolean BACK_LEFT_ENCODER_INVERTED = false;

    public static final double BACK_RIGHT_ENCODER_OFFSET_ROTATIONS = 0.360595703125;
    public static final boolean BACK_RIGHT_DRIVE_INVERTED = true;
    public static final boolean BACK_RIGHT_STEER_INVERTED = true;
    public static final boolean BACK_RIGHT_ENCODER_INVERTED = false;

    // #endregion

    // #region 🛑🚀 SEGURANÇA - LIMITES DE VELOCIDADE

    // 🛑🚀 VELOCIDADE MÁXIMA DO ROBÔ: mude aqui para o piloto andar mais ou menos.
    // Limite operacional comum ao robô real e à simulação, abaixo do máximo físico.
    public static final double kMaxLinearSpeed = 2.5; // m/s
    // 🚀 Aceleração do piloto.
    // Mantém a aceleração anterior (2,0/s no joystick × 2,0 m/s), por eixo.
    public static final double kDriverLinearAcceleration = 4.0; // m/s²

    // 🛑🚀 Velocidade máxima de rotação
    public static final double kMaxAngularSpeed = Math.PI; // rad/s

    // #endregion

    // #region 🎯🚀 CONTROLE - PID E FEEDFORWARD DO DRIVE (MÓDULO SIMULADO)

    // 🧪 Estes ganhos são só do módulo simulado.
    // 🎯 Os do robô real ficam em ModuleIOTalonFX (driveGains e steerGains).
    // PID de velocidade do Drive
    public static final double kDriveKp = 1.0;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.0;

    // Feedforward do Drive

    // Tensão mínima aproximada para começar a movimentar
    // Valor inicial para a simulação
    public static final double kDriveKs = 0.2;

    // Aproximação inicial baseada em:
    // 12 V / 4.58 m/s
    public static final double kDriveKv =
        12.0 / 4.58;

    // #endregion

    // #region SEM INSTÂNCIAS

    private DriveConstants() {}

    // #endregion
}
