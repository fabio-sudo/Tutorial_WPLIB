package frc.robot;

// #region IMPORTS

import java.util.List;

// #endregion

/**
 * Mapa ÚNICO dos dispositivos CAN do robô 9460 (barramento do roboRIO).
 *
 * Todo ID de motor, encoder e giroscópio vem daqui. Os subsistemas não
 * escrevem números soltos: trocou um ID no Phoenix Tuner X ou no REV Hardware
 * Client, troque só aqui. Os valores são os do robô de competição.
 *
 * Regra do projeto: um número por dispositivo, sem repetir nem entre tipos
 * diferentes (o CAN até aceitaria TalonFX e CANcoder com o mesmo ID, mas
 * isso confunde na hora de achar o dispositivo). HardwareMapTest confere.
 */
public final class CanIds {

    // #region SWERVE (PHOENIX TUNER: TALONFX + CANCODER)

    public static final int PIGEON = 3;

    public static final int FRONT_LEFT_DRIVE = 13;
    public static final int FRONT_LEFT_STEER = 11;
    public static final int FRONT_LEFT_ENCODER = 12;

    public static final int FRONT_RIGHT_DRIVE = 19;
    public static final int FRONT_RIGHT_STEER = 20;
    public static final int FRONT_RIGHT_ENCODER = 18;

    public static final int BACK_LEFT_DRIVE = 17;
    public static final int BACK_LEFT_STEER = 16;
    public static final int BACK_LEFT_ENCODER = 21;

    public static final int BACK_RIGHT_DRIVE = 14;
    public static final int BACK_RIGHT_STEER = 10;
    public static final int BACK_RIGHT_ENCODER = 15;

    // #endregion

    // #region CICLO DA BOLA (30 A 37)

    public static final int SHOOTER_TOWER = 30;      // Spark Flex: alimentador
    public static final int INTAKE_ROLLER = 31;      // Spark Flex: rolete do intake
    public static final int SHOOTER_PIVOT = 32;      // TalonFX: gira o shooter
    public static final int SHOOTER_TOP_RIGHT = 33;  // Spark Flex: roda de disparo
    public static final int INTAKE_LEFT = 34;        // TalonFX: articulador, líder
    public static final int INTAKE_RIGHT = 35;       // TalonFX: articulador, segue invertido
    public static final int CENTRIFUGE = 36;         // TalonFX: centrífuga
    public static final int SHOOTER_TOP_LEFT = 37;   // Spark Flex: roda de disparo

    // #endregion

    // #region BANCADA E A DEFINIR

    // Telescópio da equipe 9458: ID futuro reservado, usado apenas em simulação.
    public static final int TELESCOPIO_9458 = 50;

    // Telescópio 9460: ID futuro reservado. A habilitação física é separada
    // do ID, para não consultar motores ainda não instalados no barramento.
    public static final int NOT_DEFINED = -1;
    public static final int TELESCOPIO_9460 = 51;

    // IDs reservados, ainda sem motores instalados no robô atual.
    // Mudar ID não basta: no REAL não se cria nenhum dos dois dispositivos.
    // SIM e REPLAY continuam disponíveis, independentemente desta opção.
    public static final boolean ELEVATORS_HARDWARE_ENABLED = false;

    // O ID do telescópio 9460 já foi preenchido (não é NOT_DEFINED).
    public static final boolean TELESCOPIO_9460_DEFINED = TELESCOPIO_9460 >= 0;
    // Motor em uso no robô real: tem ID e o hardware está ligado.
    public static final boolean TELESCOPIO_9460_ENABLED = ELEVATORS_HARDWARE_ENABLED && TELESCOPIO_9460_DEFINED;

    // #endregion

    // #region LISTA PARA CONFERÊNCIA

    /** Um dispositivo: nome, ID e tipo (para conferir no Tuner X / Hardware Client). */
    public record Device(String name, int id, String type) {}

    public static final List<Device> ALL = List.of(
        new Device("Pigeon 2", PIGEON, "Pigeon2"),
        new Device("Swerve FL drive", FRONT_LEFT_DRIVE, "TalonFX"),
        new Device("Swerve FL steer", FRONT_LEFT_STEER, "TalonFX"),
        new Device("Swerve FL encoder", FRONT_LEFT_ENCODER, "CANcoder"),
        new Device("Swerve FR drive", FRONT_RIGHT_DRIVE, "TalonFX"),
        new Device("Swerve FR steer", FRONT_RIGHT_STEER, "TalonFX"),
        new Device("Swerve FR encoder", FRONT_RIGHT_ENCODER, "CANcoder"),
        new Device("Swerve BL drive", BACK_LEFT_DRIVE, "TalonFX"),
        new Device("Swerve BL steer", BACK_LEFT_STEER, "TalonFX"),
        new Device("Swerve BL encoder", BACK_LEFT_ENCODER, "CANcoder"),
        new Device("Swerve BR drive", BACK_RIGHT_DRIVE, "TalonFX"),
        new Device("Swerve BR steer", BACK_RIGHT_STEER, "TalonFX"),
        new Device("Swerve BR encoder", BACK_RIGHT_ENCODER, "CANcoder"),
        new Device("Shooter tower", SHOOTER_TOWER, "SparkFlex"),
        new Device("Intake roller", INTAKE_ROLLER, "SparkFlex"),
        new Device("Shooter pivô", SHOOTER_PIVOT, "TalonFX"),
        new Device("Shooter top direito", SHOOTER_TOP_RIGHT, "SparkFlex"),
        new Device("Intake esquerdo (líder)", INTAKE_LEFT, "TalonFX"),
        new Device("Intake direito", INTAKE_RIGHT, "TalonFX"),
        new Device("Centrífuga", CENTRIFUGE, "TalonFX"),
        new Device("Shooter top esquerdo", SHOOTER_TOP_LEFT, "SparkFlex"),
        new Device("Telescópio 9458 (reservado)", TELESCOPIO_9458, "TalonFX"),
        new Device("Telescópio 9460 (reservado)", TELESCOPIO_9460, "TalonFX"));

    // #endregion

    // #region SEM INSTÂNCIAS

    private CanIds() {}

    // #endregion
}
