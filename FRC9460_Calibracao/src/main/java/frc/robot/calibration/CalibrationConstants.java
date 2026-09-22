package frc.robot.calibration;


public final class CalibrationConstants {


    // #region CONSTRUTOR

    private CalibrationConstants() {}

    // #endregion



    // #region TEMPOS

    // ============================================================
    // TEMPOS DOS TESTES
    // ============================================================

    // Tempo que o robô permanece parado entre os testes
    public static final double STOP_TIME = 2.0;

    // Tempo que cada velocidade permanece ativa
    public static final double STEP_TIME = 4.0;

    // #endregion



    // #region VELOCIDADES DO DRIVE

    // ============================================================
    // VELOCIDADES DO DRIVE
    //
    // Unidade: metros por segundo
    // ============================================================

    public static final double DRIVE_LOW = 0.25;

    public static final double DRIVE_MEDIUM = 0.50;

    public static final double DRIVE_HIGH = 1.00;

    // #endregion



    // #region MOVIMENTO LATERAL

    // ============================================================
    // VELOCIDADE LATERAL
    //
    // Unidade: metros por segundo
    // ============================================================

    public static final double STRAFE_SPEED = 0.25;

    // #endregion



    // #region ROTAÇÃO

    // ============================================================
    // VELOCIDADE DE ROTAÇÃO
    //
    // Unidade: radianos por segundo
    // ============================================================

    public static final double ROTATION_SPEED = 0.50;

    // #endregion

        // #region TESTE KS

    // ============================================================
    // TESTE EXPERIMENTAL DE kS
    // ============================================================
    //
    // Vamos começar com tensões muito pequenas.
    // O objetivo é descobrir aproximadamente em qual tensão
    // o drivetrain começa a se movimentar.
    //
    // IMPORTANTE:
    // Isso é apenas uma ESTIMATIVA experimental.
    // Depois vamos comparar com o SysId.
    //
    // ============================================================

    // Tempo aplicado em cada tensão
    public static final double KS_STEP_TIME = 1.5;

    // Tempo parado entre as tensões
    public static final double KS_STOP_TIME = 1.0;

    // Tensões utilizadas no teste
    public static final double KS_VOLTAGE_1 = 0.10;
    public static final double KS_VOLTAGE_2 = 0.20;
    public static final double KS_VOLTAGE_3 = 0.30;
    public static final double KS_VOLTAGE_4 = 0.40;
    public static final double KS_VOLTAGE_5 = 0.50;
    public static final double KS_VOLTAGE_6 = 0.60;

    // #endregion
}