package frc.robot;

/**
 * Faixas de limite de corrente recomendadas pelos fabricantes.
 *
 * Os limites de cada mecanismo ficam nas constantes do próprio subsistema;
 * HardwareMapTest confere que todos estão dentro destas faixas. Para mudar
 * um limite, mude no mecanismo e mantenha dentro da faixa.
 *
 * Fontes (consultadas em 27/10/2026):
 * - CTRE, Phoenix 6 CurrentLimitsConfigs: supply 70 A caindo para 40 A após
 *   1 s como ponto de partida; supply comum de 20 a 80 A conforme o disjuntor;
 *   estator 120 A como ponto de partida, comum de 80 a 160 A, e MENOR em
 *   mecanismo com batente (pivô, intake).
 * - REV, Suggested Current Limits: NEO 40 a 60 A. FAQ do NEO Vortex: o padrão
 *   de 80 A do Spark Flex serve para a maioria dos usos. Spark Flex: evitar
 *   mais de 60 A por 3 minutos. Usar sempre junto com o disjuntor certo.
 */
public final class MotorLimits {

    // #region ⚡ DISJUNTOR DE CADA MOTOR (PDH)

    // Cada motor em um canal com disjuntor de 40 A.
    public static final double BREAKER_AMPS = 40.0;

    // #endregion

    // #region ⚡ CTRE TALONFX (KRAKEN X60 E X44)

    public static final double TALON_SUPPLY_MIN_AMPS = 20.0;
    public static final double TALON_SUPPLY_MAX_AMPS = 80.0;
    // Ponto de partida da CTRE: 70 A por até 1 s, depois 40 A (o disjuntor).
    public static final double TALON_SUPPLY_START_AMPS = 70.0;
    public static final double TALON_SUPPLY_LOWER_AMPS = 40.0;
    public static final double TALON_SUPPLY_LOWER_TIME_SECONDS = 1.0;
    public static final double TALON_STATOR_MAX_AMPS = 160.0;

    // #endregion

    // #region ⚡ REV SPARK FLEX

    // NEO: 40 a 60 A. NEO Vortex: até 80 A (padrão), mas o Spark Flex não deve
    // passar de 60 A por minutos seguidos. Usamos 60 A como teto do projeto.
    public static final int SPARK_NEO_MIN_AMPS = 40;
    public static final int SPARK_NEO_MAX_AMPS = 60;
    public static final int SPARK_FLEX_MAX_AMPS = 60;

    // #endregion

    // #region SEM INSTÂNCIAS

    private MotorLimits() {}

    // #endregion
}
