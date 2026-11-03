package frc.robot.util;

/**
 * Zero que sobrevive ao deploy, para mecanismos sem encoder absoluto
 * (pivô do shooter e intake).
 *
 * O TalonFX liga com a posição dentro de UMA volta do rotor (o sensor do
 * rotor só é absoluto em uma volta) e só perde a posição quando ele mesmo
 * desliga. Reiniciar o código (deploy) não desliga o Talon. Antes, o código
 * zerava a cada início: um deploy com o shooter virado 100° mudava o
 * "centro" para lá, e os ±130° passavam a alcançar a fiação. Com o intake
 * aberto, ele tentava abrir mais 21 voltas além do batente.
 *
 * Agora o zero é gravado com uma marca: o zero do mecanismo vira
 * MARK_ROTATIONS no Talon, não 0. Ao iniciar o código:
 * - perto de 0: o Talon acabou de ligar, então zera aqui (como antes);
 * - dentro do curso, em volta da marca: só o código reiniciou, mantém o zero;
 * - qualquer outro valor: posição desconhecida; o mecanismo não se move até
 *   a equipe posicionar e zerar com o robô desabilitado.
 *
 * No Phoenix Tuner X a posição aparece somada à marca (zero = 100 voltas).
 */
public final class TalonZeroMark {

    // #region VALORES

    /** O zero do mecanismo fica em 100 voltas do rotor dentro do Talon. */
    public static final double MARK_ROTATIONS = 100.0;

    // Ao ligar, o Talon fica dentro de uma volta do rotor. 2 voltas dão folga.
    public static final double FRESH_BOOT_WINDOW_ROTATIONS = 2.0;

    // Folga além do curso: mecanismo empurrado à mão além do soft limit.
    public static final double TRAVEL_MARGIN_ROTATIONS = 5.0;

    // #endregion

    // #region 🧭 DECISÃO AO INICIAR O CÓDIGO

    // 🚧 ZERAR_AQUI confia que a equipe posicionou o mecanismo: não há sensor que confirme.
    public enum Decision { ZERAR_AQUI, MANTER_ZERO, POSICAO_DESCONHECIDA }

    /**
     * O que fazer com a posição lida no Talon ao iniciar o código.
     * minRotations e maxRotations são o curso do mecanismo, em voltas do rotor.
     */
    public static Decision onStartup(double talonRotations, double minRotations, double maxRotations) {
        if (!Double.isFinite(talonRotations)) return Decision.POSICAO_DESCONHECIDA;
        if (Math.abs(talonRotations) <= FRESH_BOOT_WINDOW_ROTATIONS) return Decision.ZERAR_AQUI;
        double mechanism = toMechanism(talonRotations);
        boolean insideTravel = mechanism >= minRotations - TRAVEL_MARGIN_ROTATIONS
            && mechanism <= maxRotations + TRAVEL_MARGIN_ROTATIONS;
        return insideTravel ? Decision.MANTER_ZERO : Decision.POSICAO_DESCONHECIDA;
    }

    // #endregion

    // #region 📏 CONVERSÃO MECANISMO <-> TALON

    public static double toTalon(double mechanismRotations) {
        return mechanismRotations + MARK_ROTATIONS;
    }

    public static double toMechanism(double talonRotations) {
        return talonRotations - MARK_ROTATIONS;
    }

    // #endregion

    private TalonZeroMark() {}
}
