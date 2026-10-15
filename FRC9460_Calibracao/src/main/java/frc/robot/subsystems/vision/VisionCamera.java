package frc.robot.subsystems.vision;

// #region  - IMPORTS

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;

// #endregion

public final class VisionCamera {

    // #region  - IDENTIFICAÇÃO E COMUNICAÇÃO

    // Nome usado pelo nosso código: "Rear" ou "Front".
    // O nome de rede da Limelight continua configurado no IO.
    final String name;

    // Prefixo dos registros desta câmera.
    // Exemplos: "Vision/Rear" e "Vision/Front".
    final String logPrefix;

    // Implementação que fornece os dados:
    // Limelight real, simulação ou IO vazio para replay.
    private final VisionIO io;

    // Cada câmera possui seu próprio objeto de entradas.
    // A classe AutoLogged é gerada pelo AdvantageKit.
    final VisionIOInputsAutoLogged inputs =
        new VisionIOInputsAutoLogged();

    // #endregion

    // #region  - CONTROLE DE FRAMES

    // Timestamp da última medição processada desta câmera.
    // Será usado pelo Vision para evitar fusão duplicada.
    double lastMegaTag2TimestampSeconds = -1.0;

    // Instante de processamento usado para calcular
    // o intervalo entre medições válidas desta câmera.
    double lastSeenVisionFrameTime = -1.0;

    // Resultado da validação no ciclo atual.
    boolean visionValidNow = false;

    // #endregion

    // #region  - ÚLTIMA MEDIÇÃO ENVIADA À FUSÃO

    // Última pose que passou pelos filtros e foi enviada
    // ao estimador de posição do robô.
    Pose2d latestValidVisionPose = null;

    // Quantidade de tags daquela medição.
    int latestValidTagCount = 0;

    // Instante em que aquela medição foi processada.
    // O Vision preencherá este campo usando seu relógio.
    double lastValidFrameTime = -1.0;

    // #endregion

    // #region  - DADOS PARA O SEED MANUAL

    // Pose disponível para a recuperação manual.
    // Como no código atual, deve passar pelos filtros
    // antes de ser armazenada aqui.
    Pose2d latestRawVisionPose = null;

    // Quantidade de tags usada nessa pose.
    int latestRawVisionTagCount = 0;

    // Instante em que essa pose foi processada.
    double lastRawVisionFrameTime = -1.0;

    // #endregion

    // #region  - CONSTRUTOR

    public VisionCamera(String name, VisionIO io) {

        this.name = Objects.requireNonNull(name);
        this.io = Objects.requireNonNull(io);

        // Evita nomes vazios ou que alterem a organização
        // dos caminhos usados nos logs.
        if (name.isBlank() || name.contains("/")) {
            throw new IllegalArgumentException(
                "Use um nome simples para a câmera: Rear ou Front."
            );
        }

        logPrefix = "Vision/" + name;
    }

    // #endregion

    // #region  - ATUALIZAÇÃO E REGISTRO DAS ENTRADAS

    /*
     * O Vision chamará este método uma vez por ciclo
     * para cada câmera.
     *
     * A ordem é:
     * 1. Fornecer a orientação de campo do robô.
     * 2. Consultar os dados da câmera.
     * 3. Processar as entradas pelo AdvantageKit.
     *
     * A validação e a fusão acontecerão depois,
     * dentro do Vision, usando o objeto inputs.
     */
    void updateInputs(
        double robotYawDegrees,
        double robotYawRateDegPerSec
    ) {
        io.setRobotOrientation(
            robotYawDegrees,
            robotYawRateDegPerSec
        );

        io.updateInputs(inputs);

        // Execução normal: registra as entradas.
        // Replay: preenche as entradas a partir do log,
        // quando a fonte de replay estiver configurada.
        Logger.processInputs(
            logPrefix + "/Inputs",
            inputs
        );
    }

    // #endregion
}