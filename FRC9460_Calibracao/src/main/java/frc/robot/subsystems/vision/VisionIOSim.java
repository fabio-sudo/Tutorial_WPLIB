package frc.robot.subsystems.vision;

// #region  - IMPORTS

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;

// #endregion

public class VisionIOSim implements VisionIO {

    // #region  - ESTADO DA CÂMERA SIMULADA

    // Último frame, usado nos indicadores e gráficos individuais da câmera.
    private final VisionIOInputs frame = new VisionIOInputs();

    // Cada instância possui seu próprio lote. Nenhuma câmera aguarda a outra.
    // O cenário e o Vision acessam este IO na thread principal do robô.
    private final List<PoseObservation> pendingObservations = new ArrayList<>();

    // Fornece o relógio utilizado para calcular a idade do frame.
    private final DoubleSupplier timestampSupplier;

    // #endregion

    // #region  - CONSTRUTORES

    // Utilizado na simulação normal do projeto.
    public VisionIOSim() {
        this(Timer::getTimestamp);
    }

    // Permite fornecer um relógio controlado nos testes.
    public VisionIOSim(DoubleSupplier timestampSupplier) {
        this.timestampSupplier =
            Objects.requireNonNull(timestampSupplier);

        // A câmera virtual começa conectada, sem estimativa.
        frame.connected = true;
        frame.usesObservationBatch = true;
    }

    // #endregion

    // #region  - PUBLICAÇÃO DE FRAMES CONTROLADOS

    /*
     * O cenário de teste chama este método quando deseja
     * representar a chegada de um NOVO frame da câmera.
     *
     * pose: pose medida no referencial azul do campo.
     * tagCount: quantidade de tags utilizadas.
     * primaryTagId: ID do alvo principal.
     * avgTagDistanceMeters: distância média das tags.
     * captureTimestampSeconds: instante da captura.
     * latencyMilliseconds: latência informada pela câmera.
     *
     * Mantém a API dos testes controlados existentes, que representam
     * uma medição MegaTag2 em 2D. Cada pose disponível também entra no lote.
     * Para estimativas PhotonLib, utilize publishObservation, preservando 3D,
     * ambiguidade e identificação do algoritmo.
     *
     * Os valores são entregues ao Vision para validação.
     * Assim, também podemos publicar dados inválidos
     * propositalmente e verificar se os filtros os rejeitam.
     */
    public void publishFrame(
        Pose2d pose,
        int tagCount,
        int primaryTagId,
        double avgTagDistanceMeters,
        double captureTimestampSeconds,
        double latencyMilliseconds
    ) {
        // Uma câmera desconectada não publica novos frames.
        if (!frame.connected) {
            return;
        }

        updateLatestFrame(pose, tagCount, primaryTagId, avgTagDistanceMeters,
            captureTimestampSeconds, latencyMilliseconds);

        if (frame.hasPoseEstimate) {
            pendingObservations.add(new PoseObservation(
                captureTimestampSeconds,
                new Pose3d(pose),
                0.0,
                tagCount,
                avgTagDistanceMeters,
                PoseObservationType.MEGATAG_2
            ));
        }
    }

    // #endregion

    // #region  - PUBLICAÇÃO DE OBSERVAÇÕES PHOTONLIB

    /**
     * Enfileira uma estimativa completa produzida pelo cenário PhotonLib.
     *
     * O timestamp já deve estar no mesmo relógio utilizado pelo Vision/Drive.
     * A latência é apenas informativa e não altera esse timestamp.
     * As verificações de qualidade da medição pertencem ao Vision;
     * o IO preserva os valores numéricos recebidos, inclusive para testes.
     *
     * Os indicadores individuais representam o último frame publicado;
     * o lote conserva todas as observações recebidas entre duas leituras.
     */
    public void publishObservation(
        PoseObservation observation,
        int primaryTagId,
        double latencyMilliseconds
    ) {
        Objects.requireNonNull(observation, "A observação não pode ser null");
        Objects.requireNonNull(observation.pose(), "A pose não pode ser null");
        Objects.requireNonNull(observation.type(), "O algoritmo não pode ser null");

        if (!frame.connected) {
            return;
        }

        updateLatestFrame(
            observation.pose().toPose2d(),
            observation.tagCount(),
            primaryTagId,
            observation.averageTagDistanceMeters(),
            observation.timestampSeconds(),
            latencyMilliseconds
        );
        pendingObservations.add(observation);

        // A solução multi-tag do PhotonVision resolve o heading pelas tags, como
        // o MegaTag1: serve de MegaTag1 no simulador completo.
        if (observation.type() == PoseObservationType.PHOTONVISION && observation.tagCount() >= 2) {
            publishMegaTag1(observation.pose().toPose2d(), observation.tagCount(),
                observation.averageTagDistanceMeters(), observation.timestampSeconds());
        }
    }

    // #endregion

    // #region  - TAGS USADAS NA POSE

    /** Todas as tags da solução (PhotonLib), para as linhas de visão no 2D e no 3D. */
    public void setUsedTagIds(int[] ids) {
        frame.tagIds = ids.clone();
        // Em ordem, como a Limelight: senão o log regrava a lista a cada quadro.
        Arrays.sort(frame.tagIds);
    }

    // #endregion

    // #region  - MEGATAG1 (HEADING PELAS TAGS)

    /**
     * Publica uma leitura MegaTag1: pose calculada só pelas tags, com o heading
     * delas. Independe da leitura MegaTag2: os testes controlam as duas.
     */
    public void publishMegaTag1(
        Pose2d pose,
        int tagCount,
        double avgTagDistanceMeters,
        double captureTimestampSeconds
    ) {
        if (!frame.connected) {
            return;
        }
        frame.megaTag1Pose = pose;
        frame.megaTag1TagCount = tagCount;
        frame.megaTag1AvgTagDistanceMeters = avgTagDistanceMeters;
        frame.megaTag1TimestampSeconds = captureTimestampSeconds;
    }

    // #endregion

    // #region  - INDICADORES DO ÚLTIMO FRAME

    // Atualiza os campos individuais sem alterar as observações já na fila.
    private void updateLatestFrame(
        Pose2d pose,
        int tagCount,
        int primaryTagId,
        double avgTagDistanceMeters,
        double captureTimestampSeconds,
        double latencyMilliseconds
    ) {
        // Avança somente quando um novo frame é publicado.
        frame.heartbeat += 1.0;

        // Informações das tags.
        frame.hasTarget = tagCount > 0;

        frame.tagId = frame.hasTarget
            ? primaryTagId
            : -1;

        frame.tagCount = tagCount;
        frame.avgTagDistanceMeters = avgTagDistanceMeters;
        // Só a principal é conhecida aqui; o cenário PhotonLib informa todas (setUsedTagIds).
        frame.tagIds = frame.hasTarget && primaryTagId >= 0 ? new int[] {primaryTagId} : new int[0];

        // Esta simulação inicial exercita a localização.
        // Não calcula geometria de mira ou área da imagem.
        frame.txDegrees = 0.0;
        frame.tyDegrees = 0.0;
        frame.targetAreaPercent = 0.0;

        // #region 🧪 VALIDAÇÃO DA POSE SIMULADA

        // Uma estimativa de localização só está disponível
        // quando a câmera encontrou tags e conseguiu
        // calcular uma posição para o robô.
        frame.hasPoseEstimate =
            tagCount > 0 && pose != null;

        // Informa ao Vision se existe uma pose utilizável.
        frame.hasPose = frame.hasPoseEstimate;

        // Quando não existe uma pose válida,
        // mantém um objeto padrão para evitar null.
        // O Vision rejeitará o frame pelos indicadores acima.
        frame.pose = frame.hasPose
            ? pose
            : new Pose2d();

        // #endregion

        // O cenário já fornece o timestamp da captura.
        // Não descontamos a latência novamente.
        frame.captureTimestampSeconds =
            captureTimestampSeconds;

        frame.latencyMilliseconds =
            latencyMilliseconds;
    }

    // #endregion

    // #region  - LEITURA PELO SUBSISTEMA VISION

    @Override
    public void updateInputs(VisionIOInputs inputs) {

        // Estado de comunicação.
        inputs.connected = frame.connected;
        inputs.heartbeat = frame.heartbeat;

        // O instante de leitura avança a cada consulta.
        inputs.readTimestampSeconds =
            timestampSupplier.getAsDouble();

        // Informações do alvo.
        inputs.hasTarget = frame.hasTarget;
        inputs.txDegrees = frame.txDegrees;
        inputs.tyDegrees = frame.tyDegrees;
        inputs.targetAreaPercent = frame.targetAreaPercent;
        inputs.tagId = frame.tagId;
        inputs.tagIds = frame.tagIds;

        // Estimativa de localização.
        inputs.hasPoseEstimate = frame.hasPoseEstimate;
        inputs.hasPose = frame.hasPose;
        inputs.pose = frame.pose;

        // Timestamps e latência do último frame publicado.
        inputs.captureTimestampSeconds =
            frame.captureTimestampSeconds;

        inputs.latencyMilliseconds =
            frame.latencyMilliseconds;

        // Informações usadas para calcular a confiança.
        inputs.tagCount = frame.tagCount;

        inputs.avgTagDistanceMeters =
            frame.avgTagDistanceMeters;

        // MegaTag1: último valor publicado (um quadro repetido tem o mesmo timestamp).
        inputs.megaTag1Pose = frame.megaTag1Pose;
        inputs.megaTag1TagCount = frame.megaTag1TagCount;
        inputs.megaTag1AvgTagDistanceMeters = frame.megaTag1AvgTagDistanceMeters;
        inputs.megaTag1TimestampSeconds = frame.megaTag1TimestampSeconds;

        // Um novo array preserva o lote entregue, mesmo após limpar a fila.
        // A próxima consulta recebe somente as observações que chegarem depois.
        inputs.usesObservationBatch = frame.usesObservationBatch;
        inputs.poseObservations = pendingObservations.toArray(PoseObservation[]::new);
        pendingObservations.clear();
    }

    // #endregion

    // #region  - CONTROLE DA CONEXÃO SIMULADA

    public void setConnected(boolean connected) {
        frame.connected = connected;

        if (!connected) {
            // Não entrega observações pendentes de uma câmera desconectada.
            // Reconectar também exige um novo frame para recuperar os alvos.
            pendingObservations.clear();
            frame.hasTarget = false;
            frame.txDegrees = 0.0;
            frame.tyDegrees = 0.0;
            frame.targetAreaPercent = 0.0;
            frame.tagId = -1;
            frame.tagIds = new int[0];
            frame.hasPoseEstimate = false;
            frame.hasPose = false;
            frame.pose = new Pose2d();
            frame.captureTimestampSeconds = 0.0;
            frame.latencyMilliseconds = 0.0;
            frame.tagCount = 0;
            frame.avgTagDistanceMeters = 0.0;
            frame.megaTag1Pose = new Pose2d();
            frame.megaTag1TagCount = 0;
            frame.megaTag1AvgTagDistanceMeters = 0.0;
            frame.megaTag1TimestampSeconds = 0.0;
        }
    }

    // #endregion
}
