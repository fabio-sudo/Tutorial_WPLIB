package frc.robot.subsystems.vision;

// #region  - IMPORTS

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

// #endregion

public class VisionIOSim implements VisionIO {

    // #region  - ESTADO DA CÂMERA SIMULADA

    // Guarda o último frame publicado pelo cenário de teste.
    private final VisionIOInputs frame = new VisionIOInputs();

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
    }

    // #endregion

    // #region  - PUBLICAÇÃO DE UM FRAME

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

        // Avança somente quando um novo frame é publicado.
        frame.heartbeat += 1.0;

        // Informações das tags.
        frame.hasTarget = tagCount > 0;

        frame.tagId = frame.hasTarget
            ? primaryTagId
            : -1;

        frame.tagCount = tagCount;
        frame.avgTagDistanceMeters = avgTagDistanceMeters;

        // Esta simulação inicial exercita a localização.
        // Não calcula geometria de mira ou área da imagem.
        frame.txDegrees = 0.0;
        frame.tyDegrees = 0.0;
        frame.targetAreaPercent = 0.0;

        // Disponibilidade da estimativa e da pose.
        frame.hasPoseEstimate = true;
        frame.hasPose = pose != null;

        frame.pose = frame.hasPose
            ? pose
            : new Pose2d();

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
    }

    // #endregion

    // #region  - CONTROLE DA CONEXÃO SIMULADA

    public void setConnected(boolean connected) {
        frame.connected = connected;
    }

    // #endregion
}