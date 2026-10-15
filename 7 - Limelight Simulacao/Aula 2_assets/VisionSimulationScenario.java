package frc.robot.subsystems.vision;

// #region IMPORTS

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;


import edu.wpi.first.math.geometry.Pose3d;

// #region PHOTONLIB

// Representa uma câmera PhotonVision.
import org.photonvision.PhotonCamera;

// Simula a câmera e gera suas observações.
import org.photonvision.simulation.PhotonCameraSim;

// Configura resolução, FPS, latência e ruído.
import org.photonvision.simulation.SimCameraProperties;

// Representa o campo virtual com câmeras e alvos.
import org.photonvision.simulation.VisionSystemSim;

// Representa uma AprilTag virtual no campo.
import org.photonvision.simulation.VisionTargetSim;

// Define a geometria física das AprilTags.
import org.photonvision.estimation.TargetModel;

// #endregion

// #endregion

/**
 * Primeiro cenário de visão:
 * robô parado e duas câmeras virtuais funcionando.
 *
 * As medições são artificiais. Nesta etapa, ainda não calculamos
 * quais tags estão fisicamente visíveis para cada câmera.
 */
public final class VisionSimulationScenario {

    // #region CONFIGURAÇÃO DO CENÁRIO

    // Posição conhecida do robô virtual:
    // X = 2 metros, Y = 2 metros e orientação = 0 graus.
    //
    // Essa referência é independente da pose estimada pelo Drive.
    private static final Pose2d REFERENCE_POSE =
        new Pose2d(2.0, 2.0, new Rotation2d());

    // Intervalo mínimo entre publicações: aproximadamente 25 frames/s.
    private static final double FRAME_PERIOD_SECONDS = 0.040;

    // A leitura entregue representa uma captura de 30 ms atrás.
    private static final double LATENCY_SECONDS = 0.030;

    // Aguarda a formação inicial do histórico de odometria.
    private static final double STARTUP_DELAY_SECONDS = 0.250;

    // #endregion

    // #region CÂMERAS VIRTUAIS

    // Cada câmera possui seu próprio estado e seus próprios frames.
    private final VisionIOSim rearIO = new VisionIOSim();
    private final VisionIOSim frontIO = new VisionIOSim();

    private final VisionCamera rearCamera =
        new VisionCamera("Rear", rearIO);

    private final VisionCamera frontCamera =
        new VisionCamera("Front", frontIO);

    // #endregion

    // #region SIMULAÇÃO PHOTONLIB

    // Ambiente virtual que contém as câmeras
    // e todas as AprilTags do campo.
    private final VisionSystemSim visionSim =
        new VisionSystemSim("SteelBulls");

    // Representa a câmera traseira virtual.
    private final PhotonCamera rearPhoton =
        new PhotonCamera("sim-rear");

    // Representa a câmera frontal virtual.
    private final PhotonCamera frontPhoton =
        new PhotonCamera("sim-front");

    // Simulador da câmera traseira.
    // Recebe a câmera e suas propriedades.
    private final PhotonCameraSim rearPhotonSim =
        new PhotonCameraSim(
            rearPhoton,
            createCameraProperties()
        );

    // Simulador da câmera frontal.
    private final PhotonCameraSim frontPhotonSim =
        new PhotonCameraSim(
            frontPhoton,
            createCameraProperties()
        );

    // #endregion





    // #region CONTROLE DO TEMPO

    private final double startTimestampSeconds =
        Timer.getTimestamp();

    private double lastPublishTimestampSeconds =
        Double.NEGATIVE_INFINITY;

    // #endregion


    // #region CÂMERAS UTILIZADAS PELO VISION

    /**
     * Entrega as duas câmeras ao subsistema Vision.
     * A validação e a fusão continuam sendo feitas pelo Vision existente.
     */
    public VisionCamera[] getCameras() {
        return new VisionCamera[] {
            rearCamera,
            frontCamera
        };
    }

    // #endregion


    // #region CONFIGURAÇÃO DAS CÂMERAS PHOTONLIB

    /**
     * Define as propriedades utilizadas pelas
     * câmeras durante a simulação.
     */
    private static SimCameraProperties createCameraProperties() {

        // Cria um conjunto de propriedades.
        SimCameraProperties properties =
            new SimCameraProperties();

        // Resolução: 960 x 720 pixels.
        // Campo de visão diagonal: 90 graus.
        properties.setCalibration(
            960,
            720,
            Rotation2d.fromDegrees(90)
        );

        // Simula a captura de 25 imagens por segundo.
        properties.setFPS(25);

        // Latência média de processamento: 30 ms.
        properties.setAvgLatencyMs(30);

        // Sem variação aleatória de latência neste teste.
        properties.setLatencyStdDevMs(0);

        // Pequenos erros de calibração em pixels.
        properties.setCalibError(0.25, 0.08);

        // Entrega as propriedades configuradas.
        return properties;
    }

    // #endregion


    // #region ATUALIZAÇÃO DO CENÁRIO

    /**
     * Chamado a cada ciclo da simulação.
     *
     * estimatedPose é usada somente para calcular o erro registrado.
     * Ela não é usada para fabricar as medições das câmeras.
     */
    public void update(Pose2d estimatedPose) {
        double now = Timer.getTimestamp();


        // #region ATUALIZAÇÃO PHOTONLIB

        // Atualiza as câmeras com a posição
        // verdadeira do robô virtual.
        //
        // A PhotonLib calcula quais AprilTags
        // estão visíveis para cada câmera.
        visionSim.update(REFERENCE_POSE);


        // Registra as tags detectadas pela câmera traseira.
        logPhotonDetections(rearPhoton, "Rear");

        // Registra as tags detectadas pela câmera frontal.
        logPhotonDetections(frontPhoton, "Front");


        // #endregion


        // #region POSIÇÕES DAS CÂMERAS NO CAMPO

        // Usa a posição conhecida do cenário.
        // Não usa a estimativa corrigida pelo próprio Vision.
        Pose3d robotReference3d = new Pose3d(REFERENCE_POSE);

        // Combina a posição do robô com a montagem de cada câmera.
        Pose3d rearCameraPose = robotReference3d.transformBy(
            VisionCameraTransforms.ROBOT_TO_REAR
        );

        Pose3d frontCameraPose = robotReference3d.transformBy(
            VisionCameraTransforms.ROBOT_TO_FRONT
        );

        // São poses das CÂMERAS, não estimativas de pose do robô.
        Logger.recordOutput(
            "Vision/Simulation/Rear/CameraPose",
            rearCameraPose
        );

        Logger.recordOutput(
            "Vision/Simulation/Front/CameraPose",
            frontCameraPose
        );

        // #endregion

        // Registra a referência conhecida para desenhar no campo.
        Logger.recordOutput(
            "Vision/Simulation/ReferencePose",
            REFERENCE_POSE
        );

        Logger.recordOutput(
            "Vision/Simulation/Scenario",
            "STATIONARY_TWO_CAMERAS"
        );

        // Distância entre a posição estimada e a referência do cenário.
        double positionErrorMeters =
            estimatedPose.getTranslation().getDistance(
                REFERENCE_POSE.getTranslation()
            );

        Logger.recordOutput(
            "Vision/Simulation/PositionErrorMeters",
            positionErrorMeters
        );

        // Aguarda o histórico inicial de odometria.
        if (now - startTimestampSeconds < STARTUP_DELAY_SECONDS) {
            return;
        }

        // Entre publicações, o IO mantém o último frame.
        // Isso também exercita a identificação de frames repetidos.
        if (now - lastPublishTimestampSeconds < FRAME_PERIOD_SECONDS) {
            return;
        }

        lastPublishTimestampSeconds = now;

        double captureTimestampSeconds = now - LATENCY_SECONDS;
        double latencyMilliseconds = LATENCY_SECONDS * 1000.0;

        // Pequenos desvios fixos e conhecidos para o primeiro teste.
        // São posições medidas do ROBÔ, não posições das câmeras.
        Pose2d rearMeasurement = new Pose2d(
            REFERENCE_POSE.getX() + 0.020,
            REFERENCE_POSE.getY() + 0.010,
            REFERENCE_POSE.getRotation()
        );

        Pose2d frontMeasurement = new Pose2d(
            REFERENCE_POSE.getX() - 0.020,
            REFERENCE_POSE.getY() - 0.010,
            REFERENCE_POSE.getRotation()
        );

        // Quantidade, ID principal e distância são dados artificiais.
        // Ainda não representam a geometria real das tags do campo.
        rearIO.publishFrame(
            rearMeasurement,
            2,                          // Quantidade de tags.
            9,                          // ID principal.
            1.8,                        // Distância média em metros.
            captureTimestampSeconds,
            latencyMilliseconds
        );

        frontIO.publishFrame(
            frontMeasurement,
            2,
            25,
            1.8,
            captureTimestampSeconds,
            latencyMilliseconds
        );
    }

    // #endregion


    // #region INICIALIZAÇÃO DO AMBIENTE PHOTONLIB

    /**
     * Adiciona as AprilTags e as câmeras
     * ao campo virtual.
     *
     * Deve ser executado apenas uma vez
     * durante a criação do cenário.
     */
    private void configurePhotonSimulation() {

        // #region APRILTAGS

        // Confere se temos uma posição para cada ID.
        if (VisionFieldConstants.TAG_IDS.length !=
            VisionFieldConstants.TAG_POSES.length) {

            throw new IllegalStateException(
                "Quantidade de IDs e poses diferente."
            );
        }

        // Percorre as AprilTags cadastradas.
        for (int i = 0;
            i < VisionFieldConstants.TAG_IDS.length;
            i++) {

            // Obtém o identificador da tag.
            int tagId = VisionFieldConstants.TAG_IDS[i];

            // Obtém a posição oficial da tag no campo.
            Pose3d tagPose = VisionFieldConstants.TAG_POSES[i];

            // Cria uma AprilTag virtual 36h11.
            VisionTargetSim tag = new VisionTargetSim(
                tagPose,
                TargetModel.kAprilTag36h11,
                tagId
            );

            // Adiciona a tag ao campo virtual.
            visionSim.addVisionTargets(tag);
        }

        // #endregion

        // #region CÂMERA TRASEIRA

        // Posiciona a câmera traseira no robô.
        visionSim.addCamera(
            rearPhotonSim,
            VisionCameraTransforms.ROBOT_TO_REAR
        );

        // #endregion

        // #region CÂMERA FRONTAL

        // Posiciona a câmera frontal no robô.
        visionSim.addCamera(
            frontPhotonSim,
            VisionCameraTransforms.ROBOT_TO_FRONT
        );

        // #endregion

        // #region DESEMPENHO

        // Desativa o vídeo bruto.
        rearPhotonSim.enableRawStream(false);
        frontPhotonSim.enableRawStream(false);

        // Desativa o vídeo processado.
        // As detecções simuladas continuam funcionando.
        rearPhotonSim.enableProcessedStream(false);
        frontPhotonSim.enableProcessedStream(false);

        // #endregion
    }

    // #endregion



    // #region LOGS DAS DETECÇÕES PHOTONLIB

    /**
     * Registra as tags observadas por uma câmera.
     *
     * Utilizado somente para validar a simulação.
     */
    private void logPhotonDetections(
        PhotonCamera camera,
        String cameraName
    ) {

        // Obtém os frames novos ainda não processados.
        var frames = camera.getAllUnreadResults();

        // Se não houver frame novo, termina.
        if (frames.isEmpty()) {
            return;
        }

        // Seleciona o frame mais recente.
        var frame = frames.get(frames.size() - 1);

        // Conta quantas tags foram detectadas.
        int tagCount = frame.getTargets().size();

        // Extrai os identificadores das tags detectadas.
        int[] tagIds = frame.getTargets()
            .stream()
            .mapToInt(target -> target.getFiducialId())
            .toArray();

        // Registra a quantidade de tags.
        Logger.recordOutput(
            "Vision/Simulation/Photon/" +
                cameraName + "/TagCount",
            tagCount
        );

        // Registra os identificadores.
        Logger.recordOutput(
            "Vision/Simulation/Photon/" +
                cameraName + "/TagIds",
            tagIds
        );
    }

    // #endregion


    // #region Construtor INICIALIZAÇÃO DO CENÁRIO

        public VisionSimulationScenario() {

            // Configura as tags e as câmeras virtuais.
            // Executado uma única vez.
            configurePhotonSimulation();


            // O mapa é fixo durante esta execução.
            // Registramos uma vez para consulta no AdvantageScope.
            Logger.recordOutput(
                "Vision/Field/LayoutName",
                VisionFieldConstants.SELECTED_FIELD.name()
            );

            Logger.recordOutput(
                "Vision/Field/TagCount",
                VisionFieldConstants.TAG_IDS.length
            );

            Logger.recordOutput(
                "Vision/Field/TagIds",
                VisionFieldConstants.TAG_IDS
            );

            Logger.recordOutput(
                "Vision/Field/TagPoses",
                VisionFieldConstants.TAG_POSES
            );


        // #endregion
    
    }
}