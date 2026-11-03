package frc.robot.subsystems.vision;

// #region IMPORTS
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionSimulationConstants.CameraProfile;
// #endregion

/**
 * Cenário móvel das duas câmeras, exclusivo do desktop (não usado no replay).
 * A referência vem dos sensores simulados do Drive, sem correção visual.
 * PhotonLib simula detecções; não reproduz o firmware MegaTag2.
 *
 * As estimativas PhotonLib são publicadas nos IOs, registradas pelo
 * AdvantageKit
 * e validadas pelo Vision antes da fusão no Drive. A referência continua
 * independente dessas correções; não é usada como medição de visão.
 */
public final class VisionSimulationScenario implements AutoCloseable {
        // #region 📷 AMBIENTE E CÂMERAS
        private final VisionSystemSim visionSim = new VisionSystemSim("SteelBulls");
        private final SimulationPoseHistory referenceHistory = new SimulationPoseHistory();
        private final SimulatedCamera rear = new SimulatedCamera(
                        "Rear", "sim-rear", VisionCameraTransforms.ROBOT_TO_REAR, VisionSimulationConstants.REAR);
        private final SimulatedCamera front = new SimulatedCamera(
                        "Front", "sim-front", VisionCameraTransforms.ROBOT_TO_FRONT, VisionSimulationConstants.FRONT);
        private final SimulatedCamera[] cameras = { rear, front };

        /** Cada câmera possui seus próprios frames, estimador e diagnósticos. */
        private static final class SimulatedCamera implements AutoCloseable {
                final String name;
                final Transform3d robotToCamera;
                final PhotonCamera photon;
                final CameraProfile profile;
                final SimCameraProperties properties;
                final PhotonCameraSim simulator;
                final PhotonPoseEstimator estimator;
                final VisionIOSim io;
                final VisionCamera visionCamera;
                int[] detectedTagIds = new int[0];
                long processedFrameCount;

                SimulatedCamera(String name, String networkName, Transform3d transform, CameraProfile profile) {
                        this.name = name;
                        this.profile = profile;
                        robotToCamera = transform;
                        photon = new PhotonCamera(networkName);
                        properties = createCameraProperties(profile);
                        // Gerador multitag e estimador usam explicitamente o MESMO mapa.
                        simulator = new PhotonCameraSim(photon, properties,
                                        VisionFieldConstants.FIELD_LAYOUT);
                        // O corte de área do pipeline e a capacidade de decodificação são
                        // conceitos diferentes. A aproximação em pixels é parametrizada.
                        double decoderAreaPercent = profile.minDetectableAreaPixels()
                                        / (profile.widthPixels() * (double) profile.heightPixels()) * 100.0;
                        simulator.setMinTargetAreaPercent(Math.max(profile.minTargetAreaPercent(), decoderAreaPercent));
                        simulator.setMaxSightRange(profile.maxSightRangeMeters());
                        estimator = new PhotonPoseEstimator(VisionFieldConstants.FIELD_LAYOUT, transform);
                        estimator.setTagModel(new TargetModel(VisionSimulationConstants.APRILTAG_SIZE_METERS,
                                        VisionSimulationConstants.APRILTAG_SIZE_METERS));
                        io = new VisionIOSim();
                        visionCamera = new VisionCamera(name, io);
                        simulator.enableRawStream(false);
                        simulator.enableProcessedStream(false);
                }

                @Override
                public void close() {
                        simulator.close();
                        photon.close();
                }
        }
        // #endregion

        // #region INICIALIZAÇÃO
        public VisionSimulationScenario() {
                // Mapa completo para localização no campo virtual. O filtro restrito ao
                // hub das Limelights reais não limita a cobertura deste cenário desktop.
                TargetModel tagModel = new TargetModel(VisionSimulationConstants.APRILTAG_SIZE_METERS,
                                VisionSimulationConstants.APRILTAG_SIZE_METERS);
                visionSim.addVisionTargets(VisionFieldConstants.FIELD_LAYOUT.getTags().stream()
                                .map(tag -> new VisionTargetSim(tag.pose, tagModel, tag.ID))
                                .toArray(VisionTargetSim[]::new));
                for (SimulatedCamera camera : cameras) {
                        visionSim.addCamera(camera.simulator, camera.robotToCamera);
                        registerCameraConfiguration(camera);
                }
                Logger.recordOutput("Vision/Field/LayoutName", VisionFieldConstants.SELECTED_FIELD.name());
                Logger.recordOutput("Vision/Field/TagCount", VisionFieldConstants.TAG_IDS.length);
                Logger.recordOutput("Vision/Field/TagIds", VisionFieldConstants.TAG_IDS);
                Logger.recordOutput("Vision/Field/TagPoses", VisionFieldConstants.TAG_POSES);
                Logger.recordOutput("Vision/Simulation/Scenario", "MOVING_TWO_CAMERAS_FUSION");
                Logger.recordOutput("Vision/Simulation/FusionEnabled", true);
                Logger.recordOutput("Vision/Simulation/ReferenceSource", "SIMULATED_SENSOR_ODOMETRY");
        }

        /** Converte o perfil nominal em uma câmera PhotonLib nova e independente. */
        static SimCameraProperties createCameraProperties(CameraProfile profile) {
                // FOV horizontal e vertical não são FOV diagonal. Montamos uma matriz
                // ideal a partir dos dois ângulos nominais da lente padrão da LL4.
                // Distorção zero e ponto principal centrado são aproximações de simulação.
                double cx = profile.widthPixels() / 2.0;
                double cy = profile.heightPixels() / 2.0;
                double fx = cx / Math.tan(Math.toRadians(profile.horizontalFovDegrees()) / 2.0);
                double fy = cy / Math.tan(Math.toRadians(profile.verticalFovDegrees()) / 2.0);
                SimCameraProperties properties = new SimCameraProperties();
                properties.setCalibration(profile.widthPixels(), profile.heightPixels(),
                                MatBuilder.fill(Nat.N3(), Nat.N3(), fx, 0, cx, 0, fy, cy, 0, 0, 1),
                                VecBuilder.fill(0, 0, 0, 0, 0, 0, 0, 0));
                properties.setFPS(profile.framesPerSecond());
                properties.setExposureTimeMs(profile.exposureMilliseconds());
                properties.setAvgLatencyMs(profile.averageLatencyMilliseconds());
                properties.setLatencyStdDevMs(profile.latencyStdDevMilliseconds());
                properties.setCalibError(profile.averagePixelError(), profile.pixelErrorStdDev());
                return properties;
        }

        // Configuração registrada uma vez, para verificar os parâmetros no Scope.
        private static void registerCameraConfiguration(SimulatedCamera camera) {
                String p = "Vision/Simulation/" + camera.name + "/Configuration/";
                CameraProfile profile = camera.profile;
                Logger.recordOutput(p + "Model", "Limelight 4 - nominal");
                Logger.recordOutput(p + "WidthPixels", profile.widthPixels());
                Logger.recordOutput(p + "HeightPixels", profile.heightPixels());
                Logger.recordOutput(p + "HorizontalFovDegrees", camera.properties.getHorizFOV().getDegrees());
                Logger.recordOutput(p + "VerticalFovDegrees", camera.properties.getVertFOV().getDegrees());
                Logger.recordOutput(p + "ConfiguredFPS", profile.framesPerSecond());
                Logger.recordOutput(p + "ExposureMilliseconds", profile.exposureMilliseconds());
                Logger.recordOutput(p + "AverageLatencyMilliseconds", profile.averageLatencyMilliseconds());
                Logger.recordOutput(p + "LatencyStdDevMilliseconds", profile.latencyStdDevMilliseconds());
                Logger.recordOutput(p + "AveragePixelError", profile.averagePixelError());
                Logger.recordOutput(p + "PixelErrorStdDev", profile.pixelErrorStdDev());
                Logger.recordOutput(p + "PipelineMinTargetAreaPercent", profile.minTargetAreaPercent());
                Logger.recordOutput(p + "SimMinDetectableAreaPixels", camera.simulator.getMinTargetAreaPixels());
                Logger.recordOutput(p + "MaxRangeConfigured", Double.isFinite(profile.maxSightRangeMeters()));
                Logger.recordOutput(p + "MaxSightRangeMeters", Double.isFinite(profile.maxSightRangeMeters())
                                ? profile.maxSightRangeMeters()
                                : -1.0);
                Logger.recordOutput(p + "DetectorDownscaleReference",
                                VisionSimulationConstants.PIPELINE_DETECTOR_DOWNSCALE);
                Logger.recordOutput(p + "AprilTagSizeMeters", VisionSimulationConstants.APRILTAG_SIZE_METERS);
        }

        public VisionCamera[] getCameras() {
                return new VisionCamera[] { rear.visionCamera, front.visionCamera };
        }
        // #endregion

        // #region 🧪 ATUALIZAÇÃO DO MUNDO VIRTUAL
        /** simulationPose movimenta o mundo; estimatedPose é somente uma comparação. */
        public void update(Pose2d simulationPose, Pose2d estimatedPose) {
                // PhotonLib 2026 utiliza o relógio FPGA internamente. O histórico precisa
                // estar no mesmo domínio temporal dos timestamps das suas capturas.
                referenceHistory.add(Timer.getFPGATimestamp(), simulationPose);
                registerScenarioOutputs(simulationPose, estimatedPose);
                visionSim.update(simulationPose);
                for (SimulatedCamera camera : cameras) {
                        processPhotonCamera(camera);
                }
        }

        private void registerScenarioOutputs(Pose2d simulationPose, Pose2d estimatedPose) {
                Pose3d robotReference = new Pose3d(simulationPose);
                Logger.recordOutput("Vision/Simulation/ReferencePose", simulationPose);
                Logger.recordOutput("Vision/Simulation/PositionErrorMeters",
                                estimatedPose.getTranslation().getDistance(simulationPose.getTranslation()));
                Logger.recordOutput("Vision/Simulation/RotationErrorDegrees",
                                Math.abs(estimatedPose.getRotation().minus(simulationPose.getRotation()).getDegrees()));
                for (SimulatedCamera camera : cameras) {
                        Logger.recordOutput("Vision/Simulation/" + camera.name + "/CameraPose",
                                        robotReference.transformBy(camera.robotToCamera));
                }
        }
        // #endregion

        // #region 📷 DETECÇÕES E ESTIMATIVAS PHOTONLIB
        private void processPhotonCamera(SimulatedCamera camera) {
                String p = "Vision/Simulation/Photon/" + camera.name + "/";
                var frames = camera.photon.getAllUnreadResults();
                Logger.recordOutput(p + "HasNewFrame", !frames.isEmpty());
                // Consome cada resultado somente uma vez. Os logs mostram o último
                // frame; o contador considera todos os frames processados.
                for (var frame : frames) {
                        camera.processedFrameCount++;
                        camera.detectedTagIds = frame.getTargets().stream()
                                        .mapToInt(target -> target.getFiducialId()).toArray();
                        Logger.recordOutput(p + "ProcessedFrameCount", camera.processedFrameCount);
                        Logger.recordOutput(p + "TagCount", camera.detectedTagIds.length);
                        Logger.recordOutput(p + "TagIds", camera.detectedTagIds);
                        Logger.recordOutput(p + "HasTargets", frame.hasTargets());
                        Logger.recordOutput(p + "CaptureTimestampSeconds", frame.getTimestampSeconds());
                        Logger.recordOutput(p + "CaptureAgeSeconds",
                                        Timer.getFPGATimestamp() - frame.getTimestampSeconds());
                        // Array vazio remove alvos antigos da visualização ao perder as tags.
                        Pose3d[] tagPoses = Arrays.stream(camera.detectedTagIds)
                                        .mapToObj(VisionFieldConstants.FIELD_LAYOUT::getTagPose)
                                        .flatMap(java.util.Optional::stream).toArray(Pose3d[]::new);
                        Logger.recordOutput(p + "TagPoses", tagPoses);

                        var estimate = camera.estimator.estimateCoprocMultiTagPose(frame);
                        boolean multiTag = estimate.isPresent();
                        Logger.recordOutput(p + "MultiTagSuccess", multiTag);
                        List<PhotonTrackedTarget> usedTargets = getUsedTargets(frame, multiTag);
                        if (estimate.isEmpty() && !usedTargets.isEmpty()) {
                                // O fallback calcula a pose com UMA tag. Entrega somente essa
                                // tag ao estimador, para não contar todos os alvos como multitag.
                                var singleTagFrame = new PhotonPipelineResult(
                                                frame.metadata, usedTargets, Optional.empty());
                                estimate = camera.estimator.estimateLowestAmbiguityPose(singleTagFrame);
                        }
                        Logger.recordOutput(p + "HasEstimatedPose", estimate.isPresent());
                        Logger.recordOutput(p + "UsedTagCount",
                                        estimate.isPresent() ? usedTargets.size() : 0);
                        Logger.recordOutput(p + "UsedTagIds", estimate.isPresent()
                                        ? usedTargets.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray()
                                        : new int[0]);
                        Logger.recordOutput(p + "EstimatedRobotPoses", estimate
                                        .map(result -> new Pose3d[] { result.estimatedPose })
                                        .orElseGet(() -> new Pose3d[0]));

                        // Não deixa erro antigo aparecer como se fosse uma medição atual.
                        Logger.recordOutput(p + "ReferenceAtCaptureAvailable", false);
                        Logger.recordOutput(p + "EstimationErrorMeters", Double.NaN);
                        Logger.recordOutput(p + "RotationErrorDegrees", Double.NaN);
                        if (estimate.isEmpty()) {
                                // Atualiza perda de alvos/pose sem inventar uma observação.
                                camera.io.publishFrame(null, frame.getTargets().size(),
                                                frame.hasTargets() ? frame.getBestTarget().getFiducialId() : -1,
                                                0.0, frame.getTimestampSeconds(),
                                                (Timer.getFPGATimestamp() - frame.getTimestampSeconds()) * 1000.0);
                                continue;
                        }
                        var result = estimate.get();
                        Pose2d robotPose = result.estimatedPose.toPose2d();
                        // Campo singular mantido por compatibilidade. Consulte HasEstimatedPose;
                        // no 3D prefira EstimatedRobotPoses, que fica vazio quando não há pose.
                        Logger.recordOutput(p + "EstimatedRobotPose", robotPose);
                        Logger.recordOutput(p + "PoseTimestampSeconds", result.timestampSeconds);
                        var reference = referenceHistory.sample(result.timestampSeconds);
                        Logger.recordOutput(p + "ReferenceAtCaptureAvailable", reference.isPresent());
                        if (reference.isPresent()) {
                                Pose2d atCapture = reference.get();
                                Logger.recordOutput(p + "ReferencePoseAtCapture", atCapture);
                                Logger.recordOutput(p + "EstimationErrorMeters",
                                                robotPose.getTranslation().getDistance(atCapture.getTranslation()));
                                Logger.recordOutput(p + "RotationErrorDegrees",
                                                Math.abs(robotPose.getRotation().minus(atCapture.getRotation())
                                                                .getDegrees()));
                        }
                        double distance = usedTargets.stream()
                                        .mapToDouble(target -> target.getBestCameraToTarget().getTranslation()
                                                        .getNorm())
                                        .average().orElse(Double.NaN);
                        double ambiguity = multiTag
                                        ? frame.getMultiTagResult().orElseThrow().estimatedPose.ambiguity
                                        : usedTargets.get(0).getPoseAmbiguity();

                        // PhotonLib fornece captura no relógio FPGA, a mesma época utilizada
                        // pelo Drive/AdvantageKit. O relógio do AdvantageKit é amostrado por
                        // ciclo; isso não muda a época da captura. Não subtrair latência.
                        camera.io.publishObservation(new PoseObservation(
                                        result.timestampSeconds, result.estimatedPose, ambiguity,
                                        usedTargets.size(), distance, PoseObservationType.PHOTONVISION),
                                        usedTargets.isEmpty() ? -1 : usedTargets.get(0).getFiducialId(),
                                        (Timer.getFPGATimestamp() - result.timestampSeconds) * 1000.0);
                        camera.io.setUsedTagIds(usedTargets.stream()
                                        .mapToInt(PhotonTrackedTarget::getFiducialId).toArray());
                }
        }

        /**
         * Seleciona somente as tags efetivamente utilizadas na solução.
         * Na PhotonLib 2026.3.4, targetsUsed pode conter todos os alvos do frame,
         * inclusive no fallback. O multitag fornece sua própria lista de IDs.
         */
        static List<PhotonTrackedTarget> getUsedTargets(PhotonPipelineResult frame, boolean multiTag) {
                if (multiTag) {
                        var ids = frame.getMultiTagResult().orElseThrow().fiducialIDsUsed;
                        return frame.getTargets().stream()
                                        .filter(target -> ids.contains((short) target.getFiducialId()))
                                        .toList();
                }
                return frame.getTargets().stream()
                                .filter(target -> VisionFieldConstants.FIELD_LAYOUT.getTagPose(target.getFiducialId())
                                                .isPresent())
                                .filter(target -> Double.isFinite(target.getPoseAmbiguity())
                                                && target.getPoseAmbiguity() >= 0.0 && target.getPoseAmbiguity() < 10.0)
                                .min(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity))
                                .map(List::of).orElseGet(List::of);
        }
        // #endregion

        /**
         * Desliga a entrega de uma câmera virtual para testar a independência dos IOs.
         */
        public void setCameraConnected(boolean frontCamera, boolean connected) {
                (frontCamera ? front : rear).io.setConnected(connected);
        }

        // Consultas de diagnóstico para validar detecções em testes desktop.
        public int[] getDetectedTagIds(boolean frontCamera) {
                return (frontCamera ? front : rear).detectedTagIds.clone();
        }

        public long getProcessedFrameCount(boolean frontCamera) {
                return (frontCamera ? front : rear).processedFrameCount;
        }

        @Override
        public void close() {
                for (SimulatedCamera camera : cameras) {
                        visionSim.removeCamera(camera.simulator);
                        camera.close();
                }
        }
}
