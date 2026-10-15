package frc.robot.subsystems.vision;

// #region IMPORTS
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
// #endregion

/**
 * Cenário móvel das duas câmeras, exclusivo do desktop (não usado no replay).
 * A referência vem dos sensores simulados do Drive, sem correção visual.
 * PhotonLib simula detecções; não reproduz o firmware MegaTag2.
 *
 * ETAPA ATUAL: diagnóstico, sem fusão PhotonLib -> Vision/Drive.
 * Não publica coordenadas artificiais fixas. Os IOs ficam sem estimativa
 * até implementarmos a integração das observações em uma próxima etapa.
 */
public final class VisionSimulationScenario implements AutoCloseable {
    // #region AMBIENTE E CÂMERAS
    private final VisionSystemSim visionSim = new VisionSystemSim("SteelBulls");
    private final SimulationPoseHistory referenceHistory = new SimulationPoseHistory();
    private final SimulatedCamera rear = new SimulatedCamera(
            "Rear", "sim-rear", VisionCameraTransforms.ROBOT_TO_REAR);
    private final SimulatedCamera front = new SimulatedCamera(
            "Front", "sim-front", VisionCameraTransforms.ROBOT_TO_FRONT);
    private final SimulatedCamera[] cameras = {rear, front};

    /** Cada câmera possui seus próprios frames, estimador e diagnósticos. */
    private static final class SimulatedCamera implements AutoCloseable {
        final String name;
        final Transform3d robotToCamera;
        final PhotonCamera photon;
        final PhotonCameraSim simulator;
        final PhotonPoseEstimator estimator;
        final VisionCamera visionCamera;
        int[] detectedTagIds = new int[0];
        long processedFrameCount;

        SimulatedCamera(String name, String networkName, Transform3d transform) {
            this.name = name;
            robotToCamera = transform;
            photon = new PhotonCamera(networkName);
            // Gerador multitag e estimador usam explicitamente o MESMO mapa.
            simulator = new PhotonCameraSim(photon, createCameraProperties(),
                    VisionFieldConstants.FIELD_LAYOUT);
            estimator = new PhotonPoseEstimator(VisionFieldConstants.FIELD_LAYOUT, transform);
            visionCamera = new VisionCamera(name, new VisionIOSim());
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
        // Mapa completo para diagnóstico geométrico. Os filtros de IDs das
        // Limelights reais ainda não são aplicados neste cenário diagnóstico.
        visionSim.addAprilTags(VisionFieldConstants.FIELD_LAYOUT);
        for (SimulatedCamera camera : cameras) {
            visionSim.addCamera(camera.simulator, camera.robotToCamera);
        }
        Logger.recordOutput("Vision/Field/LayoutName", VisionFieldConstants.SELECTED_FIELD.name());
        Logger.recordOutput("Vision/Field/TagCount", VisionFieldConstants.TAG_IDS.length);
        Logger.recordOutput("Vision/Field/TagIds", VisionFieldConstants.TAG_IDS);
        Logger.recordOutput("Vision/Field/TagPoses", VisionFieldConstants.TAG_POSES);
        Logger.recordOutput("Vision/Simulation/Scenario", "MOVING_TWO_CAMERAS_DIAGNOSTIC");
        Logger.recordOutput("Vision/Simulation/FusionEnabled", false);
        Logger.recordOutput("Vision/Simulation/ReferenceSource", "SIMULATED_SENSOR_ODOMETRY");
    }

    private static SimCameraProperties createCameraProperties() {
        // Parâmetros de ensaio, não uma calibração óptica das Limelights reais.
        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        properties.setFPS(25);
        properties.setAvgLatencyMs(30);
        properties.setLatencyStdDevMs(0);
        properties.setCalibError(0.25, 0.08);
        return properties;
    }

    public VisionCamera[] getCameras() {
        return new VisionCamera[] {rear.visionCamera, front.visionCamera};
    }
    // #endregion

    // #region ATUALIZAÇÃO DO MUNDO VIRTUAL
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

    // #region DETECÇÕES E ESTIMATIVAS PHOTONLIB
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
            Logger.recordOutput(p + "MultiTagSuccess", estimate.isPresent());
            if (estimate.isEmpty()) {
                estimate = camera.estimator.estimateLowestAmbiguityPose(frame);
            }
            Logger.recordOutput(p + "HasEstimatedPose", estimate.isPresent());
            Logger.recordOutput(p + "UsedTagCount",
                    estimate.map(result -> result.targetsUsed.size()).orElse(0));
            Logger.recordOutput(p + "EstimatedRobotPoses", estimate
                    .map(result -> new Pose3d[] {result.estimatedPose}).orElseGet(() -> new Pose3d[0]));

            // Não deixa erro antigo aparecer como se fosse uma medição atual.
            Logger.recordOutput(p + "ReferenceAtCaptureAvailable", false);
            Logger.recordOutput(p + "EstimationErrorMeters", Double.NaN);
            Logger.recordOutput(p + "RotationErrorDegrees", Double.NaN);
            if (estimate.isEmpty()) {
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
                        Math.abs(robotPose.getRotation().minus(atCapture.getRotation()).getDegrees()));
            }
            // Sem publishFrame: esta etapa observa PhotonLib sem corrigir o Drive.
        }
    }
    // #endregion

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
