package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

/**
 * Gerencia as câmeras independentemente e envia suas medições ao único
 * estimador do Drive. Não controla motores e não reseta a pose automaticamente.
 */
public class Vision extends SubsystemBase {

    // #region CONFIGURAÇÃO

    // Valores de aceitação, confiança e fila centralizados em VisionConstants.
    // A decisão e o cálculo continuam neste subsistema.

    private final Drive drive;
    private final DoubleSupplier clock;
    private final List<CameraState> cameras = new ArrayList<>();
    private long resetGeneration;
    private long sequence = 0;
    private String diagnosisSummary = "SEM_LEITURA";
    private static final double[] NO_FIELD_OBJECT = new double[0];

    // Inicialização automática da pose pela MegaTag1 e monitor de heading.
    private final PoseInitializer poseInitializer = new PoseInitializer();
    private final Alert headingAlert = new Alert(
            "Visão: o heading do robô difere das tags em mais de 6 graus. Desabilitado e parado ele se "
                    + "corrige sozinho; se persistir, refaça Start + A com a frente do robô para a parede azul.",
            AlertType.kWarning);
    private double headingDivergentSince = Double.NaN;
    private int autoInitCount;
    private boolean autoInitAppliedNow;

    // Códigos do diagnóstico (ver updateDiagnosis). O texto completo acrescenta números.
    private static final String DIAG_OK = "OK_CORRIGINDO";
    private static final String DIAG_DISCONNECTED = "SEM_CONEXAO";
    private static final String DIAG_NO_TAG = "SEM_TAG_ACEITA";
    private static final String DIAG_TARGET_WITHOUT_POSE = "VE_TAG_SEM_POSE";
    private static final String DIAG_NO_GYRO = "GIROSCOPIO_INDISPONIVEL";
    private static final String DIAG_SPINNING = "GIRANDO_RAPIDO";
    private static final String DIAG_STALE = "CAPTURA_ATRASADA";
    private static final String DIAG_FUTURE = "TIMESTAMP_NO_FUTURO";
    private static final String DIAG_REJECTED = "REJEITADA";
    private static final String DIAG_NOT_APPLIED = "VALIDA_MAS_NAO_APLICADA";

    // A sequência desempata timestamps iguais de maneira determinística.
    private final PriorityQueue<Measurement> pending = new PriorityQueue<>(
            Comparator.comparingDouble(Measurement::timestamp)
                    .thenComparingLong(Measurement::sequence));

    private static final class CameraState {
        final VisionCamera camera;
        // Objeto do widget Field do Elastic, montado uma vez.
        final String fieldKey;
        // Memória limitada de capturas vistas, inclusive as rejeitadas. Um
        // timestamp futuro inválido não pode bloquear capturas corretas menores.
        final Set<Double> observedTimestamps = new LinkedHashSet<>();
        PoseObservation latestObservation;
        final List<Pose3d> validatedPoses = new ArrayList<>();
        final List<Pose3d> rejectedPoses = new ArrayList<>();
        final List<Pose2d> appliedPoses = new ArrayList<>();
        // Nomes completos dos campos do log, montados uma vez (ver key()).
        final Map<String, String> logKeys = new HashMap<>();
        String lastAppliedAlgorithm = "NONE";
        long receivedCount;
        double lastSeedCaptureTimestamp = Double.NEGATIVE_INFINITY;
        double lastConflictTimestamp = Double.NEGATIVE_INFINITY;
        double lastBaseStdDev = SINGLE_TAG_FAR_STD_DEV_METERS;
        double lastAppliedStdDev = SINGLE_TAG_FAR_STD_DEV_METERS;
        double lastPoseError = 0.0;
        double lastGap = -1.0;
        boolean newMeasurement;
        boolean measurementAccepted;
        boolean camerasDisagree;
        String fusionResult = "NO_NEW_FRAME";
        long queuedCount;
        long rejectedCount;
        // Destino final do último frame na fila: "ACEITA" ou o motivo da recusa.
        String lastFrameOutcome = "NENHUM_FRAME";
        // Diagnóstico em uma linha (ver updateDiagnosis). O texto é refeito a 4 Hz
        // ou quando o código muda, para não gerar lixo a cada ciclo.
        String diagnosisCode = "";
        String diagnosis = "SEM_LEITURA";
        double diagnosisTextDue = Double.NEGATIVE_INFINITY;
        double diagnosisSince = Double.NaN;
        boolean diagnosisReported;
        // Tags usadas: refeitas só quando a lista de IDs muda.
        int[] loggedTagIds = new int[0];
        Translation3d[] usedTagPoints = new Translation3d[0];
        EnumSet<ObservationRejection> statusReasons = EnumSet.noneOf(ObservationRejection.class);

        CameraState(VisionCamera camera) {
            this.camera = Objects.requireNonNull(camera);
            fieldKey = "Campo/MegaTag2" + camera.name;
        }
    }

    // Snapshot imutável: a próxima leitura não modifica um frame já na fila.
    private record Measurement(CameraState state, Pose2d pose, double timestamp,
            double stdDev, int tagCount, PoseObservationType type, long sequence) {
    }

    /** Motivos dos filtros, registrados para a última observação da câmera. */
    private enum ObservationRejection {
        DISCONNECTED("Disconnected"), NO_ESTIMATE("NoEstimate"), NO_POSE("NoPose"),
        NO_TAGS("NoTags"), INVALID_POSE("InvalidPose"),
        INVALID_DISTANCE("InvalidDistanceData"), INVALID_TIMESTAMP("InvalidTimestamp"),
        ROTATION("Rotation"), STALE("StaleMeasurement"), FUTURE("FutureTimestamp"),
        UNKNOWN_TYPE("UnknownType"), AMBIGUITY("Ambiguity"), HEIGHT("Height"),
        TILT("Tilt"), OUTSIDE_FIELD("OutsideField");

        final String logName;

        ObservationRejection(String logName) {
            this.logName = logName;
        }
    }

    // #endregion

    // #region CONSTRUÇÃO E ESCOLHA DOS IOs

    /** Mantém compatibilidade com new Vision(drive) no RobotContainer. */
    public Vision(Drive drive) {
        this(drive, createDefaultCameras());
    }

    /** Permite usar somente Rear, somente Front ou ambas, com qualquer IO. */
    public Vision(Drive drive, VisionCamera... cameraList) {
        this(drive, Timer::getTimestamp, cameraList);
    }

    /** Relógio injetável para exercitar timestamps sem esperar tempo real. */
    public Vision(Drive drive, DoubleSupplier clock, VisionCamera... cameraList) {
        this.drive = Objects.requireNonNull(drive);
        this.clock = Objects.requireNonNull(clock);
        Set<String> names = new HashSet<>();
        for (VisionCamera camera : cameraList) {
            Objects.requireNonNull(camera);
            if (!names.add(camera.name)) {
                throw new IllegalArgumentException("Nome de câmera repetido: " + camera.name);
            }
            cameras.add(new CameraState(camera));
        }
        resetGeneration = drive.getPoseResetGeneration();
    }

    private static VisionCamera[] createDefaultCameras() {
        List<VisionCamera> result = new ArrayList<>();
        result.add(new VisionCamera("Rear", createIO(false)));
        // Replay precisa ler a frontal quando ela estiver presente no arquivo,
        // mesmo que a câmera física esteja desativada neste computador.
        // Sem esses inputs no log, o IO vazio permanece desconectado.
        if (FRONT_CAMERA_ENABLED || Logger.hasReplaySource()) {
            result.add(new VisionCamera("Front", createIO(true)));
        }
        return result.toArray(VisionCamera[]::new);
    }

    private static VisionIO createIO(boolean front) {
        // Replay usa as entradas gravadas, sem consultar NetworkTables.
        // A fonte de replay deve estar configurada em Robot.java.
        if (Logger.hasReplaySource()) {
            return new VisionIO() {
            };
        }
        if (RobotBase.isReal()) {
            return front ? new VisionIOLimelightFront() : new VisionIOLimelightRear();
        }
        // É um simulador de entradas controladas, não uma simulação óptica.
        return new VisionIOSim();
    }

    // #endregion

    // #region 🔁 CICLO PRINCIPAL

    @Override
    public void periodic() {
        double now = clock.getAsDouble();
        if (!Double.isFinite(now)) {
            return;
        }
        discardFramesBeforeReset();
        double yaw = drive.getRotation().getDegrees();
        double yawRate = drive.getYawVelocityDegreesPerSecond();

        // Uma câmera nunca depende de hasTarget, connected ou Valid da outra.
        for (CameraState state : cameras) {
            state.newMeasurement = false;
            state.measurementAccepted = false;
            state.camerasDisagree = false;
            state.fusionResult = "NO_NEW_FRAME";
            state.validatedPoses.clear();
            state.rejectedPoses.clear();
            state.appliedPoses.clear();
            state.camera.updateInputs(yaw, yawRate);
            validateAndQueue(state, now, yawRate);
        }

        processPending(now);
        updateAutoInitialization(now);
        boolean diagnosisChanged = false;
        for (CameraState state : cameras) {
            diagnosisChanged |= updateDiagnosis(state, now);
            logCamera(state, now);
        }
        if (diagnosisChanged) {
            diagnosisSummary = buildDiagnosisSummary();
        }
        Logger.recordOutput("Vision/Diagnostico", diagnosisSummary);
        Logger.recordOutput("Vision/Summary/ConfiguredCameras", cameras.size());
        Logger.recordOutput("Vision/Summary/PendingFrames", pending.size());
        Logger.recordOutput("Vision/Summary/ReorderSeconds", REORDER_SECONDS);
        Logger.recordOutput("Vision/Summary/PairWindowSeconds", PAIR_WINDOW_SECONDS);
    }

    private void discardFramesBeforeReset() {
        if (resetGeneration == drive.getPoseResetGeneration()) {
            return;
        }
        pending.clear();
        resetGeneration = drive.getPoseResetGeneration();
        for (CameraState state : cameras) {
            state.camera.latestRawVisionPose = null;
            state.camera.latestValidVisionPose = null;
            state.camera.lastRawVisionFrameTime = -1.0;
            state.camera.lastValidFrameTime = -1.0;
            state.lastSeedCaptureTimestamp = Double.NEGATIVE_INFINITY;
            // Não zeramos observedTimestamps: o mesmo frame não vira novo
            // só porque o piloto solicitou um reset do estimador.
        }
    }

    // #endregion

    // #region 📷 FILTROS INDEPENDENTES POR CÂMERA

    private void validateAndQueue(CameraState state, double now, double yawRate) {
        VisionCamera camera = state.camera;
        VisionIO.VisionIOInputs in = camera.inputs;

        if (in.usesObservationBatch) {
            // A leitura já passou por Logger.processInputs. No replay, estes
            // mesmos lotes vêm do log, sem consultar PhotonLib ou o cenário.
            PoseObservation[] observations = in.poseObservations;
            if (observations != null && observations.length > 0) {
                state.latestObservation = observations[observations.length - 1];
                // Ordena uma cópia: não modifica as entradas registradas.
                Arrays.stream(observations)
                        .sorted(Comparator.nullsLast(Comparator.comparingDouble(
                                PoseObservation::timestampSeconds)))
                        .forEach(observation -> considerObservation(state, observation,
                                true, observation != null && observation.pose() != null, now, yawRate));
            }
            // Lote vazio significa nenhuma medição nova. Os campos individuais
            // servem aos indicadores; nunca viram um frame substituto para fusão.
        } else {
            // Compatibilidade com os IOs Limelight e com logs anteriores.
            state.latestObservation = new PoseObservation(in.captureTimestampSeconds,
                    in.pose == null ? null : new Pose3d(in.pose), 0.0, in.tagCount,
                    in.avgTagDistanceMeters, PoseObservationType.MEGATAG_2);
            if (in.hasPoseEstimate || (Double.isFinite(in.captureTimestampSeconds)
                    && in.captureTimestampSeconds > 0.0)) {
                considerObservation(state, state.latestObservation,
                        in.hasPoseEstimate, in.hasPose, now, yawRate);
            }
        }

        // Mantém Valid/Seed coerentes entre frames (25 FPS versus ciclo de 50 Hz).
        // Perder as tags, desconectar ou envelhecer a captura invalida o estado.
        PoseObservation statusObservation = state.latestObservation;
        if (!in.hasPoseEstimate || !in.hasPose) {
            // Um frame novo sem solução não entra no lote, mas deve atualizar
            // NoTags/NoPose e o timestamp dos indicadores de disponibilidade.
            statusObservation = new PoseObservation(in.captureTimestampSeconds,
                    null, 0.0, in.tagCount, in.avgTagDistanceMeters,
                    in.usesObservationBatch ? PoseObservationType.PHOTONVISION : PoseObservationType.MEGATAG_2);
        }
        var reasons = validateObservation(state, statusObservation,
                in.hasPoseEstimate, in.hasPose, now, yawRate);
        camera.visionValidNow = reasons.isEmpty();
        state.statusReasons = reasons;
        // 🚧 ATENÇÃO (pente fino 31/10/2026): são 30 campos por câmera a cada ciclo, com nomes montados na hora.
        for (ObservationRejection reason : ObservationRejection.values()) {
            boolean rejected = reasons.contains(reason);
            Logger.recordOutput(camera.logPrefix + "/Fusion/Reject/" + reason.logName, rejected);
            // Alias para as abas AdvantageScope já configuradas no projeto.
            Logger.recordOutput(camera.logPrefix + "/MegaTag2/Reject/" + reason.logName, rejected);
        }
        double captureAge = now - in.captureTimestampSeconds;
        Logger.recordOutput(camera.logPrefix + "/MegaTag2/RotationRateValid",
                !reasons.contains(ObservationRejection.ROTATION));
        Logger.recordOutput(camera.logPrefix + "/Timing/CaptureAgeSeconds", captureAge);
        Logger.recordOutput(camera.logPrefix + "/Timing/CaptureTimestampAvailable",
                !reasons.contains(ObservationRejection.INVALID_TIMESTAMP));
    }

    /** Aplica os mesmos filtros temporais às leituras individuais e aos lotes. */
    private EnumSet<ObservationRejection> validateObservation(CameraState state,
            PoseObservation observation, boolean hasEstimate, boolean hasPose,
            double now, double yawRate) {
        EnumSet<ObservationRejection> reasons = EnumSet.noneOf(ObservationRejection.class);
        Pose3d pose = observation == null ? null : observation.pose();
        double timestamp = observation == null ? Double.NaN : observation.timestampSeconds();
        double distance = observation == null ? Double.NaN : observation.averageTagDistanceMeters();
        int tagCount = observation == null ? 0 : observation.tagCount();

        if (!state.camera.inputs.connected)
            reasons.add(ObservationRejection.DISCONNECTED);
        if (!hasEstimate || observation == null)
            reasons.add(ObservationRejection.NO_ESTIMATE);
        if (!hasPose || pose == null)
            reasons.add(ObservationRejection.NO_POSE);
        if (tagCount <= 0)
            reasons.add(ObservationRejection.NO_TAGS);
        boolean finitePose = pose != null && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY()) && Double.isFinite(pose.getZ())
                && Double.isFinite(pose.getRotation().getX())
                && Double.isFinite(pose.getRotation().getY())
                && Double.isFinite(pose.getRotation().getZ());
        if (pose != null && !finitePose)
            reasons.add(ObservationRejection.INVALID_POSE);
        if (!Double.isFinite(distance) || distance < 0.0)
            reasons.add(ObservationRejection.INVALID_DISTANCE);
        boolean validTime = Double.isFinite(timestamp) && timestamp > 0.0
                && Double.isFinite(state.camera.inputs.readTimestampSeconds);
        if (!validTime)
            reasons.add(ObservationRejection.INVALID_TIMESTAMP);
        if (!drive.isGyroHealthy() || !Double.isFinite(yawRate)
                || Math.abs(yawRate) > MAX_VISION_YAW_RATE_DEG_PER_SEC) {
            reasons.add(ObservationRejection.ROTATION);
        }
        if (validTime && now - timestamp > MAX_VISION_CAPTURE_AGE_SECONDS) {
            reasons.add(ObservationRejection.STALE);
        }
        if (validTime && timestamp - now > MAX_VISION_FUTURE_TOLERANCE_SECONDS) {
            reasons.add(ObservationRejection.FUTURE);
        }
        if (observation != null && observation.type() == null)
            reasons.add(ObservationRejection.UNKNOWN_TYPE);

        // 🛑📷 Robô fora do campo não existe: vale para MegaTag2 e PhotonLib. Com o heading
        // errado, o MegaTag2 espelha a pose em volta das tags e pode jogá-la para fora.
        if (finitePose && (pose.getX() < 0.0 || pose.getX() > VisionFieldConstants.FIELD_LENGTH_METERS
                || pose.getY() < 0.0 || pose.getY() > VisionFieldConstants.FIELD_WIDTH_METERS)) {
            reasons.add(ObservationRejection.OUTSIDE_FIELD);
        }

        // MegaTag2 permanece com os filtros existentes. PhotonLib também precisa
        // avaliar a solução 3D e a ambiguidade do fallback com uma única tag.
        if (observation != null && observation.type() == PoseObservationType.PHOTONVISION) {
            if (tagCount == 1 && (!Double.isFinite(observation.ambiguity())
                    || observation.ambiguity() < 0.0
                    || observation.ambiguity() > MAX_PHOTON_SINGLE_TAG_AMBIGUITY)) {
                reasons.add(ObservationRejection.AMBIGUITY);
            }
            if (finitePose) {
                if (Math.abs(pose.getZ()) > MAX_PHOTON_HEIGHT_METERS)
                    reasons.add(ObservationRejection.HEIGHT);
                if (Math.abs(pose.getRotation().getX()) > MAX_PHOTON_TILT_RADIANS
                        || Math.abs(pose.getRotation().getY()) > MAX_PHOTON_TILT_RADIANS) {
                    reasons.add(ObservationRejection.TILT);
                }
            }
        }
        return reasons;
    }

    private void considerObservation(CameraState state, PoseObservation observation,
            boolean hasEstimate, boolean hasPose, double now, double yawRate) {
        double timestamp = observation == null ? Double.NaN : observation.timestampSeconds();
        if (Double.isFinite(timestamp) && timestamp > 0.0) {
            if (!state.observedTimestamps.add(timestamp)) {
                state.fusionResult = "REPEATED_FRAME";
                return;
            }
            if (state.observedTimestamps.size() > MAX_REMEMBERED_TIMESTAMPS) {
                var oldest = state.observedTimestamps.iterator();
                oldest.next();
                oldest.remove();
            }
        }
        state.receivedCount++;
        var reasons = validateObservation(state, observation, hasEstimate, hasPose, now, yawRate);
        if (!reasons.isEmpty()) {
            if (observation != null && observation.pose() != null) {
                state.rejectedPoses.add(observation.pose());
            }
            reject(state, "INVALID_FRAME");
            return;
        }

        state.newMeasurement = true;
        state.validatedPoses.add(observation.pose());
        VisionCamera camera = state.camera;
        camera.lastMegaTag2TimestampSeconds = timestamp;
        state.lastGap = camera.lastSeenVisionFrameTime < 0.0
                ? -1.0
                : now - camera.lastSeenVisionFrameTime;
        camera.lastSeenVisionFrameTime = now;
        state.lastBaseStdDev = calculateVisionXYStdDev(
                observation.tagCount(), observation.averageTagDistanceMeters());
        if (pending.size() >= MAX_PENDING_FRAMES) {
            reject(state, "QUEUE_FULL");
            return;
        }
        pending.add(new Measurement(state, observation.pose().toPose2d(), timestamp, state.lastBaseStdDev,
                observation.tagCount(), observation.type(), sequence++));
        state.queuedCount++;
        state.fusionResult = "QUEUED";
    }

    /** Valores iniciais do projeto: menor desvio dá mais peso à medição. */
    private static double calculateVisionXYStdDev(int tagCount, double distance) {
        if (tagCount >= 2) {
            return distance <= NEAR_TAG_DISTANCE_METERS ? MULTI_TAG_NEAR_STD_DEV_METERS
                    : distance <= MEDIUM_TAG_DISTANCE_METERS ? MULTI_TAG_MEDIUM_STD_DEV_METERS
                            : MULTI_TAG_FAR_STD_DEV_METERS;
        }
        return distance <= NEAR_TAG_DISTANCE_METERS ? SINGLE_TAG_NEAR_STD_DEV_METERS
                : distance <= MEDIUM_TAG_DISTANCE_METERS ? SINGLE_TAG_MEDIUM_STD_DEV_METERS
                        : SINGLE_TAG_FAR_STD_DEV_METERS;
    }

    // #endregion

    // #region 📷 ORDENAÇÃO E COMPARAÇÃO ENTRE CÂMERAS

    private void processPending(double now) {
        while (!pending.isEmpty()) {
            Measurement oldest = pending.peek();
            double end = oldest.timestamp() + PAIR_WINDOW_SECONDS;
            // Espera limitada. Mesmo que a outra câmera nunca responda,
            // uma medição pode seguir sozinha após esta janela.
            if (end > now - REORDER_SECONDS) {
                break;
            }
            List<Measurement> batch = new ArrayList<>();
            while (!pending.isEmpty() && pending.peek().timestamp() <= end) {
                Measurement m = pending.remove();
                if (!m.state().camera.inputs.connected) {
                    reject(m.state(), "DISCONNECTED");
                } else if (now - m.timestamp() > MAX_QUEUED_AGE_SECONDS) {
                    reject(m.state(), "EXPIRED_IN_QUEUE");
                } else if (drive.samplePoseAt(m.timestamp()).isEmpty()) {
                    reject(m.state(), "NO_ODOMETRY_AT_CAPTURE");
                } else {
                    batch.add(m);
                }
            }
            if (!batch.isEmpty()) {
                fuseBatch(batch, now);
            }
        }
    }

    private void fuseBatch(List<Measurement> batch, double now) {
        Set<CameraState> contributors = new HashSet<>();
        Set<CameraState> conflicting = new HashSet<>();
        for (Measurement m : batch) {
            contributors.add(m.state());
        }

        // Compara capturas próximas, compensando o deslocamento estimado
        // entre elas. Comparar poses de instantes diferentes sem compensação
        // confundiria o movimento real do robô com um erro de câmera.
        for (int i = 0; i < batch.size(); i++) {
            Measurement a = batch.get(i);
            for (int j = i + 1; j < batch.size(); j++) {
                Measurement b = batch.get(j);
                if (a.state() == b.state()) {
                    continue;
                }
                Pose2d odomA = drive.samplePoseAt(a.timestamp()).orElseThrow();
                Pose2d odomB = drive.samplePoseAt(b.timestamp()).orElseThrow();
                Pose2d aAtB = new Pose2d(a.pose().getTranslation(), odomA.getRotation())
                        .transformBy(new Transform2d(odomA, odomB));
                double difference = aAtB.getTranslation().getDistance(b.pose().getTranslation());
                double limit = DISAGREEMENT_MARGIN_METERS + DISAGREEMENT_STD_MULTIPLIER
                        * Math.hypot(a.stdDev(), b.stdDev());
                Logger.recordOutput("Vision/Summary/CameraDifferenceMeters", difference);
                Logger.recordOutput("Vision/Summary/CameraDifferenceLimitMeters", limit);
                if (difference > limit) {
                    conflicting.add(a.state());
                    conflicting.add(b.state());
                }
            }
        }

        for (Measurement m : batch) {
            CameraState state = m.state();
            if (conflicting.contains(state)) {
                // Sem uma referência externa não sabemos qual está errada.
                // Este grupo é rejeitado e a odometria continua normalmente.
                // A próxima leitura coerente pode ser aceita sem reset manual.
                state.camerasDisagree = true;
                state.lastConflictTimestamp = Math.max(state.lastConflictTimestamp, m.timestamp());
                reject(state, "CAMERAS_DISAGREE");
                continue;
            }

            // Nosso IO informa somente o ID principal, não a lista completa.
            // Portanto não afirmamos saber quais tags se repetem nas câmeras.
            // Aplicamos uma redução conservadora do peso em grupos com mais
            // de uma câmera, mesmo que as tags sejam diferentes. É um ajuste
            // inicial, não um modelo estatístico completo de correlação.
            double effectiveStdDev = m.stdDev() * Math.sqrt(contributors.size());
            Optional<Pose2d> reference = drive.samplePoseAt(m.timestamp());
            state.lastPoseError = reference.orElse(drive.getPose()).getTranslation()
                    .getDistance(m.pose().getTranslation());

            // Erro relativo ao estimador é diagnóstico. Não bloqueamos aquisição
            // só porque a odometria começou longe da posição correta do campo.
            boolean accepted = drive.tryAddVisionMeasurement(state.camera.name,
                    m.pose(), m.timestamp(), effectiveStdDev);
            state.fusionResult = drive.getLastVisionResult();
            if (!accepted) {
                state.rejectedCount++;
                state.lastFrameOutcome = "DRIVE_RECUSOU_" + state.fusionResult;
                continue;
            }
            state.lastFrameOutcome = "ACEITA";
            state.measurementAccepted = true;
            state.lastAppliedStdDev = effectiveStdDev;
            state.lastAppliedAlgorithm = m.type().name();
            state.appliedPoses.add(m.pose());
            VisionCamera camera = state.camera;
            camera.latestValidVisionPose = m.pose();
            camera.latestValidTagCount = m.tagCount();
            camera.lastValidFrameTime = now;
            // Seed usa somente uma medição coerente e aplicada, ainda recente.
            camera.latestRawVisionPose = m.pose();
            camera.latestRawVisionTagCount = m.tagCount();
            camera.lastRawVisionFrameTime = now;
            state.lastSeedCaptureTimestamp = m.timestamp();
        }
    }

    private void reject(CameraState state, String reason) {
        state.fusionResult = reason;
        state.lastFrameOutcome = reason;
        state.rejectedCount++;
    }

    // #endregion

    // #region 📊 DIAGNÓSTICO: POR QUE A POSE NÃO ESTÁ SENDO CORRIGIDA

    /**
     * Uma linha por câmera dizendo o que impede a correção AGORA, na ordem em
     * que um problema esconde os seguintes (sem conexão, o resto não importa).
     * Teste de 29/10/2026: a Limelight mostrava tags na imagem, o shooter
     * esperou a sessão inteira por AGUARDANDO_LIMELIGHT_CORRIGIR_POSE e o
     * motivo só aparecia espalhado em vários campos Reject/*.
     * Retorna true quando o texto foi refeito (o resumo precisa ser montado de novo).
     */
    private boolean updateDiagnosis(CameraState state, double now) {
        String code = diagnosisCode(state, now);
        if (!code.equals(state.diagnosisCode)) {
            state.diagnosisCode = code;
            state.diagnosisSince = now;
            state.diagnosisReported = false;
            state.diagnosisTextDue = Double.NEGATIVE_INFINITY;
        }
        boolean rebuilt = now >= state.diagnosisTextDue;
        if (rebuilt) {
            state.diagnosisTextDue = now + DIAGNOSIS_TEXT_PERIOD_SECONDS;
            state.diagnosis = describe(code, state, now);
        }
        // Driver Station: só problemas que duram e que não são o normal
        // (OK, ou simplesmente nenhuma tag à vista). Uma vez por mudança.
        if (!state.diagnosisReported && now - state.diagnosisSince >= DIAGNOSIS_WARNING_SECONDS
                && !code.equals(DIAG_OK) && !code.equals(DIAG_NO_TAG)) {
            state.diagnosisReported = true;
            DriverStation.reportWarning("[FRC9460] Visão " + state.camera.name + ": " + state.diagnosis, false);
        }
        return rebuilt;
    }

    private String diagnosisCode(CameraState state, double now) {
        EnumSet<ObservationRejection> reasons = state.statusReasons;
        if (reasons.contains(ObservationRejection.DISCONNECTED)) return DIAG_DISCONNECTED;
        if (reasons.contains(ObservationRejection.NO_TAGS)) {
            return state.camera.inputs.hasTarget ? DIAG_TARGET_WITHOUT_POSE : DIAG_NO_TAG;
        }
        if (reasons.contains(ObservationRejection.ROTATION)) {
            return drive.isGyroHealthy() ? DIAG_SPINNING : DIAG_NO_GYRO;
        }
        if (reasons.contains(ObservationRejection.STALE)) return DIAG_STALE;
        if (reasons.contains(ObservationRejection.FUTURE)) return DIAG_FUTURE;
        if (!reasons.isEmpty()) return DIAG_REJECTED;
        VisionCamera c = state.camera;
        return c.lastValidFrameTime >= 0.0 && now - c.lastValidFrameTime <= DIAGNOSIS_RECENT_SECONDS
                ? DIAG_OK
                : DIAG_NOT_APPLIED;
    }

    private String describe(String code, CameraState state, double now) {
        VisionIO.VisionIOInputs in = state.camera.inputs;
        double ageMs = (now - in.captureTimestampSeconds) * 1000.0;
        return switch (code) {
            case DIAG_DISCONNECTED -> code + ": heartbeat da câmera parado no NetworkTables";
            case DIAG_NO_TAG -> code + ": a câmera não informou nenhuma tag usada (tv=0). Se a imagem"
                    + " mostra tags, confira LimelightConstants.VALID_TAG_IDS e o pipeline";
            case DIAG_TARGET_WITHOUT_POSE -> code + ": vê a tag " + in.tagId
                    + " mas a pose MegaTag2 veio sem tags (pipeline sem localização 3D ou tag fora do mapa)";
            case DIAG_NO_GYRO -> code + ": sem Pigeon não há odometria nem MegaTag2";
            case DIAG_SPINNING -> code + ": robô girando acima de "
                    + Math.round(MAX_VISION_YAW_RATE_DEG_PER_SEC) + " graus/s";
            case DIAG_STALE -> code + ": captura com " + Math.round(ageMs) + " ms (limite "
                    + Math.round(MAX_VISION_CAPTURE_AGE_SECONDS * 1000.0) + " ms)";
            case DIAG_FUTURE -> code + ": captura " + Math.round(-ageMs)
                    + " ms à frente do relógio do robô (sincronia do NetworkTables)";
            case DIAG_REJECTED -> code + ": " + state.statusReasons.iterator().next().logName;
            case DIAG_NOT_APPLIED -> code + ": último frame " + state.lastFrameOutcome;
            default -> code;
        };
    }

    /** Diagnóstico de uma câmera ("Rear" ou "Front"). */
    public String getDiagnosis(String cameraName) {
        return findCamera(cameraName).diagnosis;
    }

    /**
     * Resumo para o painel e para o aviso do tiro: a câmera que está
     * corrigindo, ou o motivo de cada uma.
     */
    public String getDiagnosis() {
        return diagnosisSummary;
    }

    private String buildDiagnosisSummary() {
        for (CameraState state : cameras) {
            if (state.diagnosisCode.equals(DIAG_OK)) {
                return state.camera.name + ": " + state.diagnosis;
            }
        }
        if (cameras.size() == 1) {
            return cameras.get(0).camera.name + ": " + cameras.get(0).diagnosis;
        }
        StringBuilder text = new StringBuilder();
        for (CameraState state : cameras) {
            if (text.length() > 0) text.append(" | ");
            text.append(state.camera.name).append(": ").append(state.diagnosis);
        }
        return text.toString();
    }

    // #endregion

    // #region 📷 INICIALIZAÇÃO AUTOMÁTICA DA POSE (MEGATAG1) E MONITOR DE HEADING

    /**
     * Desabilitado e parado: se as tags mostram que a pose do estimador está
     * desalinhada (heading errado, que espelha o MegaTag2), define X, Y e heading
     * pelas tags. Durante o jogo o heading não é tocado: só alerta se divergir.
     * Toda a decisão está em PoseInitializer; aqui só se liga ao robô e ao log.
     */
    private void updateAutoInitialization(double now) {
        PoseInitializer.Candidate candidate = bestMegaTag1(now);
        boolean stationary = drive.isGyroHealthy() && drive.isStationary();
        boolean eligible = AUTO_INIT_ENABLED && stationary && DriverStation.isDisabled();
        autoInitAppliedNow = false;
        var pose = poseInitializer.update(now, eligible, candidate, drive.getPose());
        if (pose.isPresent() && drive.initializePoseFromVision(pose.get())) {
            autoInitCount++;
            autoInitAppliedNow = true;
        }
        updateHeadingMonitor(now, candidate, stationary);

        Logger.recordOutput("Vision/AutoInit/Estado", poseInitializer.state().name());
        Logger.recordOutput("Vision/AutoInit/FramesJuntados", poseInitializer.framesCollected());
        Logger.recordOutput("Vision/AutoInit/ErroHeadingGraus", poseInitializer.headingErrorDegrees());
        Logger.recordOutput("Vision/AutoInit/ErroPosicaoMetros", poseInitializer.positionErrorMeters());
        Logger.recordOutput("Vision/AutoInit/AplicadaAgora", autoInitAppliedNow);
        Logger.recordOutput("Vision/AutoInit/Aplicacoes", autoInitCount);
        Logger.recordOutput("Vision/AutoInit/Resultado", drive.getVisionInitResult());
    }

    /** O heading do estimador diverge das tags há mais de HEADING_ALERT_SECONDS (robô parado). */
    public boolean isHeadingAlertActive() {
        return headingAlert.get();
    }

    /** Quantas vezes a pose foi definida pelas tags desde que o programa iniciou. */
    public int getAutoInitCount() {
        return autoInitCount;
    }

    /** A melhor leitura MegaTag1 fresca entre as câmeras: mais tags, depois mais perto. */
    private PoseInitializer.Candidate bestMegaTag1(double now) {
        PoseInitializer.Candidate best = null;
        for (CameraState state : cameras) {
            VisionIO.VisionIOInputs in = state.camera.inputs;
            if (!in.connected || now - in.megaTag1TimestampSeconds > MAX_VISION_CAPTURE_AGE_SECONDS) {
                continue;
            }
            var candidate = new PoseInitializer.Candidate(in.megaTag1Pose, in.megaTag1TagCount,
                    in.megaTag1AvgTagDistanceMeters, in.megaTag1TimestampSeconds);
            if (!PoseInitializer.isUsable(candidate)) {
                continue;
            }
            if (best == null || candidate.tagCount() > best.tagCount()
                    || (candidate.tagCount() == best.tagCount()
                            && candidate.avgTagDistanceMeters() < best.avgTagDistanceMeters())) {
                best = candidate;
            }
        }
        return best;
    }

    /**
     * Parado e vendo tags boas, o heading do estimador não pode divergir delas
     * por muito tempo. Com o robô habilitado nada é corrigido: o heading não muda
     * no jogo (giraria a referência do piloto), então vira alerta no painel.
     */
    private void updateHeadingMonitor(double now, PoseInitializer.Candidate candidate, boolean stationary) {
        boolean divergent = candidate != null && stationary
                && Math.abs(MathUtil.inputModulus(candidate.pose().getRotation()
                        .minus(drive.getRotation()).getDegrees(), -180.0, 180.0))
                        > AUTO_INIT_HEADING_TOLERANCE_DEGREES;
        if (!divergent) {
            headingDivergentSince = Double.NaN;
        } else if (Double.isNaN(headingDivergentSince)) {
            headingDivergentSince = now;
        }
        boolean alert = divergent && now - headingDivergentSince >= HEADING_ALERT_SECONDS;
        headingAlert.set(alert);
        Logger.recordOutput("Vision/AutoInit/HeadingDivergente", alert);
    }

    // #endregion

    // #region 📷 SEED MANUAL

    private boolean seedReady(CameraState state, double now) {
        return state.camera.visionValidNow && state.camera.latestRawVisionPose != null
                && state.camera.latestRawVisionTagCount >= 2
                && state.lastSeedCaptureTimestamp > state.lastConflictTimestamp
                && now >= state.lastSeedCaptureTimestamp
                && now - state.lastSeedCaptureTimestamp <= MAX_VISION_CAPTURE_AGE_SECONDS;
    }

    /**
     * Ação explícita do piloto. Preserva o heading de campo e exige robô parado.
     */
    public void trySeedDrivePose() {
        double now = clock.getAsDouble();
        discardFramesBeforeReset();
        CameraState best = cameras.stream().filter(s -> seedReady(s, now))
                .min(Comparator.comparingDouble(s -> s.lastAppliedStdDev)).orElse(null);
        boolean accepted = best != null && drive.isStationary();
        Logger.recordOutput("Vision/Summary/SeedAccepted", accepted);
        for (CameraState state : cameras) {
            Logger.recordOutput(state.camera.logPrefix + "/MegaTag2/SeedAccepted",
                    accepted && state == best);
        }
        if (accepted) {
            drive.seedPoseFromVision(best.camera.latestRawVisionPose);
            discardFramesBeforeReset();
        }
    }

    // #endregion

    // #region 📊 LOGS PARA O ADVANTAGESCOPE

    /**
     * Nome completo do campo no log ("Vision/Rear" + "/TX"), montado só na
     * primeira vez. Antes, os ~70 nomes por câmera eram concatenados a cada
     * ciclo e viravam lixo para o coletor do Java no roboRIO.
     */
    private static String key(CameraState state, String suffix) {
        String key = state.logKeys.get(suffix);
        if (key == null) {
            key = state.camera.logPrefix + suffix;
            state.logKeys.put(suffix, key);
        }
        return key;
    }

    private void logCamera(CameraState state, double now) {
        VisionCamera c = state.camera;
        VisionIO.VisionIOInputs in = c.inputs;
        Logger.recordOutput(key(state, "/Camera/Connected"), in.connected);
        Logger.recordOutput(key(state, "/Camera/Heartbeat"), in.heartbeat);
        Logger.recordOutput(key(state, "/GyroHealthy"), drive.isGyroHealthy());
        Logger.recordOutput(key(state, "/RobotYawDegrees"), drive.getRotation().getDegrees());
        Logger.recordOutput(key(state, "/HasTarget"), in.hasTarget);
        Logger.recordOutput(key(state, "/TX"), in.txDegrees);
        Logger.recordOutput(key(state, "/TY"), in.tyDegrees);
        Logger.recordOutput(key(state, "/TA"), in.targetAreaPercent);
        Logger.recordOutput(key(state, "/TagID"), in.tagId);
        Logger.recordOutput(key(state, "/Timing/ReportedLatencyMs"), in.latencyMilliseconds);
        Logger.recordOutput(key(state, "/Timing/CaptureAgeLimitSeconds"),
                MAX_VISION_CAPTURE_AGE_SECONDS);
        Logger.recordOutput(key(state, "/Timing/FutureToleranceSeconds"),
                MAX_VISION_FUTURE_TOLERANCE_SECONDS);

        // Caminhos comuns aos algoritmos. Os aliases MegaTag2 abaixo preservam
        // as abas antigas; ObservationType identifica a origem da medição.
        Logger.recordOutput(key(state, "/Fusion/ObservationType"), state.latestObservation == null
                || state.latestObservation.type() == null ? "NONE" : state.latestObservation.type().name());
        Logger.recordOutput(key(state, "/Fusion/LastAppliedAlgorithm"), state.lastAppliedAlgorithm);
        Logger.recordOutput(key(state, "/Fusion/Valid"), c.visionValidNow);
        Logger.recordOutput(key(state, "/Fusion/NewMeasurement"), state.newMeasurement);
        Logger.recordOutput(key(state, "/Fusion/MeasurementAccepted"), state.measurementAccepted);
        Logger.recordOutput(key(state, "/Fusion/FusionResult"), state.fusionResult);
        Logger.recordOutput(key(state, "/Fusion/ReceivedCount"), state.receivedCount);
        Logger.recordOutput(key(state, "/Fusion/AcceptedCount"), drive.getVisionAcceptedCount(c.name));
        Logger.recordOutput(key(state, "/Fusion/QueuedCount"), state.queuedCount);
        Logger.recordOutput(key(state, "/Fusion/RejectedCount"), state.rejectedCount);
        Logger.recordOutput(key(state, "/Fusion/Reject/CamerasDisagree"), state.camerasDisagree);
        Logger.recordOutput(key(state, "/Fusion/XYStdDevMeters"), state.lastAppliedStdDev);
        Logger.recordOutput(key(state, "/Fusion/PoseErrorMeters"), state.lastPoseError);
        Logger.recordOutput(key(state, "/Fusion/BatchSize"), in.usesObservationBatch && in.poseObservations != null
                ? in.poseObservations.length
                : 0);
        Logger.recordOutput(key(state, "/Fusion/ValidatedPoses"), state.validatedPoses.toArray(Pose3d[]::new));
        Logger.recordOutput(key(state, "/Fusion/RejectedPoses"), state.rejectedPoses.toArray(Pose3d[]::new));
        // AppliedPoses é um pulso da aplicação no Drive, depois da janela de
        // reordenação. Os arrays ficam vazios no próximo ciclo sem observações.
        Logger.recordOutput(key(state, "/Fusion/AppliedPoses"), state.appliedPoses.toArray(Pose2d[]::new));
        Logger.recordOutput(key(state, "/MegaTag2/Valid"), c.visionValidNow);
        Logger.recordOutput(key(state, "/MegaTag2/NewMeasurement"), state.newMeasurement);
        // Accepted é um pulso de aplicação, alguns ciclos depois de NewMeasurement.
        Logger.recordOutput(key(state, "/MegaTag2/MeasurementAccepted"), state.measurementAccepted);
        Logger.recordOutput(key(state, "/MegaTag2/FusionResult"), state.fusionResult);
        Logger.recordOutput(key(state, "/MegaTag2/AcceptedCount"), drive.getVisionAcceptedCount(c.name));
        Logger.recordOutput(key(state, "/MegaTag2/RejectedCount"), state.rejectedCount);
        Logger.recordOutput(key(state, "/MegaTag2/Reject/CamerasDisagree"), state.camerasDisagree);
        Logger.recordOutput(key(state, "/MegaTag2/TagCount"), in.tagCount);
        Logger.recordOutput(key(state, "/MegaTag2/AvgTagDist"), in.avgTagDistanceMeters);
        Logger.recordOutput(key(state, "/MegaTag2/YawRateDegPerSec"), drive.getYawVelocityDegreesPerSecond());
        Logger.recordOutput(key(state, "/MegaTag2/YawRateLimitDegPerSec"),
                MAX_VISION_YAW_RATE_DEG_PER_SEC);
        Logger.recordOutput(key(state, "/MegaTag2/BaseXYStdDevMeters"), state.lastBaseStdDev);
        Logger.recordOutput(key(state, "/MegaTag2/XYStdDevMeters"), state.lastAppliedStdDev);
        Logger.recordOutput(key(state, "/MegaTag2/PoseErrorMeters"), state.lastPoseError);
        Logger.recordOutput(key(state, "/MegaTag2/VisionGapSeconds"), state.lastGap);
        Logger.recordOutput(key(state, "/MegaTag2/FrameAgeSeconds"), c.lastValidFrameTime < 0.0
                ? -1.0
                : now - c.lastValidFrameTime);
        Logger.recordOutput(key(state, "/MegaTag2/RawFrameAgeSeconds"), c.lastRawVisionFrameTime < 0.0
                ? -1.0
                : now - c.lastRawVisionFrameTime);
        Logger.recordOutput(key(state, "/MegaTag2/SeedCaptureAgeSeconds"), Double.isFinite(state.lastSeedCaptureTimestamp)
                ? now - state.lastSeedCaptureTimestamp
                : -1.0);
        Logger.recordOutput(key(state, "/MegaTag2/SeedVisionReady"), seedReady(state, now));
        Logger.recordOutput(key(state, "/MegaTag2/SeedReady"), seedReady(state, now) && drive.isStationary());
        boolean outlier = state.lastPoseError > 1.0;
        Logger.recordOutput(key(state, "/MegaTag2/OutlierSuspected"), outlier);
        Logger.recordOutput(key(state, "/MegaTag2/Diagnostic/DistanceAbovePreferred"),
                in.avgTagDistanceMeters > MAX_TAG_DISTANCE_METERS);
        // Estes valores ficam na última pose quando não há tags. Leia Valid junto.
        if (c.visionValidNow) {
            Logger.recordOutput(key(state, "/MegaTag2/Pose"), in.pose);
            Logger.recordOutput(key(state, "/MegaTag2/X"), in.pose.getX());
            Logger.recordOutput(key(state, "/MegaTag2/Y"), in.pose.getY());
        }
        if (c.latestRawVisionPose != null) {
            Logger.recordOutput(key(state, "/MegaTag2/RawSeedPose"), c.latestRawVisionPose);
        }

        // Heading pelas tags (MegaTag1) menos o heading do robô. O MegaTag2 e
        // a mira usam o heading do Pigeon, definido à mão (Start + A): se ele
        // estiver X° errado, a pose gira em volta das tags e o tiro sai X°
        // torto, sem nenhum outro campo perceber. Com uma tag só o MegaTag1
        // é ambíguo; vale com o robô parado. NaN = sem leitura confiável.
        boolean megaTag1Usable = in.megaTag1TagCount >= MEGATAG1_HEADING_MIN_TAGS;
        Logger.recordOutput(key(state, "/MegaTag1/ErroHeadingGraus"), megaTag1Usable
                ? MathUtil.inputModulus(in.megaTag1Pose.getRotation().getDegrees()
                        - drive.getRotation().getDegrees(), -180.0, 180.0)
                : Double.NaN);
        Logger.recordOutput(key(state, "/MegaTag1/Pose"), in.megaTag1Pose);
        Logger.recordOutput(key(state, "/MegaTag1/RoboParado"), drive.isStationary());

        // Tags usadas na pose: linhas robô -> tag no 2D e no 3D do AdvantageScope.
        // Só o ponto: a linha não usa a rotação, e Pose3d ocupa mais que o dobro no log.
        if (!Arrays.equals(in.tagIds, state.loggedTagIds)) {
            state.loggedTagIds = in.tagIds.clone();
            state.usedTagPoints = Arrays.stream(in.tagIds)
                    .mapToObj(VisionFieldConstants.FIELD_LAYOUT::getTagPose)
                    .flatMap(Optional::stream)
                    .map(Pose3d::getTranslation)
                    .toArray(Translation3d[]::new);
        }
        Logger.recordOutput(key(state, "/TagsUsadas"), state.usedTagPoints);

        // Widget Field do Elastic: cada filho de Campo/ é um objeto no mapa.
        // Vazio esconde o fantasma quando não há leitura válida.
        Logger.recordOutput(state.fieldKey, c.visionValidNow
                ? new double[] {in.pose.getX(), in.pose.getY(), in.pose.getRotation().getDegrees()}
                : NO_FIELD_OBJECT);
    }

    /** Último resultado do ciclo. Útil também nos testes sem AdvantageScope. */
    public String getFusionResult(String cameraName) {
        return findCamera(cameraName).fusionResult;
    }

    // #endregion

    // #region CONSULTAS DE ALVO

    private CameraState findCamera(String name) {
        return cameras.stream().filter(s -> s.camera.name.equals(name)).findFirst()
                .orElseThrow(() -> new IllegalArgumentException("Câmera não configurada: " + name));
    }

    // As chamadas antigas continuam consultando a traseira.
    // Os overloads permitem selecionar explicitamente Rear ou Front.
    public boolean hasTarget() {
        return hasTarget("Rear");
    }

    public double getTX() {
        return getTX("Rear");
    }

    public double getTY() {
        return getTY("Rear");
    }

    public double getTA() {
        return getTA("Rear");
    }

    public int getTagID() {
        return getTagID("Rear");
    }

    public boolean hasTarget(String name) {
        return findCamera(name).camera.inputs.hasTarget;
    }

    public double getTX(String name) {
        return findCamera(name).camera.inputs.txDegrees;
    }

    public double getTY(String name) {
        return findCamera(name).camera.inputs.tyDegrees;
    }

    public double getTA(String name) {
        return findCamera(name).camera.inputs.targetAreaPercent;
    }

    public int getTagID(String name) {
        return findCamera(name).camera.inputs.tagId;
    }

    // #endregion
}
