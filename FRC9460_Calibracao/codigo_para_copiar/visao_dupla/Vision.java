package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drive.Drive;

/**
 * Gerencia as câmeras independentemente e envia suas medições ao único
 * estimador do Drive. Não controla motores e não reseta a pose automaticamente.
 */
public class Vision extends SubsystemBase {

    // #region CONFIGURAÇÃO

    // A frontal já é suportada, mas seus valores de montagem ainda são exemplos.
    // Troque para true depois de medir e corrigir as constantes FRONT_*.
    private static final boolean FRONT_CAMERA_ENABLED = false;

    // Segura medições até terem pelo menos 80 ms de idade, dando tempo para
    // ordenar capturas que chegaram por caminhos diferentes. Não espera uma
    // resposta da outra câmera. Acrescenta atraso à correção, não à odometria.
    private static final double REORDER_SECONDS = 0.080;
    private static final double PAIR_WINDOW_SECONDS = 0.020;
    private static final double MAX_QUEUED_AGE_SECONDS = 0.500;
    private static final int MAX_PENDING_FRAMES = 128;

    // Comparação entre câmeras: parâmetros iniciais para validar em campo.
    // Limiar = margem + multiplicador * combinação dos desvios de X/Y.
    // Não é um certificado de precisão nem um limite oficial da Limelight.
    private static final double DISAGREEMENT_MARGIN_METERS = 0.15;
    private static final double DISAGREEMENT_STD_MULTIPLIER = 3.0;

    private final Drive drive;
    private final DoubleSupplier clock;
    private final List<CameraState> cameras = new ArrayList<>();
    private long resetGeneration;
    private long sequence = 0;

    // A sequência desempata timestamps iguais de maneira determinística.
    private final PriorityQueue<Measurement> pending = new PriorityQueue<>(
            Comparator.comparingDouble(Measurement::timestamp)
                    .thenComparingLong(Measurement::sequence));

    private static final class CameraState {
        final VisionCamera camera;
        double lastObservedTimestamp = Double.NEGATIVE_INFINITY;
        double lastSeedCaptureTimestamp = Double.NEGATIVE_INFINITY;
        double lastConflictTimestamp = Double.NEGATIVE_INFINITY;
        double lastBaseStdDev = 1.2;
        double lastAppliedStdDev = 1.2;
        double lastPoseError = 0.0;
        double lastGap = -1.0;
        boolean newMeasurement;
        boolean measurementAccepted;
        boolean camerasDisagree;
        String fusionResult = "NO_NEW_FRAME";
        long queuedCount;
        long rejectedCount;

        CameraState(VisionCamera camera) {
            this.camera = Objects.requireNonNull(camera);
        }
    }

    // Snapshot imutável: a próxima leitura não modifica um frame já na fila.
    private record Measurement(CameraState state, Pose2d pose, double timestamp,
            double stdDev, int tagCount, long sequence) {}

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
        if (FRONT_CAMERA_ENABLED) {
            result.add(new VisionCamera("Front", createIO(true)));
        }
        return result.toArray(VisionCamera[]::new);
    }

    private static VisionIO createIO(boolean front) {
        // Replay usa as entradas gravadas, sem consultar NetworkTables.
        // A fonte de replay deve estar configurada em Robot.java.
        if (Logger.hasReplaySource()) {
            return new VisionIO() {};
        }
        if (RobotBase.isReal()) {
            return front ? new VisionIOLimelightFront() : new VisionIOLimelightRear();
        }
        // É um simulador de entradas controladas, não uma simulação óptica.
        return new VisionIOSim();
    }

    // #endregion

    // #region CICLO PRINCIPAL

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
            state.camera.updateInputs(yaw, yawRate);
            validateAndQueue(state, now, yawRate);
        }

        processPending(now);
        for (CameraState state : cameras) {
            logCamera(state, now);
        }
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
            // Não zeramos lastObservedTimestamp: o mesmo frame não vira novo
            // só porque o piloto solicitou um reset do estimador.
        }
    }

    // #endregion

    // #region FILTROS INDEPENDENTES POR CÂMERA

    private void validateAndQueue(CameraState state, double now, double yawRate) {
        VisionCamera camera = state.camera;
        VisionIO.VisionIOInputs in = camera.inputs;
        String p = camera.logPrefix + "/MegaTag2/";
        double timestamp = in.captureTimestampSeconds;
        // O helper já descontou a latência. Não descontar novamente.
        double captureAge = in.readTimestampSeconds - timestamp;

        boolean noEstimate = !in.hasPoseEstimate;
        boolean noPose = !in.hasPose || in.pose == null;
        boolean noTags = in.tagCount <= 0;
        boolean invalidPose = !noPose && (!Double.isFinite(in.pose.getX())
                || !Double.isFinite(in.pose.getY())
                || !Double.isFinite(in.pose.getRotation().getRadians()));
        boolean invalidDistance = !Double.isFinite(in.avgTagDistanceMeters)
                || in.avgTagDistanceMeters < 0.0;
        boolean invalidTimestamp = !Double.isFinite(timestamp) || timestamp <= 0.0
                || !Double.isFinite(in.readTimestampSeconds) || !Double.isFinite(captureAge);
        boolean rotation = !drive.isGyroHealthy() || !Double.isFinite(yawRate)
                || Math.abs(yawRate) > LimelightConstants.MAX_VISION_YAW_RATE_DEG_PER_SEC;
        boolean stale = !invalidTimestamp
                && captureAge > LimelightConstants.MAX_VISION_CAPTURE_AGE_SECONDS;
        boolean future = !invalidTimestamp
                && captureAge < -LimelightConstants.MAX_VISION_FUTURE_TOLERANCE_SECONDS;

        camera.visionValidNow = !(noEstimate || noPose || noTags || invalidPose
                || invalidDistance || invalidTimestamp || rotation || stale || future);
        Logger.recordOutput(p + "Reject/NoEstimate", noEstimate);
        Logger.recordOutput(p + "Reject/NoPose", noPose);
        Logger.recordOutput(p + "Reject/NoTags", noTags);
        Logger.recordOutput(p + "Reject/InvalidPose", invalidPose);
        Logger.recordOutput(p + "Reject/InvalidDistanceData", invalidDistance);
        Logger.recordOutput(p + "Reject/InvalidTimestamp", invalidTimestamp);
        Logger.recordOutput(p + "Reject/Rotation", rotation);
        Logger.recordOutput(p + "Reject/StaleMeasurement", stale);
        Logger.recordOutput(p + "Reject/FutureTimestamp", future);
        Logger.recordOutput(p + "RotationRateValid", !rotation);
        Logger.recordOutput(camera.logPrefix + "/Timing/CaptureAgeSeconds", captureAge);
        Logger.recordOutput(camera.logPrefix + "/Timing/CaptureTimestampAvailable", !invalidTimestamp);

        // Detecta mudança de frame, sem exigir que o timestamp observado cresça.
        // Um timestamp futuro rejeitado não pode bloquear os próximos corretos.
        // O mesmo frame rejeitado e congelado não vira novo só porque o tempo passou.
        // O Drive continua impedindo a aplicação duplicada ou fora de ordem.
        boolean unseen = Double.isFinite(timestamp) && timestamp > 0.0
                && Double.compare(timestamp, state.lastObservedTimestamp) != 0;
        if (unseen) {
            state.lastObservedTimestamp = timestamp;
        }
        if (!camera.visionValidNow || !unseen) {
            state.fusionResult = camera.visionValidNow ? "REPEATED_FRAME" : "INVALID_FRAME";
            return;
        }

        state.newMeasurement = true;
        camera.lastMegaTag2TimestampSeconds = timestamp;
        state.lastGap = camera.lastSeenVisionFrameTime < 0.0
                ? -1.0 : now - camera.lastSeenVisionFrameTime;
        camera.lastSeenVisionFrameTime = now;
        state.lastBaseStdDev = calculateVisionXYStdDev(in.tagCount, in.avgTagDistanceMeters);
        if (pending.size() >= MAX_PENDING_FRAMES) {
            reject(state, "QUEUE_FULL");
            return;
        }
        pending.add(new Measurement(state, in.pose, timestamp, state.lastBaseStdDev,
                in.tagCount, sequence++));
        state.queuedCount++;
        state.fusionResult = "QUEUED";
    }

    /** Valores iniciais do projeto: menor desvio dá mais peso à medição. */
    private static double calculateVisionXYStdDev(int tagCount, double distance) {
        if (tagCount >= 2) {
            return distance <= 2.0 ? 0.25 : distance <= 3.0 ? 0.40 : 0.60;
        }
        return distance <= 2.0 ? 0.60 : distance <= 3.0 ? 0.85 : 1.20;
    }

    // #endregion

    // #region ORDENAÇÃO E COMPARAÇÃO ENTRE CÂMERAS

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
                if (now - m.timestamp() > MAX_QUEUED_AGE_SECONDS) {
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
                continue;
            }
            state.measurementAccepted = true;
            state.lastAppliedStdDev = effectiveStdDev;
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
        state.rejectedCount++;
    }

    // #endregion

    // #region SEED MANUAL

    private boolean seedReady(CameraState state, double now) {
        return state.camera.visionValidNow && state.camera.latestRawVisionPose != null
                && state.camera.latestRawVisionTagCount >= 2
                && state.lastSeedCaptureTimestamp > state.lastConflictTimestamp
                && now >= state.lastSeedCaptureTimestamp
                && now - state.lastSeedCaptureTimestamp
                        <= LimelightConstants.MAX_VISION_CAPTURE_AGE_SECONDS;
    }

    /** Ação explícita do piloto. Preserva o heading de campo e exige robô parado. */
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

    // #region LOGS PARA O ADVANTAGESCOPE

    private void logCamera(CameraState state, double now) {
        VisionCamera c = state.camera;
        VisionIO.VisionIOInputs in = c.inputs;
        String root = c.logPrefix;
        String p = root + "/MegaTag2/";
        Logger.recordOutput(root + "/Camera/Connected", in.connected);
        Logger.recordOutput(root + "/Camera/Heartbeat", in.heartbeat);
        Logger.recordOutput(root + "/GyroHealthy", drive.isGyroHealthy());
        Logger.recordOutput(root + "/RobotYawDegrees", drive.getRotation().getDegrees());
        Logger.recordOutput(root + "/HasTarget", in.hasTarget);
        Logger.recordOutput(root + "/TX", in.txDegrees);
        Logger.recordOutput(root + "/TY", in.tyDegrees);
        Logger.recordOutput(root + "/TA", in.targetAreaPercent);
        Logger.recordOutput(root + "/TagID", in.tagId);
        Logger.recordOutput(root + "/Timing/ReportedLatencyMs", in.latencyMilliseconds);
        Logger.recordOutput(root + "/Timing/CaptureAgeLimitSeconds",
                LimelightConstants.MAX_VISION_CAPTURE_AGE_SECONDS);
        Logger.recordOutput(root + "/Timing/FutureToleranceSeconds",
                LimelightConstants.MAX_VISION_FUTURE_TOLERANCE_SECONDS);
        Logger.recordOutput(p + "Valid", c.visionValidNow);
        Logger.recordOutput(p + "NewMeasurement", state.newMeasurement);
        // Accepted é um pulso de aplicação, alguns ciclos depois de NewMeasurement.
        Logger.recordOutput(p + "MeasurementAccepted", state.measurementAccepted);
        Logger.recordOutput(p + "FusionResult", state.fusionResult);
        Logger.recordOutput(p + "AcceptedCount", drive.getVisionAcceptedCount(c.name));
        Logger.recordOutput(p + "QueuedCount", state.queuedCount);
        Logger.recordOutput(p + "RejectedCount", state.rejectedCount);
        Logger.recordOutput(p + "Reject/CamerasDisagree", state.camerasDisagree);
        Logger.recordOutput(p + "TagCount", in.tagCount);
        Logger.recordOutput(p + "RawTagCount", in.tagCount);
        Logger.recordOutput(p + "AvgTagDist", in.avgTagDistanceMeters);
        Logger.recordOutput(p + "TimestampSeconds", in.captureTimestampSeconds);
        Logger.recordOutput(p + "YawRateDegPerSec", drive.getYawVelocityDegreesPerSecond());
        Logger.recordOutput(p + "YawRateLimitDegPerSec",
                LimelightConstants.MAX_VISION_YAW_RATE_DEG_PER_SEC);
        Logger.recordOutput(p + "BaseXYStdDevMeters", state.lastBaseStdDev);
        Logger.recordOutput(p + "XYStdDevMeters", state.lastAppliedStdDev);
        Logger.recordOutput(p + "PoseErrorMeters", state.lastPoseError);
        Logger.recordOutput(p + "VisionGapSeconds", state.lastGap);
        Logger.recordOutput(p + "FrameAgeSeconds", c.lastValidFrameTime < 0.0
                ? -1.0 : now - c.lastValidFrameTime);
        Logger.recordOutput(p + "RawFrameAgeSeconds", c.lastRawVisionFrameTime < 0.0
                ? -1.0 : now - c.lastRawVisionFrameTime);
        Logger.recordOutput(p + "SeedCaptureAgeSeconds", Double.isFinite(state.lastSeedCaptureTimestamp)
                ? now - state.lastSeedCaptureTimestamp : -1.0);
        Logger.recordOutput(p + "SeedVisionReady", seedReady(state, now));
        Logger.recordOutput(p + "SeedReady", seedReady(state, now) && drive.isStationary());
        boolean outlier = state.lastPoseError > 1.0;
        Logger.recordOutput(p + "Diagnostic/OutlierSuspected", outlier);
        Logger.recordOutput(p + "OutlierSuspected", outlier);
        Logger.recordOutput(p + "Diagnostic/DistanceAbovePreferred",
                in.avgTagDistanceMeters > LimelightConstants.MAX_TAG_DISTANCE_METERS);
        // Estes valores ficam na última pose quando não há tags. Leia Valid junto.
        if (c.visionValidNow) {
            Logger.recordOutput(p + "Pose", in.pose);
            Logger.recordOutput(p + "X", in.pose.getX());
            Logger.recordOutput(p + "Y", in.pose.getY());
            Logger.recordOutput(p + "RotationDegrees", in.pose.getRotation().getDegrees());
        }
        if (c.latestRawVisionPose != null) {
            Logger.recordOutput(p + "RawSeedPose", c.latestRawVisionPose);
        }
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
    public boolean hasTarget() { return hasTarget("Rear"); }
    public double getTX() { return getTX("Rear"); }
    public double getTY() { return getTY("Rear"); }
    public double getTA() { return getTA("Rear"); }
    public int getTagID() { return getTagID("Rear"); }
    public boolean hasTarget(String name) { return findCamera(name).camera.inputs.hasTarget; }
    public double getTX(String name) { return findCamera(name).camera.inputs.txDegrees; }
    public double getTY(String name) { return findCamera(name).camera.inputs.tyDegrees; }
    public double getTA(String name) { return findCamera(name).camera.inputs.targetAreaPercent; }
    public int getTagID(String name) { return findCamera(name).camera.inputs.tagId; }

    // #endregion
}
