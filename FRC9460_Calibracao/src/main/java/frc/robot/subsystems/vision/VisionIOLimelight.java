package frc.robot.subsystems.vision;

// #region  - IMPORTS

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.LimelightHelpers;

// #endregion

public class VisionIOLimelight implements VisionIO {

    // #region  - COMUNICAÇÃO

    private final String cameraName;
    private final NetworkTableEntry heartbeatEntry;

    // Critério inicial para diagnosticar atividade da câmera.
    private static final double HEARTBEAT_TIMEOUT_SECONDS = 0.5;

    private double lastHeartbeat = Double.NaN;

    private double lastHeartbeatChangeSeconds =
        Double.NEGATIVE_INFINITY;

    // #endregion

    // #region  - REENVIO DA CONFIGURAÇÃO

    // Montagem, filtro de IDs e modo de IMU guardados para o reenvio.
    // O código antigo (ProjetoFrc) reenviava a cada 1 s: um envio só na
    // construção pode se perder se a câmera ainda não conectou ao
    // NetworkTables, e ela fica com a configuração antiga.
    private static final double CONFIG_RESEND_PERIOD_SECONDS = 1.0;

    private final double[] cameraPose;
    private final int[] validTagIds;
    private double nextConfigSeconds = Double.NEGATIVE_INFINITY;

    // #endregion

    // #region  - CONFIGURAÇÃO DA CÂMERA

    public VisionIOLimelight(
        String cameraName,
        double forwardMeters,
        double rightMeters,
        double upMeters,
        double rollDegrees,
        double pitchDegrees,
        double yawDegrees,
        int[] validTagIds
    ) {
        this.cameraName = cameraName;

        heartbeatEntry =
            LimelightHelpers.getLimelightNTTableEntry(
                cameraName,
                "hb"
            );

        cameraPose = new double[] {
            forwardMeters, rightMeters, upMeters,
            rollDegrees, pitchDegrees, yawDegrees
        };
        this.validTagIds = validTagIds.clone();

        applyConfiguration();
    }

    /** Envia montagem, filtro de IDs e modo de IMU. Chamado a cada 1 s. */
    private void applyConfiguration() {
        // Posição física da câmera em relação ao robô.
        LimelightHelpers.setCameraPose_RobotSpace(
            cameraName,
            cameraPose[0],
            cameraPose[1],
            cameraPose[2],
            cameraPose[3],
            cameraPose[4],
            cameraPose[5]
        );

        // IDs permitidos para a localização. Um array vazio faria a câmera
        // rejeitar todas as tags: nesse caso vale o filtro do pipeline.
        if (validTagIds.length > 0) {
            LimelightHelpers.SetFiducialIDFiltersOverride(
                cameraName,
                validTagIds
            );
        }

        // Aplica o modo escolhido para esta calibração a cada câmera criada.
        // Modo 0: MegaTag2 usa a orientação externa enviada pelo robô,
        // baseada no Pigeon e na referência de campo do estimador.
        LimelightHelpers.SetIMUMode(
            cameraName,
            LimelightConstants.IMU_MODE
        );
    }

    // #endregion

    // #region  - ORIENTAÇÃO PARA O MEGATAG2

    @Override
    public void setRobotOrientation(
        double yawDegrees,
        double yawRateDegPerSec
    ) {
        // Evita enviar valores inválidos à câmera.
        if (!Double.isFinite(yawDegrees)
            || !Double.isFinite(yawRateDegPerSec)) {
            return;
        }

        LimelightHelpers.SetRobotOrientation(
            cameraName,
            yawDegrees,
            yawRateDegPerSec,
            0.0,
            0.0,
            0.0,
            0.0
        );
    }

    // #endregion

    // #region  - LEITURA DAS ENTRADAS

    @Override
    public void updateInputs(VisionIOInputs inputs) {

        // Informações do alvo principal.
        inputs.hasTarget = LimelightHelpers.getTV(cameraName);
        inputs.txDegrees = LimelightHelpers.getTX(cameraName);
        inputs.tyDegrees = LimelightHelpers.getTY(cameraName);
        inputs.targetAreaPercent = LimelightHelpers.getTA(cameraName);

        inputs.tagId = inputs.hasTarget
            ? (int) LimelightHelpers.getFiducialID(cameraName)
            : -1;

        // NaN indica que o heartbeat ainda não está disponível.
        inputs.heartbeat = heartbeatEntry.getDouble(Double.NaN);

        // Uma única leitura da estimativa por atualização.
        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                cameraName
            );

        // Registra o instante local após consultar os dados.
        inputs.readTimestampSeconds = Timer.getFPGATimestamp();

        if (inputs.readTimestampSeconds >= nextConfigSeconds) {
            nextConfigSeconds = inputs.readTimestampSeconds + CONFIG_RESEND_PERIOD_SECONDS;
            applyConfiguration();
        }

        updateConnectionStatus(inputs);

        inputs.hasPoseEstimate = estimate != null;
        inputs.hasPose = estimate != null && estimate.pose != null;

        // Os campos são preenchidos em todos os ciclos.
        // Assim, ausência de dados não mantém uma pose antiga
        // marcada como disponível.
        inputs.pose = inputs.hasPose
            ? estimate.pose
            : new Pose2d();

        inputs.captureTimestampSeconds = estimate != null
            ? estimate.timestampSeconds
            : 0.0;

        inputs.latencyMilliseconds = estimate != null
            ? estimate.latency
            : 0.0;

        inputs.tagCount = estimate != null
            ? estimate.tagCount
            : 0;

        inputs.avgTagDistanceMeters = estimate != null
            ? estimate.avgTagDist
            : 0.0;

        inputs.tagIds = usedTagIds(estimate);

        // MegaTag1: o yaw vem das próprias tags. Serve para conferir o heading e
        // para a inicialização automática (Vision.updateAutoInitialization).
        LimelightHelpers.PoseEstimate megaTag1 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
        boolean megaTag1Valid = megaTag1 != null && megaTag1.pose != null;
        inputs.megaTag1Pose = megaTag1Valid ? megaTag1.pose : new Pose2d();
        inputs.megaTag1TagCount = megaTag1Valid ? megaTag1.tagCount : 0;
        inputs.megaTag1AvgTagDistanceMeters = megaTag1Valid ? megaTag1.avgTagDist : 0.0;
        inputs.megaTag1TimestampSeconds = megaTag1Valid ? megaTag1.timestampSeconds : 0.0;
    }

    // #endregion

    // #region  - TAGS USADAS NA POSE

    private static int[] usedTagIds(LimelightHelpers.PoseEstimate estimate) {
        if (estimate == null || estimate.rawFiducials == null) {
            return new int[0];
        }
        int[] ids = new int[estimate.rawFiducials.length];
        for (int i = 0; i < ids.length; i++) {
            ids[i] = estimate.rawFiducials[i].id;
        }
        // Em ordem: a Limelight muda a ordem a cada quadro, e o log gravaria tudo de novo.
        Arrays.sort(ids);
        return ids;
    }

    // #endregion

    // #region  - DIAGNÓSTICO DO HEARTBEAT

    private void updateConnectionStatus(VisionIOInputs inputs) {

        if (Double.isFinite(inputs.heartbeat)) {

            // Aguarda observar uma mudança real do contador.
            if (Double.isFinite(lastHeartbeat)
                && inputs.heartbeat != lastHeartbeat) {

                lastHeartbeatChangeSeconds =
                    inputs.readTimestampSeconds;
            }

            lastHeartbeat = inputs.heartbeat;

        } else {
            lastHeartbeat = Double.NaN;

            lastHeartbeatChangeSeconds =
                Double.NEGATIVE_INFINITY;
        }

        inputs.connected =
            Double.isFinite(inputs.heartbeat)
            && inputs.readTimestampSeconds
                - lastHeartbeatChangeSeconds
                <= HEARTBEAT_TIMEOUT_SECONDS;
    }

    // #endregion
}
