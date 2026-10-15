package frc.robot.subsystems.vision;

// #region  - IMPORTS

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

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

        // Posição física da câmera em relação ao robô.
        LimelightHelpers.setCameraPose_RobotSpace(
            cameraName,
            forwardMeters,
            rightMeters,
            upMeters,
            rollDegrees,
            pitchDegrees,
            yawDegrees
        );

        // IDs permitidos para a localização.
        LimelightHelpers.SetFiducialIDFiltersOverride(
            cameraName,
            validTagIds
        );

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
