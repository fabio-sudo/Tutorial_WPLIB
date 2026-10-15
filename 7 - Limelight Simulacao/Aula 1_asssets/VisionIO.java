package frc.robot.subsystems.vision;

// #region - IMPORTS

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

// #endregion

public interface VisionIO {

    // #region - ENTRADAS DA CÂMERA

    @AutoLog
    public static class VisionIOInputs {

        // Comunicação com a câmera.
        public boolean connected = false;
        public double heartbeat = 0.0;

        // Instante em que o IO leu os dados, em segundos.
        public double readTimestampSeconds = 0.0;

        // Informações do alvo principal.
        public boolean hasTarget = false;
        public double txDegrees = 0.0;
        public double tyDegrees = 0.0;
        public double targetAreaPercent = 0.0;
        public int tagId = -1;

        // Disponibilidade dos dados de localização.
        public boolean hasPoseEstimate = false;
        public boolean hasPose = false;

        // Pose no referencial azul do campo.
        public Pose2d pose = new Pose2d();

        // Timestamp da captura, já corrigido pela latência.
        public double captureTimestampSeconds = 0.0;

        // Latência informada pela câmera, em milissegundos.
        public double latencyMilliseconds = 0.0;

        // Dados utilizados para calcular a confiança.
        public int tagCount = 0;
        public double avgTagDistanceMeters = 0.0;
    }

    // #endregion

    // #region - MÉTODOS DO IO

    // Preenche as entradas com uma leitura da câmera.
    default void updateInputs(VisionIOInputs inputs) {}

    // Fornece a orientação de campo utilizada pelo MegaTag2.
    default void setRobotOrientation(
        double yawDegrees,
        double yawRateDegPerSec
    ) {}

    // #endregion
}