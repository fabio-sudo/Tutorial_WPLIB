from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
dest = Path(__file__).resolve().parent
s = (root / 'src/main/java/frc/robot/subsystems/drive/Drive.java').read_text(encoding='utf-8-sig')
s = s.replace('🆕 ', '').replace('NEW - ', '')
s = '\n'.join(line.rstrip() for line in s.splitlines() if line.strip()) + '\n'
s = s.replace('import org.littletonrobotics.junction.Logger;', '''import org.littletonrobotics.junction.Logger;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;''')
s = s.replace('public class Drive extends SubsystemBase {', '''/** Um único estimador combina os módulos, o Pigeon e as câmeras. */
public class Drive extends SubsystemBase {

    // #region HISTÓRICO E CONTROLE TEMPORAL DA VISÃO
    // Timer.getTimestamp acompanha o relógio do AdvantageKit durante replay.
    private DoubleSupplier timestampSupplier = Timer::getTimestamp;
    private double firstOdometryTimestamp = Double.NaN;
    private double lastOdometryTimestamp = Double.NaN;
    private double lastVisionTimestamp = Double.NEGATIVE_INFINITY;
    private final Map<String, Double> lastVisionByCamera = new HashMap<>();
    private final Map<String, Long> visionAcceptedByCamera = new HashMap<>();
    private long poseResetGeneration = 0;
    private String lastVisionResult = "NOT_REQUESTED";
    // A WPILib 2026 guarda 1,5 s. Usamos margem de 100 ms na consulta.
    private static final double SAFE_HISTORY_SECONDS = 1.4;
    // #endregion
''')
s = s.replace('''    // Atualmente utiliza:
    // - Pigeon 2
    // - Posição dos quatro módulos
    //
    // FUTURAMENTE:
    // - Receberá também a pose do MegaTag2.
    //
    // Por enquanto NÃO utilizaremos dados da Limelight.''', '''    // Combina Pigeon 2, posição dos módulos e medições MegaTag2.
    // Vision valida e organiza as câmeras; somente Drive altera o estimador.''')
s = s.replace('''        if (RobotBase.isReal()) {''','''        // Replay não cria dispositivos CAN nem integra sensores simulados.
        // Robot.java precisa configurar a fonte de replay antes do container.
        if (Logger.hasReplaySource()) {
            frontLeft = new Module(new ModuleIO() {}, 0);
            frontRight = new Module(new ModuleIO() {}, 1);
            backLeft = new Module(new ModuleIO() {}, 2);
            backRight = new Module(new ModuleIO() {}, 3);
            gyroIO = new GyroIO() {};
        } else if (RobotBase.isReal()) {''', 1)
s = s.replace('''    Drive(GyroIO gyroIO, ModuleIO frontLeftIO, ModuleIO frontRightIO,
            ModuleIO backLeftIO, ModuleIO backRightIO) {
        this.gyroIO = gyroIO;''','''    public Drive(GyroIO gyroIO, ModuleIO frontLeftIO, ModuleIO frontRightIO,
            ModuleIO backLeftIO, ModuleIO backRightIO) {
        this(gyroIO, frontLeftIO, frontRightIO, backLeftIO, backRightIO,
                Timer::getTimestamp);
    }

    /** Permite testar a fusão sem hardware, usando sensores e relógio controlados. */
    public Drive(GyroIO gyroIO, ModuleIO frontLeftIO, ModuleIO frontRightIO,
            ModuleIO backLeftIO, ModuleIO backRightIO, DoubleSupplier clock) {
        timestampSupplier = Objects.requireNonNull(clock);
        this.gyroIO = Objects.requireNonNull(gyroIO);''')
s = s.replace('''            pose = poseEstimator.update(getRawGyroRotation(), getModulePositions());''','''            double now = timestampSupplier.getAsDouble();
            // Mesmo relógio utilizado na fila da visão e no replay.
            if (Double.isFinite(now)
                    && (!Double.isFinite(lastOdometryTimestamp) || now >= lastOdometryTimestamp)) {
                pose = poseEstimator.updateWithTime(now, getRawGyroRotation(), getModulePositions());
                if (!Double.isFinite(firstOdometryTimestamp)) {
                    firstOdometryTimestamp = now;
                }
                lastOdometryTimestamp = now;
            }''')
# Os dois resets existentes invalidam também os frames pendentes do Vision.
s = s.replace('''        poseEstimator.resetPosition(''', '''        clearVisionHistoryAfterReset();
        poseEstimator.resetPosition(''')
start = s.index('        // ============================================================\n        // FUSÃO DE VISÃO - MEGATAG2')
end = s.index('    // ============================================================\n    // VERIFICA SE O ROBÔ ESTÁ PARADO', start)
s = s[:start] + '''    // #region FUSÃO DE VISÃO

    /** Compatibilidade com chamadas antigas. Vision usa a versão identificada. */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        addVisionMeasurement(visionPose, timestampSeconds, 0.70);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
            double xyStdDevMeters) {
        tryAddVisionMeasurement("Legacy", visionPose, timestampSeconds, xyStdDevMeters);
    }

    /**
     * Aplica uma medição já validada pelo Vision.
     * Retorna true somente quando ela foi enviada a um histórico válido.
     * Timestamps iguais de câmeras diferentes são permitidos.
     * Repetir o mesmo timestamp da mesma câmera não é permitido.
     */
    public boolean tryAddVisionMeasurement(String cameraName, Pose2d visionPose,
            double timestampSeconds, double xyStdDevMeters) {
        if (cameraName == null || cameraName.isBlank() || cameraName.contains("/")
                || visionPose == null || !Double.isFinite(visionPose.getX())
                || !Double.isFinite(visionPose.getY())
                || !Double.isFinite(visionPose.getRotation().getRadians())
                || !Double.isFinite(timestampSeconds) || timestampSeconds <= 0.0
                || !Double.isFinite(xyStdDevMeters) || xyStdDevMeters <= 0.0) {
            return rejectVision("INVALID_DATA");
        }
        if (!isGyroHealthy()) {
            return rejectVision("GYRO_UNAVAILABLE");
        }
        if (timestampSeconds <= lastVisionByCamera.getOrDefault(
                cameraName, Double.NEGATIVE_INFINITY)) {
            return rejectVision("DUPLICATE_OR_OLDER_CAMERA_FRAME");
        }
        // A WPILib remove correções posteriores quando recebe visão antiga.
        // A fila do Vision reduz esse caso; esta proteção impede retroceder.
        if (timestampSeconds < lastVisionTimestamp) {
            return rejectVision("OUT_OF_ORDER");
        }
        Optional<Pose2d> atCapture = samplePoseAt(timestampSeconds);
        if (atCapture.isEmpty()) {
            return rejectVision("NO_ODOMETRY_AT_CAPTURE");
        }

        // MegaTag2 usa o yaw que fornecemos. Não tratamos esse yaw como uma
        // segunda medição independente do Pigeon. Mantemos o heading histórico.
        Pose2d translationOnly = new Pose2d(
                visionPose.getTranslation(), atCapture.get().getRotation());
        poseEstimator.addVisionMeasurement(translationOnly, timestampSeconds,
                VecBuilder.fill(xyStdDevMeters, xyStdDevMeters, 9999999.0));
        pose = poseEstimator.getEstimatedPosition();
        lastVisionTimestamp = timestampSeconds;
        lastVisionByCamera.put(cameraName, timestampSeconds);
        visionAcceptedByCamera.merge(cameraName, 1L, Long::sum);
        lastVisionResult = "ACCEPTED";

        Logger.recordOutput("Robot/Pose", pose);
        Logger.recordOutput("Drive/Vision/LastResult", lastVisionResult);
        Logger.recordOutput("Drive/Vision/LastCamera", cameraName);
        Logger.recordOutput("Drive/Vision/LastTimestampSeconds", timestampSeconds);
        Logger.recordOutput("Drive/Vision/XYStdDevMeters", xyStdDevMeters);
        Logger.recordOutput("Drive/Vision/" + cameraName + "/XYStdDevMeters", xyStdDevMeters);
        Logger.recordOutput("Drive/Vision/" + cameraName + "/AcceptedCount",
                getVisionAcceptedCount(cameraName));
        return true;
    }

    private boolean rejectVision(String reason) {
        lastVisionResult = reason;
        Logger.recordOutput("Drive/Vision/LastResult", reason);
        return false;
    }

    /**
     * Consulta a estimativa histórica somente quando há cobertura real.
     * sampleAt da WPILib sozinho limita o tempo às pontas do histórico;
     * por isso verificamos os limites antes de chamá-lo.
     */
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        if (!Double.isFinite(timestampSeconds)
                || !Double.isFinite(firstOdometryTimestamp)
                || !Double.isFinite(lastOdometryTimestamp)
                || timestampSeconds < firstOdometryTimestamp
                || timestampSeconds > lastOdometryTimestamp
                || lastOdometryTimestamp - timestampSeconds > SAFE_HISTORY_SECONDS) {
            return Optional.empty();
        }
        return poseEstimator.sampleAt(timestampSeconds);
    }

    private void clearVisionHistoryAfterReset() {
        firstOdometryTimestamp = Double.NaN;
        lastOdometryTimestamp = Double.NaN;
        lastVisionTimestamp = Double.NEGATIVE_INFINITY;
        lastVisionByCamera.clear();
        poseResetGeneration++;
        Logger.recordOutput("Drive/Vision/PoseResetGeneration", poseResetGeneration);
    }

    public long getPoseResetGeneration() {
        return poseResetGeneration;
    }

    public String getLastVisionResult() {
        return lastVisionResult;
    }

    public long getVisionAcceptedCount(String cameraName) {
        return visionAcceptedByCamera.getOrDefault(cameraName, 0L);
    }

    // #endregion

''' + s[end:]
# Espaço entre seções, sem milhares de linhas vazias entre argumentos.
s = re.sub(r'\n(    // #region)', r'\n\n\1', s)
s = re.sub(r'\n{3,}', '\n\n', s)
(dest/'Drive.java').write_text(s, encoding='utf-8')
print('Drive completo preparado fora de src.')
