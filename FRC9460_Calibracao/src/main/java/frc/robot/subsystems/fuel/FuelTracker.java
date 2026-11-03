package frc.robot.subsystems.fuel;

// #region IMPORTS

import static frc.robot.subsystems.fuel.FuelConstants.*;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;


// #endregion

/**
 * Conta coletas e disparos PELOS SENSORES, do mesmo jeito no robô real, na
 * simulação e no replay. Assim dá para comparar com o que aconteceu de verdade
 * (vídeo, contagem da equipe ou, na simulação, os contadores do FuelSimulation).
 *
 * - Coleta: o rolo puxando e a corrente do motor subindo (a bola sendo
 *   esmagada pelo rolo pede mais força). Bolas muito juntas viram uma só.
 * - Disparo: com a centrífuga ligada, o RPM do top cai de repente (a bola
 *   rouba energia da roda ao sair).
 *
 * A cada disparo detectado, grava duas trajetórias para comparar com onde a
 * bola chegou (FuelSimulation ou a anotação da equipe no FuelDashboard):
 * - Fuel/Tiro/Planejado: com o RPM alvo e o ângulo alvo do pivô;
 * - Fuel/Tiro/Sensores: com o RPM medido logo antes da bola e o encoder.
 * Junto vai o estado da visão (Fuel/Tiro/Visao): se a Limelight já tinha
 * corrigido a pose, há quanto tempo e, na simulação, o erro da pose.
 *
 * Também desenha a trajetória PREVISTA contínua (onde a bola iria agora).
 * Os limites de detecção estão em FuelConstants e precisam ser calibrados
 * no robô real.
 */
public class FuelTracker extends SubsystemBase implements AutoCloseable {

    // #region ESTADO

    private record Sample(double time, double rpm) {}

    private final FuelRobotState robot;
    private final DoubleSupplier clock;

    private int collections;
    private int shots;
    private boolean collectArmed = true;
    private int highCurrentReadings;
    private boolean rollerSpunUp;
    private int predictionCycle;
    private boolean predictionVisible = true;
    private double lastShotSeconds = Double.NEGATIVE_INFINITY;
    private final Deque<Sample> topHistory = new ArrayDeque<>();
    private int plannedHits;
    private int sensorHits;
    private int shotsWithRecentVision;
    private int shotsWithoutRecentVision;
    // Constância entre os disparos (estatística em janela) e onde cada bola chegou.
    private final ShotConsistency consistency = new ShotConsistency();
    private final Deque<Pose3d> recentArrivals = new ArrayDeque<>();
    private double lastShotIntervalSeconds = Double.NaN;
    private Translation2d ringHub;
    private Translation3d[] ring = new Translation3d[0];

    public FuelTracker(FuelRobotState robot) {
        this(robot, Timer::getTimestamp);
    }

    public FuelTracker(FuelRobotState robot, DoubleSupplier clock) {
        this.robot = Objects.requireNonNull(robot);
        this.clock = Objects.requireNonNull(clock);
    }

    // #endregion

    // #region 🔁 PERIODIC

    @Override
    public void periodic() {
        double now = clock.getAsDouble();
        boolean collectedNow = detectCollection(now);
        boolean shotNow = detectShot(now);
        if (shotNow) {
            recordShot();
        }

        Logger.recordOutput("Fuel/Contagem/ColetasDetectadas", collections);
        Logger.recordOutput("Fuel/Contagem/DisparosDetectados", shots);
        Logger.recordOutput("Fuel/Contagem/NoDepositoEstimado", Math.max(0, collections - shots));
        Logger.recordOutput("Fuel/Contagem/ColetaAgora", collectedNow);
        Logger.recordOutput("Fuel/Contagem/DisparoAgora", shotNow);
        // Limites desenhados junto dos sinais, para calibrar no robô real.
        Logger.recordOutput("Fuel/Contagem/LimiteCorrenteColetaAmps", COLLECT_CURRENT_AMPS);
        Logger.recordOutput("Fuel/Contagem/LimiteQuedaRpm", SHOT_RPM_DROP);
        Logger.recordOutput("Fuel/Contagem/RaioDoHubMetros", HUB_OPENING_RADIUS_METERS);

        logPrediction();
        logHubRing();
    }

    // #endregion

    // #region DETECÇÃO DE COLETA (CORRENTE DO ROLO)

    private boolean detectCollection(double now) {
        double current = robot.rollerCurrentAmps();
        // A corrente de partida do motor não é bola: só conta depois que o
        // rolo chegou na velocidade de trabalho desde que foi ligado.
        if (!robot.rollerPulling()) {
            rollerSpunUp = false;
        } else if (robot.rollerRpm() >= ROLLER_COLLECT_MIN_RPM) {
            rollerSpunUp = true;
        }
        boolean high = robot.rollerPulling() && rollerSpunUp && current >= COLLECT_CURRENT_AMPS;
        if (!high) {
            highCurrentReadings = 0;
            // Só rearma quando a corrente volta ao normal: uma bola, uma contagem.
            if (current < COLLECT_CURRENT_AMPS * COLLECT_REARM_FRACTION) {
                collectArmed = true;
            }
            return false;
        }
        // Conta leituras seguidas (não tempo): 0,02 s em ponto flutuante às
        // vezes dava 0,0199999 e a bola não contava.
        highCurrentReadings++;
        if (collectArmed && highCurrentReadings >= COLLECT_MIN_READINGS) {
            collectArmed = false;
            collections++;
            return true;
        }
        return false;
    }

    // #endregion

    // #region DETECÇÃO DE DISPARO (QUEDA DE RPM DO TOP)

    private boolean detectShot(double now) {
        double rpm = robot.topRpm();
        topHistory.addLast(new Sample(now, rpm));
        while (topHistory.size() > 1 && now - topHistory.peekFirst().time() > SHOT_DROP_WINDOW_SECONDS) {
            topHistory.removeFirst();
        }
        // Queda em relação ao RPM de ~60 ms atrás.
        double drop = topHistory.peekFirst().rpm() - rpm;
        Logger.recordOutput("Fuel/Contagem/QuedaRpmTop", drop);
        // A centrífuga girando empurra bola; o top comandado garante que a queda
        // não é só a roda desacelerando solta depois que o tiro parou.
        boolean centrifugeSpinning = robot.centrifugeRpm() >= FEED_MIN_CENTRIFUGE_RPM;
        if (centrifugeSpinning && robot.topCommanded() && drop >= SHOT_RPM_DROP
            && now - lastShotSeconds >= SHOT_REARM_SECONDS) {
            lastShotIntervalSeconds = Double.isFinite(lastShotSeconds) ? now - lastShotSeconds : Double.NaN;
            lastShotSeconds = now;
            shots++;
            return true;
        }
        return false;
    }

    // #endregion

    // #region TRAJETÓRIA PREVISTA

    private void logPrediction() {
        if (robot.topRpm() < PREDICTION_MIN_TOP_RPM) {
            // Apaga a linha uma vez, quando o top para.
            if (predictionVisible) {
                Logger.recordOutput("Fuel/Previsao/Trajetoria", new Translation3d[0]);
                predictionVisible = false;
            }
            return;
        }
        // 10 Hz bastam para ver a linha; a cada ciclo o log ficava enorme.
        if (predictionVisible && predictionCycle++ % PREDICTION_LOG_EVERY_CYCLES != 0) {
            return;
        }
        predictionVisible = true;
        ShotComparison.Estimate now = ShotComparison.estimate(robot.shooterFieldPosition(),
            robot.encoderShooterHeadingDegrees(), robot.topRpm(), robot.towerRpm(), robot.centrifugeRpm(),
            robot.fieldSpeeds());
        Logger.recordOutput("Fuel/Previsao/Trajetoria", now.trajectory());
        Logger.recordOutput("Fuel/Previsao/VelocidadeSaidaMps", now.exitSpeedMps());
        Logger.recordOutput("Fuel/Previsao/DistanciaDoCentroDoHubMetros", now.distanceFromHubCenterMeters());
        Logger.recordOutput("Fuel/Previsao/AcertaOHub", now.hits());
    }

    // #endregion

    // #region 📊 REGISTRO DE CADA DISPARO (PLANEJADO x SENSORES)

    /**
     * Chamado no ciclo em que o disparo foi detectado. O RPM do top usado nos
     * sensores é o de ~60 ms atrás: no instante da detecção a bola já tirou
     * energia da roda.
     */
    private void recordShot() {
        double topBeforeBall = topHistory.peekFirst().rpm();
        ShotComparison.Estimate planned = ShotComparison.estimate(robot.shooterFieldPosition(),
            robot.plannedShooterHeadingDegrees(), robot.topTargetRpm(), robot.towerTargetRpm(),
            robot.centrifugeTargetRpm(), robot.fieldSpeeds());
        ShotComparison.Estimate sensors = ShotComparison.estimate(robot.shooterFieldPosition(),
            robot.encoderShooterHeadingDegrees(), topBeforeBall, robot.towerRpm(), robot.centrifugeRpm(),
            robot.fieldSpeeds());
        plannedHits += planned.hits() ? 1 : 0;
        sensorHits += sensors.hits() ? 1 : 0;

        Logger.recordOutput("Fuel/Tiro/Numero", shots);
        logEstimate("Fuel/Tiro/Planejado/", planned);
        logEstimate("Fuel/Tiro/Sensores/", sensors);
        recordConsistency(topBeforeBall, planned, sensors);
        Logger.recordOutput("Fuel/Tiro/Planejado/TotalAcertos", plannedHits);
        Logger.recordOutput("Fuel/Tiro/Sensores/TotalAcertos", sensorHits);
        // Quanto o que aconteceu se afastou do planejado.
        Logger.recordOutput("Fuel/Tiro/Diferenca/VelocidadeSaidaMps", sensors.exitSpeedMps() - planned.exitSpeedMps());
        Logger.recordOutput("Fuel/Tiro/Diferenca/DirecaoGraus", MathUtil.inputModulus(
            sensors.headingDegrees() - planned.headingDegrees(), -180.0, 180.0));
        logVisionAtShot();
    }

    /**
     * A mira usa a pose do Drive: se a Limelight não corrige há tempo, a pose
     * pode ter escorregado com as rodas. Errou com visão velha? Olhe aqui antes
     * de mexer no RPM ou no ângulo.
     */
    private void logVisionAtShot() {
        boolean recent = robot.visionCorrectionRecent();
        shotsWithRecentVision += recent ? 1 : 0;
        shotsWithoutRecentVision += recent ? 0 : 1;
        Logger.recordOutput("Fuel/Tiro/Visao/PoseCorrigidaDesdeReset", robot.visionCorrectedSinceReset());
        Logger.recordOutput("Fuel/Tiro/Visao/CorrecaoRecente", recent);
        Logger.recordOutput("Fuel/Tiro/Visao/SegundosDesdeCorrecao", robot.visionCorrectionAgeSeconds());
        Logger.recordOutput("Fuel/Tiro/Visao/UltimaCamera", robot.lastVisionCamera());
        Logger.recordOutput("Fuel/Tiro/Visao/PoseEstimada", robot.pose());
        Logger.recordOutput("Fuel/Tiro/Visao/TirosComVisaoRecente", shotsWithRecentVision);
        Logger.recordOutput("Fuel/Tiro/Visao/TirosSemVisaoRecente", shotsWithoutRecentVision);
        // Só a simulação sabe a pose verdadeira; no robô real fica igual (erro 0).
        double poseError = robot.physicalPose().getTranslation().getDistance(robot.pose().getTranslation());
        Logger.recordOutput("Fuel/Tiro/Visao/PoseVerdadeira", robot.physicalPose());
        Logger.recordOutput("Fuel/Tiro/Visao/ErroDaPoseMetros", poseError);
    }

    // #endregion

    // #region CONSTÂNCIA E PONTOS DE CHEGADA (3D)

    /**
     * Uma linha por disparo para a estatística e os pontos 3D. ONDE DEVIA ter
     * chegado = o centro da boca do hub, na altura da boca. ONDE CHEGOU = o
     * ponto em que a bola desce por essa altura, calculado com o RPM e o
     * ângulo MEDIDOS (sensores) e com os PEDIDOS (planejado). No robô real
     * nenhum sensor vê a bola: a verdade é a anotação da equipe (FuelDashboard).
     */
    private void recordConsistency(double topBeforeBall, ShotComparison.Estimate planned,
            ShotComparison.Estimate sensors) {
        Translation2d hubTarget = robot.hubTarget();
        Translation3d hub = new Translation3d(hubTarget.getX(), hubTarget.getY(),
            ShooterConstants.TARGET_HEIGHT_METERS);
        double robotSpeed = Math.hypot(robot.fieldSpeeds().vxMetersPerSecond,
            robot.fieldSpeeds().vyMetersPerSecond);
        consistency.record(new ShotConsistency.Shot(shots, robot.hubDistanceMeters(), topBeforeBall,
            robot.topTargetRpm(), topBeforeBall - robot.topRpm(), sensors.exitSpeedMps(),
            robot.aimErrorDegrees(), sensors.distanceFromHubCenterMeters(), robotSpeed,
            lastShotIntervalSeconds, planned.hits(), sensors.hits()));

        Logger.recordOutput("Fuel/Tiro/Devia", ShotComparison.arrivalMarker(hub));
        logArrival("Fuel/Tiro/Planejado/", planned, hub);
        logArrival("Fuel/Tiro/Sensores/", sensors, hub);
        sensors.arrival().ifPresent(point -> {
            recentArrivals.addLast(ShotComparison.arrivalMarker(point));
            if (recentArrivals.size() > CONSISTENCY_WINDOW_SHOTS) {
                recentArrivals.removeFirst();
            }
        });
        // A nuvem dos últimos disparos em volta do centro: o tamanho dela é a constância.
        Logger.recordOutput("Fuel/Consistencia/Chegadas", recentArrivals.toArray(Pose3d[]::new));
    }

    /** Ponto de chegada (Pose3d[] de 0 ou 1) e a linha do centro do hub até ele. */
    private static void logArrival(String prefix, ShotComparison.Estimate estimate, Translation3d hub) {
        Logger.recordOutput(prefix + "Chegada", estimate.arrival()
            .map(point -> new Pose3d[] {ShotComparison.arrivalMarker(point)})
            .orElse(new Pose3d[0]));
        Logger.recordOutput(prefix + "LinhaDoErro", estimate.arrival()
            .map(point -> new Translation3d[] {hub, point})
            .orElse(new Translation3d[0]));
    }

    /**
     * Anel da abertura do hub (raio de HUB_OPENING_RADIUS_METERS, na altura da
     * boca): mostra no 3D se o ponto de chegada caiu dentro ou fora. Só muda
     * quando o hub mirado muda; o log grava apenas o que muda.
     */
    private void logHubRing() {
        Translation2d hubTarget = robot.hubTarget();
        if (!hubTarget.equals(ringHub)) {
            ringHub = hubTarget;
            int points = 24;
            ring = new Translation3d[points + 1];
            for (int i = 0; i <= points; i++) {
                double angle = 2.0 * Math.PI * i / points;
                ring[i] = new Translation3d(hubTarget.getX() + HUB_OPENING_RADIUS_METERS * Math.cos(angle),
                    hubTarget.getY() + HUB_OPENING_RADIUS_METERS * Math.sin(angle),
                    ShooterConstants.TARGET_HEIGHT_METERS);
            }
        }
        Logger.recordOutput("Fuel/Tiro/BocaDoHub", ring);
    }

    // #endregion

    // #region 📊 LOG DE UMA ESTIMATIVA

    private static void logEstimate(String prefix, ShotComparison.Estimate estimate) {
        Logger.recordOutput(prefix + "Trajetoria", estimate.trajectory());
        Logger.recordOutput(prefix + "VelocidadeSaidaMps", estimate.exitSpeedMps());
        Logger.recordOutput(prefix + "DirecaoGraus", estimate.headingDegrees());
        Logger.recordOutput(prefix + "DistanciaDoCentroDoHubMetros", estimate.distanceFromHubCenterMeters());
        Logger.recordOutput(prefix + "Acerta", estimate.hits());
    }

    // #endregion

    // #region CONSULTAS (TESTES)

    public int getCollections() { return collections; }
    public int getShotsWithoutRecentVision() {
        return shotsWithoutRecentVision;
    }

    public int getShots() { return shots; }

    public ShotConsistency getConsistency() { return consistency; }

    // #endregion

    // #region ENCERRAMENTO (TESTES)

    @Override
    public void close() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    // #endregion
}
