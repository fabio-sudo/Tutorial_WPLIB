package frc.robot.subsystems.fuel;

// #region IMPORTS

import static frc.robot.subsystems.fuel.FuelConstants.*;

import java.io.File;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.VisionFieldConstants;

// #endregion

/**
 * O "mundo" das bolas, só no computador (não existe no robô nem no replay).
 *
 * A cada ciclo:
 * 1. COLETA: intake aberto + rolo puxando + bola na boca do intake = a bola
 *    sai do chão e entra no depósito do robô;
 * 2. LANÇAMENTO: centrífuga e tower girando + bola no depósito = a bola sai
 *    do shooter com a velocidade calculada pelos RPMs MEDIDOS das rodas e a
 *    direção REAL do shooter;
 * 3. VOO: gravidade e arrasto. Descendo pela altura do hub dentro da
 *    abertura = ACERTO; caiu no chão = ERRO (a bola fica no campo e pode
 *    ser coletada de novo).
 *
 * Estes contadores são a "verdade" da simulação; o FuelTracker conta pelos
 * sensores e os dois podem ser comparados no layout.
 */
public class FuelSimulation {

    // #region ESTADO DO MUNDO

    private static final double LOOP_SECONDS = 0.02;
    private static final double FLIGHT_STEP_SECONDS = 0.005;
    private static final double TRACE_STEP_SECONDS = 0.02;
    // Tempo que o rolo leva para engolir a bola (o mesmo da carga simulada).
    private static final double PULL_SECONDS = frc.robot.subsystems.roller.RollerConstants.SIM_BALL_LOAD_SECONDS;
    private static final double FIELD_LENGTH = VisionFieldConstants.FIELD_LENGTH_METERS;
    private static final double FIELD_WIDTH = VisionFieldConstants.FIELD_WIDTH_METERS;

    /** Uma bola no ar, com o rastro do voo para desenhar. */
    private static final class FlyingBall {
        BallisticModel.State state;
        final double launchTime;
        final int number;
        final List<Translation3d> trace = new ArrayList<>();
        double traceClock;
        boolean crossedTargetHeight;
        double errorAtTargetHeight = Double.NaN;
        // Onde a bola desceu pela altura da boca do hub (null = nem chegou nela).
        Translation3d arrival;

        FlyingBall(BallisticModel.State state, double launchTime, int number) {
            this.state = state;
            this.launchTime = launchTime;
            this.number = number;
            trace.add(state.position());
        }
    }

    private final FuelRobotState robot;
    private final DoubleSupplier clock;
    private final List<Translation3d> floor = new ArrayList<>();
    private final List<FlyingBall> flying = new ArrayList<>();

    // Bola sendo puxada pelo rolo. Se o rolo parar antes de prendê-la
    // (2/3 da puxada), ela volta para o chão na boca do intake; depois disso
    // ela já está presa e entra mesmo que o botão seja solto.
    private Translation3d ballBeingPulled;
    private double pullGripSeconds;
    private double pullFinishSeconds;
    private boolean floorChanged = true;

    // #endregion

    // #region 🧪 CONTADORES (VERDADE DA SIMULAÇÃO)

    private int hopper;
    private int collected;
    private int launched;
    private int scored;
    private int missed;
    private double lastCollectSeconds = Double.NEGATIVE_INFINITY;
    private double lastLaunchSeconds = Double.NEGATIVE_INFINITY;
    private List<Translation3d> lastShotTrace = List.of();
    private final Deque<Pose3d> recentArrivals = new ArrayDeque<>();

    // #endregion

    // #region CONSTRUÇÃO

    public FuelSimulation(FuelRobotState robot) {
        this(robot, Timer::getTimestamp, loadInitialPositions());
    }

    /** Para os testes: relógio e bolas iniciais escolhidos. */
    public FuelSimulation(FuelRobotState robot, DoubleSupplier clock, List<Translation3d> initialBalls) {
        this.robot = Objects.requireNonNull(robot);
        this.clock = Objects.requireNonNull(clock);
        floor.addAll(initialBalls);
        hopper = PRELOADED_BALLS;
    }

    /** Lê as 456 posições de largada do campo 2026 (pasta deploy). */
    public static List<Translation3d> loadInitialPositions() {
        List<Translation3d> balls = new ArrayList<>();
        File file = new File(Filesystem.getDeployDirectory(), INITIAL_POSITIONS_FILE);
        try {
            JsonNode root = new ObjectMapper().readTree(file);
            for (JsonNode ball : root.get("bolas")) {
                balls.add(new Translation3d(ball.get(0).asDouble(), ball.get(1).asDouble(), ball.get(2).asDouble()));
            }
        } catch (Exception e) {
            DriverStation.reportWarning("Simulação das bolas sem posições iniciais: " + file, false);
        }
        return balls;
    }

    // #endregion

    // #region 🔁 CICLO (CHAMADO EM simulationPeriodic)

    public void update() {
        double now = clock.getAsDouble();
        collect(now);
        launch(now);
        fly(now);
        log();
    }

    // #endregion

    // #region 1. COLETA

    private void collect(double now) {
        if (ballBeingPulled != null) {
            if (!robot.rollerPulling() && now < pullGripSeconds) {
                floor.add(ballBeingPulled);
                floorChanged = true;
                ballBeingPulled = null;
            } else if (now >= pullFinishSeconds || !robot.rollerPulling()) {
                hopper++;
                collected++;
                ballBeingPulled = null;
            }
            return;
        }
        if (!robot.intakeReadyToCollect() || !robot.rollerPulling()
            || robot.rollerRpm() < ROLLER_COLLECT_MIN_RPM
            || hopper >= HOPPER_CAPACITY || now - lastCollectSeconds < COLLECT_PERIOD_SECONDS) {
            return;
        }
        // A bola entra pela boca do intake onde o robô ESTÁ, não onde o código acha.
        Pose2d pose = robot.physicalPose();
        for (Iterator<Translation3d> it = floor.iterator(); it.hasNext();) {
            Translation3d ball = it.next();
            if (ball.getZ() > CAPTURE_MAX_HEIGHT_METERS) {
                continue;
            }
            // Bola no referencial do robô: está na boca do intake?
            Translation2d inRobot = ball.toTranslation2d().minus(pose.getTranslation())
                .rotateBy(pose.getRotation().unaryMinus());
            if (inRobot.getX() >= CAPTURE_MIN_X_METERS && inRobot.getX() <= CAPTURE_MAX_X_METERS
                && Math.abs(inRobot.getY()) <= CAPTURE_HALF_WIDTH_METERS) {
                it.remove();
                floorChanged = true;
                ballBeingPulled = ball;
                pullGripSeconds = now + PULL_SECONDS * 2.0 / 3.0;
                pullFinishSeconds = now + PULL_SECONDS;
                lastCollectSeconds = now;
                robot.onBallCollected();
                return;
            }
        }
    }

    // #endregion

    // #region 2. LANÇAMENTO

    private void launch(double now) {
        // A centrífuga empurra a bola só quando está comandada e girando: ao
        // parar ela FREIA (Centrifuga.stop) e corta a alimentação na hora.
        boolean wheelsFeeding = robot.centrifugeRunning()
            && robot.centrifugeRpm() >= FEED_MIN_CENTRIFUGE_RPM
            && robot.towerRpm() >= FEED_MIN_TOWER_RPM;
        if (hopper <= 0 || !wheelsFeeding || now - lastLaunchSeconds < FEED_PERIOD_SECONDS) {
            return;
        }
        // Velocidade pelos RPMs MEDIDOS: roda abaixo do alvo = bola mais curta.
        double exitSpeed = ShotComparison.ballExitSpeed(
            robot.topRpm(), robot.towerRpm(), robot.centrifugeRpm());
        // O shooter mira pela pose estimada, mas a bola sai de onde o robô ESTÁ:
        // o erro da visão desloca o tiro como no robô real.
        Translation2d poseError = robot.physicalPose().getTranslation().minus(robot.pose().getTranslation());
        double headingError = robot.physicalPose().getRotation().minus(robot.pose().getRotation()).getDegrees();
        Translation2d shooter = robot.shooterFieldPosition().plus(poseError);
        double heading = robot.shooterHeadingDegrees() + headingError;
        Translation3d start = new Translation3d(shooter.getX(), shooter.getY(),
            ShooterConstants.RELEASE_HEIGHT_METERS);
        Translation3d velocity = BallisticModel.launchVelocity(
            heading, exitSpeed, LAUNCH_ANGLE_DEGREES, robot.fieldSpeeds());
        flying.add(new FlyingBall(new BallisticModel.State(start, velocity), now, launched + 1));
        hopper--;
        launched++;
        lastLaunchSeconds = now;
        robot.onBallLaunched();
        Logger.recordOutput("Fuel/Simulacao/UltimoLancamento/VelocidadeSaidaMps", exitSpeed);
        Logger.recordOutput("Fuel/Simulacao/UltimoLancamento/DirecaoGraus", heading);
        Logger.recordOutput("Fuel/Simulacao/UltimoLancamento/ErroDaPoseMetros", poseError.getNorm());
    }

    // #endregion

    // #region 3. VOO, HUB E CHÃO

    private void fly(double now) {
        double hubHeight = ShooterConstants.TARGET_HEIGHT_METERS;
        int steps = (int) Math.round(LOOP_SECONDS / FLIGHT_STEP_SECONDS);
        for (Iterator<FlyingBall> it = flying.iterator(); it.hasNext();) {
            FlyingBall ball = it.next();
            for (int i = 0; i < steps; i++) {
                Translation3d before = ball.state.position();
                ball.state = BallisticModel.step(ball.state, FLIGHT_STEP_SECONDS);
                Translation3d after = ball.state.position();
                ball.traceClock += FLIGHT_STEP_SECONDS;
                if (ball.traceClock >= TRACE_STEP_SECONDS) {
                    ball.trace.add(after);
                    ball.traceClock = 0.0;
                }

                // Descendo pela altura do hub: dentro da abertura é acerto.
                if (!ball.crossedTargetHeight && before.getZ() >= hubHeight && after.getZ() < hubHeight) {
                    ball.crossedTargetHeight = true;
                    double f = (before.getZ() - hubHeight) / (before.getZ() - after.getZ());
                    Translation2d crossing = before.toTranslation2d()
                        .interpolate(after.toTranslation2d(), f);
                    ball.errorAtTargetHeight = distanceToNearestHub(crossing);
                    ball.arrival = new Translation3d(crossing.getX(), crossing.getY(), hubHeight);
                    if (ball.errorAtTargetHeight <= HUB_OPENING_RADIUS_METERS) {
                        scored++;
                        finishShot(ball, now, "ACERTO", ball.errorAtTargetHeight);
                        it.remove();
                        break;
                    }
                }

                // Tocou o chão: errou e a bola fica no campo.
                if (after.getZ() <= BALL_RADIUS_METERS) {
                    missed++;
                    floor.add(new Translation3d(
                        MathUtil.clamp(after.getX(), BALL_RADIUS_METERS, FIELD_LENGTH - BALL_RADIUS_METERS),
                        MathUtil.clamp(after.getY(), BALL_RADIUS_METERS, FIELD_WIDTH - BALL_RADIUS_METERS),
                        BALL_RADIUS_METERS));
                    floorChanged = true;
                    boolean reachedHeight = ball.crossedTargetHeight;
                    finishShot(ball, now, reachedHeight ? "ERRO" : "NAO_CHEGOU_NA_ALTURA_DO_HUB",
                        reachedHeight ? ball.errorAtTargetHeight : distanceToNearestHub(after.toTranslation2d()));
                    it.remove();
                    break;
                }
            }
        }
    }

    private static double distanceToNearestHub(Translation2d point) {
        return ShotComparison.distanceToNearestHub(point);
    }

    private void finishShot(FlyingBall ball, double now, String result, double errorMeters) {
        ball.trace.add(ball.state.position());
        lastShotTrace = List.copyOf(ball.trace);
        // ONDE CHEGOU (verdade da simulação), para comparar com Fuel/Tiro.
        Logger.recordOutput("Fuel/Simulacao/UltimoTiro/Numero", ball.number);
        Logger.recordOutput("Fuel/Simulacao/UltimoTiro/Resultado", result);
        Logger.recordOutput("Fuel/Simulacao/UltimoTiro/DistanciaDoCentroDoHubMetros", errorMeters);
        Logger.recordOutput("Fuel/Simulacao/UltimoTiro/TempoDeVooSegundos", now - ball.launchTime);
        Logger.recordOutput("Fuel/Simulacao/UltimoTiro/Trajetoria", points(lastShotTrace));
        // ONDE CHEGOU de verdade (física), para o 3D comparar com a previsão pelos
        // sensores (Fuel/Tiro/Sensores/Chegada) e com o centro do hub (Fuel/Tiro/Devia).
        Logger.recordOutput("Fuel/Simulacao/UltimoTiro/Chegada", ball.arrival == null
            ? new Pose3d[0]
            : new Pose3d[] {ShotComparison.arrivalMarker(ball.arrival)});
        Translation2d hubTarget = robot.hubTarget();
        Logger.recordOutput("Fuel/Simulacao/UltimoTiro/LinhaDoErro", ball.arrival == null
            ? new Translation3d[0]
            : new Translation3d[] {new Translation3d(hubTarget.getX(), hubTarget.getY(),
                ShooterConstants.TARGET_HEIGHT_METERS), ball.arrival});
        if (ball.arrival != null) {
            recentArrivals.addLast(ShotComparison.arrivalMarker(ball.arrival));
            if (recentArrivals.size() > CONSISTENCY_WINDOW_SHOTS) {
                recentArrivals.removeFirst();
            }
        }
        Logger.recordOutput("Fuel/Simulacao/ChegadasRecentes", recentArrivals.toArray(Pose3d[]::new));
    }

    // #endregion

    // #region 📊 TELEMETRIA

    private void log() {
        // As bolas do chão só mudam quando uma é coletada ou cai: gravar só
        // nessas horas evita repetir centenas de poses a cada 20 ms.
        if (floorChanged) {
            Logger.recordOutput("Fuel/Simulacao/BolasNoChao", points(floor));
            floorChanged = false;
        }
        Logger.recordOutput("Fuel/Simulacao/BolasNoAr",
            flying.stream().map(b -> b.state.position()).toArray(Translation3d[]::new));
        Logger.recordOutput("Fuel/Simulacao/NoDeposito", hopper);
        Logger.recordOutput("Fuel/Simulacao/Coletadas", collected);
        Logger.recordOutput("Fuel/Simulacao/Lancadas", launched);
        Logger.recordOutput("Fuel/Simulacao/Acertos", scored);
        Logger.recordOutput("Fuel/Simulacao/Erros", missed);
        Logger.recordOutput("Fuel/Simulacao/NoAr", flying.size());
        Logger.recordOutput("Fuel/Simulacao/NoChao", floor.size());
    }

    // Só a posição (Translation3d): as bolas são esferas, a rotação não importa.
    private static Translation3d[] points(List<Translation3d> points) {
        return points.toArray(Translation3d[]::new);
    }

    // #endregion

    // #region CONSULTAS (TESTES)

    public int getHopperCount() { return hopper; }
    public int getCollected() { return collected; }
    public int getLaunched() { return launched; }
    public int getScored() { return scored; }
    public int getMissed() { return missed; }
    public int getFloorCount() { return floor.size(); }
    public int getFlyingCount() { return flying.size(); }
    public List<Translation3d> getLastShotTrace() { return lastShotTrace; }

    /** Para os testes começarem com bolas no depósito. */
    public void setHopperCount(int count) {
        hopper = MathUtil.clamp(count, 0, HOPPER_CAPACITY);
    }

    // #endregion
}
