package frc.robot.commands.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.centrifuga.Centrifuga;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.Aim;
import frc.robot.subsystems.shooter.ShotCalculator.MovingTarget;
import frc.robot.subsystems.shooter.ShotCalculator.Shot;

// #endregion

/**
 * Sequência de tiro, portada do ShooterTrackAndShootCommand da competição.
 *
 * Enquanto o botão está apertado, a cada ciclo:
 * 1. confere se a pose é confiável (Limelight já corrigiu);
 * 2. calcula o alvo (hub da aliança, compensado se o robô anda) e os RPMs;
 * 3. o pivô começa a mirar 0,25 s antes das rodas;
 * 4. liga top e tower SÓ com o hub ao alcance (dentro da curva de tiro e do
 *    curso de ±130° do pivô); fora disso as rodas ficam paradas;
 * 5. só liga a centrífuga (pasta própria, empurra a bola) quando top, tower e pivô
 *    estiverem prontos ("pronto por tempo" exige 85% do RPM nas rodas);
 * 6. soltou o botão: para tudo.
 *
 * Fica em commands/ porque coordena DOIS subsistemas (shooter e centrífuga).
 *
 * Diferença para a competição: com o hub fora do curso o código antigo
 * atirava com o pivô no limite (direção errada). Aqui ele espera o robô girar.
 */
public class ShooterTrackAndShootCommand extends Command {

    // #region ESTADO DA SEQUÊNCIA

    private final Shooter shooter;
    private final Centrifuga centrifuga;
    // Por que a visão ainda não corrigiu (Vision.getDiagnosis); vazio nos testes.
    private final Supplier<String> visionDiagnosis;

    // Instante em que a preparação começou (NaN = parada).
    private double preparationStartSeconds = Double.NaN;

    // A centrífuga continua ligada até este instante, mesmo que um
    // indicador oscile por um ciclo.
    private double feedGraceDeadlineSeconds = Double.NEGATIVE_INFINITY;
    private double readySinceSeconds = Double.NaN;

    // Diagnóstico ao soltar o botão: a centrífuga chegou a ligar? Se não, por quê.
    // No teste de 29/10/2026 o top girava e a centrífuga nunca ligava, sem aviso.
    private double pressedAtSeconds = Double.NaN;
    private int feedRequests;
    private boolean centrifugeRanThisPress;
    private String centrifugeRefusal = "";
    private String lastState = "INICIANDO";
    private String lastDetail = "";
    private double nextDetailSeconds;

    // #endregion

    // #region CONSTRUÇÃO

    public ShooterTrackAndShootCommand(Shooter shooter, Centrifuga centrifuga) {
        this(shooter, centrifuga, () -> "");
    }

    /** Com o diagnóstico da visão no aviso de AGUARDANDO_LIMELIGHT_CORRIGIR_POSE. */
    public ShooterTrackAndShootCommand(Shooter shooter, Centrifuga centrifuga,
            Supplier<String> visionDiagnosis) {
        this.shooter = shooter;
        this.centrifuga = centrifuga;
        this.visionDiagnosis = visionDiagnosis;
        addRequirements(shooter, centrifuga);
        setName("Shooter - mirar e atirar");
    }

    // #endregion

    // #region 🔁 CICLO DO COMANDO

    @Override
    public void initialize() {
        resetPreparation();
        stopAll();
        pressedAtSeconds = shooter.now();
        feedRequests = 0;
        centrifugeRanThisPress = false;
        centrifugeRefusal = "";
        lastDetail = "";
        nextDetailSeconds = Double.NEGATIVE_INFINITY;
        logState("INICIANDO", false, false, false, false, false, false, false);
    }

    @Override
    public void execute() {
        double now = shooter.now();

        // 🛑📷 1. Sem correção da visão, a pose é só o chute inicial da odometria.
        if (!shooter.isPoseTrusted()) {
            String waiting = "AGUARDANDO_LIMELIGHT_CORRIGIR_POSE";
            boolean refresh = detailDue(now, waiting);
            idle(waiting);
            // Teste de 29/10/2026: o aviso só dizia "aguardando", sem o motivo.
            if (refresh) {
                lastDetail = visionDiagnosis.get();
            }
            Logger.recordOutput("Shooter/Sequencia/Detalhe", lastDetail);
            return;
        }
        // 🛑 Falha de sensor/zero não ganha tolerância temporal e não pode ser
        // contornada pelo fallback de RPM. Não envia partida antes da validação.
        if (!shooter.areWheelsHealthy() || !shooter.isPivotHealthy()) {
            idle("SENSOR_SHOOTER_INDISPONIVEL");
            return;
        }

        // 🧭🚀 2. Alvo e RPMs. Parado, o alvo virtual é o próprio hub.
        Pose2d pose = shooter.getPose();
        Translation2d hub = shooter.getHubTarget();
        MovingTarget moving = ShotCalculator.compensateForMotion(pose, shooter.getFieldSpeeds(), hub);
        Shot shot = ShotCalculator.forPose(pose, moving.virtualTarget());
        if (!shot.valid()) {
            idle("TRAJETORIA_INVALIDA");
            return;
        }

        // 🧭 3. O pivô mira primeiro.
        if (Double.isNaN(preparationStartSeconds)) {
            preparationStartSeconds = now;
        }
        double elapsed = now - preparationStartSeconds;
        Aim aim = shooter.aimAt(moving.virtualTarget());
        if (elapsed < PIVOT_AIM_HEADSTART_SECONDS) {
            shooter.stopWheels();
            centrifuga.stop();
            logState("PIVO_MIRANDO_PRIMEIRO", false, false, false, false, aim.reachable(), false, false);
            return;
        }

        // 🛑 4. SEGURANÇA: rodas só aceleram com o hub ao alcance. Antes elas iam
        //    para o RPM máximo mesmo com o hub longe ou fora do curso do pivô
        //    (teste de 28/10/2026, mirando no hub errado).
        boolean inRange = !BLOCK_SHOTS_BEYOND_CURVE
            || shot.distanceMeters() <= SHOT_MAX_DISTANCE_METERS + SHOT_DISTANCE_MARGIN_METERS;
        boolean reachable = aim.reachable();
        Logger.recordOutput("Shooter/Sequencia/TempoPreparacaoSegundos", elapsed);
        Logger.recordOutput("Shooter/Sequencia/DistanciaMetros", shot.distanceMeters());
        Logger.recordOutput("Shooter/Sequencia/DentroDaCurva", inRange);
        if (!inRange || !reachable) {
            shooter.stopWheels();
            centrifuga.stop();
            feedGraceDeadlineSeconds = Double.NEGATIVE_INFINITY;
            readySinceSeconds = Double.NaN;
            String blockedState = !inRange ? "LONGE_DEMAIS_CHEGUE_MAIS_PERTO" : "HUB_FORA_DO_CURSO_GIRE_O_ROBO";
            if (detailDue(now, blockedState)) {
                lastDetail = String.format("distância %.2f m, pivô precisa de %.0f°",
                    shot.distanceMeters(), aim.desiredPivotDegrees());
            }
            logState(!inRange ? "LONGE_DEMAIS_CHEGUE_MAIS_PERTO" : "HUB_FORA_DO_CURSO_GIRE_O_ROBO",
                false, false, false, false, reachable, shooter.isPivotAtTarget(), false);
            return;
        }

        // 🚀 CONTROLE: rodas de disparo e alimentador no RPM da curva.
        shooter.setShotSpeeds(shot.topRpm(), shot.towerRpm());
        boolean timeFallback = USE_SPINUP_TIME_FALLBACK && elapsed >= SPINUP_FALLBACK_SECONDS;
        boolean topAtSpeed = shooter.isTopAtSpeed();
        boolean towerAtSpeed = shooter.isTowerAtSpeed();
        // 🛑 SEGURANÇA: pelo tempo, só com a roda girando de verdade (85% do pedido).
        boolean topReady = topAtSpeed
            || (timeFallback && spinningNear(shooter.getTopRpm(), shot.topRpm()));
        boolean towerReady = towerAtSpeed
            || (timeFallback && spinningNear(shooter.getTowerRpm(), shot.towerRpm()));
        boolean pivotReady = shooter.isPivotAtTarget();

        // 🛑 5. Confirma prontidão estável antes de ligar, para não dar pulsos
        // de partida a cada passagem momentânea pela tolerância do pivô.
        boolean ready = topReady && towerReady && pivotReady;
        // Queda de RPM dentro da margem (a bola passando) não reinicia a confirmação.
        // Antes, 1 ciclo abaixo da tolerância desligava a centrífuga 3 ciclos depois:
        // a margem (80 ms) acaba antes de uma nova confirmação (100 ms). Mira perdida reinicia.
        boolean wheelDipInGrace = pivotReady && now <= feedGraceDeadlineSeconds;
        if (!ready && !wheelDipInGrace) readySinceSeconds = Double.NaN;
        else if (ready && Double.isNaN(readySinceSeconds)) readySinceSeconds = now;
        boolean stable = ready && now - readySinceSeconds >= FEED_READY_STABLE_SECONDS;
        if (stable) {
            feedGraceDeadlineSeconds = now + CENTRIFUGE_READY_GRACE_SECONDS;
        }
        // Margem de RPM só com a mira ainda correta. Perdendo mira, corta.
        boolean feeding = pivotReady && (stable || now <= feedGraceDeadlineSeconds);
        // A centrífuga ainda pode recusar (sensor ou Talon sem resposta): isRunning diz se aceitou.
        boolean centrifugeRefused = false;
        if (feeding) {
            centrifuga.setRpm(shot.centrifugeRpm());
            feedRequests++;
            centrifugeRanThisPress |= centrifuga.isRunning();
            centrifugeRefused = !centrifuga.isRunning();
            if (centrifugeRefused) centrifugeRefusal = centrifuga.getStatus();
        } else {
            centrifuga.stop();
        }

        boolean readyOnlyByTime = feeding && (!topAtSpeed || !towerAtSpeed);
        String state = centrifugeRefused ? "CENTRIFUGA_RECUSOU"
            : feeding ? (readyOnlyByTime ? "DISPARANDO_PRONTO_POR_TEMPO" : "DISPARANDO")
            : !pivotReady ? "PIVO_BUSCANDO_ALVO"
            : ready ? "CONFIRMANDO_PRONTIDAO" : "ACELERANDO_TOP_E_TOWER";
        // Texto humano a 4 Hz ou na mudança de estado; controle e números seguem a 50 Hz.
        if (detailDue(now, state)) {
            lastDetail = String.format("pivô a %.1f° do alvo, tolerância %.2f°; top %s; tower %s",
                Math.abs(shooter.getPivotTargetDegrees() - shooter.getPivotDegrees()),
                PIVOT_TOLERANCE_MOTOR_ROTATIONS * PIVOT_DEGREES_PER_MOTOR_ROTATION,
                topReady ? "pronto" : "abaixo do RPM", towerReady ? "pronta" : "abaixo do RPM");
        }
        Logger.recordOutput("Shooter/Sequencia/Detalhe", lastDetail);
        logState(state, ready, feeding, topReady, towerReady, reachable, pivotReady, readyOnlyByTime);
    }

    /** A roda gira com pelo menos 85% do RPM pedido (SPINUP_FALLBACK_MIN_RPM_FRACTION). */
    private static boolean spinningNear(double measuredRpm, double targetRpm) {
        return targetRpm > 0.0 && measuredRpm >= SPINUP_FALLBACK_MIN_RPM_FRACTION * targetRpm;
    }

    @Override
    public void end(boolean interrupted) {
        resetPreparation();
        stopAll();
        // Segurou o botão e a centrífuga não girou: a Driver Station diz qual trava segurou.
        // Se a sequência pediu e a centrífuga recusou, diz o estado dela.
        if (!centrifugeRanThisPress && shooter.now() - pressedAtSeconds >= NO_FEED_WARNING_SECONDS) {
            String why = feedRequests > 0
                ? "a sequência pediu " + feedRequests + " vez(es) e a centrífuga recusou (estado "
                    + centrifugeRefusal + ", configuração " + centrifuga.getConfigurationStatus() + ")"
                : "último estado " + lastState + (lastDetail.isEmpty() ? "" : " (" + lastDetail + ")");
            DriverStation.reportWarning("[FRC9460] Tiro sem ligar a centrífuga: " + why, false);
        }
        logState(interrupted ? "PARADO" : "FINALIZADO", false, false, false, false, false, false, false);
    }

    /** Último estado da sequência antes de soltar o botão (aviso na Driver Station e testes). */
    public String getLastState() {
        return lastState;
    }

    /** A centrífuga aceitou girar em algum momento deste aperto. */
    public boolean centrifugeRanThisPress() {
        return centrifugeRanThisPress;
    }

    /** Quantas vezes a sequência pediu para alimentar neste aperto. */
    public int feedRequests() {
        return feedRequests;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // #endregion

    // #region AUXILIARES

    private boolean detailDue(double now, String state) {
        if (!state.equals(lastState) || now >= nextDetailSeconds) {
            nextDetailSeconds = now + 0.25;
            return true;
        }
        return false;
    }

    private void idle(String state) {
        resetPreparation();
        stopAll();
        if (!state.equals(lastState)) {
            lastDetail = "";
        }
        logState(state, false, false, false, false, false, false, false);
    }

    private void stopAll() {
        shooter.stopAll();
        centrifuga.stop();
    }

    private void resetPreparation() {
        preparationStartSeconds = Double.NaN;
        feedGraceDeadlineSeconds = Double.NEGATIVE_INFINITY;
        readySinceSeconds = Double.NaN;
    }

    private void logState(String state, boolean ready, boolean feeding, boolean topReady,
            boolean towerReady, boolean reachable, boolean pivotReady, boolean readyOnlyByTime) {
        lastState = state;
        Logger.recordOutput("Shooter/Sequencia/Estado", state);
        Logger.recordOutput("Shooter/Sequencia/Pronto", ready);
        Logger.recordOutput("Shooter/Sequencia/Alimentando", feeding);
        Logger.recordOutput("Shooter/Sequencia/TopPronto", topReady);
        Logger.recordOutput("Shooter/Sequencia/TowerPronto", towerReady);
        Logger.recordOutput("Shooter/Sequencia/PivoPronto", pivotReady);
        Logger.recordOutput("Shooter/Sequencia/HubNoCurso", reachable);
        Logger.recordOutput("Shooter/Sequencia/ProntoPorTempo", readyOnlyByTime);
    }

    // #endregion
}
