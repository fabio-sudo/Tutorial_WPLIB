package frc.robot.subsystems.shooter;

// #region IMPORTS

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

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
 * 4. liga top e tower;
 * 5. só liga a centrífuga (que empurra a bola) quando top, tower e pivô
 *    estiverem prontos e o hub couber no curso de ±90°;
 * 6. soltou o botão: para tudo.
 *
 * Diferença para a competição: com o hub fora do curso o código antigo
 * atirava com o pivô no limite (direção errada). Aqui ele espera o robô girar.
 */
public class ShooterTrackAndShootCommand extends Command {

    // #region ESTADO DA SEQUÊNCIA

    private final Shooter shooter;

    // Instante em que a preparação começou (NaN = parada).
    private double preparationStartSeconds = Double.NaN;

    // A centrífuga continua ligada até este instante, mesmo que um
    // indicador oscile por um ciclo.
    private double feedGraceDeadlineSeconds = Double.NEGATIVE_INFINITY;

    // #endregion

    public ShooterTrackAndShootCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        setName("Shooter - mirar e atirar");
    }

    // #region CICLO DO COMANDO

    @Override
    public void initialize() {
        resetPreparation();
        shooter.stopAll();
        logState("INICIANDO", false, false, false, false, false, false, false);
    }

    @Override
    public void execute() {
        double now = shooter.now();

        // 1. Sem correção da visão, a pose é só o chute inicial da odometria.
        if (!shooter.isPoseTrusted()) {
            idle("AGUARDANDO_LIMELIGHT_CORRIGIR_POSE");
            return;
        }

        // 2. Alvo e RPMs. Parado, o alvo virtual é o próprio hub.
        Pose2d pose = shooter.getPose();
        Translation2d hub = shooter.getHubTarget();
        MovingTarget moving = ShotCalculator.compensateForMotion(pose, shooter.getFieldSpeeds(), hub);
        Shot shot = ShotCalculator.forPose(pose, moving.virtualTarget());
        if (!shot.valid()) {
            idle("TRAJETORIA_INVALIDA");
            return;
        }

        // 3. O pivô mira primeiro.
        if (Double.isNaN(preparationStartSeconds)) {
            preparationStartSeconds = now;
        }
        double elapsed = now - preparationStartSeconds;
        Aim aim = shooter.aimAt(moving.virtualTarget());
        if (elapsed < PIVOT_AIM_HEADSTART_SECONDS) {
            shooter.stopWheels();
            logState("PIVO_MIRANDO_PRIMEIRO", false, false, false, false, aim.reachable(), false, false);
            return;
        }

        // 4. Rodas de disparo e alimentador.
        shooter.setShotSpeeds(shot.topRpm(), shot.towerRpm());
        boolean timeFallback = USE_SPINUP_TIME_FALLBACK && elapsed >= SPINUP_FALLBACK_SECONDS;
        boolean topAtSpeed = shooter.isTopAtSpeed();
        boolean towerAtSpeed = shooter.isTowerAtSpeed();
        boolean topReady = topAtSpeed || timeFallback;
        boolean towerReady = towerAtSpeed || timeFallback;
        boolean pivotReady = shooter.isPivotAtTarget();
        boolean reachable = aim.reachable();

        // 5. Centrífuga só com tudo pronto (mais a margem de um ciclo).
        boolean ready = topReady && towerReady && pivotReady && reachable;
        if (ready) {
            feedGraceDeadlineSeconds = now + CENTRIFUGE_READY_GRACE_SECONDS;
        }
        boolean feeding = ready || now <= feedGraceDeadlineSeconds;
        if (feeding) {
            shooter.setCentrifugeRpm(shot.centrifugeRpm());
        } else {
            shooter.stopCentrifuge();
        }

        boolean readyOnlyByTime = feeding && (!topAtSpeed || !towerAtSpeed);
        String state = feeding ? (readyOnlyByTime ? "DISPARANDO_PRONTO_POR_TEMPO" : "DISPARANDO")
            : !reachable ? "HUB_FORA_DO_CURSO_GIRE_O_ROBO"
            : !pivotReady ? "PIVO_BUSCANDO_ALVO"
            : "ACELERANDO_TOP_E_TOWER";
        logState(state, ready, feeding, topReady, towerReady, reachable, pivotReady, readyOnlyByTime);
        Logger.recordOutput("Shooter/Sequencia/TempoPreparacaoSegundos", elapsed);
        Logger.recordOutput("Shooter/Sequencia/DistanciaMetros", shot.distanceMeters());
    }

    @Override
    public void end(boolean interrupted) {
        resetPreparation();
        shooter.stopAll();
        logState(interrupted ? "PARADO" : "FINALIZADO", false, false, false, false, false, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // #endregion

    // #region AUXILIARES

    private void idle(String state) {
        resetPreparation();
        shooter.stopAll();
        logState(state, false, false, false, false, false, false, false);
    }

    private void resetPreparation() {
        preparationStartSeconds = Double.NaN;
        feedGraceDeadlineSeconds = Double.NEGATIVE_INFINITY;
    }

    private static void logState(String state, boolean ready, boolean feeding, boolean topReady,
            boolean towerReady, boolean reachable, boolean pivotReady, boolean readyOnlyByTime) {
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
