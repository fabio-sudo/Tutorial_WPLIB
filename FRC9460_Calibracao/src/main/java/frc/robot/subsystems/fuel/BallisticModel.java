package frc.robot.subsystems.fuel;

// #region IMPORTS

import static frc.robot.subsystems.fuel.FuelConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// #endregion

/**
 * Física do voo da bola: gravidade + arrasto do ar. Sem estado; a mesma
 * conta serve para a simulação (bolas voando) e para a trajetória prevista
 * que aparece também no robô real.
 */
public final class BallisticModel {

    // #region ESTADO DA BOLA

    /** Posição (m) e velocidade (m/s) no campo. Z = altura do centro da bola. */
    public record State(Translation3d position, Translation3d velocity) {}

    // #endregion

    // #region LANÇAMENTO

    /**
     * Velocidade inicial: rapidez de saída na direção do shooter, inclinada
     * pelo ângulo de lançamento, MAIS a velocidade do robô (a bola herda).
     */
    public static Translation3d launchVelocity(double headingDegrees, double exitSpeed,
            double launchAngleDegrees, ChassisSpeeds robotFieldSpeeds) {
        double heading = Math.toRadians(headingDegrees);
        double elevation = Math.toRadians(launchAngleDegrees);
        double horizontal = exitSpeed * Math.cos(elevation);
        return new Translation3d(
            horizontal * Math.cos(heading) + robotFieldSpeeds.vxMetersPerSecond,
            horizontal * Math.sin(heading) + robotFieldSpeeds.vyMetersPerSecond,
            exitSpeed * Math.sin(elevation));
    }

    // #endregion

    // #region UM PASSO DE TEMPO

    /** Euler semi-implícito: primeiro a velocidade, depois a posição. */
    public static State step(State state, double dtSeconds) {
        Translation3d v = state.velocity();
        double speed = v.getNorm();
        // Arrasto: oposto ao movimento, proporcional à velocidade ao quadrado.
        double drag = DRAG_PER_METER * speed;
        Translation3d next = new Translation3d(
            v.getX() - drag * v.getX() * dtSeconds,
            v.getY() - drag * v.getY() * dtSeconds,
            v.getZ() - (GRAVITY + drag * v.getZ()) * dtSeconds);
        return new State(state.position().plus(next.times(dtSeconds)), next);
    }

    // #endregion

    // #region TRAJETÓRIA COMPLETA

    /** Pontos do voo até a bola tocar o chão (ou o tempo máximo). */
    public static List<Translation3d> predict(State start, double maxSeconds, double stepSeconds) {
        List<Translation3d> points = new ArrayList<>();
        State state = start;
        points.add(state.position());
        for (double t = 0.0; t < maxSeconds; t += stepSeconds) {
            state = step(state, stepSeconds);
            points.add(state.position());
            if (state.position().getZ() <= BALL_RADIUS_METERS && state.velocity().getZ() < 0.0) {
                break;
            }
        }
        return points;
    }

    /**
     * Um ponto a cada {@code outputSeconds} (e o último): basta para desenhar
     * a linha e deixa o log bem menor que gravar todos os passos.
     */
    public static Translation3d[] sample(List<Translation3d> path, double stepSeconds, double outputSeconds) {
        int every = Math.max(1, (int) Math.round(outputSeconds / stepSeconds));
        List<Translation3d> points = new ArrayList<>();
        for (int i = 0; i < path.size(); i += every) {
            points.add(path.get(i));
        }
        if ((path.size() - 1) % every != 0) {
            points.add(path.get(path.size() - 1));
        }
        return points.toArray(Translation3d[]::new);
    }

    /**
     * Onde a trajetória atravessa a altura do alvo DESCENDO (como a bola
     * entra no hub). Vazio se não chega a essa altura.
     */
    public static Optional<Translation2d> descendingCrossing(List<Translation3d> path, double heightMeters) {
        for (int i = 1; i < path.size(); i++) {
            Translation3d a = path.get(i - 1);
            Translation3d b = path.get(i);
            if (a.getZ() >= heightMeters && b.getZ() < heightMeters) {
                double f = (a.getZ() - heightMeters) / (a.getZ() - b.getZ());
                return Optional.of(new Translation2d(
                    a.getX() + f * (b.getX() - a.getX()),
                    a.getY() + f * (b.getY() - a.getY())));
            }
        }
        return Optional.empty();
    }

    // #endregion

    // #region TEMPO DE VOO

    /**
     * Tempo (s) até a bola descer pela altura do alvo: a mesma conta de step(),
     * só com números (double), sem criar objetos. Roda a cada ciclo na mira do
     * tiro andando, também no roboRIO (objetos por passo pausam o programa no
     * coletor de lixo, como no ArmModel). Ignora a velocidade do robô, que muda
     * pouco o arrasto. NaN se a bola não chega a essa altura.
     */
    public static double timeToDescendThrough(double exitSpeed, double launchAngleDegrees,
            double startHeightMeters, double targetHeightMeters, double stepSeconds, double maxSeconds) {
        double elevation = Math.toRadians(launchAngleDegrees);
        double horizontal = exitSpeed * Math.cos(elevation);
        double vertical = exitSpeed * Math.sin(elevation);
        double height = startHeightMeters;
        for (double t = 0.0; t < maxSeconds; t += stepSeconds) {
            double drag = DRAG_PER_METER * Math.hypot(horizontal, vertical);
            horizontal -= drag * horizontal * stepSeconds;
            vertical -= (GRAVITY + drag * vertical) * stepSeconds;
            double next = height + vertical * stepSeconds;
            if (height >= targetHeightMeters && next < targetHeightMeters) {
                return t + stepSeconds * (height - targetHeightMeters) / (height - next);
            }
            height = next;
        }
        return Double.NaN;
    }

    // #endregion

    // #region SEM INSTÂNCIAS

    private BallisticModel() {}

    // #endregion
}
