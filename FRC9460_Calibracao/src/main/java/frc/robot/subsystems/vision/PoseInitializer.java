package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Decide QUANDO a pose do robô deve ser definida pelas tags (MegaTag1): X, Y e
 * heading de campo de uma vez. Não conhece robô, câmera nem relógio: recebe o
 * que foi lido e devolve a pose a aplicar, ou nada.
 *
 * Por que existe: o MegaTag2 aceita o heading do Pigeon como verdade. Com o
 * heading errado a pose sai espelhada em volta das tags e continua sendo
 * "corrigida". O MegaTag1 acha o heading pelas próprias tags e é o único que
 * pode desmentir o Pigeon.
 *
 * Regras (limites em VisionConstants):
 * - só vale quando quem chama diz que é seguro (desabilitado e parado);
 * - a leitura precisa ter tags de sobra e estar perto: AUTO_INIT_MIN_TAGS e
 *   AUTO_INIT_MAX_TAG_DISTANCE_METERS, dentro do campo;
 * - AUTO_INIT_STABLE_FRAMES quadros NOVOS seguidos precisam concordar
 *   (dispersão de posição e de heading dentro do limite);
 * - com os quadros estáveis, só aplica se o estimador estiver desalinhado:
 *   heading além de AUTO_INIT_HEADING_TOLERANCE_DEGREES ou posição além de
 *   AUTO_INIT_POSITION_TOLERANCE_METERS. Alinhado, não faz nada, então não há
 *   reset repetido, e depois de aplicar a condição deixa de valer.
 */
public final class PoseInitializer {

    // #region ENTRADA E ESTADOS

    /** Uma leitura MegaTag1 de uma câmera. */
    public record Candidate(Pose2d pose, int tagCount, double avgTagDistanceMeters, double timestampSeconds) {}

    /** O que o inicializador está fazendo, para o log e o painel. */
    public enum State {
        /** Habilitado, em movimento ou desligado: nunca mexe na pose. */
        INELEGIVEL,
        /** Seguro, mas sem leitura boa (sem tags, longe, poucas tags, fora do campo). */
        SEM_LEITURA_BOA,
        /** Juntando quadros novos. */
        JUNTANDO_FRAMES,
        /** Os quadros discordam entre si: câmera tremendo, tags trocando. */
        INSTAVEL,
        /** Quadros estáveis e a pose do estimador já concorda com eles. */
        ALINHADA,
        /** Este ciclo devolveu uma pose para aplicar. */
        APLICADA
    }

    // #endregion

    // #region ESTADO

    private final Deque<Candidate> frames = new ArrayDeque<>();
    private double lastNewFrameSeconds = Double.NEGATIVE_INFINITY;
    private State state = State.INELEGIVEL;
    private double headingErrorDegrees = Double.NaN;
    private double positionErrorMeters = Double.NaN;

    // #endregion

    // #region 🔁 CICLO

    /**
     * @param now        relógio do robô, em segundos
     * @param eligible   é seguro redefinir a pose agora (desabilitado, parado, gyro bom)
     * @param candidate  melhor leitura MegaTag1 do ciclo, ou null
     * @param current    pose atual do estimador
     * @return a pose (X, Y e heading) a aplicar; vazio se nada deve mudar
     */
    public Optional<Pose2d> update(double now, boolean eligible, Candidate candidate, Pose2d current) {
        if (!eligible) {
            frames.clear();
            state = State.INELEGIVEL;
            headingErrorDegrees = Double.NaN;
            positionErrorMeters = Double.NaN;
            return Optional.empty();
        }
        if (!frames.isEmpty() && now - lastNewFrameSeconds > AUTO_INIT_FRAME_TIMEOUT_SECONDS) {
            frames.clear();
        }
        if (candidate != null && isUsable(candidate)) {
            // Só quadro NOVO conta: a Limelight repete o último valor entre quadros.
            if (frames.isEmpty() || candidate.timestampSeconds() > frames.peekLast().timestampSeconds()) {
                frames.addLast(candidate);
                lastNewFrameSeconds = now;
                while (frames.size() > AUTO_INIT_STABLE_FRAMES) {
                    frames.removeFirst();
                }
            }
        } else if (frames.isEmpty()) {
            state = State.SEM_LEITURA_BOA;
            return Optional.empty();
        }
        if (frames.size() < AUTO_INIT_STABLE_FRAMES) {
            state = frames.isEmpty() ? State.SEM_LEITURA_BOA : State.JUNTANDO_FRAMES;
            return Optional.empty();
        }

        Pose2d mean = average(frames);
        if (!agree(frames, mean)) {
            state = State.INSTAVEL;
            return Optional.empty();
        }
        headingErrorDegrees = MathUtil.inputModulus(
            mean.getRotation().minus(current.getRotation()).getDegrees(), -180.0, 180.0);
        positionErrorMeters = mean.getTranslation().getDistance(current.getTranslation());
        if (Math.abs(headingErrorDegrees) <= AUTO_INIT_HEADING_TOLERANCE_DEGREES
                && positionErrorMeters <= AUTO_INIT_POSITION_TOLERANCE_METERS) {
            state = State.ALINHADA;
            return Optional.empty();
        }
        state = State.APLICADA;
        frames.clear();
        return Optional.of(mean);
    }

    // #endregion

    // #region QUALIDADE E CONCORDÂNCIA

    /** A leitura serve para definir ou conferir a pose? (tags, distância, campo, números finitos). */
    public static boolean isUsable(Candidate c) {
        Pose2d pose = c.pose();
        return pose != null
            && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians())
            && c.tagCount() >= AUTO_INIT_MIN_TAGS
            && Double.isFinite(c.avgTagDistanceMeters())
            && c.avgTagDistanceMeters() > 0.0
            && c.avgTagDistanceMeters() <= AUTO_INIT_MAX_TAG_DISTANCE_METERS
            && Double.isFinite(c.timestampSeconds()) && c.timestampSeconds() > 0.0
            && pose.getX() >= 0.0 && pose.getX() <= VisionFieldConstants.FIELD_LENGTH_METERS
            && pose.getY() >= 0.0 && pose.getY() <= VisionFieldConstants.FIELD_WIDTH_METERS;
    }

    /** Média da posição e média CIRCULAR do heading (350° e 10° dão 0°, não 180°). */
    private static Pose2d average(Deque<Candidate> frames) {
        double x = 0.0;
        double y = 0.0;
        double sin = 0.0;
        double cos = 0.0;
        for (Candidate c : frames) {
            x += c.pose().getX();
            y += c.pose().getY();
            sin += c.pose().getRotation().getSin();
            cos += c.pose().getRotation().getCos();
        }
        return new Pose2d(new Translation2d(x / frames.size(), y / frames.size()),
            new Rotation2d(Math.atan2(sin, cos)));
    }

    private static boolean agree(Deque<Candidate> frames, Pose2d mean) {
        for (Candidate c : frames) {
            double distance = c.pose().getTranslation().getDistance(mean.getTranslation());
            double heading = Math.abs(MathUtil.inputModulus(
                c.pose().getRotation().minus(mean.getRotation()).getDegrees(), -180.0, 180.0));
            if (distance > AUTO_INIT_MAX_FRAME_SPREAD_METERS || heading > AUTO_INIT_MAX_FRAME_SPREAD_DEGREES) {
                return false;
            }
        }
        return true;
    }

    // #endregion

    // #region CONSULTAS

    public State state() { return state; }
    public int framesCollected() { return frames.size(); }
    /** Heading das tags menos o do estimador, em graus. NaN = sem quadros estáveis. */
    public double headingErrorDegrees() { return headingErrorDegrees; }
    /** Distância entre a posição das tags e a do estimador. NaN = sem quadros estáveis. */
    public double positionErrorMeters() { return positionErrorMeters; }

    // #endregion
}
