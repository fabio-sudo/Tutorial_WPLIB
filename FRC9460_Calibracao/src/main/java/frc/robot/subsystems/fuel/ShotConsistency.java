package frc.robot.subsystems.fuel;

// #region IMPORTS

import static frc.robot.subsystems.fuel.FuelConstants.*;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

import org.littletonrobotics.junction.Logger;

// #endregion

/**
 * Constância dos tiros: o quanto cada disparo se parece com o anterior.
 *
 * Cada disparo detectado entra numa janela dos últimos
 * CONSISTENCY_WINDOW_SHOTS tiros. Para cada medida o log guarda o último
 * valor, a média (viés: o robô erra sempre para o mesmo lado?) e o desvio
 * (dispersão: cada tiro sai diferente?). Dois problemas diferentes pedem
 * ajustes diferentes:
 *
 * - viés grande, desvio pequeno: erro sistemático. Ajuste de calibração
 *   (curva de RPM, offset do shooter, heading);
 * - desvio grande: o robô não repete. Mecânica, alimentação, bateria,
 *   pivô que não assenta, pose ruidosa. Mexer na curva não resolve.
 *
 * É só conta sobre sinais já gravados: dá o mesmo resultado no robô real, na
 * simulação e no replay. No replay, ReplayOutputs/Fuel/Consistencia mostra o
 * que o código ATUAL diria dos mesmos tiros.
 *
 * Nomes no log: Fuel/Consistencia/&lt;Medida&gt;/{Ultimo,Media,Desvio,Minimo,Maximo}.
 */
public final class ShotConsistency {

    // #region UM DISPARO

    /**
     * @param number            número do disparo detectado
     * @param distanceMeters    do shooter ao hub
     * @param topRpmBeforeBall  RPM do top ~60 ms antes da bola (a bola já tirou energia da roda na detecção)
     * @param topTargetRpm      RPM do top que o código pediu
     * @param rpmDrop           queda de RPM do top na detecção
     * @param exitSpeedMps      velocidade de saída da bola, pelos sensores
     * @param aimErrorDegrees   direção do alvo menos direção do shooter (0 = apontando certo)
     * @param missMeters        onde a bola cai, a partir do centro do hub, pelos sensores; negativo = não chega à altura do hub
     * @param robotSpeedMps     velocidade do robô no disparo
     * @param intervalSeconds   desde o disparo anterior; NaN no primeiro
     * @param plannedHit        o modelo diz que o tiro PLANEJADO acerta
     * @param sensorsHit        o modelo diz que o tiro pelos SENSORES acerta
     */
    public record Shot(int number, double distanceMeters, double topRpmBeforeBall, double topTargetRpm,
            double rpmDrop, double exitSpeedMps, double aimErrorDegrees, double missMeters,
            double robotSpeedMps, double intervalSeconds, boolean plannedHit, boolean sensorsHit) {}

    // #endregion

    // #region ESTADO

    private final RollingStats exitSpeed = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final RollingStats topRpm = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final RollingStats topRpmError = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final RollingStats rpmDrop = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final RollingStats aimError = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final RollingStats miss = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final RollingStats distance = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final RollingStats robotSpeed = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final RollingStats interval = new RollingStats(CONSISTENCY_WINDOW_SHOTS);
    private final Deque<Boolean> plannedHits = new ArrayDeque<>();
    private final Deque<Boolean> sensorHits = new ArrayDeque<>();
    private int shotsRecorded;
    private String diagnosis = diagnose();

    // #endregion

    // #region 📊 REGISTRO DE UM DISPARO

    /** Chamado no ciclo em que o disparo foi detectado. */
    public void record(Shot shot) {
        shotsRecorded++;
        exitSpeed.add(shot.exitSpeedMps());
        topRpm.add(shot.topRpmBeforeBall());
        topRpmError.add(shot.topTargetRpm() - shot.topRpmBeforeBall());
        rpmDrop.add(shot.rpmDrop());
        aimError.add(shot.aimErrorDegrees());
        // Bola que nem chega à altura do hub (distância negativa) é um erro
        // grave, mas não tem "distância do centro": fica de fora da média.
        miss.add(shot.missMeters() >= 0.0 ? shot.missMeters() : Double.NaN);
        distance.add(shot.distanceMeters());
        robotSpeed.add(shot.robotSpeedMps());
        interval.add(shot.intervalSeconds());
        push(plannedHits, shot.plannedHit());
        push(sensorHits, shot.sensorsHit());
        diagnosis = diagnose();
        log();
    }

    private static void push(Deque<Boolean> window, boolean value) {
        window.addLast(value);
        if (window.size() > CONSISTENCY_WINDOW_SHOTS) {
            window.removeFirst();
        }
    }

    private static double rate(Deque<Boolean> window) {
        if (window.isEmpty()) {
            return Double.NaN;
        }
        return window.stream().filter(Boolean::booleanValue).count() / (double) window.size();
    }

    // #endregion

    // #region 📊 DIAGNÓSTICO EM TEXTO

    /**
     * Uma linha dizendo o que está inconstante, para o painel ao vivo.
     * Vazio de problemas = CONSTANTE. Limites em FuelConstants (valores iniciais).
     */
    private String diagnose() {
        int shots = Math.min(shotsRecorded, CONSISTENCY_WINDOW_SHOTS);
        if (shots < CONSISTENCY_MIN_SHOTS) {
            return "POUCOS_TIROS_" + shots + "_DE_" + CONSISTENCY_MIN_SHOTS;
        }
        List<String> problems = new ArrayList<>();
        double speedCv = exitSpeed.stdDev() / exitSpeed.mean();
        if (speedCv > CONSISTENCY_SPEED_MAX_CV) {
            problems.add("VELOCIDADE_INCONSTANTE");
        }
        // Pedido - medido: positivo = a roda não chegou; negativo = passou (no
        // simulador o Vortex roda ~7% acima). Os dois lados mudam onde a bola cai.
        if (topRpmError.mean() > CONSISTENCY_TOP_RPM_MAX_MEAN_ERROR) {
            problems.add("TOP_ABAIXO_DO_PEDIDO");
        } else if (topRpmError.mean() < -CONSISTENCY_TOP_RPM_MAX_MEAN_ERROR) {
            problems.add("TOP_ACIMA_DO_PEDIDO");
        }
        if (aimError.stdDev() > CONSISTENCY_AIM_MAX_SIGMA_DEGREES) {
            problems.add("MIRA_INCONSTANTE");
        }
        if (Math.abs(aimError.mean()) > CONSISTENCY_AIM_MAX_BIAS_DEGREES) {
            problems.add("MIRA_COM_VIES");
        }
        if (miss.mean() > CONSISTENCY_MISS_MAX_MEAN_FRACTION_OF_RADIUS * HUB_OPENING_RADIUS_METERS) {
            problems.add("ERRO_DE_QUEDA_ALTO");
        }
        return problems.isEmpty() ? "CONSTANTE" : String.join("+", problems);
    }

    // #endregion

    // #region 📊 LOG

    private void log() {
        logStats("VelocidadeSaidaMps", exitSpeed);
        logStats("TopRpmNoDisparo", topRpm);
        logStats("TopRpmErro", topRpmError);
        logStats("QuedaRpmTop", rpmDrop);
        logStats("ErroMiraGraus", aimError);
        logStats("ErroQuedaMetros", miss);
        logStats("DistanciaMetros", distance);
        logStats("VelocidadeRoboMps", robotSpeed);
        logStats("IntervaloSegundos", interval);
        Logger.recordOutput("Fuel/Consistencia/TirosNaJanela", Math.min(shotsRecorded, CONSISTENCY_WINDOW_SHOTS));
        Logger.recordOutput("Fuel/Consistencia/TaxaAcertoPlanejado", rate(plannedHits));
        Logger.recordOutput("Fuel/Consistencia/TaxaAcertoSensores", rate(sensorHits));
        Logger.recordOutput("Fuel/Consistencia/CvVelocidade", exitSpeed.stdDev() / exitSpeed.mean());
        Logger.recordOutput("Fuel/Consistencia/Diagnostico", diagnosis);
    }

    private static void logStats(String name, RollingStats stats) {
        String prefix = "Fuel/Consistencia/" + name + "/";
        Logger.recordOutput(prefix + "Ultimo", stats.last());
        Logger.recordOutput(prefix + "Media", stats.mean());
        Logger.recordOutput(prefix + "Desvio", stats.stdDev());
        Logger.recordOutput(prefix + "Minimo", stats.min());
        Logger.recordOutput(prefix + "Maximo", stats.max());
    }

    // #endregion

    // #region CONSULTAS (TESTES)

    public RollingStats exitSpeed() { return exitSpeed; }
    public RollingStats aimError() { return aimError; }
    public RollingStats miss() { return miss; }
    public RollingStats topRpmError() { return topRpmError; }
    public String diagnosis() { return diagnosis; }
    public int shotsRecorded() { return shotsRecorded; }

    // #endregion
}
