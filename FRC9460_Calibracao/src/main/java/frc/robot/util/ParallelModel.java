package frc.robot.util;

// #region IMPORTS

import java.util.Objects;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;

// #endregion

/**
 * Modelo em paralelo (REAL x SIMULADO), o mesmo conceito do telescópio 9458.
 *
 * A cada ciclo, a física da simulação recebe a tensão que o motor REALMENTE
 * aplicou e prevê posição, velocidade e corrente. O log mostra lado a lado o
 * que o mecanismo fez e o que a física esperava:
 * - diferença pequena: o modelo está bom, a simulação vale para treinar;
 * - diferença que cresce devagar: atrito, inércia ou redução diferentes;
 * - salto de diferença: batida, travamento, bola presa ou zero errado.
 *
 * Na simulação o "real" é o XIOSim, com os efeitos que o modelo não tem
 * (carga da bola, erro do zero manual, batente da fiação): a diferença mostra
 * exatamente esses efeitos.
 *
 * Regras (iguais às do 9458):
 * - desabilitado ou leitura inválida: o modelo é realinhado ao real;
 * - habilitado: o modelo anda sozinho, só com a tensão medida;
 * - replay: não roda (o log original já traz as saídas do modelo).
 * Mecanismos de posição (pivô, intake, telescópio) também se realinham quando
 * real E modelo estão parados: o modelo não tem o PID que segura o real, e
 * escorregaria devagar durante a partida. Assim cada movimento é comparado do
 * início, e o erro no fim fica em DiferencaAoParar. Travamento continua
 * aparecendo: o real para, o modelo segue andando e a diferença cresce.
 *
 * Tensão: a medida no ciclo corresponde ao intervalo anterior, como a
 * posição. Os XIOSim informam a média do ciclo; no robô real é a amostra do
 * sinal de tensão do motor, boa enquanto a tensão não muda muito em 20 ms.
 *
 * Nomes no log (prefixo = nome do mecanismo, ex.: "Centrifuga"):
 * - prefixo/Modelo/Valido, ComparacaoAtiva, FonteTensao, TensaoVolts;
 * - prefixo/Comparacao/Real*, Simulado*, Diferenca* (diferença = real - simulado);
 * - prefixo/Comparacao/DiferencaMaxima*, Tolerancia*, Divergindo;
 * - prefixo/Comparacao/DiferencaAoParar*, MovimentosComparados (posição).
 */
public final class ParallelModel {

    // #region 📊 UNIDADES NO LOG

    /** Nome no log e conversão da unidade do IO: (valor - zero) × escala. */
    public record Unit(String name, double scale, double zero) {

        public static Unit of(String name, double scale) {
            return new Unit(name, scale, 0.0);
        }

        double convert(double value) {
            return (value - zero) * scale;
        }
    }

    // #endregion

    // #region CONFIGURAÇÃO

    /** Abaixo disto a saída do motor é neutra (Coast ou Brake, conforme o mecanismo). */
    public static final double NEUTRAL_VOLTS = 0.05;

    // A diferença precisa passar da tolerância por este tempo para contar:
    // a queda de RPM de uma bola ou o tranco de uma partida não contam.
    private static final double DIVERGENCE_SECONDS = 0.5;

    private final String prefix;
    private final MechanismModel model;
    private final Unit position;
    private final Unit velocity;
    private final double tolerance;
    private final double restVelocity;
    private final Debouncer divergence = new Debouncer(DIVERGENCE_SECONDS, Debouncer.DebounceType.kRising);

    // #endregion

    // #region ESTADO

    private boolean ready;
    private boolean comparing;
    private boolean diverging;
    private double modelVolts;
    private double measuredPosition;
    private double measuredVelocity;
    private double measuredCurrentAmps;
    private double modeledPosition = Double.NaN;
    private double modeledVelocity = Double.NaN;
    private double modeledCurrentAmps = Double.NaN;
    private double maxDifference;
    private boolean moving;
    private double stopDifference;
    private int comparedMovements;

    // #endregion

    // #region CONSTRUÇÃO

    /**
     * @param prefix    nome do mecanismo no log (ex.: "Shooter/Pivo")
     * @param factory   cria a física (o XIOSim do mecanismo); não roda no replay
     * @param position  unidade da posição; null para rodas e rolos (só velocidade)
     * @param velocity  unidade da velocidade
     * @param tolerance diferença aceitável na grandeza principal (posição, ou
     *                  velocidade quando não há posição), já na unidade do log
     */
    public ParallelModel(String prefix, Supplier<MechanismModel> factory,
                         Unit position, Unit velocity, double tolerance) {
        this(prefix, factory, position, velocity, tolerance, 0.0);
    }

    /**
     * @param restVelocity abaixo desta velocidade (unidade do log) o mecanismo
     *                     está parado; real e modelo parados = realinha.
     *                     0 = nunca (rodas e rolos, que se estabilizam sozinhos)
     */
    public ParallelModel(String prefix, Supplier<MechanismModel> factory,
                         Unit position, Unit velocity, double tolerance, double restVelocity) {
        this.prefix = Objects.requireNonNull(prefix);
        this.velocity = Objects.requireNonNull(velocity);
        this.position = position;
        this.tolerance = tolerance;
        this.restVelocity = restVelocity;
        this.model = Logger.hasReplaySource() ? null : Objects.requireNonNull(factory.get());
    }

    // #endregion

    // #region 🧪 PASSO DA FÍSICA (CHAMAR LOGO DEPOIS DE LER OS SENSORES)

    /**
     * @param measurementValid motor respondendo e, se houver, zero válido
     * @param volts            tensão medida no motor (média dos motores juntos)
     * @param currentAmps      corrente de um motor (média dos motores juntos)
     */
    public void update(boolean measurementValid, double position, double velocity,
                       double volts, double currentAmps) {
        measuredPosition = position;
        measuredVelocity = velocity;
        measuredCurrentAmps = currentAmps;
        comparing = false;
        modelVolts = 0.0;

        if (model == null) {
            return;
        }
        if (!measurementValid || !Double.isFinite(volts)
            || !Double.isFinite(position) || !Double.isFinite(velocity)) {
            // Leitura inválida não pode contaminar o modelo.
            ready = false;
            diverging = divergence.calculate(false);
            return;
        } else if (DriverStation.isDisabled() || !ready) {
            // Desabilitado (ou recém-recuperado): alinha o modelo ao real.
            model.setModelState(position, velocity);
            ready = true;
            moving = false;
            maxDifference = 0.0;
        } else {
            model.stepModel(volts);
            modelVolts = volts;
            comparing = true;
        }

        // Previsão deste ciclo (antes de um eventual realinhamento).
        modeledPosition = model.modelPosition();
        modeledVelocity = model.modelVelocity();
        modeledCurrentAmps = model.modelCurrentAmps();
        double difference = comparing ? primaryDifference() : 0.0;
        maxDifference = Math.max(maxDifference, Math.abs(difference));
        diverging = divergence.calculate(comparing && Math.abs(difference) > tolerance);
        if (comparing) {
            realignWhenBothStopped(position, velocity, difference);
        }
    }

    /** Fim de um movimento: guarda o erro e recomeça do real. */
    private void realignWhenBothStopped(double position, double velocity, double difference) {
        if (restVelocity <= 0.0) {
            return;
        }
        boolean bothStopped = Math.abs(this.velocity.convert(velocity)) <= restVelocity
            && Math.abs(this.velocity.convert(modeledVelocity)) <= restVelocity;
        if (!bothStopped) {
            moving = true;
            return;
        }
        if (moving) {
            stopDifference = difference;
            comparedMovements++;
            moving = false;
        }
        model.setModelState(position, velocity);
    }

    // #endregion

    // #region CONSULTAS

    /** Habilitado e comparando: o modelo anda com a tensão medida. */
    public boolean isComparing() {
        return comparing;
    }

    /** Posição na unidade publicada (graus/cm); NaN quando não há modelo válido. */
    public double getModeledPositionConverted() {
        return model != null && ready && position != null
            ? position.convert(modeledPosition) : Double.NaN;
    }

    /** Real e modelo diferentes além da tolerância por mais de 0,5 s. */
    public boolean isDiverging() {
        return diverging;
    }

    private Unit primaryUnit() {
        return position != null ? position : velocity;
    }

    private double primaryDifference() {
        return position != null
            ? position.convert(measuredPosition) - position.convert(modeledPosition)
            : velocity.convert(measuredVelocity) - velocity.convert(modeledVelocity);
    }

    // #endregion

    // #region 📊 TELEMETRIA (NOMES USADOS PELOS LAYOUTS)

    public void log() {
        String m = prefix + "/Modelo/";
        String c = prefix + "/Comparacao/";
        Logger.recordOutput(m + "Valido", model != null && ready);
        Logger.recordOutput(m + "ComparacaoAtiva", comparing);
        Logger.recordOutput(m + "FonteTensao", model == null ? "SEM_MODELO_NO_REPLAY" : "TENSAO_MEDIDA");
        Logger.recordOutput(m + "TensaoVolts", modelVolts);

        if (position != null) {
            logTriple(c, position, measuredPosition, modeledPosition);
        }
        logTriple(c, velocity, measuredVelocity, modeledVelocity);
        Logger.recordOutput(c + "RealCorrenteAmps", measuredCurrentAmps);
        if (Double.isFinite(modeledCurrentAmps)) {
            Logger.recordOutput(c + "SimuladoCorrenteAmps", modeledCurrentAmps);
            Logger.recordOutput(c + "DiferencaCorrenteAmps", measuredCurrentAmps - modeledCurrentAmps);
        }

        Unit primary = primaryUnit();
        Logger.recordOutput(c + "DiferencaMaxima" + primary.name(), maxDifference);
        Logger.recordOutput(c + "Tolerancia" + primary.name(), tolerance);
        Logger.recordOutput(c + "Divergindo", diverging);
        if (restVelocity > 0.0) {
            Logger.recordOutput(c + "DiferencaAoParar" + primary.name(), stopDifference);
            Logger.recordOutput(c + "MovimentosComparados", comparedMovements);
        }
    }

    private static void logTriple(String c, Unit unit, double measured, double modeled) {
        double real = unit.convert(measured);
        Logger.recordOutput(c + "Real" + unit.name(), real);
        if (Double.isFinite(modeled)) {
            double simulated = unit.convert(modeled);
            Logger.recordOutput(c + "Simulado" + unit.name(), simulated);
            Logger.recordOutput(c + "Diferenca" + unit.name(), real - simulated);
        }
    }

    // #endregion
}
