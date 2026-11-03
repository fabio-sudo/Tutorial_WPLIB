package frc.robot.commands.shooter;

import static frc.robot.subsystems.centrifuga.CentrifugaConstants.*;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.centrifuga.Centrifuga;

/**
 * Uma tentativa curta de reverter 1/4 de volta antes do tiro.
 *
 * Não conseguiu (corrente alta, tempo esgotado, não parou, sensor oscilando):
 * para de reverter, avisa e segue para o tiro normal, como o código de
 * competição sem destravamento. A sequência de tiro tem as próprias travas.
 * Antes (até 29/10/2026) qualquer falha bloqueava o tiro até soltar o B; no
 * robô a reversão fraca falhava sempre e a centrífuga nunca empurrava a bola.
 *
 * Só SENTIDO_INCORRETO continua bloqueando: o motor andou para a frente com
 * pedido de reverter, sinal de inversão ou encoder errados.
 */
public class CentrifugaDestravarCommand extends Command {
    private enum Phase { FREANDO_ANTES, REVERSAO, FREANDO_DEPOIS, PRONTO, FALHA }
    private final Centrifuga centrifuga;
    private Phase phase;
    private double phaseStart;
    private double stoppedSince;
    private double overcurrentSince;
    private double startRotations;
    private String fault = "";

    public CentrifugaDestravarCommand(Centrifuga centrifuga) {
        this.centrifuga = centrifuga;
        addRequirements(centrifuga);
        setName("Centrifuga - destravar antes do tiro");
    }

    @Override public void initialize() {
        fault = "";
        startRotations = centrifuga.getOutputRotations();
        overcurrentSince = Double.NaN;
        enter(Phase.FREANDO_ANTES);
        centrifuga.stop();
    }

    @Override public void execute() {
        double now = Timer.getTimestamp(); // segue o relógio do replay
        if (!centrifuga.canUnjam()) fail("SENSOR_OU_HABILITACAO_INVALIDO");
        switch (phase) {
            case FREANDO_ANTES, FREANDO_DEPOIS -> {
                centrifuga.stop();
                if (Math.abs(centrifuga.getRpm()) <= UNJAM_STOPPED_ROTOR_RPM) {
                    if (Double.isNaN(stoppedSince)) stoppedSince = now;
                    if (now - stoppedSince >= UNJAM_STOPPED_SECONDS) {
                        if (phase == Phase.FREANDO_ANTES) {
                            startRotations = centrifuga.getOutputRotations();
                            enter(Phase.REVERSAO);
                        } else enter(Phase.PRONTO);
                    }
                } else stoppedSince = Double.NaN;
                if (phase != Phase.PRONTO && phase != Phase.REVERSAO
                    && now - phaseStart >= UNJAM_BRAKE_TIMEOUT_SECONDS) fail("NAO_PAROU");
            }
            case REVERSAO -> {
                double travel = startRotations - centrifuga.getOutputRotations();
                if (centrifuga.getCurrentAmps() >= UNJAM_CURRENT_AMPS) {
                    if (Double.isNaN(overcurrentSince)) overcurrentSince = now;
                } else overcurrentSince = Double.NaN;
                if (!Double.isNaN(overcurrentSince)
                    && now - overcurrentSince >= UNJAM_OVERCURRENT_SECONDS) fail("CORRENTE_ALTA");
                else if (travel < -0.03) fail("SENTIDO_INCORRETO");
                else if (now - phaseStart >= UNJAM_TIMEOUT_SECONDS) fail("TEMPO_ESGOTADO");
                else if (travel >= UNJAM_OUTPUT_ROTATIONS) {
                    centrifuga.stop();
                    enter(Phase.FREANDO_DEPOIS);
                } else centrifuga.reverseForUnjam();
            }
            case PRONTO, FALHA -> centrifuga.stop();
        }
        Logger.recordOutput("Centrifuga/Destravar/Estado", phase.toString());
        Logger.recordOutput("Centrifuga/Destravar/Falha", fault);
        Logger.recordOutput("Centrifuga/Destravar/BloqueiaTiro", phase == Phase.FALHA && blocksShot());
        Logger.recordOutput("Centrifuga/Destravar/VoltasSaida",
            startRotations - centrifuga.getOutputRotations());
    }

    private void enter(Phase next) {
        phase = next;
        phaseStart = Timer.getTimestamp();
        stoppedSince = Double.NaN;
    }

    private void fail(String reason) {
        if (phase != Phase.FALHA) {
            fault = reason;
            DriverStation.reportWarning("[FRC9460] Centrífuga: destravar falhou (" + reason + "). "
                + (blocksShot() ? "Tiro bloqueado: conferir inversão do motor." : "Seguindo para o tiro."), false);
        }
        phase = Phase.FALHA;
        centrifuga.stop();
    }

    private boolean blocksShot() {
        return "SENTIDO_INCORRETO".equals(fault);
    }

    @Override public boolean isFinished() {
        return phase == Phase.PRONTO || (phase == Phase.FALHA && !blocksShot());
    }
    @Override public void end(boolean interrupted) {
        centrifuga.stop();
        if (interrupted) Logger.recordOutput("Centrifuga/Destravar/Estado", "CANCELADO");
    }
}
