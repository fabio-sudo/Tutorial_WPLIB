package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

import frc.robot.subsystems.centrifuga.CentrifugaIOTalonFX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.ShooterPivotIOTalonFX;
import frc.robot.subsystems.telescopio.TelescopioIOTalonFX;
import frc.robot.util.PhoenixUtil;

/**
 * optimizeBusUtilization desacelera todo sinal sem frequência pedida. O risco é
 * um sinal que o código usa cair junto para 4 Hz: o mecanismo pareceria
 * desconectado ou o seguidor da intake perderia o líder.
 *
 * Confere no código-fonte de cada IO, e não no simulador do Phoenix: ali todo
 * sinal do dispositivo informa a maior frequência pedida (até a temperatura, que
 * ninguém pediu, aparece a 50 Hz), então um sinal esquecido passaria sem aviso.
 */
class CanSignalRatesTest {

    private static final Path SUBSYSTEMS = Path.of("src/main/java/frc/robot/subsystems");
    private static final List<String> REAL_IOS = List.of(
        "intake/IntakeIOTalonFX.java",
        "shooter/ShooterPivotIOTalonFX.java",
        "centrifuga/CentrifugaIOTalonFX.java",
        "telescopio/TelescopioIOTalonFX.java",
        "drive/ModuleIOTalonFX.java",
        "drive/GyroIOPigeon2.java");

    // Ex.: position = motor.getPosition(); getValue... é o uso, não a criação do sinal.
    private static final Pattern READ_SIGNAL = Pattern.compile("(\\w+)\\s*=\\s*\\w+\\.get(?!Value)[A-Z]\\w*\\(\\)\\s*;");
    private static final Pattern RATE_CALL = Pattern.compile("setUpdateFrequencyForAll\\(([^)]*)\\)");
    private static final Pattern IDENTIFIER = Pattern.compile("[A-Za-z_]\\w*");
    private static final Pattern CHAINED_READ =
        Pattern.compile("\\.get[A-Z]\\w*\\(\\)\\s*\\.(getValue|getValueAsDouble|refresh)\\(");

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    private static String source(String file) throws Exception {
        return Files.readString(SUBSYSTEMS.resolve(file));
    }

    /** Sinais que o IO guarda para ler e não têm frequência pedida. */
    static Set<String> signalsWithoutRate(String source) {
        Set<String> rated = ratedSignals(source);
        Set<String> missing = new TreeSet<>();
        Matcher read = READ_SIGNAL.matcher(source);
        while (read.find()) {
            if (!rated.contains(read.group(1))) missing.add(read.group(1));
        }
        return missing;
    }

    private static Set<String> ratedSignals(String source) {
        Set<String> rated = new TreeSet<>();
        Matcher call = RATE_CALL.matcher(source);
        while (call.find()) {
            Matcher name = IDENTIFIER.matcher(call.group(1));
            while (name.find()) rated.add(name.group());
        }
        return rated;
    }

    @Test void theCheckFindsAForgottenSignal() {
        // Prova do próprio teste: sem isto, ele poderia passar sem conferir nada.
        String forgotten = """
            position = motor.getPosition();
            reference = motor.getClosedLoopReference();
            applySignalRates("x", () -> BaseStatusSignal.setUpdateFrequencyForAll(HZ, position), motor);
            """;
        assertEquals(Set.of("reference"), signalsWithoutRate(forgotten));
    }

    @Test void everySignalARealIoReadsHasARequestedRate() throws Exception {
        for (String file : REAL_IOS) {
            String source = source(file);
            assertTrue(READ_SIGNAL.matcher(source).find(), file + ": nenhum sinal encontrado");
            assertEquals(Set.of(), signalsWithoutRate(source), file);
        }
    }

    @Test void noRealIoReadsASignalOutsideTheRatedList() throws Exception {
        // Leitura direta (motor.getDeviceTemp().getValue()) escaparia da conferência acima.
        for (String file : REAL_IOS) {
            assertFalse(CHAINED_READ.matcher(source(file)).find(), file);
        }
    }

    @Test void theIntakeLeaderKeepsWhatTheFollowerCopies() throws Exception {
        // PositionVoltage copia MotorVoltage; DutyCycleOut copia DutyCycle.
        String source = source("intake/IntakeIOTalonFX.java");
        assertTrue(source.contains("voltage = leader.getMotorVoltage()"));
        assertTrue(source.contains("dutyCycle = leader.getDutyCycle()"));
        assertTrue(ratedSignals(source).containsAll(Set.of("voltage", "dutyCycle")));
    }

    @Test void theCancoderIsNeverSlowedDown() throws Exception {
        // O steer usa FusedCANcoder: a posição do CANcoder chega a ele pelo CAN.
        assertTrue(source("drive/ModuleIOTalonFX.java").contains("}, driveMotor, steerMotor);"));
    }

    @Test void everySwerveConfigurationIsRetriedAndChecked() throws Exception {
        // Com o CAN cheio na partida, uma configuração pode expirar. Sem tentar de novo
        // e sem olhar o resultado, a falha passava sem nenhum aviso.
        String source = source("drive/ModuleIOTalonFX.java");
        assertEquals(3, Pattern.compile("getConfigurator\\(\\)").matcher(source).results().count());
        assertEquals(3, Pattern.compile("reportKeptRunning\\(\\s*PhoenixUtil\\.tryUntilOk\\(\\(\\) -> \\w+\\.getConfigurator")
            .matcher(source).results().count());
    }

    @Test void theSwerveModuleChecksItsThreeDevicesWithoutThrowing() {
        new ModuleIOTalonFX(56, 57, 58, 0.0, false, false, false).updateInputs(new ModuleIO.ModuleIOInputs());
    }

    @Test void theIntakeChecksItsFollowerWithoutThrowing() {
        new IntakeIOTalonFX().updateInputs(new IntakeIO.IntakeIOInputs());
    }

    @Test void unusedSignalsAreSlowedAndNeverTurnedOff() {
        // A 0 Hz o hasResetOccurred pararia, e é ele que invalida o zero dos mecanismos.
        assertTrue(PhoenixUtil.OPTIMIZE_CAN_BUS);
        assertEquals(4.0, PhoenixUtil.UNUSED_SIGNAL_HZ);
    }

    @Test void everyRealIoStartsWithTheBusOptimized() {
        // Cria os IOs de verdade: a chamada aos dispositivos não pode lançar nada.
        new IntakeIOTalonFX();
        new ShooterPivotIOTalonFX();
        new CentrifugaIOTalonFX();
        new TelescopioIOTalonFX(55);
        new ModuleIOTalonFX(51, 52, 53, 0.0, false, false, false);
        new GyroIOPigeon2(54);
    }
}
