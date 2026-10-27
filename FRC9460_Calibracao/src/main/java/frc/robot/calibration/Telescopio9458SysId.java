package frc.robot.calibration;


// #region IMPORTS

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.telescopio_9458.TelescopioConstants_9458.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import frc.robot.subsystems.telescopio_9458.Telescopio_9458;

// #endregion



/**
 * SysId do telescópio 9458.
 *
 * Mede kS, kG, kV e kA em volts e METROS DA PONTA, as mesmas unidades de
 * REAL_KS/KG/KV/KA em TelescopioConstants_9458. Forward = subir (abrir).
 *
 * O telescópio sempre começa fechado; cada botão sobe, pausa e desce,
 * terminando fechado de novo:
 *
 *   R1 (segurar): quasistático subindo + quasistático descendo
 *   L1 (segurar): dinâmico subindo     + dinâmico descendo
 *
 * No SysId, abrir o .wpilog do AdvantageKit (pendrive), análise "Elevator", e escolher:
 *
 *   estado do teste: /RealOutputs/Telescopio9458/SysIdTestState
 *   posição:         /Telescopio9458/Inputs/ExtensionMeters
 *   velocidade:      /Telescopio9458/Inputs/VelocityMetersPerSec
 *   tensão:          /Telescopio9458/Inputs/AppliedVolts
 *
 * Os mesmos dados vão também para o log da CTRE (.hoot), que o roboRIO grava
 * sem pendrive; exportado pelo Phoenix Tuner X para .wpilog, usar:
 *
 *   estado do teste: Telescopio9458/SysIdTestState
 *   posição:         Telescopio9458/PosicaoMetros
 *   velocidade:      Telescopio9458/VelocidadeMetrosPorSeg
 *   tensão:          Telescopio9458/TensaoVolts
 *
 * Procedimento: config/advantagescope/docs/telescopio_9458/SYSID.md
 */
public final class Telescopio9458SysId {


    // #region ATRIBUTOS

    private final Telescopio_9458 telescopio;

    // Rotina oficial de SysId da WPILib
    private final SysIdRoutine sysIdRoutine;

    // Também grava no log da CTRE (.hoot). Desligado nos testes automáticos.
    private final boolean writeToSignalLogger;

    // #endregion



    // #region CONSTRUTOR

    public Telescopio9458SysId(Telescopio_9458 telescopio) {
        this(telescopio, true);
    }

    Telescopio9458SysId(Telescopio_9458 telescopio, boolean writeToSignalLogger) {

        this.telescopio = telescopio;
        this.writeToSignalLogger = writeToSignalLogger;

        sysIdRoutine =
            new SysIdRoutine(

                // Rampa 0,5 V/s, degrau 2 V e tempo máximo de 10 s por teste.
                new SysIdRoutine.Config(
                    Volts.of(SYSID_RAMP_VOLTS_PER_SEC).per(Second),
                    Volts.of(SYSID_STEP_VOLTS),
                    Seconds.of(SYSID_TIMEOUT_SECONDS),

                    // Separado do "SysIdTestState" do swerve,
                    // para os dois não se misturarem no mesmo log.
                    state -> {
                        Logger.recordOutput(
                            "Telescopio9458/SysIdTestState",
                            state.toString()
                        );
                        if (writeToSignalLogger) {
                            SignalLogger.writeString(
                                "Telescopio9458/SysIdTestState",
                                state.toString()
                            );
                        }
                    }
                ),

                new SysIdRoutine.Mechanism(

                    // Tensão pura no motor, sem PID. O subsistema continua
                    // cortando por segurança, por travamento e nas paradas do curso.
                    voltage -> telescopio.runCharacterization(
                        voltage.in(Volts)
                    ),

                    // O AdvantageKit já grava posição, velocidade e tensão em
                    // Telescopio9458/Inputs. Aqui vão as mesmas medidas para o
                    // .hoot, em metros da ponta, a cada ciclo do teste.
                    log -> {
                        if (writeToSignalLogger) {
                            SignalLogger.writeDouble(
                                "Telescopio9458/PosicaoMetros",
                                telescopio.getExtensionMeters(), "m");
                            SignalLogger.writeDouble(
                                "Telescopio9458/VelocidadeMetrosPorSeg",
                                telescopio.getVelocityMetersPerSecond(), "m/s");
                            SignalLogger.writeDouble(
                                "Telescopio9458/TensaoVolts",
                                telescopio.getAppliedVolts(), "V");
                        }
                    },

                    telescopio,

                    "Telescopio9458"
                )
            );
    }

    // #endregion



    // #region BOTÕES (SEGURAR)

    /** R1: rampa lenta subindo, pausa em cima e rampa lenta descendo. */
    public Command quasistaticUpAndDown() {
        return upAndDown(
            safeTest(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                SysIdRoutine.Direction.kForward,
                "QUASISTATICO SUBINDO"
            ),
            safeTest(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                SysIdRoutine.Direction.kReverse,
                "QUASISTATICO DESCENDO"
            ),
            "QUASISTATICO"
        );
    }

    /** L1: degrau de 2 V subindo, pausa em cima e degrau descendo. */
    public Command dynamicUpAndDown() {
        return upAndDown(
            safeTest(
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                SysIdRoutine.Direction.kForward,
                "DINAMICO SUBINDO"
            ),
            safeTest(
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
                SysIdRoutine.Direction.kReverse,
                "DINAMICO DESCENDO"
            ),
            "DINAMICO"
        );
    }

    /**
     * Sobe, pausa em 0 V e desce. Cada aperto é uma nova tentativa depois de
     * um travamento; se travar, o resto da sequência não roda.
     */
    private Command upAndDown(Command up, Command down, String name) {

        return Commands.sequence(
                up,
                Commands.waitSeconds(SYSID_PAUSE_SECONDS),
                down
            )
            .until(telescopio::isCharacterizationStalled)
            .beforeStarting(() -> {
                telescopio.clearCharacterizationStall();
                // Garante o .hoot gravando (no roboRIO ele já inicia sozinho).
                if (writeToSignalLogger) SignalLogger.start();
            })
            .withName("Telescopio 9458 - SysId " + name + " (sobe e desce)");
    }

    // #endregion



    // #region SEGURANÇA

    /**
     * Termina o teste perto da ponta do curso ou se travar e, ao sair por
     * qualquer motivo (soltou o botão, parada, travamento ou tempo), volta a 0 V.
     */
    private Command safeTest(
        Command test,
        SysIdRoutine.Direction direction,
        String testName
    ) {

        BooleanSupplier endOfTravel =
            direction == SysIdRoutine.Direction.kForward
                ? () -> telescopio.getExtensionMeters() >= SYSID_UPPER_STOP_METERS
                : () -> telescopio.getExtensionMeters() <= SYSID_LOWER_STOP_METERS;

        return test
            .until(() -> endOfTravel.getAsBoolean() || telescopio.isCharacterizationStalled())
            .beforeStarting(() -> {
                Logger.recordOutput("Telescopio9458/SysId/Teste", testName);
                Logger.recordOutput("Telescopio9458/SysId/Resultado", "EM_ANDAMENTO");
            })
            .finallyDo(interrupted -> {

                telescopio.stop();

                Logger.recordOutput(
                    "Telescopio9458/SysId/Resultado",
                    telescopio.isCharacterizationStalled()
                        ? "TRAVOU"
                        : interrupted
                            ? "CANCELADO"
                            : endOfTravel.getAsBoolean()
                                ? "CHEGOU_NA_PARADA"
                                : "TEMPO_ESGOTADO"
                );
            });
    }

    // #endregion
}
