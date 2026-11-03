package frc.robot.controls;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

class DriverMechanismBindingsTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private XboxControllerSim pad;
    private int intakeDirection;
    private boolean rollerOn, aiming, shooting;

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        scheduler.getActiveButtonLoop().clear();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        var controller = new CommandXboxController(0);
        pad = new XboxControllerSim(controller.getHID());
        var intake = new SubsystemBase() {};
        var roller = new SubsystemBase() {};
        var shooter = new SubsystemBase() {};
        var centrifuge = new SubsystemBase() {};
        DriverMechanismBindings.bind(controller,
            intake.runEnd(() -> intakeDirection = -1, () -> intakeDirection = 0),
            intake.runEnd(() -> intakeDirection = 1, () -> intakeDirection = 0),
            roller.runEnd(() -> rollerOn = true, () -> rollerOn = false),
            shooter.runEnd(() -> aiming = true, () -> aiming = false),
            Commands.runEnd(() -> shooting = true, () -> shooting = false, shooter, centrifuge));
        tick();
    }
    private void tick() { DriverStationSim.notifyNewData(); scheduler.run(); }
    /** Confere no código-fonte: o RobotContainer só nasce com o robô inteiro. */
    private static String robotContainerCode() throws Exception {
        return Files.readString(Path.of("src/main/java/frc/robot/RobotContainer.java")).lines()
            .filter(line -> !line.strip().startsWith("//")).collect(Collectors.joining("\n"));
    }
    @AfterEach void cleanup() {
        scheduler.cancelAll();
        scheduler.getActiveButtonLoop().clear();
        scheduler.unregisterAllSubsystems();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }
    @Test void bumpersSelectDirectionAndOppositeButtonsCancel() {
        pad.setRawButton(6, true); tick(); assertEquals(-1, intakeDirection);
        pad.setRawButton(5, true); tick(); assertEquals(0, intakeDirection);
        pad.setRawButton(6, false); tick(); assertEquals(1, intakeDirection);
        pad.setRawButton(5, false); tick(); assertEquals(0, intakeDirection);
    }
    @Test void rightTriggerOnlyRunsRollerAndReleaseStopsIt() {
        pad.setRightTriggerAxis(1.0); tick();
        assertTrue(rollerOn); assertEquals(0, intakeDirection); assertFalse(aiming);
        pad.setRightTriggerAxis(0.0); tick(); assertFalse(rollerOn);
    }
    @Test void bTakesPriorityOverAimAndReleasingReturnsToHeldAim() {
        pad.setLeftTriggerAxis(1.0); tick(); assertTrue(aiming); assertFalse(shooting);
        pad.setBButton(true); tick(); assertTrue(shooting); assertFalse(aiming);
        pad.setBButton(false); tick(); assertFalse(shooting); assertTrue(aiming);
        pad.setLeftTriggerAxis(0.0); tick(); assertFalse(aiming);
    }
    @Test void disabledAndModifiersCancelMechanisms() {
        pad.setBButton(true); pad.setRightTriggerAxis(1.0); tick();
        assertTrue(shooting); assertTrue(rollerOn);
        pad.setBackButton(true); tick(); assertFalse(shooting); assertFalse(rollerOn);
        pad.setBackButton(false); tick(); assertTrue(shooting);
        DriverStationSim.setEnabled(false); tick(); assertFalse(shooting); assertFalse(rollerOn);
    }
    @Test void noButtonRunsTheCentrifugeAlone() throws Exception {
        // Ela só gira pela sequência de tiro, com rodas e mira prontas.
        assertFalse(Pattern.compile("centrifuga\\.\\w+Command\\(").matcher(robotContainerCode()).find());
    }
    @Test void calibrationIsAllowedOnlyWithoutTheFms() {
        assertTrue(DriverMechanismBindings.calibrationAllowed().getAsBoolean());
        DriverStationSim.setFmsAttached(true); DriverStationSim.notifyNewData();
        assertFalse(DriverMechanismBindings.calibrationAllowed().getAsBoolean());
    }
    @Test void everyCalibrationButtonNeedsTheFmsTrigger() throws Exception {
        var bindings = Arrays.stream(robotContainerCode().split(";"))
            .filter(s -> s.matches("(?s).*(CalibrationCommands\\.|new DriveTwoMetersCommand|new Rotate360Command).*"))
            .toList();
        assertEquals(8, bindings.size(), "Back + Y, 3 setas, A, B e Start + L3/R3");
        for (String binding : bindings) {
            assertTrue(binding.contains(".and(calibracaoLiberada)"), binding);
        }
    }
}
