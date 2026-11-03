package frc.robot.controls;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

class OptionalOperatorControllerTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private OptionalOperatorController controller;
    private boolean running;
    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }
    @BeforeEach void setup() {
        scheduler.getActiveButtonLoop().clear();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        controller = new OptionalOperatorController(1);
        controller.rightBumper().whileTrue(Commands.runEnd(() -> running = true, () -> running = false));
    }
    private void tick() { DriverStationSim.notifyNewData(); scheduler.run(); }
    @AfterEach void cleanup() {
        scheduler.cancelAll(); scheduler.getActiveButtonLoop().clear();
        DriverStationSim.setEnabled(false); DriverStationSim.notifyNewData();
    }
    @Test void absentControllerIsNeutralAndConnectingLaterWorks() {
        DriverStationSim.setJoystickButtonCount(1, 0);
        DriverStationSim.setJoystickAxisCount(1, 0);
        DriverStationSim.setJoystickPOVCount(1, 0);
        tick(); assertFalse(running); assertEquals(0.0, controller.getRightX());
        DriverStationSim.setJoystickButtonCount(1, 10);
        DriverStationSim.setJoystickAxisCount(1, 6);
        DriverStationSim.setJoystickButton(1, 6, true);
        DriverStationSim.setJoystickAxis(1, 4, 0.5);
        tick(); assertTrue(running); assertEquals(0.5, controller.getRightX(), 1e-6);
        DriverStationSim.setJoystickButtonCount(1, 0);
        DriverStationSim.setJoystickAxisCount(1, 0);
        tick(); assertFalse(running); assertEquals(0.0, controller.getRightX());
    }
    @Test void deviceWithoutButtonSixDoesNotTriggerTheShooter() {
        DriverStationSim.setJoystickButtonCount(1, 4);
        DriverStationSim.setJoystickAxisCount(1, 2);
        DriverStationSim.setJoystickButton(1, 6, true);
        tick(); assertFalse(running); assertEquals(0.0, controller.getRightX());
    }
}
