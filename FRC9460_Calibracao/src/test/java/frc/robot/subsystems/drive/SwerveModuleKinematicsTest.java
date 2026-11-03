package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Funções críticas do swerve com módulos falsos: optimize, escala pelo
 * cosseno, conversões e cinemática. Não prova offsets dos CANcoders,
 * inversões dos motores nem atrito: isso é teste no robô (rodas no ar).
 */
class SwerveModuleKinematicsTest {
    private static final double EPSILON = 1e-6;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void cleanup() {
        scheduler.cancelAll();
        scheduler.unregisterAllSubsystems();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    // #region MÓDULO: OPTIMIZE E COSSENO

    private static FakeModule moduleAt(double turnDegrees) {
        FakeModule io = new FakeModule();
        io.turnRad = Math.toRadians(turnDegrees);
        return io;
    }

    private static void request(FakeModule io, double speed, double degrees) {
        Module module = new Module(io, 0);
        module.periodic();
        module.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(degrees)));
    }

    /** Mesmo ângulo, ignorando voltas completas. */
    private static void assertSameAngle(double expectedDegrees, double actualRad) {
        assertEquals(0.0, Rotation2d.fromRadians(actualRad).minus(Rotation2d.fromDegrees(expectedDegrees))
            .getDegrees(), 1e-6);
    }

    @Test void halfTurnRequestReversesTheWheelInsteadOfSpinningTheModule() {
        FakeModule io = moduleAt(0.0);
        request(io, 1.0, 180.0);
        assertSameAngle(0.0, io.turnSetpoint);
        assertEquals(-1.0, io.velocitySetpoint, EPSILON);
    }

    @Test void speedIsScaledByTheCosineOfTheSteeringError() {
        FakeModule io = moduleAt(0.0);
        request(io, 1.0, 60.0);
        assertSameAngle(60.0, io.turnSetpoint);
        assertEquals(0.5, io.velocitySetpoint, EPSILON, "60° de erro: metade da velocidade");

        FakeModule sideways = moduleAt(0.0);
        request(sideways, 1.0, 90.0);
        assertEquals(0.0, sideways.velocitySetpoint, EPSILON, "90° de erro: roda não empurra");
    }

    @Test void crossingPlusMinus180TakesTheShortWay() {
        FakeModule io = moduleAt(170.0);
        request(io, 1.0, -170.0);
        assertSameAngle(-170.0, io.turnSetpoint);
        assertEquals(Math.cos(Math.toRadians(20.0)), io.velocitySetpoint, EPSILON,
            "20° de erro pelo caminho curto, sem inverter a roda");
    }

    @Test void stateAndPositionUseTheWheelRadius() {
        FakeModule io = moduleAt(30.0);
        io.driveVelocityRadPerSec = 10.0;
        io.drivePositionRad = 2.0 * Math.PI;
        Module module = new Module(io, 0);
        module.periodic();
        assertEquals(10.0 * kWheelRadiusMeters, module.getState().speedMetersPerSecond, EPSILON);
        assertEquals(2.0 * Math.PI * kWheelRadiusMeters, module.getPosition().distanceMeters, EPSILON);
        assertEquals(30.0, module.getState().angle.getDegrees(), EPSILON);
    }

    // #endregion

    // #region CINEMÁTICA DO CHASSI (DRIVE COM MÓDULOS IDEAIS)

    private FakeModule[] modules;
    private Drive drive;

    /** Módulos que viram na hora: o segundo ciclo já não tem erro de direção. */
    private void driveWithIdealSteering(double vx, double vy, double omega) {
        modules = new FakeModule[] {new FakeModule(), new FakeModule(), new FakeModule(), new FakeModule()};
        for (FakeModule module : modules) module.idealSteering = true;
        drive = new Drive(new FakeGyro(), modules[0], modules[1], modules[2], modules[3]);
        for (int cycle = 0; cycle < 3; cycle++) {
            drive.drive(vx, vy, omega);
            drive.periodic();
        }
    }

    @Test void requestAboveTheSpeedLimitIsDesaturated() {
        driveWithIdealSteering(10.0, 0.0, 0.0);
        for (FakeModule module : modules) {
            assertEquals(kMaxLinearSpeed, module.velocitySetpoint, EPSILON);
            assertSameAngle(0.0, module.turnSetpoint);
        }
    }

    @Test void strafingLeftPointsEveryModuleSideways() {
        driveWithIdealSteering(0.0, 1.0, 0.0);
        for (FakeModule module : modules) {
            // +Y do robô: sin(ângulo) × velocidade = 1 m/s, sem componente em X.
            assertEquals(1.0, Math.sin(module.turnSetpoint) * module.velocitySetpoint, EPSILON);
            assertEquals(0.0, Math.cos(module.turnSetpoint) * module.velocitySetpoint, EPSILON);
        }
    }

    @Test void spinningInPlaceUsesTheDiagonalsWithEqualSpeeds() {
        double omega = 1.0;
        driveWithIdealSteering(0.0, 0.0, omega);
        double radius = Math.hypot(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0);
        for (FakeModule module : modules) {
            assertEquals(Math.abs(Math.cos(module.turnSetpoint)), Math.abs(Math.sin(module.turnSetpoint)),
                EPSILON, "módulo na diagonal");
            assertEquals(omega * radius, Math.abs(module.velocitySetpoint), EPSILON);
        }
        // Frente esquerda e traseira direita na mesma diagonal; as outras duas na outra.
        assertSameAngle(Math.toDegrees(modules[0].turnSetpoint), modules[3].turnSetpoint);
        assertSameAngle(Math.toDegrees(modules[1].turnSetpoint), modules[2].turnSetpoint);
    }

    @Test void zeroRequestKeepsEveryWheelStopped() {
        driveWithIdealSteering(0.0, 0.0, 0.0);
        for (FakeModule module : modules) {
            assertEquals(0.0, module.velocitySetpoint, EPSILON);
        }
    }

    // #endregion

    // #region FALSOS

    private static class FakeGyro implements GyroIO {
        @Override
        public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = true;
            inputs.configurationOk = true;
        }
    }

    private static class FakeModule implements ModuleIO {
        boolean idealSteering;
        double turnRad, driveVelocityRadPerSec, drivePositionRad;
        double velocitySetpoint, turnSetpoint;

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.encoderConnected = true;
            inputs.turnPositionRad = turnRad;
            inputs.driveVelocityRadPerSec = driveVelocityRadPerSec;
            inputs.drivePositionRad = drivePositionRad;
        }

        @Override
        public void setDriveVelocity(double speed) {
            velocitySetpoint = speed;
        }

        @Override
        public void setTurnPosition(double radians) {
            turnSetpoint = radians;
            if (idealSteering) turnRad = radians;
        }
    }

    // #endregion
}
