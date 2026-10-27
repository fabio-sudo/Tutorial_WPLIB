package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveStartupInputsAutoLogged;
import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Executado em uma JVM separada: Logger.start é único por processo.
 * Exercita WPILOG real -> processInputs -> Vision -> estimador -> novo WPILOG.
 * Os dados de teste são sintéticos; não representam uma coleta física.
 */
class VisionReplayIntegrationTest {
    @Test void replaysBothCamerasAndPreservesOriginalInputsWithoutRunningTheWorld() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        Path directory = Path.of(System.getProperty("replay.validationDir"));
        Files.createDirectories(directory);
        Path input = directory.resolve("visao-entrada.wpilog");
        Path output = directory.resolve("visao-replay.wpilog");
        writeFixture(input);
        byte[] original = Files.readAllBytes(input);

        // Somente o teste usa os AdvancedHooks; Robot continua usando LoggedRobot.
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.setReplaySource(new WPILOGReader(input.toString()));
        Logger.addDataReceiver(new WPILOGWriter(output.toString(),
            WPILOGWriter.AdvantageScopeOpenBehavior.NEVER));
        Logger.start();
        try {
            Drive drive = new Drive();
            Vision vision = new Vision(drive); // Frontal física está desativada.
            for (int cycle = 0; cycle < 40; cycle++) {
                if (cycle > 0) Logger.AdvancedHooks.invokePeriodicBeforeUser();
                drive.periodic();
                vision.periodic();
                if (cycle == 1) {
                    assertEquals(10.02, Timer.getTimestamp(), 1e-6,
                        "O relógio da lógica deve vir do arquivo, não do computador");
                    assertEquals(2.0, drive.getPose().getX(), 1e-9);
                    assertEquals(2.0, drive.getPose().getY(), 1e-9);
                }
                assertFalse(drive.isSimulationReferenceReady(),
                    "Replay não integra outra física nem movimenta câmeras virtuais");
                Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
            }
            assertEquals(4, drive.getVisionAcceptedCount("Rear"),
                "Inclui o lote com dois frames, ignora a repetição e continua sem a frontal");
            assertEquals(1, drive.getVisionAcceptedCount("Front"),
                "A configuração da frontal física não pode apagar seus inputs gravados");
            assertTrue(drive.getPose().getX() > 2.0 && drive.getPose().getX() < 2.3);
            assertEquals(0.0, drive.getPose().getRotation().getRadians(), 1e-9);
            assertEquals(1, drive.getPoseResetGeneration(), "A pose inicial é aplicada uma vez");
        } finally {
            Logger.end();
            CommandScheduler.getInstance().unregisterAllSubsystems();
        }
        assertArrayEquals(original, Files.readAllBytes(input), "O arquivo original é preservado");

        // O novo arquivo deve conter saídas originais E saídas recalculadas.
        // WPILOGReader ignora ReplayOutputs deliberadamente para não reaplicar
        // resultados antigos. A inspeção do arquivo pronto usa DataLogReader.
        DataLogReader result = new DataLogReader(output.toString());
        assertTrue(result.isValid());
        Map<Integer, String> entries = new HashMap<>();
        boolean foundComparison = false;
        boolean foundOriginal = false;
        for (var record : result) {
            if (record.isStart()) entries.put(record.getStartData().entry, record.getStartData().name);
            if (record.isControl()) continue;
            String name = entries.get(record.getEntry());
            if ("/ReplayOutputs/Robot/Pose".equals(name)) {
                Pose2d recalculated = Pose2d.struct.unpack(ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN));
                if (recalculated.getX() > 2.0 && recalculated.getX() < 2.3) foundComparison = true;
            } else if ("/RealOutputs/Robot/Pose".equals(name)) {
                Pose2d recorded = Pose2d.struct.unpack(ByteBuffer.wrap(record.getRaw()).order(ByteOrder.LITTLE_ENDIAN));
                assertEquals(99.0, recorded.getX(), 1e-9);
                foundOriginal = true;
            }
        }
        assertTrue(foundOriginal, "A saída original também precisa continuar no arquivo");
        assertTrue(foundComparison, "O resultado precisa ser gravado em ReplayOutputs");
    }

    private static void writeFixture(Path path) {
        WPILOGWriter writer = new WPILOGWriter(path.toString(),
            WPILOGWriter.AdvantageScopeOpenBehavior.NEVER);
        writer.start();
        double[] now = {10.0};
        VisionIOSim rear = new VisionIOSim(() -> now[0]);
        VisionIOSim front = new VisionIOSim(() -> now[0]);
        var rearInputs = new VisionIOInputsAutoLogged();
        var frontInputs = new VisionIOInputsAutoLogged();
        var startup = new DriveStartupInputsAutoLogged();
        startup.initializeSimulationPose = true;
        startup.initialPose = new Pose2d(2.0, 2.0, new Rotation2d());
        var gyro = new GyroIOInputsAutoLogged();
        gyro.connected = true;
        gyro.configurationOk = true;
        var module = new ModuleIOInputsAutoLogged();
        for (int cycle = 0; cycle < 50; cycle++) {
            now[0] = 10.0 + cycle * 0.02;
            if (cycle == 6) {
                publish(rear, 10.08, 2.20, 9);
                publish(rear, 10.10, 2.22, 9);
                publish(front, 10.10, 2.21, 25);
            }
            if (cycle == 7) publish(rear, 10.10, 2.22, 9); // Repetido.
            if (cycle == 14) front.setConnected(false);
            if (cycle == 16) {
                // Compatibilidade com o formato MegaTag2 usado pela Limelight.
                rear.publishFrame(new Pose2d(2.23, 2.0, new Rotation2d()), 2, 9, 1.8, 10.30, 20.0);
            }
            if (cycle == 20) publish(rear, 10.38, 2.24, 9);
            rear.updateInputs(rearInputs);
            front.updateInputs(frontInputs);
            LogTable table = new LogTable(Math.round(now[0] * 1_000_000));
            startup.toLog(table.getSubtable("Drive/Startup"));
            gyro.toLog(table.getSubtable("Drive/Gyro"));
            for (int index = 0; index < 4; index++) module.toLog(table.getSubtable("Drive/Module" + index));
            rearInputs.toLog(table.getSubtable("Vision/Rear/Inputs"));
            // As primeiras linhas não têm frontal: deve permanecer desconectada.
            if (cycle >= 6) frontInputs.toLog(table.getSubtable("Vision/Front/Inputs"));
            table.getSubtable("RealOutputs").put("Robot/Pose", new Pose2d(99, 99, new Rotation2d()));
            writer.putTable(table);
        }
        writer.end();
    }

    private static void publish(VisionIOSim io, double capture, double x, int id) {
        io.publishObservation(new PoseObservation(capture,
            new Pose3d(new Pose2d(x, 2.0, new Rotation2d())), 0.0, 2, 1.8,
            PoseObservationType.PHOTONVISION), id, 20.0);
    }
}
