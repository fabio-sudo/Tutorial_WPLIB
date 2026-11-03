package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;

/**
 * O cenário que enganou o robô em 28 e 29/10/2026: o Pigeon liga com um heading
 * (0°), as tags dizem outro (180°). O MegaTag2 espelha a pose e o shooter confia
 * nela. Aqui o Drive e o Vision reais recebem leituras MegaTag1 controladas.
 */
class VisionAutoInitTest {

    private double now;
    private Drive drive;
    private Vision vision;
    private VisionIOSim rear;
    private VisionMultiCameraTest.SensorGyro gyro;
    private VisionMultiCameraTest.SensorModule[] modules;

    // O robô está em (3,5; 4,0) olhando para 180° (parede azul), como nas tags 25/26.
    private static final Pose2d TAGS_SEE = new Pose2d(3.5, 4.0, Rotation2d.fromDegrees(180.0));
    // Na metade vermelha, de costas para a parede vermelha (frente para -X).
    private static final Pose2d RED_HALF = new Pose2d(13.0, 4.0, Rotation2d.fromDegrees(180.0));

    @BeforeAll
    static void init() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setup() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        setEnabled(false);
        now = 10.0;
        gyro = new VisionMultiCameraTest.SensorGyro();
        modules = new VisionMultiCameraTest.SensorModule[] {
            new VisionMultiCameraTest.SensorModule(), new VisionMultiCameraTest.SensorModule(),
            new VisionMultiCameraTest.SensorModule(), new VisionMultiCameraTest.SensorModule()};
        drive = new Drive(gyro, modules[0], modules[1], modules[2], modules[3], () -> now);
        rear = new VisionIOSim(() -> now);
        vision = new Vision(drive, () -> now, new VisionCamera("Rear", rear));
        drive.periodic();
        tick(0.1);
    }

    @AfterEach
    void cleanup() {
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    private void setEnabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
    }

    /** Avança o relógio e roda Drive e Vision, como o laço do robô. */
    private void tick(double seconds) {
        now += seconds;
        drive.periodic();
        vision.periodic();
    }

    /** Uma câmera entregando quadros MegaTag1 novos a ~30 Hz. */
    private void tagsSee(Pose2d pose, int tagCount, double distance, int frames) {
        for (int i = 0; i < frames; i++) {
            rear.publishMegaTag1(pose, tagCount, distance, now);
            tick(0.033);
        }
    }

    @Test
    void wrongHeadingIsCorrectedByTheTagsWhileDisabledAndStationary() {
        assertEquals(0.0, drive.getRotation().getDegrees(), 1e-6, "o Pigeon ligou achando que olha para 0°");

        tagsSee(TAGS_SEE, 2, 2.5, AUTO_INIT_STABLE_FRAMES + 2);

        assertEquals(1, vision.getAutoInitCount());
        assertEquals(1, drive.getVisionInitCount());
        assertEquals(TAGS_SEE.getX(), drive.getPose().getX(), 0.05);
        assertEquals(TAGS_SEE.getY(), drive.getPose().getY(), 0.05);
        assertEquals(180.0, Math.abs(drive.getRotation().getDegrees()), 1.0);
        assertEquals("APLICADA", drive.getVisionInitResult());
        // O Vision manda este heading à câmera: o MegaTag2 já sai do lado certo.
        assertFalse(drive.hasVisionCorrectionSinceReset(),
            "o shooter só confia na pose depois do primeiro MegaTag2 aplicado com o heading novo");
    }

    @Test
    void itAppliesOnlyOnceAndKeepsHoldingTheNewHeading() {
        tagsSee(TAGS_SEE, 2, 2.5, 60);
        assertEquals(1, vision.getAutoInitCount(), "alinhada: não reaplica");
        assertEquals(1, drive.getVisionInitCount());
        assertEquals(1L, drive.getPoseResetGeneration() - 0L, "um reset só");
    }

    @Test
    void afterTheInitializationTheMegaTag2StillFusesAndTheShooterCanTrustThePose() {
        tagsSee(TAGS_SEE, 2, 2.5, AUTO_INIT_STABLE_FRAMES + 2);
        assertEquals(1, vision.getAutoInitCount());
        // MegaTag2 com o heading certo, no mesmo lugar: o estimador aceita.
        for (int i = 0; i < 6; i++) {
            rear.publishFrame(TAGS_SEE, 2, 25, 2.5, now - 0.04, 20.0);
            tick(0.05);
        }
        tick(0.2);
        assertTrue(drive.getVisionAcceptedCount("Rear") >= 1, "MegaTag2 continua fundindo");
        assertTrue(drive.hasVisionCorrectionSinceReset());
    }

    @Test
    void enabledRobotNeverHasItsHeadingChangedOnlyAnAlert() {
        setEnabled(true);
        // 40 quadros a 33 ms = 1,3 s: passa do HEADING_ALERT_SECONDS (1 s).
        tagsSee(TAGS_SEE, 2, 2.5, 40);
        assertEquals(0, vision.getAutoInitCount(), "heading não muda no jogo");
        assertEquals(0.0, drive.getRotation().getDegrees(), 1e-6);
        assertTrue(vision.isHeadingAlertActive(), "parado, vendo as tags e divergindo há mais de 1 s: alerta");
    }

    @Test
    void headingAlertWaitsForTheConfiguredTimeBeforeFiring() {
        setEnabled(true);
        int framesBeforeLimit = (int) (HEADING_ALERT_SECONDS / 0.033) - 5;
        tagsSee(TAGS_SEE, 2, 2.5, framesBeforeLimit);
        assertFalse(vision.isHeadingAlertActive(), "um tropeço de uma ou duas leituras não vira alerta");
    }

    @Test
    void movingRobotIsNeverInitializedNorAlerted() {
        modules[0].speed = 1.0; // uma roda girando: não está parado.
        tagsSee(TAGS_SEE, 2, 2.5, 25);
        assertEquals(0, vision.getAutoInitCount());
        assertFalse(vision.isHeadingAlertActive());
        assertEquals(0.0, drive.getRotation().getDegrees(), 1e-6);
    }

    @Test
    void anAlignedPoseIsLeftAlone() {
        // As tags concordam com o estimador (dentro da tolerância): nada muda.
        Pose2d near = new Pose2d(0.2, 0.1, Rotation2d.fromDegrees(2.0));
        long resetsBefore = drive.getPoseResetGeneration();
        tagsSee(near, 2, 2.5, 30);
        assertEquals(0, vision.getAutoInitCount());
        assertEquals(resetsBefore, drive.getPoseResetGeneration(), "nenhum reset do estimador");
        assertFalse(vision.isHeadingAlertActive());
    }

    @Test
    void oneTagIsNotEnoughToDefineTheHeading() {
        tagsSee(TAGS_SEE, 1, 2.5, 25);
        assertEquals(0, vision.getAutoInitCount());
        assertEquals(0.0, drive.getRotation().getDegrees(), 1e-6);
    }

    // #region REFERÊNCIA DO PILOTO: O LADO VEM DAS TAGS, NÃO DA DRIVER STATION

    @Test
    void theRedHalfPointsThePilotForwardAwayFromTheRedWall() {
        // A Driver Station diz azul de propósito: a direção não pode olhar para ela.
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        tagsSee(RED_HALF, 2, 2.5, AUTO_INIT_STABLE_FRAMES + 2);
        assertEquals("TAGS_LADO_VERMELHO", drive.getDriverReferenceSource());
        // Robô olhando para -X: a frente do piloto é a frente do robô.
        assertEquals(0.0, drive.getDriverRotation().getDegrees(), 1.0);
    }

    @Test
    void theBlueHalfKeepsThePilotForwardTowardPlusX() {
        // Red 1 é o padrão da Driver Station fora do FMS (teste de 28/10/2026).
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        tagsSee(TAGS_SEE, 2, 2.5, AUTO_INIT_STABLE_FRAMES + 2);
        assertEquals("TAGS_LADO_AZUL", drive.getDriverReferenceSource());
        assertEquals(180.0, Math.abs(drive.getDriverRotation().getDegrees()), 1.0);
    }

    @Test
    void megaTag2AloneAlsoDefinesTheSideWhenTheInitializationDoesNotRun() {
        // Heading já certo e só MegaTag2 chegando: a inicialização pelas tags não roda.
        Pose2d facingTheRedWall = new Pose2d(13.0, 4.0, new Rotation2d());
        for (int i = 0; i < 40; i++) {
            rear.publishFrame(facingTheRedWall, 2, 25, 1.5, now - 0.04, 20.0);
            tick(0.05);
        }
        tick(0.2);
        assertEquals(0, vision.getAutoInitCount());
        assertTrue(drive.getPose().getX() > VisionFieldConstants.FIELD_LENGTH_METERS / 2.0);
        assertEquals("TAGS_LADO_VERMELHO", drive.getDriverReferenceSource());
        assertEquals(180.0, Math.abs(drive.getDriverRotation().getDegrees()), 1.0);
    }

    @Test
    void theSideLocksOnTheFirstEnableSoCrossingTheFieldDoesNotFlipThePilot() {
        tagsSee(TAGS_SEE, 2, 2.5, AUTO_INIT_STABLE_FRAMES + 2);
        setEnabled(true);
        tick(0.02);
        // O intervalo entre autônomo e teleoperado, com o robô já na outra metade.
        setEnabled(false);
        tick(0.02);
        tagsSee(RED_HALF, 2, 2.5, AUTO_INIT_STABLE_FRAMES + 2);
        assertEquals(2, vision.getAutoInitCount(), "a pose continua sendo corrigida");
        assertEquals("TAGS_LADO_AZUL", drive.getDriverReferenceSource());
        assertEquals(180.0, Math.abs(drive.getDriverRotation().getDegrees()), 1.0);
    }

    @Test
    void thePilotYSurvivesTheTagsAndKeepsTheSameDirectionOnTheFloor() {
        drive.zeroDriverHeading();
        assertEquals("BOTAO_Y", drive.getDriverReferenceSource());
        // O heading de campo muda 180° no número; o robô não saiu do lugar.
        tagsSee(TAGS_SEE, 2, 2.5, AUTO_INIT_STABLE_FRAMES + 2);
        assertEquals(1, vision.getAutoInitCount());
        assertEquals(180.0, Math.abs(drive.getRotation().getDegrees()), 1.0);
        assertEquals(0.0, drive.getDriverRotation().getDegrees(), 1.0);
        assertEquals("BOTAO_Y", drive.getDriverReferenceSource());
    }

    @Test
    void startPlusAReleasesALockedSide() {
        setEnabled(true);
        tick(0.02);
        setEnabled(false);
        tick(0.02);
        drive.setFieldHeading(Rotation2d.k180deg);
        tagsSee(RED_HALF, 2, 2.5, AUTO_INIT_STABLE_FRAMES + 2);
        assertEquals("TAGS_LADO_VERMELHO", drive.getDriverReferenceSource());
    }

    // #endregion

    @Test
    void driveRefusesToInitializeWhileEnabledEvenIfAskedDirectly() {
        setEnabled(true);
        drive.periodic();
        assertFalse(drive.initializePoseFromVision(TAGS_SEE));
        assertEquals("RECUSADA_HABILITADO", drive.getVisionInitResult());
        assertEquals(0L, drive.getVisionInitCount());
    }
}
