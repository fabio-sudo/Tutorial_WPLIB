package frc.robot.subsystems.vision;


// #region IMPORTS

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drive.Drive;

// #endregion



public class Vision extends SubsystemBase {


    // #region DRIVE

    private final Drive drive;

    // #endregion



    // #region POSE INICIAL - MEGATAG2


    // ============================================================
    // ÚLTIMA POSE VISUAL ACEITA
    // ============================================================

    private Pose2d latestValidVisionPose =
        null;


    // ============================================================
    // ESTADO ATUAL DA VISÃO
    // ============================================================

    private boolean visionValidNow =
        false;


    // ============================================================
    // ÚLTIMA MEDIÇÃO VISUAL ACEITA
    // ============================================================
    //
    // Utilizado principalmente pelo:
    //
    // - Seed
    // - FrameAgeSeconds
    //
    // IMPORTANTE:
    //
    // Só é atualizado quando a medição passa
    // também pelo filtro de outlier.
    //
    // ============================================================

    private double lastValidFrameTime =
        -1.0;


    // ============================================================
    // ÚLTIMO FRAME VISUAL RECEBIDO
    // ============================================================
    //
    // Utilizado para medir a continuidade da visão.
    //
    // Mesmo que uma medição seja rejeitada como outlier,
    // este tempo continua sendo atualizado.
    //
    // Isso permite distinguir:
    //
    // - câmera entregando frames ruins
    //
    // de
    //
    // - câmera que realmente ficou sem visão.
    //
    // ============================================================

    private double lastSeenVisionFrameTime =
        -1.0;


    // ============================================================
    // TAGS DA ÚLTIMA MEDIÇÃO ACEITA
    // ============================================================

    private int latestValidTagCount =
        0;


    // #endregion



    // #region CONTROLE DE NOVAS MEDIÇÕES


    // ============================================================
    // TIMESTAMP DO ÚLTIMO FRAME PROCESSADO
    // ============================================================
    //
    // Evita processar duas vezes
    // exatamente o mesmo frame MegaTag2.
    //
    // ============================================================

    private double lastMegaTag2TimestampSeconds =
        -1.0;


    // #endregion



    // #region CONSTRUTOR


    public Vision(Drive drive) {


        this.drive =
            drive;


        // ========================================================
        // POSIÇÃO FÍSICA DA LIMELIGHT NO ROBÔ
        // ========================================================

        LimelightHelpers.setCameraPose_RobotSpace(

            LimelightConstants.REAR_NAME,

            LimelightConstants.REAR_FORWARD,

            LimelightConstants.REAR_RIGHT,

            LimelightConstants.REAR_UP,

            LimelightConstants.REAR_ROLL,

            LimelightConstants.REAR_PITCH,

            LimelightConstants.REAR_YAW

        );


        // ========================================================
        // FILTRO DE APRILTAGS
        // ========================================================
        //
        // Somente as AprilTags definidas em:
        //
        // LimelightConstants.VALID_TAG_IDS
        //
        // serão utilizadas.
        //
        // ========================================================

        LimelightHelpers.SetFiducialIDFiltersOverride(

            LimelightConstants.REAR_NAME,

            LimelightConstants.VALID_TAG_IDS

        );

    }


    // #endregion



    // #region SINCRONIZAÇÃO MANUAL DA POSE


    // ============================================================
    // MEGATAG2 - SEED MANUAL DO POSE ESTIMATOR
    // ============================================================
    //
    // Requisitos:
    //
    // - visão válida
    // - pose visual aceita disponível
    // - pelo menos duas AprilTags
    // - medição recente
    // - robô parado
    //
    // ============================================================

    public void trySeedDrivePose() {


        // ========================================================
        // IDADE DA ÚLTIMA MEDIÇÃO ACEITA
        // ========================================================

        double frameAgeSeconds =

            lastValidFrameTime >= 0.0

                ? Timer.getFPGATimestamp()
                    - lastValidFrameTime

                : -1.0;


        // ========================================================
        // CONDIÇÕES DE SEGURANÇA
        // ========================================================

        boolean allowed =

            visionValidNow

            && latestValidVisionPose != null

            && latestValidTagCount >= 2

            && frameAgeSeconds >= 0.0

            && frameAgeSeconds <= 0.25

            && drive.isStationary();


        // ========================================================
        // ADVANTAGEKIT
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/SeedAccepted",

            allowed

        );


        if (!allowed) {

            return;

        }


        // ========================================================
        // INICIALIZA POSE DO DRIVE
        // ========================================================

        drive.seedPoseFromVision(

            latestValidVisionPose

        );

    }


    // #endregion



    // #region CONFIANÇA DINÂMICA DA VISÃO


    // ============================================================
    // CÁLCULO DO DESVIO PADRÃO XY
    // ============================================================
    //
    // Quanto MENOR:
    //
    // maior confiança na Limelight.
    //
    // Quanto MAIOR:
    //
    // maior confiança na odometria.
    //
    // Baseado em:
    //
    // - número de AprilTags
    // - distância média das tags
    //
    // ============================================================

    private double calculateVisionXYStdDev(

        int tagCount,

        double avgTagDist

    ) {


        // ========================================================
        // DUAS OU MAIS APRILTAGS
        // ========================================================

        if (tagCount >= 2) {


            // ----------------------------------------------------
            // ATÉ 2 METROS
            // ----------------------------------------------------

            if (avgTagDist <= 2.0) {

                return 0.25;

            }


            // ----------------------------------------------------
            // ENTRE 2 E 3 METROS
            // ----------------------------------------------------

            if (avgTagDist <= 3.0) {

                return 0.40;

            }


            // ----------------------------------------------------
            // ACIMA DE 3 METROS
            // ----------------------------------------------------

            return 0.60;

        }


        // ========================================================
        // SOMENTE UMA APRILTAG
        // ========================================================


        // --------------------------------------------------------
        // ATÉ 2 METROS
        // --------------------------------------------------------

        if (avgTagDist <= 2.0) {

            return 0.60;

        }


        // --------------------------------------------------------
        // ENTRE 2 E 3 METROS
        // --------------------------------------------------------

        if (avgTagDist <= 3.0) {

            return 0.85;

        }


        // --------------------------------------------------------
        // ACIMA DE 3 METROS
        // --------------------------------------------------------

        return 1.20;

    }


    // #endregion



    // #region PERIODIC


    @Override
    public void periodic() {


        // #region LEITURA BÁSICA DA LIMELIGHT


        // ========================================================
        // POSSUI ALVO
        // ========================================================

        boolean hasTarget =

            LimelightHelpers.getTV(

                LimelightConstants.REAR_NAME

            );


        // ========================================================
        // TX
        // ========================================================

        double tx =

            LimelightHelpers.getTX(

                LimelightConstants.REAR_NAME

            );


        // ========================================================
        // TY
        // ========================================================

        double ty =

            LimelightHelpers.getTY(

                LimelightConstants.REAR_NAME

            );


        // ========================================================
        // ÁREA
        // ========================================================

        double ta =

            LimelightHelpers.getTA(

                LimelightConstants.REAR_NAME

            );


        // ========================================================
        // ID DA APRILTAG PRINCIPAL
        // ========================================================

        double tagId =

            LimelightHelpers.getFiducialID(

                LimelightConstants.REAR_NAME

            );


        // #endregion



        // #region PIGEON / ORIENTAÇÃO


        // ========================================================
        // YAW DO ROBÔ
        // ========================================================

        double robotYawDegrees =

            drive
                .getRotation()
                .getDegrees();


        // ========================================================
        // VELOCIDADE ANGULAR
        // ========================================================

        double robotYawRateDegPerSec =

            drive
                .getYawVelocityDegreesPerSecond();


        // ========================================================
        // VALIDAÇÃO DA VELOCIDADE ANGULAR
        // ========================================================

        boolean rotationRateValid =

            Double.isFinite(

                robotYawRateDegPerSec

            )

            && Math.abs(

                robotYawRateDegPerSec

            )
                < LimelightConstants
                    .MAX_VISION_YAW_RATE_DEG_PER_SEC;


        // ========================================================
        // ENVIA ORIENTAÇÃO DO PIGEON PARA O MEGATAG2
        // ========================================================

        LimelightHelpers.SetRobotOrientation(

            LimelightConstants.REAR_NAME,

            robotYawDegrees,

            0.0,

            0.0,

            0.0,

            0.0,

            0.0

        );


        // #endregion



        // #region MEGATAG2


        // ========================================================
        // OBTÉM POSE DO MEGATAG2
        // ========================================================

        LimelightHelpers.PoseEstimate megaTag2Estimate =

            LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(

                    LimelightConstants.REAR_NAME

                );


        // #endregion



        // #region PASSO 7 - DIAGNÓSTICO DOS FILTROS


        // ========================================================
        // REJEIÇÃO - SEM ESTIMATIVA
        // ========================================================

        boolean rejectNoEstimate =

            megaTag2Estimate == null;


        // ========================================================
        // REJEIÇÃO - SEM POSE
        // ========================================================

        boolean rejectNoPose =

            megaTag2Estimate != null

            && megaTag2Estimate.pose == null;


        // ========================================================
        // REJEIÇÃO - SEM APRILTAGS
        // ========================================================

        boolean rejectNoTags =

            megaTag2Estimate != null

            && megaTag2Estimate.tagCount <= 0;


        // ========================================================
        // REJEIÇÃO - DISTÂNCIA
        // ========================================================

        boolean rejectDistance =

            megaTag2Estimate != null

            && (

                !Double.isFinite(

                    megaTag2Estimate.avgTagDist

                )

                || megaTag2Estimate.avgTagDist
                    >= LimelightConstants
                        .MAX_TAG_DISTANCE_METERS

            );


        // ========================================================
        // REJEIÇÃO - ROTAÇÃO RÁPIDA
        // ========================================================

        boolean rejectRotation =

            !rotationRateValid;


        // #endregion



        // #region VALIDAÇÃO BÁSICA DO MEGATAG2


        // ========================================================
        // MEGATAG2 VÁLIDO
        // ========================================================
        //
        // A medição passa pela primeira etapa quando:
        //
        // - existe estimativa
        // - existe pose
        // - existe pelo menos uma tag
        // - distância é válida
        // - velocidade angular é válida
        //
        // ========================================================

        boolean megaTag2Valid =

            !rejectNoEstimate

            && !rejectNoPose

            && !rejectNoTags

            && !rejectDistance

            && !rejectRotation;


        // ========================================================
        // ESTADO BRUTO ATUAL DA VISÃO
        // ========================================================

        visionValidNow =

            megaTag2Valid;


        // #endregion



        // #region NOVA MEDIÇÃO


        // ========================================================
        // VERIFICA SE É UM FRAME NOVO
        // ========================================================

        boolean newMegaTag2Measurement =

            megaTag2Valid

            && Double.isFinite(

                megaTag2Estimate.timestampSeconds

            )

            && megaTag2Estimate.timestampSeconds > 0.0

            && megaTag2Estimate.timestampSeconds
                > lastMegaTag2TimestampSeconds;


        // ========================================================
        // ESTADOS DESTE CICLO
        // ========================================================
        //
        // São zerados a cada periodic.
        //
        // Portanto os logs mostram o estado real
        // deste ciclo e não ficam presos no valor anterior.
        //
        // ========================================================

        boolean outlierSuspected =
            false;


        boolean measurementAccepted =
            false;


        // ========================================================
        // PROCESSAMENTO DO NOVO FRAME
        // ========================================================

        if (newMegaTag2Measurement) {


            // ====================================================
            // TIMESTAMP DO FRAME PROCESSADO
            // ====================================================
            //
            // Atualizamos mesmo quando o frame será rejeitado
            // posteriormente como outlier.
            //
            // Assim nunca processamos o mesmo frame duas vezes.
            //
            // ====================================================

            lastMegaTag2TimestampSeconds =

                megaTag2Estimate.timestampSeconds;


            // ====================================================
            // TEMPO ATUAL DO ROBÔ
            // ====================================================

            double currentVisionTime =

                Timer.getFPGATimestamp();


            // ====================================================
            // INTERVALO DESDE O ÚLTIMO FRAME VISUAL
            // ====================================================

            double visionGapSeconds =

                lastSeenVisionFrameTime >= 0.0

                    ? currentVisionTime
                        - lastSeenVisionFrameTime

                    : -1.0;


            // ====================================================
            // LOG DO GAP
            // ====================================================

            Logger.recordOutput(

                "Vision/Rear/MegaTag2/VisionGapSeconds",

                visionGapSeconds

            );


            // ====================================================
            // ATUALIZA ÚLTIMO FRAME RECEBIDO
            // ====================================================
            //
            // Isto ocorre ANTES da decisão de outlier.
            //
            // Portanto:
            //
            // frame recebido ≠ frame aceito.
            //
            // ====================================================

            lastSeenVisionFrameTime =

                currentVisionTime;


            // ====================================================
            // ERRO ENTRE POSE DO ROBÔ E POSE VISUAL
            // ====================================================

            double visionPoseErrorMeters =

                drive
                    .getPose()
                    .getTranslation()
                    .getDistance(

                        megaTag2Estimate
                            .pose
                            .getTranslation()

                    );


            // ====================================================
            // LOG DO ERRO
            // ====================================================

            Logger.recordOutput(

                "Vision/Rear/MegaTag2/PoseErrorMeters",

                visionPoseErrorMeters

            );


            // ====================================================
            // DETECÇÃO DE OUTLIER
            // ====================================================
            //
            // Consideramos suspeito quando:
            //
            // 1 - a câmera estava entregando frames continuamente
            //
            //     gap <= 0.25 s
            //
            // 2 - a nova pose aparece mais de 1 metro
            //     distante da pose atual.
            //
            // Se houve uma longa perda de visão:
            //
            // gap > 0.25
            //
            // não rejeitamos automaticamente,
            // pois pode ser uma relocalização legítima.
            //
            // ====================================================

            outlierSuspected =

                visionGapSeconds >= 0.0

                && visionGapSeconds <= 0.25

                && visionPoseErrorMeters > 1.0;


            // ====================================================
            // DECISÃO FINAL
            // ====================================================

            measurementAccepted =

                !outlierSuspected;


            // ====================================================
            // CONFIANÇA DINÂMICA
            // ====================================================

            double visionXYStdDev =

                calculateVisionXYStdDev(

                    megaTag2Estimate.tagCount,

                    megaTag2Estimate.avgTagDist

                );


            // ====================================================
            // LOG DA CONFIANÇA
            // ====================================================

            Logger.recordOutput(

                "Vision/Rear/MegaTag2/XYStdDevMeters",

                visionXYStdDev

            );


            // ====================================================
            // MEDIÇÃO ACEITA
            // ====================================================
            //
            // Somente aqui:
            //
            // - atualizamos última pose válida
            // - atualizamos quantidade de tags
            // - atualizamos lastValidFrameTime
            // - enviamos ao PoseEstimator
            //
            // ====================================================

            if (measurementAccepted) {


                // ------------------------------------------------
                // ÚLTIMA POSE VISUAL ACEITA
                // ------------------------------------------------

                latestValidVisionPose =

                    megaTag2Estimate.pose;


                // ------------------------------------------------
                // TAGS DA ÚLTIMA MEDIÇÃO ACEITA
                // ------------------------------------------------

                latestValidTagCount =

                    megaTag2Estimate.tagCount;


                // ------------------------------------------------
                // ÚLTIMO TEMPO ACEITO
                // ------------------------------------------------

                lastValidFrameTime =

                    currentVisionTime;


                // ------------------------------------------------
                // FUSÃO COM POSE ESTIMATOR
                // ------------------------------------------------

                drive.addVisionMeasurement(

                    megaTag2Estimate.pose,

                    megaTag2Estimate.timestampSeconds,

                    visionXYStdDev

                );

            }

        }


        // #endregion



        // #region PRONTIDÃO PARA SEED


        // ========================================================
        // IDADE DA ÚLTIMA MEDIÇÃO ACEITA
        // ========================================================

        double frameAgeSeconds =

            lastValidFrameTime >= 0.0

                ? Timer.getFPGATimestamp()
                    - lastValidFrameTime

                : -1.0;


        // ========================================================
        // SEED READY
        // ========================================================

        boolean seedReady =

            visionValidNow

            && latestValidVisionPose != null

            && latestValidTagCount >= 2

            && lastValidFrameTime >= 0.0

            && frameAgeSeconds >= 0.0

            && frameAgeSeconds <= 0.25;


        // #endregion



        // #region ADVANTAGEKIT


        // ========================================================
        // ESTADO GERAL
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/RobotYawDegrees",

            robotYawDegrees

        );


        Logger.recordOutput(

            "Vision/Rear/HasTarget",

            hasTarget

        );


        Logger.recordOutput(

            "Vision/Rear/TX",

            tx

        );


        Logger.recordOutput(

            "Vision/Rear/TY",

            ty

        );


        Logger.recordOutput(

            "Vision/Rear/TA",

            ta

        );


        Logger.recordOutput(

            "Vision/Rear/TagID",

            tagId

        );


        // ========================================================
        // VALIDAÇÃO MEGATAG2
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/Valid",

            megaTag2Valid

        );


        Logger.recordOutput(

            "Vision/Rear/MegaTag2/NewMeasurement",

            newMegaTag2Measurement

        );


        // ========================================================
        // DECISÃO FINAL
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/MeasurementAccepted",

            measurementAccepted

        );


        // ========================================================
        // OUTLIER
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/OutlierSuspected",

            outlierSuspected

        );


        // ========================================================
        // PASSO 7 - MOTIVOS DE REJEIÇÃO
        // ========================================================


        // --------------------------------------------------------
        // SEM ESTIMATIVA
        // --------------------------------------------------------

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/Reject/NoEstimate",

            rejectNoEstimate

        );


        // --------------------------------------------------------
        // SEM POSE
        // --------------------------------------------------------

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/Reject/NoPose",

            rejectNoPose

        );


        // --------------------------------------------------------
        // SEM TAGS
        // --------------------------------------------------------

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/Reject/NoTags",

            rejectNoTags

        );


        // --------------------------------------------------------
        // DISTÂNCIA
        // --------------------------------------------------------

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/Reject/Distance",

            rejectDistance

        );


        // --------------------------------------------------------
        // ROTAÇÃO
        // --------------------------------------------------------

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/Reject/Rotation",

            rejectRotation

        );


        // --------------------------------------------------------
        // OUTLIER
        // --------------------------------------------------------

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/Reject/Outlier",

            outlierSuspected

        );


        // ========================================================
        // QUANTIDADE DE TAGS
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/TagCount",

            megaTag2Estimate != null

                ? megaTag2Estimate.tagCount

                : 0

        );


        // ========================================================
        // DISTÂNCIA MÉDIA
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/AvgTagDist",

            megaTag2Estimate != null

                ? megaTag2Estimate.avgTagDist

                : 0.0

        );


        // ========================================================
        // TIMESTAMP
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/TimestampSeconds",

            megaTag2Estimate != null

                ? megaTag2Estimate.timestampSeconds

                : 0.0

        );


        // ========================================================
        // VELOCIDADE ANGULAR
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/YawRateDegPerSec",

            robotYawRateDegPerSec

        );


        Logger.recordOutput(

            "Vision/Rear/MegaTag2/RotationRateValid",

            rotationRateValid

        );


        // ========================================================
        // SEED
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/FrameAgeSeconds",

            frameAgeSeconds

        );


        Logger.recordOutput(

            "Vision/Rear/MegaTag2/SeedReady",

            seedReady

        );


        // ========================================================
        // POSE BRUTA VÁLIDA DO MEGATAG2
        // ========================================================
        //
        // IMPORTANTE:
        //
        // Esta é a pose produzida pela Limelight.
        //
        // Ela pode aparecer aqui mesmo se posteriormente
        // for rejeitada como outlier.
        //
        // Isso é proposital para diagnóstico.
        //
        // ========================================================

        if (megaTag2Valid) {


            Logger.recordOutput(

                "Vision/Rear/MegaTag2/Pose",

                megaTag2Estimate.pose

            );


            Logger.recordOutput(

                "Vision/Rear/MegaTag2/X",

                megaTag2Estimate
                    .pose
                    .getX()

            );


            Logger.recordOutput(

                "Vision/Rear/MegaTag2/Y",

                megaTag2Estimate
                    .pose
                    .getY()

            );


            Logger.recordOutput(

                "Vision/Rear/MegaTag2/RotationDegrees",

                megaTag2Estimate
                    .pose
                    .getRotation()
                    .getDegrees()

            );

        }


        // #endregion

    }


    // #endregion



    // #region GETTERS


    public boolean hasTarget() {

        return LimelightHelpers.getTV(

            LimelightConstants.REAR_NAME

        );

    }



    public double getTX() {

        return LimelightHelpers.getTX(

            LimelightConstants.REAR_NAME

        );

    }



    public double getTY() {

        return LimelightHelpers.getTY(

            LimelightConstants.REAR_NAME

        );

    }



    public double getTA() {

        return LimelightHelpers.getTA(

            LimelightConstants.REAR_NAME

        );

    }



    public int getTagID() {

        return (int)
            LimelightHelpers.getFiducialID(

                LimelightConstants.REAR_NAME

            );

    }


    // #endregion

}