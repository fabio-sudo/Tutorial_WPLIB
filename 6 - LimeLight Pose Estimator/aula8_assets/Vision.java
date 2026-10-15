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

    private Pose2d latestValidVisionPose =
        null;


    private boolean visionValidNow =
        false;


    private double lastValidFrameTime =
        -1.0;


    // Quantidade de tags utilizadas
    // na última captura válida
    private int latestValidTagCount =
        0;

    // #endregion



    // #region CONTROLE DE NOVAS MEDIÇÕES

    // Guarda o timestamp da última
    // medição MegaTag2 processada.
    //
    // Evita utilizar duas vezes
    // exatamente o mesmo frame.

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
        //
        // Limelight traseira.
        //
        // Valores configurados no Constants.java:
        //
        // REAR_FORWARD
        // REAR_RIGHT
        // REAR_UP
        //
        // REAR_ROLL
        // REAR_PITCH
        // REAR_YAW
        //
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
        // serão utilizadas para localização.
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
    // MEGATAG2 - INICIALIZAÇÃO MANUAL DO POSE ESTIMATOR
    // ============================================================
    //
    // Utilizado para sincronizar X e Y iniciais.
    //
    // Requisitos:
    //
    // - visão válida
    // - pose disponível
    // - pelo menos duas AprilTags
    // - frame recente
    // - robô parado
    //
    // ============================================================

    public void trySeedDrivePose() {


        // ========================================================
        // IDADE DA ÚLTIMA MEDIÇÃO VÁLIDA
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


        // Se alguma condição falhar,
        // não sincroniza.

        if (!allowed) {

            return;

        }


        // ========================================================
        // INICIALIZAÇÃO DA POSIÇÃO ABSOLUTA
        // ========================================================

        drive.seedPoseFromVision(

            latestValidVisionPose

        );

    }

    // #endregion



    // #region NEW - CONFIANÇA DINÂMICA DA VISÃO

    // ============================================================
    // NEW - CÁLCULO DA CONFIANÇA DA MEDIÇÃO MEGATAG2
    // ============================================================
    //
    // O valor retornado representa o desvio padrão
    // utilizado pelo SwerveDrivePoseEstimator.
    //
    // Quanto MENOR:
    //
    //     maior a confiança na Limelight.
    //
    // Quanto MAIOR:
    //
    //     maior a confiança na odometria.
    //
    // A decisão utiliza:
    //
    // - quantidade de AprilTags
    // - distância média das AprilTags
    //
    // ============================================================

    private double calculateVisionXYStdDev(

        int tagCount,

        double avgTagDist

    ) {


        // ========================================================
        // NEW - DUAS OU MAIS APRILTAGS
        // ========================================================
        //
        // Melhor geometria para localização.
        //
        // ========================================================

        if (tagCount >= 2) {


            // ----------------------------------------------------
            // NEW - TAGS MUITO PRÓXIMAS
            // ----------------------------------------------------

            if (avgTagDist <= 2.0) {

                return 0.25;

            }


            // ----------------------------------------------------
            // NEW - DISTÂNCIA MÉDIA
            // ----------------------------------------------------

            if (avgTagDist <= 3.0) {

                return 0.40;

            }


            // ----------------------------------------------------
            // NEW - PRÓXIMO DO LIMITE DE VISÃO
            // ----------------------------------------------------

            return 0.60;

        }


        // ========================================================
        // NEW - SOMENTE UMA APRILTAG
        // ========================================================
        //
        // Ainda utilizamos a medição,
        // porém com menor confiança.
        //
        // ========================================================


        // --------------------------------------------------------
        // NEW - UMA TAG PRÓXIMA
        // --------------------------------------------------------

        if (avgTagDist <= 2.0) {

            return 0.60;

        }


        // --------------------------------------------------------
        // NEW - UMA TAG EM DISTÂNCIA MÉDIA
        // --------------------------------------------------------

        if (avgTagDist <= 3.0) {

            return 0.85;

        }


        // --------------------------------------------------------
        // NEW - UMA TAG DISTANTE
        // --------------------------------------------------------

        return 1.20;

    }

    // #endregion



    // #region PERIODIC

    @Override
    public void periodic() {


        // #region LEITURA DA LIMELIGHT


        // ========================================================
        // VERIFICA SE EXISTE ALVO
        // ========================================================

        boolean hasTarget =

            LimelightHelpers.getTV(

                LimelightConstants.REAR_NAME

            );


        // ========================================================
        // ÂNGULO HORIZONTAL
        // ========================================================

        double tx =

            LimelightHelpers.getTX(

                LimelightConstants.REAR_NAME

            );


        // ========================================================
        // ÂNGULO VERTICAL
        // ========================================================

        double ty =

            LimelightHelpers.getTY(

                LimelightConstants.REAR_NAME

            );


        // ========================================================
        // ÁREA DO ALVO NA IMAGEM
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


        // ========================================================
        // ORIENTAÇÃO DO ROBÔ - PIGEON 2
        // ========================================================

        double robotYawDegrees =

            drive
                .getRotation()
                .getDegrees();


        // ========================================================
        // NEW - VELOCIDADE ANGULAR DO ROBÔ
        // ========================================================

        double robotYawRateDegPerSec =

            drive.getYawVelocityDegreesPerSecond();


        // ========================================================
        // NEW - VALIDAÇÃO DA VELOCIDADE ANGULAR
        // ========================================================
        //
        // Em rotação muito rápida a imagem pode apresentar
        // maior borrão e a pose visual perde confiabilidade.
        //
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
        // ENVIA A ORIENTAÇÃO DO ROBÔ PARA O MEGATAG2
        // ========================================================
        //
        // MegaTag2 utiliza o heading fornecido pelo Pigeon 2.
        //
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


        // ========================================================
        // POSE DO ROBÔ CALCULADA PELO MEGATAG2
        // ========================================================

        LimelightHelpers.PoseEstimate megaTag2Estimate =

            LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(

                    LimelightConstants.REAR_NAME

                );


        // ========================================================
        // VALIDAÇÃO DO MEGATAG2
        // ========================================================
        //
        // Uma medição será considerada válida quando:
        //
        // 1 - existe uma estimativa
        // 2 - existe uma pose
        // 3 - existe pelo menos uma AprilTag
        // 4 - distância média está dentro do limite
        // 5 - velocidade angular está dentro do limite
        //
        // ========================================================

        boolean megaTag2Valid =

            megaTag2Estimate != null

            && megaTag2Estimate.pose != null

            && megaTag2Estimate.tagCount > 0

            && Double.isFinite(
                megaTag2Estimate.avgTagDist
            )

            && megaTag2Estimate.avgTagDist
                < LimelightConstants
                    .MAX_TAG_DISTANCE_METERS

            && rotationRateValid;


        // Estado atual da visão

        visionValidNow =
            megaTag2Valid;


        // ========================================================
        // VERIFICA SE CHEGOU UMA NOVA MEDIÇÃO
        // ========================================================
        //
        // Só processamos o frame se o timestamp avançou.
        //
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
        // NEW - PROCESSAMENTO DE UMA NOVA MEDIÇÃO
        // ========================================================
        //
        // Toda a fusão fica centralizada neste único bloco.
        //
        // Isso garante que:
        //
        // - um frame seja processado somente uma vez
        // - não exista dupla fusão
        // - a confiança dinâmica seja aplicada corretamente
        //
        // ========================================================

        if (newMegaTag2Measurement) {


            // ====================================================
            // NEW - ATUALIZA TIMESTAMP
            // ====================================================

            lastMegaTag2TimestampSeconds =

                megaTag2Estimate.timestampSeconds;


            // ====================================================
            // NEW - ARMAZENA ÚLTIMA POSE VÁLIDA
            // ====================================================

            latestValidVisionPose =

                megaTag2Estimate.pose;


            latestValidTagCount =

                megaTag2Estimate.tagCount;


            lastValidFrameTime =

                Timer.getFPGATimestamp();


            // ====================================================
            // NEW - DIAGNÓSTICO DE SALTO DA POSE VISUAL
            // ====================================================
            //
            // Mede a distância entre:
            //
            // - pose atual estimada pelo robô
            // - pose informada pelo MegaTag2
            //
            // POR ENQUANTO:
            //
            // NÃO rejeitamos nenhuma medição.
            //
            // Apenas registramos o erro para descobrir
            // qual é o comportamento normal do nosso robô.
            //
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
            // NEW - LOG DO ERRO DA VISÃO
            // ====================================================

            Logger.recordOutput(

                "Vision/Rear/MegaTag2/PoseErrorMeters",

                visionPoseErrorMeters

            );



            // ====================================================
            // NEW - CALCULA A CONFIANÇA DINÂMICA
            // ====================================================

            double visionXYStdDev =

                calculateVisionXYStdDev(

                    megaTag2Estimate.tagCount,

                    megaTag2Estimate.avgTagDist

                );


            // ====================================================
            // NEW - FUSÃO MEGATAG2 + POSE ESTIMATOR
            // ====================================================
            //
            // X / Y:
            //
            // confiança calculada dinamicamente.
            //
            // Theta:
            //
            // continua praticamente confiado ao Pigeon 2
            // dentro do Drive.java.
            //
            // ====================================================

            drive.addVisionMeasurement(

                megaTag2Estimate.pose,

                megaTag2Estimate.timestampSeconds,

                visionXYStdDev

            );


            // ====================================================
            // NEW - LOG DA CONFIANÇA
            // ====================================================

            Logger.recordOutput(

                "Vision/Rear/MegaTag2/XYStdDevMeters",

                visionXYStdDev

            );

        }


        // #endregion



        // #region PRONTIDÃO PARA SEED


        // ========================================================
        // TEMPO DESDE A ÚLTIMA CAPTURA VÁLIDA
        // ========================================================

        double frameAgeSeconds =

            lastValidFrameTime >= 0.0

                ? Timer.getFPGATimestamp()
                    - lastValidFrameTime

                : -1.0;


        // ========================================================
        // VERIFICA SE A MEDIÇÃO ESTÁ PRONTA PARA SEED
        // ========================================================
        //
        // Esse valor serve apenas como diagnóstico.
        //
        // O método trySeedDrivePose() ainda verifica
        // adicionalmente se o robô está parado.
        //
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
        // ESTADO GERAL DA LIMELIGHT
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
        // MEGATAG2 - VALIDAÇÃO
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
        // MEGATAG2 - QUANTIDADE DE TAGS
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/TagCount",

            megaTag2Estimate != null

                ? megaTag2Estimate.tagCount

                : 0

        );


        // ========================================================
        // MEGATAG2 - DISTÂNCIA MÉDIA
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/AvgTagDist",

            megaTag2Estimate != null

                ? megaTag2Estimate.avgTagDist

                : 0.0

        );


        // ========================================================
        // MEGATAG2 - TIMESTAMP
        // ========================================================

        Logger.recordOutput(

            "Vision/Rear/MegaTag2/TimestampSeconds",

            megaTag2Estimate != null

                ? megaTag2Estimate.timestampSeconds

                : 0.0

        );


        // ========================================================
        // NEW - VELOCIDADE ANGULAR
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
        // MEGATAG2 - POSE VÁLIDA
        // ========================================================
        //
        // Só atualizamos esses logs quando a medição atual
        // é válida.
        //
        // Se a visão for perdida, a última pose visual
        // permanece disponível no AdvantageScope.
        //
        // ========================================================

        if (megaTag2Valid) {


            Logger.recordOutput(

                "Vision/Rear/MegaTag2/Pose",

                megaTag2Estimate.pose

            );


            Logger.recordOutput(

                "Vision/Rear/MegaTag2/X",

                megaTag2Estimate.pose
                    .getX()

            );


            Logger.recordOutput(

                "Vision/Rear/MegaTag2/Y",

                megaTag2Estimate.pose
                    .getY()

            );


            Logger.recordOutput(

                "Vision/Rear/MegaTag2/RotationDegrees",

                megaTag2Estimate.pose
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