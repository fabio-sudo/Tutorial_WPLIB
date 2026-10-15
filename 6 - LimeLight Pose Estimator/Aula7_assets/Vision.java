package frc.robot.subsystems.vision;


// #region IMPORTS

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

import frc.robot.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

// #endregion



public class Vision extends SubsystemBase {


    // #region DRIVE

    private final Drive drive;

    // #endregion


    // #region POSE INICIAL - MEGATAG2

    private Pose2d latestValidVisionPose = null;

    private boolean visionValidNow = false;

    private double lastValidFrameTime = -1.0;

    // Quantidade de tags utilizadas na última captura válida
    private int latestValidTagCount = 0;

    // #endregion

    // #region CONTROLE DE NOVAS MEDIÇÕES

    // Guarda o timestamp da última medição válida observada.
    // Impede processar duas vezes o mesmo resultado.

    private double lastMegaTag2TimestampSeconds = -1.0;

    // #endregion


    // #region CONSTRUTOR

    public Vision(Drive drive) {
        
        //===========================================DRIVE
        this.drive = drive;

        // ========================================================
        // POSIÇÃO FÍSICA DA LIMELIGHT NO ROBÔ
        // ========================================================
        //
        // Câmera utilizada:
        //
        // Limelight traseira
        //
        // Os valores estão armazenados no Constants.java.
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
        // Atualmente nossa arena de teste utiliza:
        //
        // Tag 25
        // Tag 26
        //
        // Qualquer outra AprilTag será ignorada pela Limelight
        // para localização.
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

        public void trySeedDrivePose() {

            // Tempo desde a última medição válida
            double frameAgeSeconds =

                lastValidFrameTime >= 0.0
                    ? Timer.getFPGATimestamp() - lastValidFrameTime
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


            // Se alguma condição falhar, não sincroniza.
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
        //
        // Negativo = alvo para um lado
        // Positivo = alvo para o outro
        //
        // Vamos validar fisicamente o sentido depois.
        //
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
        // ID DA APRILTAG
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
                // ENVIA A ORIENTAÇÃO DO ROBÔ PARA A LIMELIGHT
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
        // Só consideramos a medição válida quando:
        //
        // 1 - existe pelo menos uma AprilTag
        // 2 - a distância média das tags é menor que 4 metros
        //
        // Isso impede que uma pose inválida (0,0,0)
        // seja utilizada quando a Limelight perde as tags.
        //
        // ========================================================

        boolean megaTag2Valid =

            megaTag2Estimate != null

            && megaTag2Estimate.pose != null

            && megaTag2Estimate.tagCount > 0

            && megaTag2Estimate.avgTagDist
                < LimelightConstants.MAX_TAG_DISTANCE_METERS;


            // ========================================================
            // VERIFICA SE CHEGOU UMA NOVA MEDIÇÃO
            // ========================================================

            boolean newMegaTag2Measurement =

                megaTag2Valid

                && Double.isFinite(
                    megaTag2Estimate.timestampSeconds
                )

                && megaTag2Estimate.timestampSeconds > 0.0

                && megaTag2Estimate.timestampSeconds
                    > lastMegaTag2TimestampSeconds;


            // Atualiza somente quando o timestamp avançou.
            if (newMegaTag2Measurement) {

                lastMegaTag2TimestampSeconds =
                    megaTag2Estimate.timestampSeconds;
            }

            // ========================================================
            // ARMAZENA A ÚLTIMA MEDIÇÃO VÁLIDA E RECENTE
            // ========================================================

            // Estado atual da visão
            visionValidNow = megaTag2Valid;

            if (newMegaTag2Measurement) {

                latestValidVisionPose =
                    megaTag2Estimate.pose;

                latestValidTagCount =
                    megaTag2Estimate.tagCount;

                lastValidFrameTime =
                    Timer.getFPGATimestamp();
            }

// ========================================================
// TESTE - PRONTIDÃO PARA SINCRONIZAÇÃO
// ========================================================

// Tempo decorrido desde a última captura válida
double frameAgeSeconds =

    lastValidFrameTime >= 0.0
        ? Timer.getFPGATimestamp() - lastValidFrameTime
        : -1.0;

        // ========================================================
        // VERIFICA SE A MEDIÇÃO ESTÁ PRONTA PARA INICIALIZAÇÃO
        // ========================================================

        boolean seedReady =

            visionValidNow

            && latestValidVisionPose != null

            // Exige pelo menos duas AprilTags
            && latestValidTagCount >= 2

            // Verifica se já recebemos uma captura válida
            && lastValidFrameTime >= 0.0

            // Verifica se o tempo é válido
            && frameAgeSeconds >= 0.0

            // A captura deve ter no máximo 250 ms
            && frameAgeSeconds <= 0.25;

            // ========================================================
            // LOGS - ADVANTAGEKIT
            // ========================================================

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/FrameAgeSeconds",
                frameAgeSeconds
            );

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/SeedReady",
                seedReady
            );


    // #endregion
    
    
    // #region ADVANTAGEKIT


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
            // MEGATAG2 - STATUS DA MEDIÇÃO
            // ========================================================


            // Indica se a medição atual pode ser utilizada
            Logger.recordOutput(
                "Vision/Rear/MegaTag2/Valid",
                megaTag2Valid
            );


            // Quantidade de AprilTags utilizadas
            Logger.recordOutput(
                "Vision/Rear/MegaTag2/TagCount",

                megaTag2Estimate != null
                    ? megaTag2Estimate.tagCount
                    : 0
            );


            // Distância média das AprilTags detectadas
            Logger.recordOutput(
                "Vision/Rear/MegaTag2/AvgTagDist",

                megaTag2Estimate != null
                    ? megaTag2Estimate.avgTagDist
                    : 0.0
            );
            // ========================================================
            // MEGATAG2 - TIMESTAMP DA MEDIÇÃO
            // ========================================================
            //
            // Registra o instante estimado de captura da medição.
            //
            // Será utilizado para compensar a latência da Limelight
            // quando ativarmos a fusão com o PoseEstimator.
            //
            // ========================================================

            Logger.recordOutput(
                "Vision/Rear/MegaTag2/TimestampSeconds",

                megaTag2Estimate != null
                    ? megaTag2Estimate.timestampSeconds
                    : 0.0
            );

            // ========================================================
            // MEGATAG2 - POSE VÁLIDA
            // ========================================================
            //
            // A pose só é publicada quando a medição é válida.
            //
            // Quando perdemos as AprilTags:
            //
            // Valid     = false
            // TagCount  = 0
            //
            // MAS:
            //
            // Pose
            // X
            // Y
            // Rotation
            //
            // permanecem na última medição válida.
            //
            // ========================================================

            if (megaTag2Valid) {


                Logger.recordOutput(
                    "Vision/Rear/MegaTag2/Pose",
                    megaTag2Estimate.pose
                );


                Logger.recordOutput(
                    "Vision/Rear/MegaTag2/X",
                    megaTag2Estimate.pose.getX()
                );


                Logger.recordOutput(
                    "Vision/Rear/MegaTag2/Y",
                    megaTag2Estimate.pose.getY()
                );


                Logger.recordOutput(
                    "Vision/Rear/MegaTag2/RotationDegrees",

                    megaTag2Estimate.pose
                        .getRotation()
                        .getDegrees()
                );
            }


            Logger.recordOutput(
            "Vision/Rear/MegaTag2/NewMeasurement",
            newMegaTag2Measurement
        );

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

        return (int) LimelightHelpers.getFiducialID(
            LimelightConstants.REAR_NAME
        );
    }

    // #endregion
}