// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    //Porta onde controle está do xBox
    public static final int kDriverControllerPort = 0;
  }


    // #region CONSTANTES DO SWERVE / DRIVETRAIN

   // ============================================================
   // CONSTANTES DO SWERVE / DRIVETRAIN
   // ============================================================

  public static class DriveConstants {


    // #region GEOMETRIA DO ROBÔ

    // Distância entre os módulos dianteiros e traseiros
    // Medida de centro a centro dos módulos
    public static final double kWheelBaseMeters =
    0.5461;

    // Distância entre os módulos da esquerda e direita
    // Medida de centro a centro dos módulos
    public static final double kTrackWidthMeters =
    0.5461;

    // Raio da roda do módulo Swerve
    public static final double kWheelRadiusMeters = 0.0508;

    // #endregion


    // #region LIMITES DE VELOCIDADE

    // Velocidade máxima usada inicialmente na simulação
    public static final double kMaxLinearSpeed = 2.0; // m/s

    // Velocidade máxima de rotação
    public static final double kMaxAngularSpeed = Math.PI; // rad/s
    // #endregion


  // ============================================================
  // CONTROLE DO DRIVE
  // ============================================================

  // #region PID DO DRIVE

  // PID de velocidade do Drive
  public static final double kDriveKp = 1.0;
  public static final double kDriveKi = 0.0;
  public static final double kDriveKd = 0.0;


  // Feedforward do Drive

  // Tensão mínima aproximada para começar a movimentar
  // Valor inicial para a simulação
  public static final double kDriveKs = 0.2;

  // Aproximação inicial baseada em:
  // 12 V / 4.58 m/s
  public static final double kDriveKv =
      12.0 / 4.58;
  
  // #endregion
  


    }


  //#region Limelight

  public static class LimelightConstants {
  // ============================================================
  // Lime Ligh Constants
  // ============================================================

      public static final String REAR_NAME = "limelight-rear";

      public static final int[] VALID_TAG_IDS = {25, 26, 9, 10};

      public static final int IMU_MODE = 0;

      public static final double REAR_FORWARD = -0.29;
      public static final double REAR_RIGHT = 0.16;
      public static final double REAR_UP = 0.46;

      public static final double REAR_ROLL = 0.0;
      public static final double REAR_PITCH = 4.0;
      public static final double REAR_YAW = 180.0;

    // ============================================================
    // VALIDAÇÃO DO MEGATAG2
    // ============================================================

    // Distância máxima média das AprilTags
    // para aceitarmos uma medição de visão.
    public static final double MAX_TAG_DISTANCE_METERS =4.0;


    // ============================================================
    // LIMITE DE VELOCIDADE ANGULAR PARA VISÃO
    // ============================================================
    //
    // Acima desse valor a imagem pode sofrer blur durante
    // a rotação e a pose do MegaTag2 fica menos confiável.
    //
    // Unidade: graus por segundo.
    //
    // ============================================================

   
   //720 o que usamos para testes
    public static final double MAX_VISION_YAW_RATE_DEG_PER_SEC =
        720.0;

  }
  
  //#region
  
    // #endregion
}










//=====================================Anotações
//ROBOT CONFIG  = PathPlanner


//Limite tenção Bateria
//As vezes a bateria esta com 13v ou 12 então manda mais doque 12

//slowrate -- Swerve Função quadratica


//Optimize Cocene scale


//================Limelight
//Confiar menos no giroscopio da camera usar o pision
//Pega somente X e Y
//Despreza a rotação
//Usar bloco Try Cath Exception = Limelight 
//Constantes no codigo
//MegaTag 1 não precisa do pision
//Megata 2 precisa do pision sexta posicao e o angulo
//Enquanto for falso que esta vendo a limelight ele fica procurando 
//Quando acha deixa de procurar pega os valores dela
//TimeStamp volta no tempo

//=================Posicao da arena e robo
//Yaw 180 uma alinaça 0 arena uim
//Teste para robo fica lado certo da arena




//=====Sistematiza 
//Monitoramento de Log




//==========Perguntar
//Cabibrar
//Picos Voltagem
//Alinhar quando começa jogar as rodas na posicao 0


//Paphplanner
//Tem como passar o x e y e ele desviar dos objetos automaticamente 
//Ele gera um json com as config do robo peso medidas
//PaphToPose mandar robo por coordenadas 
//PID de trajetoria do robo = robot config


//Java Teste de unidades