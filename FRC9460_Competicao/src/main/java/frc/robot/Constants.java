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
    public static final double kWheelBaseMeters = 0.58;

    // Distância entre os módulos da esquerda e direita
    // Medida de centro a centro dos módulos
    public static final double kTrackWidthMeters = 0.54;

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
  
    // #endregion
}
