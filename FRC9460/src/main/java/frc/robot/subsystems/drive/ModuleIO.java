package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    // ============================================================
   // DADOS DE ENTRADA DO MÓDULO
  // ============================================================ 
  @AutoLog
  public static class ModuleIOInputs{

    // Posição percorrida pela roda
    public double drivePositionRad = 0.0;

    // Velocidade de rotação da roda
    public double driveVelocityRadPerSec = 0.0;

    // Tensão aplicada no motor de tração
    public double driveAppliedVolts = 0.0;

    // Corrente consumida pelo motor de tração
    public double driveCurrentAmps = 0.0;


    // Ângulo atual do módulo
    public double turnPositionRad = 0.0;

    // Velocidade de rotação do steer
    public double turnVelocityRadPerSec = 0.0;

    // Tensão aplicada no motor de direção
    public double turnAppliedVolts = 0.0;

    // Corrente consumida pelo motor de direção
    public double turnCurrentAmps = 0.0;

  }


  // ============================================================
  // ATUALIZAÇÃO DOS SENSORES
  // ============================================================

  default void updateInputs(ModuleIOInputs inputs) {}


  // ============================================================
  // COMANDOS PARA O DRIVE MOTOR
  // ============================================================

  default void setDriveVoltage(double volts) {}


  // ============================================================
  // COMANDOS PARA O STEER MOTOR
  // ============================================================

  default void setTurnVoltage(double volts) {}


// ============================================================
// MALHA FECHADA - DRIVE
// ============================================================

default void setDriveVelocity(
    double velocityMetersPerSecond
) {}


// ============================================================
// MALHA FECHADA - STEER
// ============================================================

default void setTurnPosition(
    double angleRadians
) {}
}