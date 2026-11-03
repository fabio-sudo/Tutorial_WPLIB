// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// ============================================================
// ADVANTAGEKIT - LOGGING E TELEMETRIA
// ============================================================
import java.time.LocalDateTime;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
public Robot() {

    // ============================================================
    // ADVANTAGEKIT
    // ============================================================

    // Informações que ficarão gravadas nos logs
    Logger.recordMetadata("ProjectName", "FRC9460");
    var buildInfo = LoggingSetup.buildInfo();
    Logger.recordMetadata("SourceSHA256", buildInfo.getProperty("SourceSHA256", "DESCONHECIDO"));
    Logger.recordMetadata("BuiltAtUTC", buildInfo.getProperty("BuiltAtUTC", "DESCONHECIDO"));

    LoggingConstants.Mode mode = LoggingSetup.getMode();
    Logger.recordMetadata("RunMode", mode.name());

    if (mode == LoggingConstants.Mode.REAL) {

        // ROBÔ REAL
        // USB primeiro; fallback interno com reserva de espaço e tamanho limitado.
        Logger.addDataReceiver(new frc.robot.util.RobotLogWriter());

        // O Phoenix também gravava .hoot sozinho em /home/lvuser/logs. No teste de
        // 29/10/2026 o disco do roboRIO estava cheio ("Available disk space (0 MB)").
        // O AdvantageKit grava os inputs/outputs instrumentados, não todos os
        // sinais Phoenix em alta frequência. Um SignalLogger.start() explícito
        // (SysId do Phoenix) continua funcionando.
        SignalLogger.enableAutoLogging(false);

        // Envia os dados pela NetworkTables
        // Permite visualizar no AdvantageScope em tempo real
        Logger.addDataReceiver(new NT4Publisher());

    } else if (mode == LoggingConstants.Mode.SIM) {

        // SIMULAÇÃO
        // Guarda também os inputs: depois poderemos recalcular a fusão
        // sem executar novamente o mundo óptico ou movimentar o controle.
        // Arquivo: logs/simulacao/Simulacao_aa-MM-dd_HH-mm-ss.wpilog.
        Logger.addDataReceiver(new WPILOGWriter(
            LoggingSetup.simulationLogPath(LocalDateTime.now()),
            WPILOGWriter.AdvantageScopeOpenBehavior.NEVER));
        // Envia os dados da simulação para o AdvantageScope.
        Logger.addDataReceiver(new NT4Publisher());
    } else {
        // REPLAY: usa o relógio e os sensores gravados. Não cria as câmeras
        // PhotonLib, dispositivos CAN ou sensores de motores simulados.
        setUseTiming(false);
        String logPath = LoggingSetup.requireReplayLog(
            LogFileUtil.findReplayLog()).toString();
        Logger.recordMetadata("ReplaySource", logPath);
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(
            LogFileUtil.addPathSuffix(logPath, LoggingConstants.REPLAY_OUTPUT_SUFFIX)));
    }

    // Inicia o sistema de logging do AdvantageKit
    Logger.start();

    // ELASTIC: serve os painéis na porta 5800 (porta liberada pela FRC).
    // No Elastic: File > Download Layout baixa os painéis direto do robô ou
    // da simulação. Ficam em config/elastic/layouts; o deploy copia para o robô.
    // Gerados por config/elastic/preparar_layouts_elastic.py.
    if (mode != LoggingConstants.Mode.REPLAY) {
        WebServer.start(5800, LoggingSetup.elasticLayoutsDirectory(
            mode, Filesystem.getDeployDirectory().toPath()).toString());
    }

    // IMPORTANTE:
    // RobotContainer deve ser criado DEPOIS do Logger.start()
    m_robotContainer = new RobotContainer();
}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
@Override
public void simulationPeriodic() {
    // Produz as próximas leituras das câmeras virtuais.
    m_robotContainer.updateVisionSimulation();
    // Move as bolas: coleta, lançamento e voo.
    m_robotContainer.updateFuelSimulation();
}

}
