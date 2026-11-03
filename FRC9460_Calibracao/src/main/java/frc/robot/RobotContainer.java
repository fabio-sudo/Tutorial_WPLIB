// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// #region IMPORTS

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.intake.ColetarCommand;
import frc.robot.commands.telescopio.TelescopioAbrirCommand;
import frc.robot.commands.telescopio.TelescopioFecharCommand;
import frc.robot.commands.shooter.ShooterAimAtHubCommand;
import frc.robot.commands.shooter.ShooterTrackAndShootCommand;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.telescopio.Telescopio;
import frc.robot.subsystems.telescopio.TelescopioDashboard;
// import frc.robot.subsystems.telescopio_9458.Telescopio_9458; // telescópio 9458 fora de uso
// import frc.robot.subsystems.telescopio_9458.TelescopioDashboard_9458; // telescópio 9458 fora de uso
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterDashboard;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.centrifuga.Centrifuga;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.fuel.FuelConstants;
import frc.robot.subsystems.fuel.FuelDashboard;
import frc.robot.subsystems.fuel.FuelRobotState;
import frc.robot.subsystems.fuel.FuelSimulation;
import frc.robot.subsystems.fuel.FuelTracker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.drive.DriveConstants.kMaxAngularSpeed;
import static frc.robot.subsystems.drive.DriveConstants.kMaxLinearSpeed;
import static frc.robot.subsystems.drive.DriveConstants.kDriverLinearAcceleration;


// Testes automáticos de validação
import frc.robot.calibration.DriveTwoMetersCommand;
import frc.robot.calibration.Rotate360Command;

import frc.robot.calibration.DriveSysId;
// import frc.robot.calibration.Telescopio9458SysId; // telescópio 9458 fora de uso

// 🆕 Comandos de calibração do Swerve
import frc.robot.calibration.SwerveCalibrationCommands;

//CALIBRAÇÃO
import frc.robot.calibration.CalibrationCommands;

//Curva de Aceleração e Suavisação
import edu.wpi.first.math.filter.SlewRateLimiter;

//Limelight Vision
import frc.robot.subsystems.vision.Vision;


//Virtualização limelight
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.subsystems.vision.VisionSimulationScenario;

// #endregion



public class RobotContainer {
    private final frc.robot.util.RobotHealth robotHealth = new frc.robot.util.RobotHealth();

    // #region SUBSYSTEMS

    // Drivetrain Swerve
    private final Drive drive =
        new Drive();

    // Simulação e replay do telescópio; hardware real permanece sem configuração CAN.
    private final Telescopio telescopio = new Telescopio(Telescopio.createIO());
    private final TelescopioDashboard telescopioDashboard =
        new TelescopioDashboard(telescopio);

    // ------------------------------------------------------------
    // ANTIGO (comentado em 28/10/2026): telescópio 9458 fora de uso por enquanto.
    // Comentado, ele não roda no periodic, não cria o TalonFX CAN 5 no robô
    // 9460 nem publica no painel. Para voltar, descomente este bloco, o SysId
    // dele (mais abaixo) e os imports.
    // ------------------------------------------------------------
    // Telescópio 9458 em bancada: TalonFX CAN 5 no real, simulação física no desktop.
    // private final Telescopio_9458 telescopio9458 = new Telescopio_9458(Telescopio_9458.createIO());
    // private final TelescopioDashboard_9458 telescopio9458Dashboard =
    //     new TelescopioDashboard_9458(telescopio9458);

    
        // ============================================================
        // CENÁRIO DE VISÃO PARA SIMULAÇÃO
        // ============================================================

        // Cria o cenário somente na simulação normal.
        // No replay, as entradas virão do arquivo gravado.
        private final VisionSimulationScenario visionSimulation =
            RobotBase.isSimulation() && !Logger.hasReplaySource()
                ? new VisionSimulationScenario()
                : null;

        // Na simulação, usamos explicitamente as duas câmeras virtuais.
        // Nas demais execuções, usamos a configuração existente do Vision.
        private final Vision vision =
            visionSimulation != null
                ? new Vision(drive, visionSimulation.getCameras())
                : new Vision(drive);

    // ============================================================
    // CICLO DA BOLA: INTAKE -> ROLLER -> CENTRÍFUGA -> SHOOTER
    // ============================================================

    // Intake (abre/fecha) e rolete (puxa a bola): pastas intake e roller.
    private final Intake intake = new Intake(Intake.createIO());
    private final Roller roller = new Roller(Roller.createIO());

    // Centrífuga: empurra a bola até a tower. Pasta centrifuga.
    private final Centrifuga centrifuga = new Centrifuga(Centrifuga.createIO());

    // Shooter: mira no hub com a pose do Drive (odometria + Limelight).
    // Só atira depois que a Limelight corrigiu a pose ao menos uma vez.
    private final Shooter shooter = new Shooter(
        Shooter.createPivotIO(),
        Shooter.createTopIO(),
        Shooter.createTowerIO(),
        drive::getPose,
        drive::getFieldRelativeSpeeds,
        drive::hasVisionCorrectionSinceReset);
    private final ShooterDashboard shooterDashboard = new ShooterDashboard(shooter,
        () -> new ShooterTrackAndShootCommand(shooter, centrifuga, vision::getDiagnosis),
        () -> new ShooterAimAtHubCommand(shooter));

    // Chave da mira automática, ligada e desligada pelo B do piloto.
    // Desliga sozinha quando o robô é desabilitado.
    // private boolean autoAimEnabled = false; // antigo toggle B, arquivado

    // O que as bolas precisam saber do robô (ligação entre os subsistemas).
    private final FuelRobotState fuelRobotState = new FuelRobotState() {
        @Override public Pose2d pose() { return drive.getPose(); }
        @Override public ChassisSpeeds fieldSpeeds() { return drive.getFieldRelativeSpeeds(); }
        // Simulação: a referência sem erro de visão. Robô real e replay: a estimada.
        @Override public Pose2d physicalPose() {
            return drive.isSimulationReferenceReady() ? drive.getSimulationPose() : drive.getPose();
        }
        // Visão no momento de cada tiro (Fuel/Tiro/Visao).
        @Override public boolean visionCorrectedSinceReset() { return drive.hasVisionCorrectionSinceReset(); }
        @Override public boolean visionCorrectionRecent() { return drive.hasRecentVisionCorrection(); }
        @Override public double visionCorrectionAgeSeconds() { return drive.getVisionCorrectionAgeSeconds(); }
        @Override public String lastVisionCamera() { return drive.getLastVisionCamera(); }
        @Override public boolean intakeReadyToCollect() {
            return intake.getRaiseDegrees() <= FuelConstants.INTAKE_COLLECT_MAX_RAISE_DEGREES;
        }
        @Override public boolean rollerPulling() { return roller.isPulling(); }
        @Override public double rollerRpm() { return roller.getRpm(); }
        @Override public double rollerCurrentAmps() { return roller.getCurrentAmps(); }
        @Override public double topRpm() { return shooter.getTopRpm(); }
        @Override public boolean topCommanded() { return shooter.getTopTargetRpm() > 0.0; }
        @Override public double towerRpm() { return shooter.getTowerRpm(); }
        @Override public double centrifugeRpm() { return centrifuga.getRpm(); }
        @Override public boolean centrifugeRunning() { return centrifuga.isRunning(); }
        @Override public Translation2d shooterFieldPosition() {
            return ShotCalculator.shooterFieldPosition(drive.getPose());
        }
        @Override public double shooterHeadingDegrees() {
            return ShotCalculator.shooterFieldHeadingDegrees(drive.getPose(), shooter.getPivotPhysicalDegrees());
        }
        // Valores planejados (o que o código pediu) para comparar cada tiro.
        @Override public double topTargetRpm() { return shooter.getTopTargetRpm(); }
        @Override public double towerTargetRpm() { return shooter.getTowerTargetRpm(); }
        @Override public double centrifugeTargetRpm() { return centrifuga.getTargetRpm(); }
        @Override public double plannedShooterHeadingDegrees() {
            return ShotCalculator.shooterFieldHeadingDegrees(drive.getPose(), shooter.getPivotTargetDegrees());
        }
        @Override public double encoderShooterHeadingDegrees() {
            return ShotCalculator.shooterFieldHeadingDegrees(drive.getPose(), shooter.getPivotDegrees());
        }
        // Alvo, distância e erro de mira no disparo (Fuel/Consistencia).
        @Override public Translation2d hubTarget() { return shooter.getHubTarget(); }
        @Override public double hubDistanceMeters() { return shooter.getHubDistanceMeters(); }
        @Override public double aimErrorDegrees() { return shooter.getAimErrorDegrees(); }
        @Override public void onBallCollected() { roller.simulateBallIntake(); }
        @Override public void onBallLaunched() { shooter.simulateBallLeaving(); }
    };

    // Conta coletas e disparos pelos sensores: robô real, simulação e replay.
    private final FuelTracker fuelTracker = new FuelTracker(fuelRobotState);

    // Botões para a equipe anotar onde a bola chegou (robô real).
    private final FuelDashboard fuelDashboard = new FuelDashboard(fuelTracker::getShots);

    // Bolas no campo, no robô e no ar: só na simulação normal.
    private final FuelSimulation fuelSimulation =
        RobotBase.isSimulation() && !Logger.hasReplaySource()
            ? new FuelSimulation(fuelRobotState)
            : null;

    // Peças móveis do modelo 3D (intake e shooter), registradas por último
    // para desenhar os ângulos já atualizados neste ciclo.
    private final RobotComponentsVisualizer componentsVisualizer =
        new RobotComponentsVisualizer(intake::getRaiseDegrees, shooter::getPivotDegrees,
            intake::getModeledRaiseDegrees, shooter::getModeledPivotDegrees);

    // #endregion

    // ============================================================
    // SYSID DO DRIVE
    // ============================================================

    private final DriveSysId driveSysId =
        new DriveSysId(
            drive
        );

    // ============================================================
    // SYSID DO TELESCÓPIO 9458
    // ============================================================

    // private final Telescopio9458SysId telescopio9458SysId =
    //     new Telescopio9458SysId(
    //         telescopio9458
    //     );


    // #region 🎮 CONTROLE DO MOTORISTA

    // Controle Xbox conectado na porta definida
    private final CommandXboxController m_driverController =
        new CommandXboxController(
            OperatorConstants.kDriverControllerPort
        );

    // Controle do operador (porta 1): shooter. O controle do piloto
    // já usa quase todos os botões no Drive, telescópios e calibração.
    private final CommandXboxController m_operatorController =
        new frc.robot.controls.OptionalOperatorController(
            OperatorConstants.kOperatorControllerPort
        );


    // ============================================================
    // 🚀 SUAVIZAÇÃO DO SWERVE: limita a aceleração pedida pelo piloto
    // ============================================================

    // Frente / trás
    private final SlewRateLimiter vxLimiter =
        new SlewRateLimiter(kDriverLinearAcceleration / kMaxLinearSpeed);

    // Movimento lateral
    private final SlewRateLimiter vyLimiter =
        new SlewRateLimiter(kDriverLinearAcceleration / kMaxLinearSpeed);

    // 🚀 2,0/s = do parado ao giro máximo em 0,5 s.
    // Rotação
    private final SlewRateLimiter omegaLimiter =
        new SlewRateLimiter(2.0);


    // #endregion


    // #region 🎮 MODO DE DIREÇÃO

        // true  = Field Relative
        // false = Robot Relative
        private boolean fieldRelativeEnabled = true;

    // #endregion
    

    // #region CONSTRUTOR

    /**
     * O RobotContainer contém:
     *
     * - Subsistemas
     * - Controle
     * - Comandos
     * - Bindings
     */
    public RobotContainer() {


        // ============================================================
        // COMANDO PADRÃO DO SWERVE
        // 🎮 Analógico esquerdo anda, direito gira. Zona morta de 10% e curva quadrática.
        // 🛑 Fora do teleop habilitado o comando zera o chassi e os limitadores.
        // ============================================================

        drive.setDefaultCommand(

            drive.runEnd(

                () -> {
        if (!DriverStation.isTeleopEnabled()) {
            drive.stop();
            resetDriverLimiters();
            return;
        }
        // ====================================================
        // MOVIMENTO PARA FRENTE / TRÁS
        // ====================================================

        double vxInput =
            -MathUtil.applyDeadband(

                m_driverController.getLeftY(),

                0.10

            );

        // Curva quadrática para deixar o centro mais suave
        vxInput =
            Math.copySign(
                vxInput * vxInput,
                vxInput
            );

        // 🚀 m/s = entrada do piloto × kMaxLinearSpeed (DriveConstants).
        double vx =
            vxLimiter.calculate(vxInput)
                * kMaxLinearSpeed;



        // ====================================================
        // MOVIMENTO LATERAL
        // ====================================================

        double vyInput =
            -MathUtil.applyDeadband(

                m_driverController.getLeftX(),

                0.10

            );

        // Curva quadrática
        vyInput =
            Math.copySign(
                vyInput * vyInput,
                vyInput
            );

        double vy =
            vyLimiter.calculate(vyInput)
                * kMaxLinearSpeed;



        // ====================================================
        // ROTAÇÃO DO ROBÔ
        // ====================================================

        double omegaInput =
            -MathUtil.applyDeadband(

                m_driverController.getRightX(),

                0.10

            );

        // Curva quadrática
        omegaInput =
            Math.copySign(
                omegaInput * omegaInput,
                omegaInput
            );

        // 🚀 rad/s = entrada do piloto × kMaxAngularSpeed (DriveConstants).
        double omega =
            omegaLimiter.calculate(omegaInput)
                * kMaxAngularSpeed;



        // ====================================================
        // ESCOLHE O MODO DE DIREÇÃO
        // ====================================================

        // 🎮🧭 A frente do piloto vem do lado do campo visto pelas tags, sem usar a aliança
        // da Driver Station (Drive.updateDriverReference). Y troca para onde o robô aponta.
        if (fieldRelativeEnabled) {

            // -----------------------------------------------
            // FIELD RELATIVE
            // -----------------------------------------------

            drive.driveDriverRelative(
                vx,
                vy,
                omega
            );

        } else {

            // -----------------------------------------------
            // ROBOT RELATIVE
            // -----------------------------------------------

            drive.drive(
                vx,
                vy,
                omega
            );
        }

        },

        // Quando o comando terminar
        // ou for interrompido
        drive::stop

        )

        );


        // Configura os botões do controle
        configureBindings();

        }

    // #endregion


    // #region 🎮 BINDINGS DOS CONTROLES

    /**
     * Configuração dos botões
     * e triggers do controle Xbox.
     */
    // 🎮 MAPA DOS BOTÕES EM USO (conferido no código em 31/10/2026)
    //
    // PILOTO (porta 0)
    //   Analógico esquerdo   anda        Analógico direito   gira
    //   R1 (RB)              desce/abre o intake     L1 (LB)  sobe/fecha o intake
    //   R2 (RT)              rolete puxa             L2 (LT)  só mira
    //   B                    mira e atira
    //   X                    alterna field/robot relative
    //   Y                    zera a referência do piloto (robô parado)
    //   Seta para baixo      freio em X
    //   Start + Y / Start + A        0° / 180° de campo (desabilitado)
    //   Start + seta para baixo      pose pela visão (seed manual)
    //   Start + L3 / Start + R3      🔧 gira o robô nos dois sentidos
    //   Back + Y                     🔧 teste de tensão baixa
    //   Back + seta cima/direita/esquerda   🔧 sweep, kS, kS reverso
    //   Back + A / Back + B          🔧 anda 2 m / gira 360°
    //   🛑 Com FMS, todos os 🔧 acima ficam desligados: nada anda sozinho em partida.
    //
    // OPERADOR (porta 1, opcional)
    //   RB mira e atira   LB só mira   A pivô manual (analógico direito)
    //   X abre o intake   Y fecha o intake   B rolete
    //   Back + Start (desabilitado)  zera o pivô     Back + Y (desabilitado)  zera o intake
    //
    private void configureBindings() {

        // Seta para baixo sem Start: manter o freio em X no teleoperado.
        // Start + seta para baixo continua reservado ao seed da visão.
        m_driverController.povDown()
            .and(m_driverController.start().negate())
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(
                drive.xLockCommand()
                    .finallyDo(interrupted -> resetDriverLimiters())
            );


        // ============================================================
        // 🎮🧭 BOTÃO Y - ZERO DA REFERÊNCIA DO PILOTO
        // ============================================================

        // Preserva a pose de campo enviada à Limelight. Exige robô parado.
        // Sem Back: Back + Y é o teste de calibração de tensão baixa.
        m_driverController
            .y()
            .and(m_driverController.start().negate())
            .and(m_driverController.back().negate())
            .and(() -> DriverStation.isDisabled() || DriverStation.isTeleopEnabled())
            .onTrue(
                drive.runOnce(
                    () -> {
                        drive.zeroDriverHeading();
                        resetDriverLimiters();
                    }
                ).ignoringDisable(true)
            );

        // Start + Y: definir 0 graus de CAMPO, desabilitado e parado.
        // Antes de usar, apontar a frente do robô para +X do campo azul.
        m_driverController.y()
            .and(m_driverController.start())
            .and(DriverStation::isDisabled)
            .onTrue(drive.runOnce(drive::zeroHeading).ignoringDisable(true));

        // Start + A: definir 180 graus de CAMPO, desabilitado e parado.
        // Para o treino com a Limelight TRASEIRA vendo o hub azul (tags 25/26):
        // a frente do robô aponta para a parede da aliança AZUL (-X).
        // Sem isso o giroscópio liga em 0° e o MegaTag2 espelha o robô para o
        // meio da arena, atrás das tags (teste de 28/10/2026).
        m_driverController.a()
            .and(m_driverController.start())
            .and(DriverStation::isDisabled)
            .onTrue(drive.runOnce(() -> drive.setFieldHeading(Rotation2d.k180deg)).ignoringDisable(true));

            // ============================================================
            // 🎮 BOTÃO X - ALTERNA FIELD / ROBOT RELATIVE
            // ============================================================

            m_driverController
                .x()
                .onTrue(

                    drive.runOnce(
                        () -> {

                            fieldRelativeEnabled =
                                !fieldRelativeEnabled;

                        }
                    )
                );


        // ============================================================
        // 📦 ARQUIVADO: TELESCÓPIO 9460 - PILOTO: L1 ABRE / L2 FECHA (hoje L1/L2 são intake e mira)
        // ============================================================
        //
        // L1 (LB) segurado: abre até o fim de curso.
        // L2 (LT) segurado: fecha até o batente.
        // Soltou: o perfil freia e o telescópio mantém a altura.
        // L1 e L2 juntos não movem o telescópio.
        //
        // Só no Teleop e sem BACK e sem START: BACK + L1/L2 é o SysId
        // do swerve e START + L1/L2 é a calibração do steer (0° e 90°).
        // Um botão interrompe o comando do painel; para voltar ao painel,
        // desligue e ligue ControleAtivo.
        //
        // O telescópio 9458 não tem botão: só o painel /Tuning/Telescopio9458.
        //
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         Trigger telescopio9460Liberado =
//             m_driverController.back().negate()
//                 .and(m_driverController.start().negate())
//                 .and(DriverStation::isTeleopEnabled);
//
//         m_driverController
//             .leftBumper()
//             .and(m_driverController.leftTrigger().negate())
//             .and(telescopio9460Liberado)
//             .whileTrue(new TelescopioAbrirCommand(telescopio));
//
//         m_driverController
//             .leftTrigger()
//             .and(m_driverController.leftBumper().negate())
//             .and(telescopio9460Liberado)
//             .whileTrue(new TelescopioFecharCommand(telescopio));



        // ------------------------------------------------------------
        // ANTIGO (comentado em 28/10/2026): R1 (RB) agora sobe o intake; o 9460 passou para L1/L2 (bloco acima).
        // Para voltar, descomente o bloco e libere o botão no bloco novo.
        // ------------------------------------------------------------
        // ============================================================
        // TELESCÓPIO - RB ABRE / LB FECHA (SEGURAR)
        // ============================================================
        //
        // Segurou RB: abre até o fim de curso.
        // Segurou LB: fecha até o batente.
        // Soltou: o perfil freia e o telescópio mantém a altura.
        //
        // Sem BACK e sem START: BACK + LB/RB é SysId e
        // START + LB/RB é calibração do steer.
        // Os dois bumpers juntos não movem o telescópio.
        //
        // ============================================================

        // Trigger telescopioLiberado =
        //     m_driverController.back().negate()
        //         .and(m_driverController.start().negate())
        //         .and(DriverStation::isTeleopEnabled);

        // m_driverController
        //     .rightBumper()
        //     .and(m_driverController.leftBumper().negate())
        //     .and(telescopioLiberado)
        //     .whileTrue(new TelescopioAbrirCommand(telescopio));

        // m_driverController
        //     .leftBumper()
        //     .and(m_driverController.rightBumper().negate())
        //     .and(telescopioLiberado)
        //     .whileTrue(new TelescopioFecharCommand(telescopio));


        // ------------------------------------------------------------
        // ANTIGO (comentado em 28/10/2026): R2 (RT) agora desce o intake; o 9458 fica só pelo painel.
        // Para voltar, descomente o bloco e libere o botão no bloco novo.
        // ------------------------------------------------------------
        // ============================================================
        // TELESCÓPIO 9458 - RT ABRE / LT FECHA (SEGURAR)
        // ============================================================
        //
        // Segurou RT: abre até 0,946 m na ponta (5 cm antes do batente de 0,996 m).
        // Segurou LT: fecha até o batente.
        // Soltou: o perfil freia e o telescópio mantém a altura.
        //
        // Mesma liberação dos bumpers: BACK + LT/RT é SysId e
        // START + LT/RT é calibração do steer.
        // Os dois gatilhos juntos não movem o telescópio.
        //
        // ============================================================

        // m_driverController
        //     .rightTrigger()
        //     .and(m_driverController.leftTrigger().negate())
        //     .and(telescopioLiberado)
        //     .whileTrue(telescopio9458.extendWhileHeldCommand());

        // m_driverController
        //     .leftTrigger()
        //     .and(m_driverController.rightTrigger().negate())
        //     .and(telescopioLiberado)
        //     .whileTrue(telescopio9458.retractWhileHeldCommand());


        // ------------------------------------------------------------
        // ANTIGO (comentado em 28/10/2026): o 9458 fica só pelo painel (/Tuning/Telescopio9458).
        // Para voltar, descomente o bloco e libere o botão no bloco novo.
        // ------------------------------------------------------------
        // ============================================================
        // TELESCÓPIO 9458 - R1 / L1 NO MODO TEST: PASSOS DE 10 CM (SEGURAR)
        // ============================================================
        //
        // Somente no modo TEST da Driver Station:
        //
        // R1 (RB): sobe 10 cm a partir da altura atual e mantém.
        // L1 (LB): desce 10 cm e mantém.
        //
        // Cada aperto é mais um passo, dentro do curso (0 a 0,946 m).
        // Soltou antes de chegar: o perfil freia e para ali.
        //
        // No Test, R1/L1 sozinhos estão livres: o intake (R1) só
        // funciona no Teleop. Sem BACK e sem START: BACK + R1/L1 é o
        // SysId do swerve e START + R1/L1 é a calibração do steer.
        // Os dois juntos não fazem nada.
        //
        // SysId do 9458: a rotina continua em telescopio9458SysId.
        // Para calibrar de novo, troque os comandos abaixo por
        // telescopio9458SysId.quasistaticUpAndDown() (R1) e
        // telescopio9458SysId.dynamicUpAndDown() (L1).
        //
        // ============================================================

        // Trigger teste9458Liberado =
        //     new Trigger(DriverStation::isTestEnabled)
        //         .and(m_driverController.back().negate())
        //         .and(m_driverController.start().negate());

        // teste9458Liberado
        //     .and(m_driverController.rightBumper())
        //     .and(m_driverController.leftBumper().negate())
        //     .whileTrue(telescopio9458.stepUpWhileHeldCommand());

        // teste9458Liberado
        //     .and(m_driverController.leftBumper())
        //     .and(m_driverController.rightBumper().negate())
        //     .whileTrue(telescopio9458.stepDownWhileHeldCommand());


        // ============================================================
        // 🎮 SHOOTER - CONTROLE DO OPERADOR (PORTA 1)
        // ============================================================
        //
        // Com um controle só, use o B do piloto (mais abaixo).
        //
        // RB segurado: mira no hub, acelera e atira quando pronto.
        // LB segurado: só mira (rodas paradas).
        // A segurado: pivô manual lento pelo analógico direito (teste).
        // BACK + START com o robô DESABILITADO: centro atual vira o zero.
        //
        // Antes de ligar o robô, centralizar o shooter à mão:
        // não existe sensor, o zero é onde ele estiver ao ligar.
        //
        // ============================================================

        Trigger operadorTeleop = new Trigger(DriverStation::isTeleopEnabled);

        m_operatorController.rightBumper()
            .and(operadorTeleop)
            .whileTrue(new ShooterTrackAndShootCommand(shooter, centrifuga, vision::getDiagnosis));

        m_operatorController.leftBumper()
            .and(m_operatorController.rightBumper().negate())
            .and(operadorTeleop)
            .whileTrue(new ShooterAimAtHubCommand(shooter));

        m_operatorController.a()
            .and(m_operatorController.leftBumper().negate())
            .and(m_operatorController.rightBumper().negate())
            .and(operadorTeleop)
            .whileTrue(shooter.manualPivotCommand(
                () -> -MathUtil.applyDeadband(m_operatorController.getRightX(), 0.10)));

        m_operatorController.back()
            .and(m_operatorController.start())
            .and(DriverStation::isDisabled)
            .onTrue(shooter.zeroPivotHereCommand());

        // Back + Y, desabilitado, com o intake FECHADO à mão: zera o intake.
        // Só é preciso se o início avisar POSICAO_DESCONHECIDA (o deploy não zera mais).
        m_operatorController.back()
            .and(m_operatorController.y())
            .and(DriverStation::isDisabled)
            .onTrue(intake.zeroClosedCommand());

        // ============================================================
        // 🎮 INTAKE E ROLLER - OPERADOR (PORTA 1)
        // ============================================================
        //
        // X segurado: abre o intake devagar.  Y segurado: fecha devagar.
        // B segurado: rolete puxando.         Soltou: trava/para.
        // Igual ao código de competição, só que no controle do operador.
        //
        // ============================================================

        m_operatorController.x()
            .and(m_operatorController.y().negate())
            .and(operadorTeleop)
            .whileTrue(intake.openCommand());

        m_operatorController.y()
            .and(m_operatorController.x().negate())
            .and(operadorTeleop)
            .whileTrue(intake.closeCommand());

        m_operatorController.b()
            .and(operadorTeleop)
            .whileTrue(roller.pullCommand());

        // ============================================================
        // 📦 ARQUIVADO: INTAKE E ROLETE - PILOTO (o mapa atual está em DriverMechanismBindings)
        // ============================================================
        //
        // R1 (RB) segurado: SOBE o intake (fecha devagar).
        // R2 (RT) segurado: DESCE o intake (abre devagar).
        // Soltou: o intake trava onde está (PID de segurar no TalonFX).
        // R1 e R2 juntos não movem o intake.
        //
        // START + R1 (um toque): LIGA o rolete. Outro toque: DESLIGA.
        // Desabilitar o robô desliga o rolete (o comando é cancelado).
        //
        // Só no Teleop. R1/R2 sozinhos exigem BACK e START soltos:
        // BACK + R1/R2 é o SysId do swerve e START + R1 é o rolete.
        //
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         Trigger intakeLiberado =
//             m_driverController.back().negate()
//                 .and(m_driverController.start().negate())
//                 .and(DriverStation::isTeleopEnabled);
//
//         m_driverController
//             .rightBumper()
//             .and(m_driverController.rightTrigger().negate())
//             .and(intakeLiberado)
//             .whileTrue(intake.closeCommand());
//
//         m_driverController
//             .rightTrigger()
//             .and(m_driverController.rightBumper().negate())
//             .and(intakeLiberado)
//             .whileTrue(intake.openCommand());
//
//         m_driverController
//             .start()
//             .and(m_driverController.rightBumper())
//             .and(DriverStation::isTeleopEnabled)
//             .toggleOnTrue(roller.pullCommand());



        // ------------------------------------------------------------
        // ANTIGO (comentado em 28/10/2026): coleta pelo L3/R3 trocada por R1/R2 e START + R1.
        // Para voltar, descomente o bloco e libere o botão no bloco novo.
        // ------------------------------------------------------------
        // ============================================================
        // COLETA COM UM CONTROLE SÓ - PILOTO (PORTA 0)
        // ============================================================
        //
        // Clique no analógico ESQUERDO (L3) segurado: abre o intake e
        // puxa com o rolete. Soltou: o rolete para e o intake trava.
        // Clique no analógico DIREITO (R3) segurado: fecha o intake.
        // Sem START: START + L3/R3 é a calibração de rotação do swerve.
        //
        // ============================================================

        // m_driverController.leftStick()
        //     .and(m_driverController.start().negate())
        //     .and(DriverStation::isTeleopEnabled)
        //     .whileTrue(new ColetarCommand(intake, roller));

        // m_driverController.rightStick()
        //     .and(m_driverController.start().negate())
        //     .and(DriverStation::isTeleopEnabled)
        //     .whileTrue(intake.closeCommand());


        // ============================================================
        // 📦 ARQUIVADO: SHOOTER - PILOTO: LIGA/DESLIGA COM UM TOQUE (hoje B segurado atira)
        // ============================================================
        //
        // B (um toque): liga/desliga a MIRA AUTOMÁTICA. Ligada, o pivô
        //    segue o hub o tempo todo (rodas paradas).
        // START + B (um toque): liga/desliga o TIRO AUTOMÁTICO. Ligado,
        //    mira, acelera e atira sempre que tudo estiver pronto.
        //    Desligou: volta para a mira automática, se ela estiver ligada.
        //
        // Desabilitar o robô desliga os dois.
        // BACK + B continua sendo o teste de rotação 360°.
        //
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         // Comando padrão do shooter: segue o hub quando a chave estiver ligada.
//         shooter.setDefaultCommand(new ShooterAimAtHubCommand(shooter, () -> autoAimEnabled));
//
//         m_driverController
//             .b()
//             .and(m_driverController.back().negate())
//             .and(m_driverController.start().negate())
//             .and(DriverStation::isTeleopEnabled)
//             .onTrue(Commands.runOnce(() -> autoAimEnabled = !autoAimEnabled)
//                 .withName("Shooter - liga/desliga mira automática"));
//
//         m_driverController
//             .b()
//             .and(m_driverController.start())
//             .and(DriverStation::isTeleopEnabled)
//             .toggleOnTrue(new ShooterTrackAndShootCommand(shooter, centrifuga));
//
//         new Trigger(DriverStation::isDisabled)
//             .onTrue(Commands.runOnce(() -> autoAimEnabled = false)
//                 .ignoringDisable(true)
//                 .withName("Shooter - desliga mira automática ao desabilitar"));

        // Piloto, porta 0: segurar para funcionar; soltou, o comando termina.
        // R1 desce intake; L1 sobe; R2 roller; L2 mira; B sequência de tiro.
        // O B prepara top/tower e só alimenta após as verificações da sequência.
        frc.robot.controls.DriverMechanismBindings.bind(m_driverController,
            intake.openCommand(), intake.closeCommand(), roller.pullCommand(),
            new ShooterAimAtHubCommand(shooter),
            // B: mira e atira; a centrífuga só gira para a FRENTE.
            new ShooterTrackAndShootCommand(shooter, centrifuga, vision::getDiagnosis));

// Arquivado em 29/10/2026: o B revertia a centrífuga 1/4 de volta antes do tiro.
// No robô a reversão era fraca e só atrasava/travava o tiro.
//             Commands.sequence(
//                 shooter.runOnce(shooter::stopAll),
//                 new frc.robot.commands.shooter.CentrifugaDestravarCommand(centrifuga),
//                 new ShooterTrackAndShootCommand(shooter, centrifuga))
//                 .withName("Shooter - destravar, mirar e atirar"));



        // ============================================================
        // 🔧 TESTES DE CALIBRAÇÃO DO DRIVE - SÓ COM BACK SEGURADO
        // ============================================================
        //
        // Antes (até 29/10/2026) bastava A sozinho ou uma seta: um toque sem
        // querer no teleoperado fazia o robô andar sozinho. E Back + A
        // (andar 2 m) disparava também o A sozinho, os dois disputando o drive.
        //
        // 🛑 Com FMS estes botões não fazem nada (calibracaoLiberada).
        // BACK + Y: teste de tensão baixa (era A).
        // BACK + ↑: varredura (sweep). BACK + →: kS. BACK + ←: kS ao contrário.
        //
        // ============================================================

        Trigger calibracaoLiberada = frc.robot.controls.DriverMechanismBindings.calibrationAllowed();

        m_driverController
            .back()
            .and(m_driverController.y())
            .and(calibracaoLiberada)
            .whileTrue(
                CalibrationCommands.driveLowTest(drive)
            );

        m_driverController
            .back()
            .and(m_driverController.povUp())
            .and(calibracaoLiberada)
            .whileTrue(
                CalibrationCommands.driveSweepTest(drive)
            );

        m_driverController
            .back()
            .and(m_driverController.povRight())
            .and(calibracaoLiberada)
            .whileTrue(
                CalibrationCommands.driveKsTest(drive)
            );

        m_driverController
            .back()
            .and(m_driverController.povLeft())
            .and(calibracaoLiberada)
            .whileTrue(
                CalibrationCommands.driveKsReverseTest(
                    drive
                )
            );

// Arquivado em 29/10/2026: A sozinho e setas sozinhas iniciavam os testes.
//         m_driverController.a().whileTrue(CalibrationCommands.driveLowTest(drive));
//         m_driverController.povUp().whileTrue(CalibrationCommands.driveSweepTest(drive));
//         m_driverController.povRight().whileTrue(CalibrationCommands.driveKsTest(drive));
//         m_driverController.povLeft().whileTrue(CalibrationCommands.driveKsReverseTest(drive));

        // ============================================================
        // 📦 ARQUIVADO: SYSID - QUASISTATIC FORWARD (botões liberados em 29/10/2026)
        // ============================================================
        //
        // BACK + LB
        //
        // A tensão sobe lentamente no sentido positivo.
        //
        // Soltou qualquer um dos botões:
        // o Command é cancelado
        // e o DriveSysId zera a tensão.
        //
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         m_driverController
//             .back()
//             .and(
//                 m_driverController.leftBumper()
//             )
//             .whileTrue(
//                 driveSysId.quasistaticForward()
//             );




        // ============================================================
        // 📦 ARQUIVADO: SYSID - QUASISTATIC REVERSE
        // ============================================================
        //
        // BACK + RB
        //
        // A tensão sobe lentamente no sentido negativo.
        //
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         m_driverController
//             .back()
//             .and(
//                 m_driverController.rightBumper()
//             )
//             .whileTrue(
//                 driveSysId.quasistaticReverse()
//             );




        // ============================================================
        // 📦 ARQUIVADO: SYSID - DYNAMIC FORWARD
        // ============================================================
        //
        // BACK + LT
        //
        // Aplica o degrau positivo configurado:
        //
        // +2.0 V
        //
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         m_driverController
//             .back()
//             .and(
//                 m_driverController.leftTrigger()
//             )
//             .whileTrue(
//                 driveSysId.dynamicForward()
//             );




        // ============================================================
        // 📦 ARQUIVADO: SYSID - DYNAMIC REVERSE
        // ============================================================
        //
        // BACK + RT
        //
        // Aplica:
        //
        // -2.0 V
        //
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         m_driverController
//             .back()
//             .and(
//                 m_driverController.rightTrigger()
//             )
//             .whileTrue(
//                 driveSysId.dynamicReverse()
//             );



        // #region 🔧 CALIBRAÇÃO DO SWERVE - ÂNGULOS


        // ============================================================
        // START + L1
        // STEER = 0°
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         m_driverController
//             .start()
//             .and(
//                 m_driverController.leftBumper()
//             )
//             .whileTrue(
//
//                 SwerveCalibrationCommands.steer0(
//                     drive
//                 )
//             );




        // ------------------------------------------------------------
        // ANTIGO (comentado em 28/10/2026): START + R1 agora liga/desliga o rolete.
        // Para voltar, descomente o bloco e libere o botão no bloco novo.
        // ------------------------------------------------------------
        // ============================================================
        // START + R1
        // STEER = 45°
        // ============================================================

        // m_driverController
        //     .start()
        //     .and(
        //         m_driverController.rightBumper()
        //     )
        //     .whileTrue(

        //         SwerveCalibrationCommands.steer45(
        //             drive
        //         )
        //     );



        // ============================================================
        // START + L2
        // STEER = 90°
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         m_driverController
//             .start()
//             .and(
//                 m_driverController.leftTrigger()
//             )
//             .whileTrue(
//
//                 SwerveCalibrationCommands.steer90(
//                     drive
//                 )
//             );




        // ============================================================
        // START + R2
        // STEER = 180°
        // ============================================================

// Arquivado em 29/10/2026: mapeamento anterior, preservado para consulta.
//         m_driverController
//             .start()
//             .and(
//                 m_driverController.rightTrigger()
//             )
//             .whileTrue(
//
//                 SwerveCalibrationCommands.steer180(
//                     drive
//                 )
//
//             );





                // ============================================================
                // CALIBRAÇÃO DO SWERVE - ROTAÇÃO PURA
                // ============================================================

                // ============================================================
                // START + LEFT STICK
                // ROTAÇÃO POSITIVA
                //
                // vx    = 0
                // vy    = 0
                // omega = +0.50 rad/s
                //
                // O robô deve girar no próprio centro.
                // ============================================================
                m_driverController
                    .start()
                    .and(
                        m_driverController.leftStick()
                    )
                    .and(calibracaoLiberada)
                    .whileTrue(
                        SwerveCalibrationCommands.rotatePositive(
                            drive
                        )
                    );


                // ============================================================
                // START + RIGHT STICK
                // ROTAÇÃO NEGATIVA
                //
                // vx    = 0
                // vy    = 0
                // omega = -0.50 rad/s
                //
                // O robô deve girar no próprio centro
                // no sentido contrário.
                // ============================================================
                m_driverController
                    .start()
                    .and(
                        m_driverController.rightStick()
                    )
                    .and(calibracaoLiberada)
                    .whileTrue(
                        SwerveCalibrationCommands.rotateNegative(
                            drive
                        )
                    );
                     
                    // ============================================================
                    // TESTE AUTOMÁTICO - ANDAR 2 METROS
                    // ============================================================
                    //
                    // BACK + A
                    //
                    // SEGURAR OS DOIS BOTÕES.
                    //
                    // Enquanto estiver segurando:
                    //
                    //     robô anda para frente
                    //          ↓
                    //     odometria mede a distância
                    //          ↓
                    //     chegou em 2,00 metros
                    //          ↓
                    //     para automaticamente
                    //
                    // SEGURANÇA:
                    //
                    // Soltou BACK ou A
                    //        ↓
                    // comando é cancelado
                    //        ↓
                    // DriveTwoMetersCommand.end()
                    //        ↓
                    // drive.stop()
                    //
                    // Timeout máximo = 6 segundos
                    //
                    // ============================================================

                    m_driverController
                        .back()
                        .and(
                            m_driverController.a()
                        )
                        .and(calibracaoLiberada)
                        .whileTrue(

                            new DriveTwoMetersCommand(
                                drive
                            )
                            .withTimeout(
                                6.0
                            )
                        );



                    // ============================================================
                    // BACK + B
                    // TESTE AUTOMÁTICO - ROTAÇÃO 360°
                    //
                    // PRECISA SEGURAR.
                    //
                    // Soltou BACK ou B:
                    // cancela imediatamente.
                    //
                    // Segurança adicional:
                    // timeout de 150 segundos.
                    // ============================================================

                    m_driverController
                        .back()
                        .and(
                            m_driverController.b()
                        )
                        .and(calibracaoLiberada)
                        .whileTrue(

                            new Rotate360Command(
                                drive
                            )
                            .withTimeout(
                                150.0//Quantidade tempo que metodo roda deposi destroy
                            )
                        );



                    // ============================================================
                    // 📷🧭 LIMELIGHT - SINCRONIZAÇÃO MANUAL DA POSE
                    // ============================================================
                    //
                    // START + SETA PARA BAIXO (D-PAD ↓)
                    //
                    // Executa uma única tentativa por acionamento.
                    //
                    // ============================================================

                    m_driverController
                        .start()
                        .and(
                            m_driverController.povDown()
                        )
                        .onTrue(

                            drive.runOnce(
                                () -> vision.trySeedDrivePose()
                            )

                        );

                        }
    
    //#endregion
    
    // #endregion


    // #region AUTÔNOMO

                         private void resetDriverLimiters() {
                             vxLimiter.reset(0.0);
                             vyLimiter.reset(0.0);
                             omegaLimiter.reset(0.0);
                         }

                         // 🚧 ATENÇÃO: ainda não há autônomo: o comando termina na hora e o robô não se move.
                         /** Retorna o comando utilizado durante o período autônomo. */
                         public Command getAutonomousCommand() {

                            return Commands.none();

                        }

    // #endregion


// ============================================================
// ATUALIZAÇÃO DA VISÃO VIRTUAL
// ============================================================

public void updateVisionSimulation() {
    if (visionSimulation != null && drive.isSimulationReferenceReady()) {
        // A referência movimenta as câmeras; a estimativa serve apenas para comparação.
        visionSimulation.update(drive.getSimulationPose(), drive.getPose());
    }
}

// Bolas: coleta, lançamento e voo. Só existe na simulação normal.
public void updateFuelSimulation() {
    if (fuelSimulation != null) {
        fuelSimulation.update();
    }
}


}
