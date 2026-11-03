package frc.robot;

// #region IMPORTS

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

// #endregion

/**
 * Peças móveis do modelo 3D "FRC9460 - Robo 2026" no AdvantageScope.
 *
 * Documentação de componentes articulados do AdvantageScope: o modelo tem
 * uma parte fixa e uma lista de peças móveis (model_0 = intake, model_1 =
 * shooter). O código publica um Pose3d por peça, RELATIVO AO ROBÔ e NA ORDEM
 * DOS ARQUIVOS, em Mecanismos/Componentes; o AdvantageScope soma a pose do
 * robô no campo. Como a lista é uma só, ela é montada aqui, com o ângulo de
 * cada subsistema.
 */
public class RobotComponentsVisualizer extends SubsystemBase implements AutoCloseable {

    // #region FONTES DOS ÂNGULOS

    private final DoubleSupplier intakeRaiseDegrees;
    private final DoubleSupplier shooterPivotDegrees;
    private final DoubleSupplier modelIntakeDegrees;
    private final DoubleSupplier modelPivotDegrees;

    /**
     * @param intakeRaiseDegrees 0 = intake aberto (posição do CAD), 90 = fechado
     * @param shooterPivotDegrees ângulo do pivô lido do encoder (0 = centro)
     */
    public RobotComponentsVisualizer(DoubleSupplier intakeRaiseDegrees, DoubleSupplier shooterPivotDegrees) {
        this(intakeRaiseDegrees, shooterPivotDegrees, () -> Double.NaN, () -> Double.NaN);
    }

    public RobotComponentsVisualizer(DoubleSupplier intakeRaiseDegrees, DoubleSupplier shooterPivotDegrees,
            DoubleSupplier modelIntakeDegrees, DoubleSupplier modelPivotDegrees) {
        this.intakeRaiseDegrees = intakeRaiseDegrees;
        this.shooterPivotDegrees = shooterPivotDegrees;
        this.modelIntakeDegrees = modelIntakeDegrees;
        this.modelPivotDegrees = modelPivotDegrees;
    }

    // #endregion

    // #region POSES DAS PEÇAS

    /**
     * model_0 (intake): gira em Y no pivô; Rotation3d(0, -subida, 0) levanta.
     * model_1 (shooter): gira em Z no pivô. O CAD foi exportado apontando
     * para 175,5°; a diferença para o zero do código (180°) é descontada.
     */
    public static Pose3d[] componentPoses(double intakeRaiseDegrees, double pivotDegrees) {
        double shooterYawDegrees = ShooterConstants.PIVOT_ZERO_FACING_DEGREES + pivotDegrees
            - ShooterConstants.CAD_ZERO_FACING_DEGREES;
        return new Pose3d[] {
            new Pose3d(IntakeConstants.PIVOT_IN_ROBOT,
                new Rotation3d(0.0, -Math.toRadians(intakeRaiseDegrees), 0.0)),
            new Pose3d(ShooterConstants.TURRET_PIVOT_IN_ROBOT,
                new Rotation3d(0.0, 0.0, Math.toRadians(shooterYawDegrees)))
        };
    }

    // #endregion

    // #region 🔁 PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Mecanismos/Componentes",
            componentPoses(intakeRaiseDegrees.getAsDouble(), shooterPivotDegrees.getAsDouble()));
        double intake = modelIntakeDegrees.getAsDouble();
        double pivot = modelPivotDegrees.getAsDouble();
        boolean valid = Double.isFinite(intake) && Double.isFinite(pivot);
        Logger.recordOutput("Mecanismos/ModeloValido", valid);
        Logger.recordOutput("Mecanismos/ComponentesModelo",
            valid ? componentPoses(intake, pivot) : new Pose3d[0]);
    }

    // #endregion

    // #region ENCERRAMENTO (TESTES)

    @Override
    public void close() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    // #endregion
}
