package frc.robot;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Mesma ligação de relógio feita pelo RobotBase, sem iniciar um robô no teste. */
final class RobotTestClock {
    static void configure() {
        MathSharedStore.setMathShared(new MathShared() {
            @Override public double getTimestamp() { return Timer.getTimestamp(); }
            @Override public void reportError(String error, StackTraceElement[] trace) {
                DriverStation.reportError(error, trace);
            }
            @Override public void reportUsage(MathUsageId id, int count) {}
        });
    }
    private RobotTestClock() {}
}
