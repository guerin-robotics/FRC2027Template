package frc.lib.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/**
 * Measures how long each robot loop actually takes and complains when it exceeds the budget.
 *
 * <p><b>Why this exists.</b> The 2026 robot ran closer to 30 Hz than the 50 Hz it was scheduled
 * for, with {@code robotPeriodic} averaging 31–35 ms against a 20 ms budget, for the entire season.
 * Nothing surfaced it. It was found months later by analyzing match logs. A loop that overruns
 * silently degrades everything at once — stale control outputs, missed odometry samples, sluggish
 * button response — and none of those symptoms points at the real cause.
 *
 * <p>This makes the problem loud on day one: a number in the log, and a driver-station alert the
 * moment it goes over budget.
 *
 * <p>Publishes under {@code LoopTiming/}: current, peak and rolling-average duration in
 * milliseconds, plus a cumulative overrun count and the fraction of loops that overran.
 *
 * <p>Usage in {@code Robot.robotPeriodic()}:
 *
 * <pre>
 * loopMonitor.begin();
 * ... all periodic work ...
 * loopMonitor.end();
 * </pre>
 */
public class LoopTimeMonitor {

  /** Loop budget in milliseconds. WPILib's default period is 20 ms. */
  private static final double BUDGET_MS = 20.0;

  /**
   * Alert once the rolling average crosses this. Set above the budget so a single slow loop (which
   * happens at startup and on the first enable) does not cry wolf — this fires when the robot is
   * genuinely running slow, not on a transient.
   */
  private static final double ALERT_THRESHOLD_MS = 25.0;

  /** Number of samples in the rolling average. 50 samples is ~1 second at 20 ms. */
  private static final int WINDOW = 50;

  private final Alert overrunAlert =
      new Alert(
          "Loop time over budget — robot is running slower than 50 Hz. Check Drive and Vision.",
          AlertType.kWarning);

  private final double[] window = new double[WINDOW];
  private int windowIndex = 0;
  private boolean windowFilled = false;

  private long startMicros = 0;
  private double peakMs = 0.0;
  private long overrunCount = 0;
  private long totalLoops = 0;

  /** Call at the very top of {@code robotPeriodic()}. */
  public void begin() {
    startMicros = RobotController.getFPGATime();
  }

  /** Call at the very bottom of {@code robotPeriodic()}. */
  public void end() {
    double elapsedMs = (RobotController.getFPGATime() - startMicros) / 1000.0;

    totalLoops++;
    if (elapsedMs > BUDGET_MS) {
      overrunCount++;
    }
    if (elapsedMs > peakMs) {
      peakMs = elapsedMs;
    }

    window[windowIndex] = elapsedMs;
    windowIndex = (windowIndex + 1) % WINDOW;
    if (windowIndex == 0) {
      windowFilled = true;
    }

    int samples = windowFilled ? WINDOW : Math.max(windowIndex, 1);
    double sum = 0.0;
    for (int i = 0; i < samples; i++) {
      sum += window[i];
    }
    double averageMs = sum / samples;

    Logger.recordOutput("LoopTiming/CurrentMs", elapsedMs);
    Logger.recordOutput("LoopTiming/AverageMs", averageMs);
    Logger.recordOutput("LoopTiming/PeakMs", peakMs);
    Logger.recordOutput("LoopTiming/BudgetMs", BUDGET_MS);
    Logger.recordOutput("LoopTiming/OverrunCount", overrunCount);
    Logger.recordOutput(
        "LoopTiming/OverrunFraction", totalLoops == 0 ? 0.0 : (double) overrunCount / totalLoops);

    // Only alert once the window is full, so startup does not trip it.
    overrunAlert.set(windowFilled && averageMs > ALERT_THRESHOLD_MS);
  }

  /** Rolling-average loop time in milliseconds. Exposed for dashboard use. */
  public double getAverageMs() {
    int samples = windowFilled ? WINDOW : Math.max(windowIndex, 1);
    double sum = 0.0;
    for (int i = 0; i < samples; i++) {
      sum += window[i];
    }
    return sum / samples;
  }

  /** True when the rolling average is over the alert threshold. */
  public boolean isOverBudget() {
    return windowFilled && getAverageMs() > ALERT_THRESHOLD_MS;
  }

  /** Clears the peak, so a single startup spike doesn't dominate the whole session. */
  public void resetPeak() {
    peakMs = 0.0;
  }
}
