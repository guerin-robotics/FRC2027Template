package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Attributes loop time to named sections. {@link LoopTimeMonitor} says the loop is over budget;
 * this says where.
 *
 * <p>Call {@link #reset()} at the start of {@code robotPeriodic()}, then {@link #record(String)}
 * after each section. Each call logs the milliseconds since the previous {@code reset()} or {@code
 * record()} under {@code LoggedTracer/<name>}, so the sections sum to the loop time.
 *
 * <p>A subsystem's {@code periodic()} runs inside {@code CommandScheduler.run()}, so a {@code
 * record("Drive")} at the end of {@code Drive.periodic()} measures from the previous record — the
 * scheduler start, or the subsystem that ran before it — not Drive alone. Read the channels as
 * "time until this point", and compare adjacent ones.
 *
 * <p>Not thread-safe: use from the main robot thread only. Costs one {@code recordOutput} per call,
 * so keep sections coarse.
 */
public final class LoggedTracer {
  private static double startSeconds = Timer.getFPGATimestamp();

  /** Log keys built once per name, so a record() call allocates nothing after its first. */
  private static final Map<String, String> keys = new HashMap<>();

  private LoggedTracer() {}

  /** Starts a new measurement. Call once at the top of {@code robotPeriodic()}. */
  public static void reset() {
    startSeconds = Timer.getFPGATimestamp();
  }

  /** Logs the time since the last {@link #reset()} or {@code record()}, then restarts the clock. */
  public static void record(String name) {
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput(
        keys.computeIfAbsent(name, n -> "LoggedTracer/" + n + "MS"), (now - startSeconds) * 1000.0);
    startSeconds = now;
  }
}
