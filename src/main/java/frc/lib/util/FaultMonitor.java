package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Collapses every known fault condition into one thing the drive team can actually watch.
 *
 * <p><b>Why this exists.</b> In 2026 a camera died partway through an event and the only indication
 * was a per-camera {@code Alert} buried on the dashboard. Nobody was looking at it, and the team
 * finished the event running on three cameras without knowing. The information existed; the problem
 * was that it was one of dozens of things that would have had to be checked individually.
 *
 * <p>This gives the pit a single green/red: {@code RobotHealth/OK}. When it goes red, {@code
 * RobotHealth/ActiveFaults} says what broke.
 *
 * <p>Register conditions once during wiring, in {@code RobotContainer}:
 *
 * <pre>
 * FaultMonitor.getInstance().register("Gyro disconnected", () -&gt; !drive.isGyroConnected());
 * FaultMonitor.getInstance().register("Camera 0 disconnected", () -&gt; !vision.isCameraConnected(0));
 * </pre>
 *
 * <p>Register the condition as <b>true means broken</b>. Keep names short — they get read at a
 * glance across a pit.
 *
 * <p>This does not replace {@code Alert}. Alerts are still the right tool for detail and for the
 * driver-station log. This is the summary on top of them.
 */
public class FaultMonitor {

  private static FaultMonitor instance;

  /** Fault name -> condition, where true means the fault is present. */
  private final Map<String, BooleanSupplier> faults = new LinkedHashMap<>();

  private final List<String> activeFaults = new ArrayList<>();

  private FaultMonitor() {}

  public static FaultMonitor getInstance() {
    if (instance == null) {
      instance = new FaultMonitor();
    }
    return instance;
  }

  /**
   * Registers a fault condition.
   *
   * @param name Short, human-readable, e.g. "Camera 0 disconnected"
   * @param isFaulted Returns true when the fault is present
   */
  public void register(String name, BooleanSupplier isFaulted) {
    faults.put(name, isFaulted);
  }

  /** Call once per loop from {@code robotPeriodic()}. */
  public void periodic() {
    activeFaults.clear();

    for (var entry : faults.entrySet()) {
      boolean faulted = entry.getValue().getAsBoolean();
      Logger.recordOutput("RobotHealth/Faults/" + entry.getKey(), faulted);
      if (faulted) {
        activeFaults.add(entry.getKey());
      }
    }

    boolean healthy = activeFaults.isEmpty();

    Logger.recordOutput("RobotHealth/OK", healthy);
    Logger.recordOutput("RobotHealth/ActiveFaultCount", activeFaults.size());
    Logger.recordOutput("RobotHealth/ActiveFaults", activeFaults.toArray(new String[0]));

    // Dashboard mirror — this is the one the pit actually watches.
    SmartDashboard.putBoolean("Robot OK", healthy);
    SmartDashboard.putString("Active Faults", healthy ? "None" : String.join(", ", activeFaults));
  }

  /** True when no registered fault is currently present. */
  public boolean isHealthy() {
    return activeFaults.isEmpty();
  }
}
