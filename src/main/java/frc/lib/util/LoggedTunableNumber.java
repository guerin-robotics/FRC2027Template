package frc.lib.util;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that can be adjusted from the dashboard while the robot is running, but only when tuning
 * mode is on.
 *
 * <p><b>Why this exists.</b> Without it, changing a gain costs an edit, a build, a deploy and a
 * re-enable — call it two minutes per iteration. A tuning session is dozens of iterations, so most
 * of it is spent waiting rather than tuning. With it, you drag a number and watch the response.
 *
 * <p><b>Why it is gated.</b> When {@link Constants#tuningMode} is false this does not touch
 * NetworkTables at all — it returns the compiled-in default and costs nothing. That matters for two
 * reasons: NT traffic in the 20 ms loop is not free, and more importantly a competition robot must
 * not be one stray dashboard edit away from a different gain set. Tuning mode off is the
 * competition configuration.
 *
 * <p><b>Usage:</b>
 *
 * <pre>
 * private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 4.0);
 *
 * // Read it wherever you need the value:
 * controller.setP(kP.get());
 *
 * // Or only rebuild expensive objects when it actually changed:
 * if (kP.hasChanged(hashCode())) {
 *   controller.setP(kP.get());
 * }
 * </pre>
 *
 * <p><b>Values found this way are not final.</b> They live in NetworkTables, not in git. When a
 * tuning session lands on numbers you like, write them into the constants and commit them, or they
 * are gone the next time the robot boots. See {@code docs/characterization-and-tuning.md}.
 */
public class LoggedTunableNumber implements DoubleSupplier {

  private static final String TABLE_KEY = "Tuning";

  private final String key;
  private final double defaultValue;
  private final LoggedNetworkNumber dashboardNumber;

  /** Last value observed per caller id, for {@link #hasChanged(int)}. */
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * @param dashboardKey Key shown on the dashboard, under the "Tuning" table
   * @param defaultValue Value used when tuning mode is off, and the starting dashboard value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.defaultValue = defaultValue;
    this.dashboardNumber = Constants.tuningMode ? new LoggedNetworkNumber(key, defaultValue) : null;
  }

  /**
   * @return the dashboard value when tuning mode is on, otherwise the compiled-in default
   */
  public double get() {
    return dashboardNumber == null ? defaultValue : dashboardNumber.get();
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * Returns true when the value has changed since the last call with this id.
   *
   * <p>Use this to avoid rebuilding controllers or re-applying motor configs every loop. Pass a
   * stable id unique to the call site — {@code hashCode()} of the calling object is the usual
   * choice. Two call sites sharing an id will steal each other's change notifications.
   *
   * @param id Stable identifier for the caller
   */
  public boolean hasChanged(int id) {
    double current = get();
    Double last = lastHasChangedValues.get(id);
    if (last == null || last != current) {
      lastHasChangedValues.put(id, current);
      return true;
    }
    return false;
  }

  /**
   * Convenience for the common case of several related gains: runs {@code action} if any of the
   * given tunables changed.
   *
   * @param id Stable identifier for the caller
   * @param action What to run when something changed
   * @param tunables The tunables to watch
   */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunables) {
    boolean changed = false;
    for (LoggedTunableNumber tunable : tunables) {
      // Deliberately not short-circuiting: every tunable must record its value, or the ones
      // after the first change would report changed again on the next call.
      changed |= tunable.hasChanged(id);
    }
    if (changed) {
      action.run();
    }
  }
}
