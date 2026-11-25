package frc.lib.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.driverstation.RobotState;

/**
 * A number that can be adjusted from the dashboard while the robot is running.
 *
 * <p><b>Live only in test mode.</b> There is no tuning-mode flag: every instance publishes to
 * NetworkTables under {@code Tuning/}, but {@link #get()} returns the dashboard value only while
 * the robot is enabled in test mode, and the compiled-in default in every other mode. The field
 * never enables test mode, so a stray dashboard edit at an event reaches nothing — enforced here,
 * not by convention. The dashboard value is an AdvantageKit logged input ({@link
 * LoggedNetworkNumber}), so a replay sees exactly what the robot saw.
 *
 * <p><b>Still, only a tuning command should read one.</b> Match code reads compiled values from
 * {@code Constants.Setpoints}; a tunable there is dead weight that reads its default. Each scaffold
 * in {@code template/} has a {@code tune*} command showing the pattern. At the 2027 port that
 * command becomes a {@code @Utility} OpMode.
 *
 * <p><b>Use it for setpoints — velocities, positions, voltages — not for gains.</b> Phoenix gains
 * are tuned in Phoenix Tuner X and roboRIO-side gains in simulation with {@code /pid-tune}; either
 * way the result is committed as a plain constant. See {@code docs/tunables.md}.
 *
 * <pre>
 * private static final LoggedTunableNumber TUNE_RPM = new LoggedTunableNumber("Intake/TuneRpm", 0);
 *
 * // In the tuning command only:
 * if (TUNE_RPM.hasChanged(hashCode())) {
 *   intake.setVelocity(RPM.of(TUNE_RPM.get()));
 * }
 * </pre>
 *
 * <p><b>Values found this way are not final.</b> They live in NetworkTables, not in git. Write the
 * number you land on into {@code Constants.Setpoints} and commit it, or it is gone at the next
 * boot.
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
   * @param defaultValue The starting dashboard value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.defaultValue = defaultValue;
    this.dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
  }

  /**
   * Returns the dashboard value <b>only while the robot is enabled in test mode</b>, and the
   * compiled-in default otherwise.
   *
   * <p>This is the enforcement behind "only a tuning command may read a tunable". Tuning commands
   * run in test mode, which the field never enables, so in auto and teleop every tunable reads its
   * default no matter what is on the dashboard or left over in NetworkTables. A tunable read by
   * match code by mistake is therefore inert rather than live. Test mode is part of the logged
   * Driver Station state, so replay takes the same branch the robot did.
   */
  public double get() {
    return RobotState.isUtilityEnabled() ? dashboardNumber.get() : defaultValue;
  }

  /**
   * @return the value this tunable was constructed with, regardless of dashboard edits
   */
  public double getDefault() {
    return defaultValue;
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
