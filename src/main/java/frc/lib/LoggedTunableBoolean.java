package frc.lib;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * A boolean that can be toggled from the dashboard while the robot is running, but only when tuning
 * mode is on. The boolean counterpart to {@link LoggedTunableNumber} — see that class for the
 * reasoning and the caveat about values living in NetworkTables rather than git.
 *
 * <p>Useful for toggling a behavior under test without a redeploy: an alternate control strategy, a
 * filter you want to A/B, a debug output that is too expensive to leave on.
 *
 * <p><b>Do not gate safety behavior on one of these.</b> When tuning mode is off this always
 * returns its compiled-in default, so a interlock written as a tunable would silently take the
 * default at competition. Interlocks belong in code.
 */
public class LoggedTunableBoolean implements BooleanSupplier {

  private static final String TABLE_KEY = "Tuning";

  private final String key;
  private final boolean defaultValue;
  private final LoggedNetworkBoolean dashboardBoolean;

  private final Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  /**
   * @param dashboardKey Key shown on the dashboard, under the "Tuning" table
   * @param defaultValue Value used when tuning mode is off, and the starting dashboard value
   */
  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.defaultValue = defaultValue;
    this.dashboardBoolean =
        Constants.tuningMode ? new LoggedNetworkBoolean(key, defaultValue) : null;
  }

  /**
   * @return the dashboard value when tuning mode is on, otherwise the compiled-in default
   */
  public boolean get() {
    return dashboardBoolean == null ? defaultValue : dashboardBoolean.get();
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }

  /**
   * Returns true when the value has changed since the last call with this id.
   *
   * @param id Stable identifier for the caller
   */
  public boolean hasChanged(int id) {
    boolean current = get();
    Boolean last = lastHasChangedValues.get(id);
    if (last == null || last != current) {
      lastHasChangedValues.put(id, current);
      return true;
    }
    return false;
  }
}
