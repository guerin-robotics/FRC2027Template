package frc.lib.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * A boolean that can be toggled from the dashboard while the robot is running. The boolean
 * counterpart to {@link LoggedTunableNumber}: always live, and <b>only a tuning command may read
 * one</b> — see that class for why that rule is what makes always-live safe.
 *
 * <p>Useful inside a tuning command for toggling a behavior under test without a redeploy.
 *
 * <p><b>Never gate safety behavior on one of these.</b> It is live in every mode, so an interlock
 * written as a tunable is one dashboard click from being off. Interlocks belong in code.
 */
public class LoggedTunableBoolean implements BooleanSupplier {

  private static final String TABLE_KEY = "Tuning";

  private final String key;
  private final boolean defaultValue;
  private final LoggedNetworkBoolean dashboardBoolean;

  private final Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  /**
   * @param dashboardKey Key shown on the dashboard, under the "Tuning" table
   * @param defaultValue The starting dashboard value
   */
  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.defaultValue = defaultValue;
    this.dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
  }

  /**
   * @return the current dashboard value. Only a tuning command should call this
   */
  public boolean get() {
    return dashboardBoolean.get();
  }

  /**
   * @return the value this tunable was constructed with, regardless of dashboard edits
   */
  public boolean getDefault() {
    return defaultValue;
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
