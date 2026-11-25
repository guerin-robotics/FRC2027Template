package frc.lib.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.driverstation.RobotState;

/**
 * A boolean that can be toggled from the dashboard while the robot is running. The boolean
 * counterpart to {@link LoggedTunableNumber}: live only in test mode, and meant to be read only by
 * a tuning command — see that class.
 *
 * <p>Useful inside a tuning command for toggling a behavior under test without a redeploy.
 *
 * <p><b>Never gate safety behavior on one of these.</b> In test mode it is one dashboard click from
 * flipping, and everywhere else it silently takes its default. Interlocks belong in code.
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
   * Returns the dashboard value <b>only while the robot is enabled in test mode</b>, and the
   * compiled-in default otherwise.
   *
   * <p>This is the enforcement behind "only a tuning command may read a tunable". Tuning commands
   * run in test mode, which the field never enables, so in auto and teleop every tunable reads its
   * default no matter what is on the dashboard or left over in NetworkTables. A tunable read by
   * match code by mistake is therefore inert rather than live. Test mode is part of the logged
   * Driver Station state, so replay takes the same branch the robot did.
   */
  public boolean get() {
    return RobotState.isUtilityEnabled() ? dashboardBoolean.get() : defaultValue;
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
