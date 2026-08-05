package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks current draw, power consumption, and cumulative energy usage for every subsystem.
 *
 * <p>Each subsystem calls {@link #reportCurrentUsage(String, boolean, double...)} once per loop
 * inside its {@code periodic()} method, passing the supply-current readings (in amps) from its
 * motors. The logger aggregates the data and writes it to AdvantageKit under the {@code
 * BatteryLogger/} namespace so it can be viewed in AdvantageScope.
 *
 * <p>Current and power are instantaneous. <b>Energy is integrated once per cycle against the
 * measured interval since the last cycle</b>, not against an assumed 20 ms — so the totals stay
 * correct when loops overrun, which is the condition under which you most want to trust them. The
 * measured interval is published as {@code BatteryLogger/LoopSeconds}.
 *
 * <p>Adapted from <a
 * href="https://github.com/Mechanical-Advantage/RobotCode2026Public">6328&rsquo;s
 * BatteryLogger</a>.
 */
public class BatteryLogger {

  /**
   * Loop period used for the very first cycle, before two timestamps exist to subtract.
   *
   * <p>Every cycle after that measures the real interval. Nothing else in this class assumes a loop
   * period.
   */
  private static final double NOMINAL_LOOP_SECONDS = 0.02;

  /**
   * Ceiling on a single cycle's measured interval, in seconds.
   *
   * <p>Without this, any gap in execution — a disable, a breakpoint, the first loop after code
   * start — is integrated as if the robot had been drawing that instant's current for the whole
   * gap, and dumps a slug of phantom energy into the totals. Five nominal loops is generous enough
   * never to clip a real overrun and tight enough that a pause cannot distort the match total.
   */
  private static final double MAX_LOOP_SECONDS = 0.1;

  /** Timestamp of the previous {@link #periodicAfterScheduler()}, in microseconds. */
  private long lastTimestampMicros = 0;

  /** Measured length of the last cycle, in seconds. Logged so the correction is visible. */
  private double lastLoopSeconds = NOMINAL_LOOP_SECONDS;

  // ---- Running totals for the current loop cycle ----
  private double totalCurrent = 0.0;
  private double driveCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;

  // Battery voltage, updated each loop from RobotController
  private double batteryVoltage = 12.6;

  // RoboRIO current, set from Robot.java each loop
  private double rioCurrent = 0.0;

  // ---- Brownout tracking ----
  //
  // The 2026 robot logged 491 brownouts across 23 matches and it was never noticed at an
  // event — the data was in the logs, but nothing counted it or said so out loud. A brownout
  // cuts motor output without restarting code, so from the driver's seat it just feels like
  // the robot got weak. These counters make it a number instead of a feeling.
  private boolean wasBrownedOut = false;
  private int brownoutCount = 0;
  private double minVoltage = Double.POSITIVE_INFINITY;

  private final Alert brownoutAlert =
      new Alert("Brownout detected — check battery and drive current limits.", AlertType.kWarning);

  // ---- Per-subsystem maps ----
  private final Map<String, Double> subsystemCurrents = new HashMap<>();
  private final Map<String, Double> subsystemPowers = new HashMap<>();
  private final Map<String, Double> subsystemEnergies = new HashMap<>();

  /**
   * Reports the current draw of one or more motors belonging to a subsystem.
   *
   * @param key Hierarchical key, e.g. {@code "Drive/Module0-Drive"}. Slashes or hyphens separate
   *     levels so parent groups are automatically aggregated.
   * @param isDrive {@code true} if this current contributes to the drivetrain total.
   * @param amps One value per motor — supply current in amps.
   */
  public void reportCurrentUsage(String key, boolean isDrive, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) {
      totalAmps += Math.abs(amp);
    }

    if (isDrive) {
      driveCurrent += totalAmps;
    }

    // Power only. Energy is integrated once per cycle in periodicAfterScheduler(), against the
    // measured interval — a subsystem reporting here has no way to know how long this loop took.
    double power = totalAmps * batteryVoltage;

    totalCurrent += totalAmps;
    totalPower += power;

    // Store the leaf key
    subsystemCurrents.put(key, totalAmps);
    subsystemPowers.put(key, power);

    // Aggregate parent keys (split on "/" or "-")
    String[] keys = key.split("/|-");
    if (keys.length < 2) {
      return;
    }

    String subkey = "";
    for (int i = 0; i < keys.length - 1; i++) {
      subkey += keys[i];
      if (i < keys.length - 2) {
        subkey += "/";
      }
      subsystemCurrents.merge(subkey, totalAmps, Double::sum);
      subsystemPowers.merge(subkey, power, Double::sum);
    }
  }

  /**
   * Call once per loop <b>after</b> all subsystem {@code periodic()} methods and the command
   * scheduler have run. Logs all aggregated data to AdvantageKit and resets per-cycle totals.
   */
  public void periodicAfterScheduler() {
    // Include fixed overhead currents
    reportCurrentUsage("Controls/roboRIO", false, rioCurrent);
    reportCurrentUsage("Controls/CANcoders", false, 0.05 * 4);
    reportCurrentUsage("Controls/Pigeon", false, 0.04);
    reportCurrentUsage("Controls/CANivore", false, 0.03);
    reportCurrentUsage("Controls/Radio", false, 0.5);

    // ---- Energy integration ----
    //
    // Done here, once, against the real interval since the last cycle. Integrating inside
    // reportCurrentUsage() against a fixed 20 ms was wrong by exactly the overrun ratio: at the
    // ~30 Hz the 2026 robot actually ran, every energy figure was roughly 40% low.
    //
    // Must run after the overhead currents above are reported and before the logging loops
    // below zero the power map.
    double dtSeconds = measureLoopSeconds();
    totalEnergy += totalPower * dtSeconds;
    for (var entry : subsystemPowers.entrySet()) {
      subsystemEnergies.merge(entry.getKey(), entry.getValue() * dtSeconds, Double::sum);
    }
    Logger.recordOutput("BatteryLogger/LoopSeconds", dtSeconds);

    // ---- Brownout / voltage tracking ----
    boolean brownedOut = RobotController.isBrownedOut();
    if (brownedOut && !wasBrownedOut) {
      brownoutCount++; // count edges, not loops spent browned out
    }
    wasBrownedOut = brownedOut;

    if (batteryVoltage < minVoltage) {
      minVoltage = batteryVoltage;
    }

    Logger.recordOutput("BatteryLogger/Voltage", batteryVoltage);
    Logger.recordOutput("BatteryLogger/MinVoltage", minVoltage);
    Logger.recordOutput("BatteryLogger/BrownedOutNow", brownedOut);
    Logger.recordOutput("BatteryLogger/BrownoutCount", brownoutCount);
    brownoutAlert.set(brownoutCount > 0);

    // Log totals
    Logger.recordOutput("BatteryLogger/Current", totalCurrent);
    Logger.recordOutput("BatteryLogger/DriveCurrent", driveCurrent);
    Logger.recordOutput("BatteryLogger/Power", totalPower);
    Logger.recordOutput("BatteryLogger/Energy", joulesToWattHours(totalEnergy));

    // Log per-subsystem breakdowns
    for (var entry : subsystemCurrents.entrySet()) {
      Logger.recordOutput("BatteryLogger/Current/" + entry.getKey(), entry.getValue());
      subsystemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemPowers.entrySet()) {
      Logger.recordOutput("BatteryLogger/Power/" + entry.getKey(), entry.getValue());
      subsystemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemEnergies.entrySet()) {
      Logger.recordOutput(
          "BatteryLogger/Energy/" + entry.getKey(), joulesToWattHours(entry.getValue()));
    }

    // Reset per-cycle totals (energy accumulates across the match)
    resetTotals();
  }

  // ---- Setters called from Robot.java each loop ----

  public void setBatteryVoltage(double voltage) {
    this.batteryVoltage = voltage;
  }

  public void setRioCurrent(double amps) {
    this.rioCurrent = amps;
  }

  // ---- Getters ----

  public double getTotalCurrent() {
    return totalCurrent;
  }

  public double getDriveCurrent() {
    return driveCurrent;
  }

  public double getTotalPower() {
    return totalPower;
  }

  public double getTotalEnergy() {
    return totalEnergy;
  }

  /** Number of brownout events since code start. */
  public int getBrownoutCount() {
    return brownoutCount;
  }

  /** Lowest battery voltage seen since code start, or since {@link #resetBrownoutTracking()}. */
  public double getMinVoltage() {
    return minVoltage == Double.POSITIVE_INFINITY ? 0.0 : minVoltage;
  }

  /**
   * Clears the brownout count and minimum voltage.
   *
   * <p>Call at the start of an enable if you want per-match numbers rather than per-power-cycle.
   * Leave it alone to accumulate across a whole session.
   */
  public void resetBrownoutTracking() {
    brownoutCount = 0;
    minVoltage = Double.POSITIVE_INFINITY;
    wasBrownedOut = false;
    brownoutAlert.set(false);
  }

  // ---- Helpers ----

  private void resetTotals() {
    totalPower = 0.0;
    totalCurrent = 0.0;
    driveCurrent = 0.0;
  }

  private double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }

  /**
   * Real seconds since the previous cycle, clamped.
   *
   * <p>Uses AdvantageKit's timestamp rather than {@code Timer.getFPGATimestamp()} on purpose.
   * During replay the AdvantageKit clock follows the log, so energy replays to the same numbers the
   * robot produced; wall-clock time would make a replay disagree with the match it is replaying,
   * which defeats the point of having one.
   */
  private double measureLoopSeconds() {
    long now = Logger.getTimestamp();

    if (lastTimestampMicros == 0) {
      // First cycle — nothing to subtract from yet.
      lastTimestampMicros = now;
      lastLoopSeconds = NOMINAL_LOOP_SECONDS;
      return lastLoopSeconds;
    }

    double elapsed = (now - lastTimestampMicros) / 1.0e6;
    lastTimestampMicros = now;
    lastLoopSeconds = MathUtil.clamp(elapsed, 0.0, MAX_LOOP_SECONDS);
    return lastLoopSeconds;
  }

  /** Measured length of the last loop, in seconds. */
  public double getLastLoopSeconds() {
    return lastLoopSeconds;
  }
}
