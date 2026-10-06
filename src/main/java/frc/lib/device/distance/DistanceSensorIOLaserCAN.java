package frc.lib.device.distance;

import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.device.CanIdRegistry;

/**
 * A Grapple LaserCAN. It lives on the roboRIO bus — the driver takes no bus argument — so it claims
 * its ID against {@code "rio"}, where it can collide with a TalonFX.
 *
 * <p>Ranging mode, region of interest and timing budget are configured once at construction, with
 * the same retry {@code PhoenixUtil.tryUntilOk} gives CTRE devices: a LaserCAN that is still
 * booting rejects configuration, and one that silently keeps its defaults measures a different cone
 * than the code assumes.
 *
 * <p>{@code connected} means a measurement arrived recently. {@code valid} additionally requires
 * the sensor to call it a good one — noise, weak signal and out-of-range readings are all reported
 * as connected but invalid, which is the difference between "unplugged" and "nothing there".
 */
public class DistanceSensorIOLaserCAN implements DistanceSensorIO {

  /** The LaserCAN bus name. Must match {@code Constants.CanIds.RIO_BUS}. */
  private static final String BUS = "rio";

  private static final int CONFIG_ATTEMPTS = 5;

  private final LaserCan sensor;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * @param name Owner name for the CAN ID registry, e.g. "Indexer sensor"
   * @param canId From {@code Constants.CanIds}, set on the device in GrappleHook
   * @param mode {@code SHORT} for anything inside about 1.3 m — far less sensitive to ambient light
   * @param roi Region of interest in SPAD units, e.g. {@code new RegionOfInterest(8, 8, 16, 16)}
   *     for the full field of view; narrow it to ignore a nearby frame member
   * @param budget Longer is steadier but slower; 33 ms is a sensible start inside a 20 ms loop
   */
  public DistanceSensorIOLaserCAN(
      String name, int canId, RangingMode mode, RegionOfInterest roi, TimingBudget budget) {
    CanIdRegistry.claim(canId, BUS, name);
    sensor = new LaserCan(canId);
    configure(name, () -> sensor.setRangingMode(mode));
    configure(name, () -> sensor.setRegionOfInterest(roi));
    configure(name, () -> sensor.setTimingBudget(budget));
  }

  @Override
  public void updateInputs(DistanceSensorInputs inputs) {
    Measurement measurement = sensor.getMeasurement();
    inputs.connected = connectedDebounce.calculate(measurement != null);
    inputs.stickyBootDuringEnable = false;
    if (measurement == null) {
      inputs.valid = false;
      return;
    }
    inputs.valid = measurement.status == LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
    inputs.distance = Millimeters.of(measurement.distance_mm);
    inputs.ambientSignal = measurement.ambient;
  }

  private interface ConfigStep {
    void run() throws ConfigurationFailedException;
  }

  private static void configure(String name, ConfigStep step) {
    for (int attempt = 1; attempt <= CONFIG_ATTEMPTS; attempt++) {
      try {
        step.run();
        return;
      } catch (ConfigurationFailedException e) {
        if (attempt == CONFIG_ATTEMPTS) {
          DriverStation.reportError(
              name + " LaserCAN configuration failed after " + CONFIG_ATTEMPTS + " attempts",
              false);
        }
      }
    }
  }
}
