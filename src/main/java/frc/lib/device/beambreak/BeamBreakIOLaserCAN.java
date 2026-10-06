package frc.lib.device.beambreak;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.units.measure.Distance;
import frc.lib.device.distance.DistanceSensorIO.DistanceSensorInputs;
import frc.lib.device.distance.DistanceSensorIOLaserCAN;

/**
 * A LaserCAN used as a beam break: broken when a valid reading is closer than a threshold.
 *
 * <p>Unlike a DIO beam break, this one knows when it has gone missing, so {@code connected} is
 * real. An invalid reading — nothing in range, or swamped by light — reads as clear, never broken.
 */
public class BeamBreakIOLaserCAN implements BeamBreakIO {

  private final DistanceSensorIOLaserCAN sensor;
  private final DistanceSensorInputs reading = new DistanceSensorInputs();
  private final Distance threshold;

  /**
   * @param threshold Closer than this counts as broken. Measure it with a game piece in place and
   *     with one absent; set it between the two with margin.
   * @see DistanceSensorIOLaserCAN#DistanceSensorIOLaserCAN for the remaining parameters
   */
  public BeamBreakIOLaserCAN(
      String name,
      int canId,
      Distance threshold,
      RangingMode mode,
      RegionOfInterest roi,
      TimingBudget budget) {
    this.sensor = new DistanceSensorIOLaserCAN(name, canId, mode, roi, budget);
    this.threshold = threshold;
  }

  @Override
  public void updateInputs(BeamBreakInputs inputs) {
    sensor.updateInputs(reading);
    inputs.connected = reading.connected;
    inputs.broken = reading.valid && reading.distance.lt(threshold);
  }
}
