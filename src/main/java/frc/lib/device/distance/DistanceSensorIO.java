package frc.lib.device.distance;

import static org.wpilib.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Distance;

/**
 * A time-of-flight distance sensor: CANrange or LaserCAN on the robot, a supplier in simulation,
 * {@code new DistanceSensorIO() {}} in replay.
 */
public interface DistanceSensorIO {

  @AutoLog
  class DistanceSensorInputs {
    public boolean connected = false;

    /**
     * True when the reading is usable. A sensor can be connected and still have nothing in range,
     * or be swamped by ambient light; {@link #distance} is meaningless while this is false.
     */
    public boolean valid = false;

    public Distance distance = Meters.of(0);

    /** Ambient infrared. Rising ambient with falling validity is sunlight, not a broken sensor. */
    public double ambientSignal = 0.0;

    /** The sensor rebooted while the robot was enabled. Latches until cleared. CANrange only. */
    public boolean stickyBootDuringEnable = false;
  }

  default void updateInputs(DistanceSensorInputs inputs) {}
}
