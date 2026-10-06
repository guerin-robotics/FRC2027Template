package frc.lib.device.lights;

import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

/**
 * An LED controller: CANdle on the robot, {@code LightsIOSim} in simulation, {@code new LightsIO()
 * {}} in replay.
 *
 * <p>Deliberately small. The robot says what it wants to show — a colour, a strobe for "look at
 * me", a rainbow for "idle" — and the IO decides how the hardware does it. Anything richer belongs
 * in a command that calls these, not in vendor control requests leaking into robot code.
 */
public interface LightsIO {

  @AutoLog
  class LightsInputs {
    public boolean connected = false;
  }

  default void updateInputs(LightsInputs inputs) {}

  /** Every LED off, animations included. */
  default void off() {}

  /** Every LED one steady colour. */
  default void setSolid(Color color) {}

  /** Every LED flashing one colour. */
  default void setStrobe(Color color) {}

  /** A moving rainbow across every LED. */
  default void setRainbow() {}
}
