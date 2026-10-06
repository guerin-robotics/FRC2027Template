package frc.lib.device.beambreak;

import org.littletonrobotics.junction.AutoLog;

/**
 * A beam break: a roboRIO digital input or a LaserCAN on the robot, a supplier in simulation,
 * {@code new BeamBreakIO() {}} in replay.
 */
public interface BeamBreakIO {

  @AutoLog
  class BeamBreakInputs {
    /**
     * False only for a CAN beam break that has dropped off the bus. A DIO input has no way to tell
     * "unplugged" from "beam clear" and always reports true — wire it so a broken wire reads as
     * broken, not clear, if that matters for the mechanism.
     */
    public boolean connected = false;

    /** Something is in the beam. Already corrected for the sensor's polarity. */
    public boolean broken = false;
  }

  default void updateInputs(BeamBreakInputs inputs) {}
}
