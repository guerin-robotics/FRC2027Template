package frc.lib.device.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

/** A beam break on a roboRIO DIO port. */
public class BeamBreakIODigitalInput implements BeamBreakIO {

  private final DigitalInput input;
  private final boolean brokenWhenHigh;

  /**
   * @param port From {@code Constants.DioPorts}
   * @param brokenWhenHigh Most IR beam breaks pull the line low when the beam is broken, so this is
   *     usually false. Check it on the bench with the sensor's LED before trusting it on the robot.
   */
  public BeamBreakIODigitalInput(int port, boolean brokenWhenHigh) {
    input = new DigitalInput(port);
    this.brokenWhenHigh = brokenWhenHigh;
  }

  @Override
  public void updateInputs(BeamBreakInputs inputs) {
    inputs.connected = true;
    inputs.broken = input.get() == brokenWhenHigh;
  }
}
