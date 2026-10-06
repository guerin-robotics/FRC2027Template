package frc.lib.device.lights;

/**
 * Accepts every request and reports itself connected. What the lights were asked to show is already
 * logged by {@link Lights}, so there is nothing to simulate.
 */
public class LightsIOSim implements LightsIO {

  @Override
  public void updateInputs(LightsInputs inputs) {
    inputs.connected = true;
  }
}
