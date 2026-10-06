package frc.lib.device.beambreak;

import java.util.function.BooleanSupplier;

/**
 * Reports whatever the simulation says is in the beam — typically game-piece state from {@code
 * RobotState}.
 */
public class BeamBreakIOSim implements BeamBreakIO {

  private final BooleanSupplier broken;

  public BeamBreakIOSim(BooleanSupplier broken) {
    this.broken = broken;
  }

  @Override
  public void updateInputs(BeamBreakInputs inputs) {
    inputs.connected = true;
    inputs.broken = broken.getAsBoolean();
  }
}
