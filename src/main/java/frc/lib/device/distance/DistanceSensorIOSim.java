package frc.lib.device.distance;

import edu.wpi.first.units.measure.Distance;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Reports a distance chosen by the simulation — typically from game-piece state in {@code
 * RobotState}. An empty optional reads as "nothing in range".
 */
public class DistanceSensorIOSim implements DistanceSensorIO {

  private final Supplier<Optional<Distance>> distance;

  public DistanceSensorIOSim(Supplier<Optional<Distance>> distance) {
    this.distance = distance;
  }

  @Override
  public void updateInputs(DistanceSensorInputs inputs) {
    Optional<Distance> reading = distance.get();
    inputs.connected = true;
    inputs.valid = reading.isPresent();
    reading.ifPresent(d -> inputs.distance = d);
    inputs.ambientSignal = 0.0;
    inputs.stickyBootDuringEnable = false;
  }
}
