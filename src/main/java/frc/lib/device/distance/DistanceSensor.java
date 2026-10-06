package frc.lib.device.distance;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FaultMonitor;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * A distance sensor, owned by a subsystem.
 *
 * <p>Not a subsystem itself: the owner calls {@link #periodic()} from its own {@code periodic()}.
 * Skip it and {@code Logger.processInputs} stops running, which breaks replay.
 */
public class DistanceSensor {

  private final String name;
  private final DistanceSensorIO io;
  private final DistanceSensorInputsAutoLogged inputs = new DistanceSensorInputsAutoLogged();

  /**
   * @param name Log key and fault prefix, e.g. "Indexer/Sensor"
   */
  public DistanceSensor(String name, DistanceSensorIO io) {
    this.name = name;
    this.io = io;
  }

  /** Call once per loop from the owning subsystem's {@code periodic()}. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs); // NEVER remove — it is what makes replay work
  }

  /** Call once from {@code RobotContainer} at wiring time. */
  public void registerFaultMonitors() {
    FaultMonitor monitor = FaultMonitor.getInstance();
    monitor.register(name + " disconnected", () -> !inputs.connected);
    monitor.register(name + " rebooted while enabled", () -> inputs.stickyBootDuringEnable);
  }

  /** The distance, or empty when nothing valid is in range. Never a stale or sentinel value. */
  public Optional<Distance> getDistance() {
    return inputs.valid ? Optional.of(inputs.distance) : Optional.empty();
  }

  /** True while a valid reading lies within {@code [min, max]}. False with no valid reading. */
  public boolean isWithin(Distance min, Distance max) {
    return inputs.valid && inputs.distance.gte(min) && inputs.distance.lte(max);
  }

  /**
   * A trigger on {@link #isWithin}. Build it once, in {@code Triggers.java}, named for the robot
   * action it stands for — "game piece staged", not "sensor in range".
   */
  public Trigger within(Distance min, Distance max) {
    return new Trigger(() -> isWithin(min, max));
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public String getName() {
    return name;
  }
}
