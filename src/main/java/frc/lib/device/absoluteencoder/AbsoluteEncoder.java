package frc.lib.device.absoluteencoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.util.FaultMonitor;
import org.littletonrobotics.junction.Logger;

/**
 * A standalone absolute encoder, owned by a subsystem.
 *
 * <p>Not a subsystem itself, for the same reason {@code RollerMechanism} is not: one subsystem may
 * own several devices. The owner calls {@link #periodic()} from its own {@code periodic()} — skip
 * it and {@code Logger.processInputs} stops running, which breaks replay.
 */
public class AbsoluteEncoder {

  private final String name;
  private final AbsoluteEncoderIO io;
  private final AbsoluteEncoderInputsAutoLogged inputs = new AbsoluteEncoderInputsAutoLogged();

  /**
   * @param name Log key and fault prefix, e.g. "Turret/Encoder"
   */
  public AbsoluteEncoder(String name, AbsoluteEncoderIO io) {
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
    monitor.register(name + " magnet red", () -> inputs.connected && inputs.magnetHealth == 1);
  }

  public Angle getAbsolutePosition() {
    return inputs.absolutePosition;
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public String getName() {
    return name;
  }
}
