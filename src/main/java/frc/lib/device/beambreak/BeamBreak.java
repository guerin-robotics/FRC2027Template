package frc.lib.device.beambreak;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FaultMonitor;
import org.littletonrobotics.junction.Logger;

/**
 * A beam break, owned by a subsystem.
 *
 * <p>Not a subsystem itself: the owner calls {@link #periodic()} from its own {@code periodic()}.
 * Skip it and {@code Logger.processInputs} stops running, which breaks replay.
 */
public class BeamBreak {

  private final String name;
  private final BeamBreakIO io;
  private final BeamBreakInputsAutoLogged inputs = new BeamBreakInputsAutoLogged();

  /**
   * @param name Log key and fault prefix, e.g. "Indexer/BeamBreak"
   */
  public BeamBreak(String name, BeamBreakIO io) {
    this.name = name;
    this.io = io;
  }

  /** Call once per loop from the owning subsystem's {@code periodic()}. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs); // NEVER remove — it is what makes replay work
  }

  /** Call once from {@code RobotContainer} at wiring time. Only CAN beam breaks can disconnect. */
  public void registerFaultMonitors() {
    FaultMonitor.getInstance().register(name + " disconnected", () -> !inputs.connected);
  }

  /** Something is in the beam. False while disconnected. */
  public boolean isBroken() {
    return inputs.connected && inputs.broken;
  }

  /**
   * A trigger on {@link #isBroken}. Build it once, in {@code Triggers.java}, named for what it
   * means — "piece staged", not "beam broken".
   */
  public Trigger broken() {
    return new Trigger(this::isBroken);
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public String getName() {
    return name;
  }
}
