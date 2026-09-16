package frc.lib.mechanism.linear;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@link SubsystemBase} wrapping one {@link LinearMechanism}.
 *
 * <pre>{@code
 * public class Elevator extends LinearSubsystem {
 *   public Elevator(LinearMechanism mechanism) {
 *     super(mechanism);
 *   }
 * }
 * }</pre>
 *
 * <p>See {@code RollerSubsystem} for why the named subclass is worth writing, and for what to do
 * when one subsystem owns more than one mechanism.
 */
public class LinearSubsystem extends SubsystemBase {

  protected final LinearMechanism linear;

  public LinearSubsystem(LinearMechanism linear) {
    this.linear = linear;
    setName(linear.getName());
  }

  @Override
  public void periodic() {
    linear.periodic();
  }

  /** The mechanism itself, for anything this wrapper does not forward. */
  public LinearMechanism mechanism() {
    return linear;
  }

  /** Registers this mechanism's hardware with {@code FaultMonitor}. Call once at wiring time. */
  public void registerFaultMonitors() {
    linear.registerFaultMonitors();
  }

  // ---- Control ----

  /** Drives to a height along the Motion Magic profile. Clamped to the travel bounds. */
  public void setPosition(Distance height) {
    linear.setPosition(height);
  }

  /** Open-loop. Bring-up, characterization and the zeroing routine. */
  public void setVoltage(Voltage volts) {
    linear.setVoltage(volts);
  }

  /**
   * Open-loop torque current.
   *
   * <p>The right mode for driving into a hard stop during zeroing: current is torque, so a small
   * value is a small, bounded push rather than a voltage that turns into whatever current the
   * mechanism's impedance allows.
   */
  public void setTorqueCurrent(Current amps) {
    linear.setTorqueCurrent(amps);
  }

  /** Tells the mechanism the carriage is currently at {@code height}. */
  public void zeroAt(Distance height) {
    linear.zeroAt(height);
  }

  /**
   * Stops commanding output.
   *
   * <p>An elevator under gravity descends from here, braked but unpowered. Holding a height means
   * continuing to command it — see {@code LinearCommands.holdAt}.
   */
  public void stop() {
    linear.stop();
  }

  // ---- State ----

  public Distance getHeight() {
    return linear.getHeight();
  }

  public LinearVelocity getLinearVelocity() {
    return linear.getLinearVelocity();
  }

  public Distance getGoalPosition() {
    return linear.getGoalPosition();
  }

  public boolean isAtPosition() {
    return linear.isAtPosition();
  }

  /** Within {@code tolerance} of {@code target}, for a caller needing its own tolerance. */
  public boolean nearGoal(Distance target, Distance tolerance) {
    return linear.nearGoal(target, tolerance);
  }

  /** Is every device on this mechanism present? */
  public boolean isConnected() {
    return linear.isConnected();
  }
}
