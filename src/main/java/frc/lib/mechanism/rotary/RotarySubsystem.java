package frc.lib.mechanism.rotary;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@link SubsystemBase} wrapping one {@link RotaryMechanism}.
 *
 * <pre>{@code
 * public class Hood extends RotarySubsystem {
 *   public Hood(RotaryMechanism mechanism) {
 *     super(mechanism);
 *   }
 * }
 * }</pre>
 *
 * <p>See {@code RollerSubsystem} for why the named subclass is worth writing, and for what to do
 * when one subsystem owns more than one mechanism.
 */
public class RotarySubsystem extends SubsystemBase {

  protected final RotaryMechanism rotary;

  public RotarySubsystem(RotaryMechanism rotary) {
    this.rotary = rotary;
    setName(rotary.getName());
  }

  @Override
  public void periodic() {
    rotary.periodic();
  }

  /** The mechanism itself, for anything this wrapper does not forward. */
  public RotaryMechanism mechanism() {
    return rotary;
  }

  /** Registers this mechanism's hardware with {@code FaultMonitor}. Call once at wiring time. */
  public void registerFaultMonitors() {
    rotary.registerFaultMonitors();
  }

  // ---- Control ----

  /** Drives to an angle along the Motion Magic profile. Clamped to the travel bounds. */
  public void setPosition(Angle position) {
    rotary.setPosition(position);
  }

  /** Open-loop. Bring-up and characterization; match logic should close a loop. */
  public void setVoltage(Voltage volts) {
    rotary.setVoltage(volts);
  }

  /**
   * Stops commanding output.
   *
   * <p>Note what this does <b>not</b> do: it does not hold the mechanism where it is. A rotary
   * mechanism under gravity falls to its hard stop from here, braked but unpowered. Holding a
   * position means continuing to command it — see {@code RotaryCommands.holdAt}.
   */
  public void stop() {
    rotary.stop();
  }

  // ---- State ----

  public Angle getPosition() {
    return rotary.getPosition();
  }

  public Angle getGoalPosition() {
    return rotary.getGoalPosition();
  }

  public boolean isAtPosition() {
    return rotary.isAtPosition();
  }

  /** Within {@code tolerance} of {@code target}, for a caller needing its own tolerance. */
  public boolean nearGoal(Angle target, Angle tolerance) {
    return rotary.nearGoal(target, tolerance);
  }

  /** Is every device on this mechanism present? */
  public boolean isConnected() {
    return rotary.isConnected();
  }
}
