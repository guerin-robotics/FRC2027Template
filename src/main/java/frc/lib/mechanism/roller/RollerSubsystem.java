package frc.lib.mechanism.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@link SubsystemBase} wrapping one {@link RollerMechanism}.
 *
 * <p>The common case, and it costs nothing:
 *
 * <pre>{@code
 * public class Shooter extends RollerSubsystem {
 *   public Shooter(RollerMechanism mechanism) {
 *     super(mechanism);
 *   }
 * }
 * }</pre>
 *
 * <p>The named subclass is worth writing even when it adds nothing. It gives the mechanism's own
 * predicates a home — {@code isReadyToShoot()} belongs on {@code Shooter}, not spread across
 * command factories — and it puts a real class name in stack traces and command logs. But nothing
 * requires it: a roller with no logic of its own can be constructed directly.
 *
 * <p>A subsystem that owns <b>two</b> mechanisms does not use this class. It extends {@code
 * SubsystemBase} itself, holds the mechanisms, and calls {@code periodic()} on each. That shape is
 * why {@link RollerMechanism} is not a subsystem in the first place.
 */
public class RollerSubsystem extends SubsystemBase {

  protected final RollerMechanism roller;

  public RollerSubsystem(RollerMechanism roller) {
    this.roller = roller;
    // The scheduler's name, the log keys and the alert prefixes all become the same string, so one
    // grep finds every trace of this mechanism.
    setName(roller.getName());
  }

  @Override
  public void periodic() {
    roller.periodic();
  }

  /** The mechanism itself, for anything this wrapper does not forward. */
  public RollerMechanism mechanism() {
    return roller;
  }

  /**
   * Registers this mechanism's hardware with {@code FaultMonitor}. Call once from {@code
   * RobotContainer} at wiring time.
   */
  public void registerFaultMonitors() {
    roller.registerFaultMonitors();
  }

  // ---- Control ----

  public void setVelocity(AngularVelocity velocity) {
    roller.setVelocity(velocity);
  }

  /** Open-loop. Bring-up and characterization; match logic should use velocity control. */
  public void setVoltage(Voltage volts) {
    roller.setVoltage(volts);
  }

  public void stop() {
    roller.stop();
  }

  // ---- State ----

  public AngularVelocity getVelocity() {
    return roller.getVelocity();
  }

  public AngularVelocity getGoalVelocity() {
    return roller.getGoalVelocity();
  }

  public boolean isAtVelocity() {
    return roller.isAtVelocity();
  }

  /** Always false on a mechanism with no jam detection configured. */
  public boolean isJammed() {
    return roller.isJammed();
  }

  /** Is every device on this mechanism present? */
  public boolean isConnected() {
    return roller.isConnected();
  }
}
