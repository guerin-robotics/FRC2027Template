package frc.lib.mechanism.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

/**
 * The command factories every velocity-controlled mechanism needs.
 *
 * <p>Static factories rather than command classes, per {@code .claude/rules/03-commands.md}: they
 * compose, they are testable, and their names show up in the AdvantageKit command log. Every one
 * here names itself from the subsystem, so {@code Shooter_Velocity} and {@code Intake_Velocity} are
 * distinguishable in a log without anyone having to remember to pass a name.
 *
 * <h2>Setpoints are parameters</h2>
 *
 * <p>Nothing here reads a setpoint and nothing here hardcodes one. The caller — almost always
 * {@code RobotContainer} — supplies it from {@code Constants.Setpoints}, so a driver asking for "a
 * bit more intake speed" sends you to one file rather than on a hunt across three.
 *
 * <p>A mechanism with verbs of its own still writes its own factory class. This covers the verbs
 * every roller shares; it is not meant to cover a scoring sequence.
 */
public final class RollerCommands {

  private RollerCommands() {}

  /**
   * Runs at a fixed velocity while scheduled, and stops on the way out.
   *
   * <p>The shape for "run while the button is held". Stopping on end is not optional — a command
   * that ends without stopping leaves the mechanism running, and the next thing to notice is
   * usually a brownout.
   */
  public static Command runAtVelocity(RollerSubsystem roller, AngularVelocity velocity) {
    return Commands.startEnd(() -> roller.setVelocity(velocity), roller::stop, roller)
        .withName(roller.getName() + "_Velocity");
  }

  /**
   * Runs at a velocity that is re-read every loop.
   *
   * <p>For a setpoint that depends on something live — distance to a target, a dashboard value
   * during a tuning session. Prefer the fixed form when the value does not actually change; a
   * supplier that returns a constant is a supplier somebody will later assume is dynamic.
   */
  public static Command runAtVelocity(
      RollerSubsystem roller, Supplier<AngularVelocity> velocitySupplier) {
    return Commands.run(() -> roller.setVelocity(velocitySupplier.get()), roller)
        .finallyDo(roller::stop)
        .withName(roller.getName() + "_VelocitySupplied");
  }

  /** Open-loop voltage while scheduled. Bring-up and characterization, not match logic. */
  public static Command runAtVoltage(RollerSubsystem roller, Voltage volts) {
    return Commands.startEnd(() -> roller.setVoltage(volts), roller::stop, roller)
        .withName(roller.getName() + "_Voltage");
  }

  /** Stops, once, and finishes. */
  public static Command stop(RollerSubsystem roller) {
    return Commands.runOnce(roller::stop, roller).withName(roller.getName() + "_Stop");
  }

  /**
   * The default command: hold the mechanism stopped.
   *
   * <p>{@code Commands.run}, not {@code runOnce} — a default command must not end, or the scheduler
   * immediately reschedules it and the command log fills with one entry per loop.
   */
  public static Command idle(RollerSubsystem roller) {
    return Commands.run(roller::stop, roller).withName(roller.getName() + "_Idle");
  }

  /**
   * Spins up and finishes once at speed, or when the budget runs out.
   *
   * <p>The timeout is mandatory and is the whole point. A robot that hangs waiting for a mechanism
   * that will never reach its setpoint scores zero; one that gives up and acts anyway usually
   * scores something. Note that this <b>finishes</b> at speed rather than stopping — it is the
   * first phase of a sequence, and whatever runs next is expected to keep commanding the velocity.
   */
  public static Command spinUpTo(RollerSubsystem roller, AngularVelocity velocity, Time timeout) {
    return Commands.sequence(
            Commands.runOnce(() -> roller.setVelocity(velocity), roller),
            Commands.waitUntil(roller::isAtVelocity).withTimeout(timeout))
        .withName(roller.getName() + "_SpinUp");
  }

  /**
   * Reverses for a fixed time to clear a jam.
   *
   * <p>The response to a jam, kept separate from its detection deliberately. Reversing can eject a
   * game piece the driver wanted, which makes it a game-strategy decision rather than a hardware
   * one — the mechanism owns the signal, and the caller decides whether and when to act on it.
   *
   * <p>Bind it with {@code new Trigger(roller::isJammed).onTrue(unjam(...))} when the answer is
   * always yes, or leave it on a driver button when it is not.
   */
  public static Command unjam(
      RollerSubsystem roller, AngularVelocity reverseVelocity, Time duration) {
    return Commands.startEnd(() -> roller.setVelocity(reverseVelocity), roller::stop, roller)
        .withTimeout(duration)
        .withName(roller.getName() + "_Unjam");
  }
}
