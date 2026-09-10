package frc.lib.mechanism.linear;

import static edu.wpi.first.units.Units.InchesPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.Supplier;

/**
 * The command factories every linear position mechanism needs, including the zeroing routine.
 *
 * <p>See {@code RollerCommands} for the conventions these follow. Setpoints are parameters: the
 * heights a mechanism is commanded to live in {@code Constants.Setpoints}.
 */
public final class LinearCommands {

  private LinearCommands() {}

  /**
   * Commands a height and keeps commanding it.
   *
   * <p>Does not end. For an elevator this is almost always what is wanted — the closed loop is what
   * holds the carriage against gravity, and a command that finishes on arrival hands the subsystem
   * back to its default command, which typically stops it. An elevator that reaches its setpoint
   * and then sinks is this mistake.
   */
  public static Command holdAt(LinearSubsystem linear, Distance height) {
    return Commands.run(() -> linear.setPosition(height), linear)
        .withName(linear.getName() + "_HoldAt");
  }

  /** Commands a height re-read every loop, and keeps commanding it. */
  public static Command holdAt(LinearSubsystem linear, Supplier<Distance> heightSupplier) {
    return Commands.run(() -> linear.setPosition(heightSupplier.get()), linear)
        .withName(linear.getName() + "_HoldAtSupplied");
  }

  /**
   * Drives to a height and finishes once there, or when the budget runs out.
   *
   * <p>The timeout is mandatory. "Never arrives" covers a bound carriage, a snapped rope and a
   * setpoint outside the travel bounds, and none of them announce themselves.
   *
   * <p>A phase inside a sequence. On its own it leaves the carriage unheld.
   */
  public static Command moveTo(LinearSubsystem linear, Distance height, Time timeout) {
    return Commands.sequence(
            Commands.runOnce(() -> linear.setPosition(height), linear),
            Commands.waitUntil(linear::isAtPosition).withTimeout(timeout))
        .withName(linear.getName() + "_MoveTo");
  }

  /** {@link #moveTo} followed by {@link #holdAt}, which is what most bindings actually want. */
  public static Command moveToAndHold(LinearSubsystem linear, Distance height, Time timeout) {
    return Commands.sequence(moveTo(linear, height, timeout), holdAt(linear, height))
        .withName(linear.getName() + "_MoveToAndHold");
  }

  /** Open-loop voltage while scheduled. Bring-up and characterization, not match logic. */
  public static Command runAtVoltage(LinearSubsystem linear, Voltage volts) {
    return Commands.startEnd(() -> linear.setVoltage(volts), linear::stop, linear)
        .withName(linear.getName() + "_Voltage");
  }

  /**
   * The default command: hold wherever the carriage currently is.
   *
   * <p>Captures the height once, at schedule time. A default that re-read the measurement every
   * loop would command wherever the carriage had already sunk to, which is a loop with no restoring
   * force and looks exactly like gravity winning slowly.
   */
  public static Command holdCurrentPosition(LinearSubsystem linear) {
    return Commands.defer(
            () -> {
              Distance here = linear.getHeight();
              return Commands.run(() -> linear.setPosition(here), linear);
            },
            Set.of(linear))
        .withName(linear.getName() + "_HoldCurrent");
  }

  /**
   * Drives gently into the bottom hard stop, waits until the carriage stops moving, and declares
   * that position to be {@code heightAtStop}.
   *
   * <p>A mechanism on the motor's internal encoder has no idea where it is at boot — position reads
   * zero wherever the carriage happened to be sitting, so every setpoint is offset by however far
   * that was from the bottom. This is the routine that fixes it. A mechanism with a fused CANcoder
   * does not need it.
   *
   * <h2>The voltage has to be small, and the stator limit is not the guard you want</h2>
   *
   * <p>A voltage produces whatever current the mechanism's impedance allows, and at stall against a
   * hard stop that is as much as the configured stator limit permits. That limit is sized for the
   * mechanism doing its real job, not for a gentle push into a stop. Find this value by driving the
   * carriage down on a bench and taking the smallest voltage that still moves it against friction,
   * then watch stator current in the log the first few times it runs.
   *
   * <h2>Three guards, all necessary</h2>
   *
   * <ul>
   *   <li><b>A minimum settling time before the stall check.</b> The carriage starts at zero
   *       velocity, so a stall check that ran immediately would declare success without moving the
   *       mechanism at all, and zero it wherever it already was — the exact failure this routine
   *       exists to prevent, arrived at more confidently.
   *   <li><b>A timeout on the stall wait.</b> Mandatory, per the command rules. A carriage that
   *       never stalls is one whose rope has come off, and hanging forever on it costs the match.
   *   <li><b>A stop on the way out.</b> {@code finallyDo}, so an interrupted zeroing routine does
   *       not leave the motor driving into the hard stop.
   * </ul>
   *
   * <p>Run this once at robot startup, or bind it to a pit button. It should <b>not</b> run
   * automatically during a match: the carriage travelling to the bottom is not always safe, and it
   * is never fast.
   *
   * @param linear The mechanism to zero
   * @param zeroingVoltage Voltage to drive with. Negative, small — enough to move the carriage
   *     against friction and no more
   * @param settleTime How long to drive before the stall check is allowed to succeed. Must exceed
   *     the time it takes the carriage to start moving
   * @param stallVelocity Speed below which the carriage counts as stopped
   * @param timeout Total budget for the stall wait
   * @param heightAtStop What the carriage's height is when it is against the stop. Usually the
   *     mechanism's minimum height, but not always zero — measure it
   */
  public static Command zeroAtHardStop(
      LinearSubsystem linear,
      Voltage zeroingVoltage,
      Time settleTime,
      LinearVelocity stallVelocity,
      Time timeout,
      Distance heightAtStop) {
    return Commands.sequence(
            Commands.runOnce(() -> linear.setVoltage(zeroingVoltage), linear),
            Commands.waitTime(settleTime),
            Commands.waitUntil(
                    () ->
                        Math.abs(linear.getLinearVelocity().in(InchesPerSecond))
                            < Math.abs(stallVelocity.in(InchesPerSecond)))
                .withTimeout(timeout),
            Commands.runOnce(() -> linear.zeroAt(heightAtStop), linear))
        .finallyDo(linear::stop)
        .withName(linear.getName() + "_ZeroAtHardStop");
  }

  /**
   * Stops commanding output, once, and finishes.
   *
   * <p>An elevator descends from here. Usually a bug; {@link #holdCurrentPosition} is what "stop
   * moving" normally means.
   */
  public static Command stop(LinearSubsystem linear) {
    return Commands.runOnce(linear::stop, linear).withName(linear.getName() + "_Stop");
  }
}
