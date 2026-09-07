package frc.lib.mechanism.rotary;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

/**
 * The command factories every rotary position mechanism needs.
 *
 * <p>See {@code RollerCommands} for the conventions these follow. The one that matters most here is
 * that setpoints are parameters: the angles a mechanism is commanded to live in {@code
 * Constants.Setpoints}, not in this file and not in the mechanism's own constants.
 */
public final class RotaryCommands {

  private RotaryCommands() {}

  /**
   * Commands an angle and keeps commanding it.
   *
   * <p>Does not end. This is the shape for "go here and stay here", which for a rotary mechanism is
   * almost always what is wanted: the closed loop is what holds the arm against gravity, and a
   * command that finishes on arrival hands the subsystem back to its default command, which
   * typically stops it. An arm that reaches its setpoint and then falls is this mistake.
   */
  public static Command holdAt(RotarySubsystem rotary, Angle position) {
    return Commands.run(() -> rotary.setPosition(position), rotary)
        .withName(rotary.getName() + "_HoldAt");
  }

  /** Commands an angle re-read every loop, and keeps commanding it. */
  public static Command holdAt(RotarySubsystem rotary, Supplier<Angle> positionSupplier) {
    return Commands.run(() -> rotary.setPosition(positionSupplier.get()), rotary)
        .withName(rotary.getName() + "_HoldAtSupplied");
  }

  /**
   * Drives to an angle and finishes once there, or when the budget runs out.
   *
   * <p>The timeout is mandatory. A robot that hangs waiting for a mechanism that will never arrive
   * scores zero, and "never arrives" covers a jammed pivot, a snapped belt and a setpoint outside
   * the travel bounds — none of which announce themselves.
   *
   * <p>Use this as a phase inside a sequence, where something later keeps commanding the position.
   * On its own it leaves the mechanism unheld.
   */
  public static Command moveTo(RotarySubsystem rotary, Angle position, Time timeout) {
    return Commands.sequence(
            Commands.runOnce(() -> rotary.setPosition(position), rotary),
            Commands.waitUntil(rotary::isAtPosition).withTimeout(timeout))
        .withName(rotary.getName() + "_MoveTo");
  }

  /**
   * Drives to an angle, waits until it arrives, then holds it.
   *
   * <p>{@link #moveTo} followed by {@link #holdAt}, which is the combination most bindings actually
   * want. The timeout governs only the arrival phase; the hold continues regardless, because a
   * mechanism that did not quite arrive still needs to be held somewhere.
   */
  public static Command moveToAndHold(RotarySubsystem rotary, Angle position, Time timeout) {
    return Commands.sequence(moveTo(rotary, position, timeout), holdAt(rotary, position))
        .withName(rotary.getName() + "_MoveToAndHold");
  }

  /** Open-loop voltage while scheduled. Bring-up and characterization, not match logic. */
  public static Command runAtVoltage(RotarySubsystem rotary, Voltage volts) {
    return Commands.startEnd(() -> rotary.setVoltage(volts), rotary::stop, rotary)
        .withName(rotary.getName() + "_Voltage");
  }

  /**
   * The default command: hold wherever the mechanism currently is.
   *
   * <p>Captures the position once, at schedule time, rather than every loop — a default that
   * re-read the measurement each cycle would command wherever the arm had already sagged to, which
   * is a loop with no restoring force and looks exactly like gravity winning slowly.
   */
  public static Command holdCurrentPosition(RotarySubsystem rotary) {
    return Commands.defer(
            () -> {
              Angle here = rotary.getPosition();
              return Commands.run(() -> rotary.setPosition(here), rotary);
            },
            java.util.Set.of(rotary))
        .withName(rotary.getName() + "_HoldCurrent");
  }

  /**
   * Stops commanding output, once, and finishes.
   *
   * <p>For a mechanism under gravity this lets it fall to its hard stop. That is occasionally what
   * you want — stowing a spring-loaded arm, or making a mechanism safe to handle — and is otherwise
   * a bug. {@link #holdCurrentPosition} is what "stop moving" usually means.
   */
  public static Command stop(RotarySubsystem rotary) {
    return Commands.runOnce(rotary::stop, rotary).withName(rotary.getName() + "_Stop");
  }
}
