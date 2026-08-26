package frc.robot.commands;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.exampleLift.ExampleLift;
import frc.robot.subsystems.exampleLift.ExampleLiftConstants;
import java.util.function.DoubleSupplier;

/**
 * Command factory for a linear position-controlled mechanism.
 *
 * <p>Commands are <b>static factory methods, not classes</b>. See {@code
 * .claude/rules/01-architecture.md}.
 *
 * <p><b>Always call {@code .withName()}.</b> Names appear in AdvantageKit's command log, and bad
 * names make a match impossible to debug.
 *
 * <h2>SETPOINTS ARE PARAMETERS</h2>
 *
 * <p>Every factory below takes the height it commands as an argument, in inches. None reads a
 * setpoint, and none hardcodes one. The caller — almost always {@code RobotContainer} — supplies it
 * from {@code Constants.Setpoints}:
 *
 * <pre>
 * triggers.scoreHigh()
 *     .onTrue(ExampleLiftCommands.goToHeight(lift, Constants.Setpoints.LIFT_HIGH_INCHES));
 * </pre>
 *
 * <p>When {@code RobotContainer} is finished it should contain no bare numbers at all.
 */
public class ExampleLiftCommands {

  private ExampleLiftCommands() {}

  /**
   * Drives the carriage into its hard stop and declares that position zero.
   *
   * <p><b>This must run before any height command is trusted.</b> The motor encoder is relative and
   * reads zero at boot wherever the carriage was sitting, so until this completes every reported
   * height — and every soft limit, which bounds that same reported height — is off by an unknown
   * offset.
   *
   * <p>Three things make this safe, and all three matter:
   *
   * <ol>
   *   <li><b>It is a COMMAND, not a subsystem method</b>, so it requires the subsystem and the
   *       scheduler interrupts it if the operator commands a height mid-run. The subsystem holds
   *       only the primitive: "declare the current position to be zero."
   *   <li><b>It detects the stop with CURRENT HIGH AND VELOCITY NEAR ZERO, debounced</b> — not
   *       current alone, which spikes the instant the motor breaks static friction, long before the
   *       carriage is anywhere near the real hard stop.
   *   <li><b>On timeout it GIVES UP WITHOUT ZEROING.</b> A zero taken at an unknown position is
   *       worse than no zero: the mechanism believes it for the rest of the match and every height
   *       afterward is wrong by a silent, fixed offset. Failing loudly and leaving {@code
   *       isZeroed()} false is the correct outcome.
   * </ol>
   *
   * <p>Bind it to a button the operator can reach, and run it as part of the pre-match routine. See
   * {@code docs/new-mechanism-bringup.md} Phase 1 step 6.
   */
  public static Command zero(ExampleLift lift) {
    return Commands.sequence(
            Commands.run(() -> lift.setVoltage(ExampleLiftConstants.ZEROING_VOLTAGE), lift)
                .until(lift::isAtZeroingStall)
                .withTimeout(ExampleLiftConstants.ZEROING_TIMEOUT_SECONDS),
            Commands.either(
                Commands.runOnce(lift::zeroAtCurrentPosition, lift),
                Commands.runOnce(
                    () ->
                        DriverStation.reportError(
                            ExampleLiftConstants.NAME
                                + " zero failed: hard stop not found within "
                                + ExampleLiftConstants.ZEROING_TIMEOUT_SECONDS
                                + "s",
                            false)),
                lift::isAtZeroingStall))
        .finallyDo(interrupted -> lift.stop())
        .withName("ExampleLift_Zero");
  }

  /**
   * Commands a profiled move to a height in inches, and finishes once the carriage arrives.
   *
   * <p><b>Refuses to move if the mechanism has not been zeroed.</b> The subsystem itself does not
   * block the call — a subsystem that silently ignores commands is harder to debug than one that
   * obeys — so the guard lives here, where the refusal is visible in the command log as a command
   * that ended immediately.
   *
   * <p><b>The timeout is mandatory.</b> A bare {@code waitUntil} hangs forever if the condition
   * never becomes true, and a robot that hangs mid-sequence scores zero. See {@code
   * .claude/rules/03-commands.md}.
   *
   * <p>It does <b>not</b> stop the mechanism when it ends. That is deliberate on a gravity-loaded
   * lift: the Talon keeps holding the last commanded height, which is what you want between the
   * move and whatever happens next. A {@code finallyDo(stop)} here would drop the carriage the
   * moment the command completed.
   */
  public static Command goToHeight(ExampleLift lift, double inches) {
    return Commands.sequence(
            Commands.runOnce(() -> lift.setPositionInches(inches), lift),
            Commands.waitUntil(lift::isAtPosition)
                .withTimeout(Constants.Waits.MECHANISM_READY_SECONDS))
        .onlyIf(lift::isZeroed)
        .withName("ExampleLift_GoTo_" + Math.round(inches) + "in");
  }

  /**
   * Commands a height that is re-evaluated every loop.
   *
   * <p>Use this when the target comes from something that moves — an interpolation table keyed on
   * distance to a scoring element, for instance. {@link #goToHeight} samples its argument once;
   * this one re-reads the supplier continuously.
   *
   * <p>Runs until interrupted, so it is a natural fit for {@code whileTrue}.
   */
  public static Command trackHeight(ExampleLift lift, DoubleSupplier inchesSupplier) {
    return Commands.run(() -> lift.setPositionInches(inchesSupplier.getAsDouble()), lift)
        .onlyIf(lift::isZeroed)
        .withName("ExampleLift_Track");
  }

  /**
   * Default command: hold a safe resting height.
   *
   * <p>Runs whenever nothing else requires the subsystem, and it must <b>never end</b> — {@code
   * Commands.run}, not {@code Commands.runOnce}.
   *
   * <p><b>Do not make this a stop.</b> Zero output on a gravity-loaded lift means the carriage
   * falls. The safe idle state for this scaffold is holding a height, which is what closed-loop
   * control gives you for free.
   *
   * <p>Note the absence of an {@code onlyIf} guard: a default command that refuses to start leaves
   * the subsystem with no default at all. Before zeroing, {@code setPositionInches} commands a
   * height measured from a zero that has not been established — which is exactly why zeroing
   * belongs in the pre-match routine rather than being something the code works around.
   */
  public static Command holdAt(ExampleLift lift, double stowInches) {
    return Commands.run(() -> lift.setPositionInches(stowInches), lift)
        .withName("ExampleLift_Hold");
  }

  /**
   * Open-loop. Bring-up and characterization only — never bind this to a match control.
   *
   * <p>Bypasses gravity compensation and the closed loop, so the carriage falls at zero volts and
   * has nothing but the soft limits protecting it — and those are themselves meaningless before
   * zeroing. Use it to confirm direction of travel, with a hand on disable.
   */
  public static Command runAtVoltage(ExampleLift lift, Voltage voltage) {
    return Commands.startEnd(() -> lift.setVoltage(voltage), lift::stop, lift)
        .withName("ExampleLift_Voltage");
  }
}
