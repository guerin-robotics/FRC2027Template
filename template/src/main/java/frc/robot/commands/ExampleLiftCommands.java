package frc.robot.commands;

import static org.wpilib.units.Units.Seconds;

import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.exampleLift.ExampleLift;
import frc.robot.subsystems.exampleLift.ExampleLiftConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Voltage;

/**
 * Command factory for a linear position-controlled mechanism.
 *
 * <p>Commands are <b>static factory methods, not classes</b>. See {@code
 * .claude/rules/01-architecture.md}.
 *
 * <p><b>Every command ends in {@code .named(...)}</b> (V3 requires it). Names appear in
 * AdvantageKit's command log, and bad names make a match impossible to debug.
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
   * <p>What makes it safe:
   *
   * <ol>
   *   <li><b>It is a COMMAND, not a subsystem method</b>, so it requires the subsystem and the
   *       scheduler interrupts it if the operator commands a height mid-run. The subsystem holds
   *       only the primitive: "declare the current position to be zero."
   *   <li><b>It detects the stop with CURRENT HIGH AND VELOCITY NEAR ZERO, debounced</b> — not
   *       current alone, which spikes the instant the motor breaks static friction, long before the
   *       carriage is anywhere near the real hard stop.
   *   <li><b>A stall only counts once the creep has settled.</b> The detector runs every loop, so a
   *       carriage already jammed when the button is pressed would otherwise end the routine on its
   *       first loop and be zeroed mid-air. The creep runs for {@code settleSeconds} first, then
   *       the detector is reset, so the stall that ends zeroing is one that happened under the
   *       creep. {@code settleSeconds} must exceed the time the carriage takes to start moving.
   *   <li><b>The reverse soft limit is off while creeping</b>. It boots off — before a zero it is
   *       measured from the power-on position and means nothing — and goes on in {@code whenExited}
   *       only once the lift is zeroed.
   *   <li><b>Nothing waits on the bus except one short write.</b> The zero itself is sent without
   *       waiting and confirmed from the logged position, so a slow or missing Talon cannot stall
   *       the robot loop. Only the soft-limit write is waited for, bounded to 0.25 s; see {@code
   *       ExampleLiftIO.setReverseSoftLimitEnabled} for why.
   *   <li><b>On timeout it GIVES UP WITHOUT ZEROING.</b> A zero taken at an unknown position is
   *       worse than no zero: the mechanism believes it for the rest of the match and every height
   *       afterward is wrong by a silent, fixed offset. Failing loudly and leaving {@code
   *       isZeroed()} false is the correct outcome.
   * </ol>
   *
   * <p><b>Every value is a parameter</b>, per {@code .claude/rules/03-commands.md}: the creep
   * voltage is a commanded value and lives in {@code Constants.Setpoints}; the two times are waits
   * and live in {@code Constants.Waits}. Starting points — measure them with the carriage in front
   * of you:
   *
   * <pre>
   * // Constants.Setpoints
   * public static final Voltage LIFT_ZEROING_VOLTAGE = Volts.of(-3.0); // negative is down
   * // Constants.Waits
   * public static final double LIFT_ZEROING_SETTLE_SECONDS = 0.25;
   * public static final double LIFT_ZEROING_TIMEOUT_SECONDS = 3.0;
   * </pre>
   *
   * <p>With a follower, halve the voltage: the follower mirrors the leader, so the force into the
   * stop doubles. Bind it to a button the operator can reach, and run it as part of the pre-match
   * routine. See {@code docs/new-mechanism-bringup.md} Phase 1 step 6.
   *
   * @param creepVoltage downward creep while seeking the stop; negative is down
   * @param settleSeconds how long to creep before a stall may count
   * @param timeoutSeconds how long after settling to look for the stop before giving up
   */
  public static Command zero(
      ExampleLift lift, Voltage creepVoltage, double settleSeconds, double timeoutSeconds) {
    return lift.run(
            co -> {
              // The reverse bound has to be off to creep down to the stop. It boots off, so on a
              // first zero this writes nothing; it only costs a (waited-for) write when re-zeroing
              // a lift that was already zeroed. If that write fails, the creep is stopped by the
              // bound instead of the stop, never stalls, and the routine times out without zeroing.
              lift.setReverseSoftLimitEnabled(false);

              // Creep for settleSeconds before a stall may count.
              Timer timer = Timer.createStarted();
              while (!timer.hasElapsed(settleSeconds)) {
                lift.setVoltage(creepVoltage);
                co.yield();
              }

              // Forget anything the detector saw before or during the settle.
              lift.resetZeroingStallDetection();

              // Creep until the stall, bounded by the timeout so a missing stop cannot hang this.
              timer.restart();
              while (!lift.isAtZeroingStall() && !timer.hasElapsed(timeoutSeconds)) {
                lift.setVoltage(creepVoltage);
                co.yield();
              }

              if (lift.isAtZeroingStall()) {
                // Send the zero without waiting, keep the carriage pressed into the stop, and let
                // the subsystem confirm it from the logged position. The wait is bounded by twice
                // the subsystem's own confirmation window, as a backstop.
                lift.requestZero();
                timer.restart();
                while (lift.isZeroPending()
                    && !timer.hasElapsed(2.0 * ExampleLiftConstants.ZERO_CONFIRM_SECONDS)) {
                  lift.setVoltage(creepVoltage);
                  co.yield();
                }
              } else {
                DriverStationErrors.reportError(
                    ExampleLiftConstants.NAME
                        + " zero failed: hard stop not found within "
                        + (settleSeconds + timeoutSeconds)
                        + "s",
                    false);
              }
            })
        // Stop, then put the bound back on if — and only if — the lift is zeroed. That covers a
        // fresh zero (bound on in the new frame), an interrupted re-zero (bound back on in the old,
        // still valid frame), and a failed one (left off, because an unzeroed bound means nothing).
        // whenExited runs on completion, cancel and interruption alike; a body that can finish
        // must clean up on every exit, not only on cancel.
        .whenExited(
            () -> {
              lift.stop();
              if (lift.isZeroed()) {
                lift.setReverseSoftLimitEnabled(true);
              }
            })
        .named("ExampleLift_Zero");
  }

  /**
   * Commands a profiled move to a height in inches, and finishes once the carriage arrives.
   *
   * <p><b>Refuses to move if the mechanism has not been zeroed.</b> The subsystem itself does not
   * block the call — a subsystem that silently ignores commands is harder to debug than one that
   * obeys — so the guard lives here, where the refusal is visible in the command log as a command
   * that ended immediately.
   *
   * <p><b>The timeout is mandatory.</b> An untimed {@code waitUntil} (the command or the coroutine
   * form) hangs forever if the condition never becomes true, and a robot that hangs mid-sequence
   * scores zero. See {@code .claude/rules/03-commands.md}. The timed form returns a {@code
   * WaitResult}; this logs whether the move timed out.
   *
   * <p>It does <b>not</b> stop the mechanism when it ends. That is deliberate on a gravity-loaded
   * lift: the Talon keeps holding the last commanded height, which is what you want between the
   * move and whatever happens next. A {@code whenExited(stop)} here would drop the carriage the
   * moment the command completed.
   */
  public static Command goToHeight(ExampleLift lift, double inches) {
    return lift.run(
            co -> {
              if (!lift.isZeroed()) {
                return; // refuse: ends immediately, visible in the command log
              }
              lift.setPositionInches(inches);
              var arrived =
                  co.waitUntil(
                      lift::isAtPosition, Seconds.of(Constants.Waits.MECHANISM_READY_SECONDS));
              Logger.recordOutput(ExampleLiftConstants.NAME + "/GoTo/TimedOut", arrived.timedOut());
            })
        .named("ExampleLift_GoTo_" + Math.round(inches) + "in");
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
    return lift.run(
            co -> {
              if (!lift.isZeroed()) {
                return; // refuse: ends immediately, visible in the command log
              }
              while (true) {
                lift.setPositionInches(inchesSupplier.getAsDouble());
                co.yield();
              }
            })
        .named("ExampleLift_Track");
  }

  /**
   * Default command: hold a safe resting height.
   *
   * <p>Runs whenever nothing else requires the mechanism, and it must <b>never end</b> — {@code
   * runRepeatedly}, not a one-shot. It runs at {@code LOWEST_PRIORITY}: V3 lets a new command
   * interrupt a running one only at equal or higher priority, so a default at or above {@code
   * DEFAULT_PRIORITY} would block every ordinary command.
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
    return lift.runRepeatedly(() -> lift.setPositionInches(stowInches))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("ExampleLift_Hold");
  }

  /**
   * Open-loop. Bring-up and characterization only — never bind this to a match control.
   *
   * <p>Bypasses gravity compensation and the closed loop, so the carriage falls at zero volts and
   * has nothing but the soft limits protecting it — and those are themselves meaningless before
   * zeroing. Use it to confirm direction of travel, with a hand on disable.
   */
  public static Command runAtVoltage(ExampleLift lift, Voltage voltage) {
    return lift.run(
            co -> {
              lift.setVoltage(voltage);
              co.park();
            })
        .whenExited(lift::stop)
        .named("ExampleLift_Voltage");
  }

  /**
   * Dashboard setpoint for {@link #tuneHeight}, in inches. Read nowhere else. Starts at the bottom
   * of travel.
   */
  private static final LoggedTunableNumber TUNE_HEIGHT_INCHES =
      new LoggedTunableNumber(
          ExampleLiftConstants.NAME + "/Tune/HeightIn", ExampleLiftConstants.MIN_TRAVEL_INCHES);

  /**
   * Moves to whatever height the dashboard says, for finding a setpoint with the mechanism in front
   * of you. Edit {@code Tuning/ExampleLift/Tune/HeightIn} and the carriage follows on the next
   * loop. The goal is clamped to travel by {@code setPositionInches}, and {@code GoalWasClamped}
   * says when.
   *
   * <p><b>Refuses to run until the lift is zeroed</b>, for the same reason {@link #goToHeight}
   * does. It does not stop when it ends: the carriage keeps holding the last height, because zero
   * output on a gravity-loaded lift lets it fall.
   *
   * <p><b>Utility mode, with Start held.</b> Bind it with {@code
   * triggers.tuningEnabled.whileTrue(...)}, which is true only while the robot is enabled in
   * Utility mode AND the operator holds Start (the {@code menu()} button) — a dead-man switch, so
   * enabling a Utility OpMode for something else does not move the mechanism. It also ends the
   * moment the robot leaves Utility mode, and {@code LoggedTunableNumber.get()} returns its default
   * outside it anyway, so the dashboard value can never reach the field. It is a candidate to
   * become a {@code @Utility} OpMode of its own; keep the held button.
   *
   * <p>When the number is right, write it into {@code Constants.Setpoints} and commit it. The
   * dashboard value lives only in NetworkTables and is gone at the next boot.
   */
  public static Command tuneHeight(ExampleLift lift) {
    return lift.run(
            co -> {
              if (!lift.isZeroed()) {
                return; // refuse, as goToHeight does
              }
              // The loop condition is the guard: it refuses to start outside Utility mode and
              // ends the moment the robot leaves it, so a mode transition applies no last setpoint.
              while (org.wpilib.driverstation.RobotState.isUtilityEnabled()) {
                double inches = TUNE_HEIGHT_INCHES.get();
                Logger.recordOutput(ExampleLiftConstants.NAME + "/Tune/HeightIn", inches);
                lift.setPositionInches(inches);
                co.yield();
              }
            })
        .named("ExampleLift_Tune");
  }
}
