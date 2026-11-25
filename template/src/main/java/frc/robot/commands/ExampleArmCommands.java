package frc.robot.commands;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.exampleArm.ExampleArm;
import frc.robot.subsystems.exampleArm.ExampleArmConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

/**
 * Command factory for a rotary position-controlled mechanism.
 *
 * <p>Commands are <b>static factory methods, not classes</b>. See {@code
 * .claude/rules/01-architecture.md}.
 *
 * <p><b>Every command ends in {@code .named(...)}</b> — Commands V3 will not build one without a
 * name. Names appear in AdvantageKit's command log, and bad names make a match impossible to debug.
 * Format: {@code "SubsystemName_ActionVerb_OptionalParam"}. A body that loops must {@code
 * co.yield()} every pass; V3 has no preemption.
 *
 * <h2>SETPOINTS ARE PARAMETERS</h2>
 *
 * <p>Every factory below takes the angle it commands as an argument. None reads a setpoint, and
 * none hardcodes one. The caller — almost always {@code RobotContainer} — supplies it from {@code
 * Constants.Setpoints}:
 *
 * <pre>
 * triggers.deployIntake()
 *     .onTrue(ExampleArmCommands.goToPosition(arm, Constants.Setpoints.ARM_DEPLOYED));
 * </pre>
 *
 * <p>Keeping factories parameterized is what lets one command serve stow, deploy and score, and
 * what keeps every number a driver might want changed in one file for the pit. When {@code
 * RobotContainer} is finished it should contain no bare numbers at all — if a unit import like
 * {@code Degrees} is still needed there, a setpoint has been left behind.
 */
public class ExampleArmCommands {

  private ExampleArmCommands() {}

  /**
   * Commands a profiled move and finishes once the arm arrives.
   *
   * <p><b>The timeout is mandatory.</b> An untimed {@code waitUntil} (the command or the coroutine
   * form) hangs forever if the condition never becomes true, and a robot that hangs mid-sequence
   * scores zero. This is a competition safety rule, not a style preference — see {@code
   * .claude/rules/03-commands.md}. The timed form returns a {@code WaitResult}; this logs whether
   * the move timed out, so a log can tell "arrived" from "gave up".
   *
   * <p>It does <b>not</b> stop the mechanism when it ends. That is deliberate on a gravity-loaded
   * arm: the Talon keeps holding the last commanded position, which is what you want between the
   * move and whatever happens next. A {@code whenExited(stop)} here would let the arm fall the
   * moment the command completed.
   */
  public static Command goToPosition(ExampleArm arm, Angle position) {
    return arm.run(
            co -> {
              arm.setPosition(position);
              var arrived =
                  co.waitUntil(
                      arm::isAtPosition, Seconds.of(Constants.Waits.MECHANISM_READY_SECONDS));
              Logger.recordOutput(ExampleArmConstants.NAME + "/GoTo/TimedOut", arrived.timedOut());
            })
        .named("ExampleArm_GoTo_" + Math.round(position.in(Degrees)) + "deg");
  }

  /**
   * Commands a position that is re-evaluated every loop.
   *
   * <p>Use this when the target comes from something that moves — an interpolation table keyed on
   * distance to a scoring element, for instance. {@link #goToPosition} samples its argument once;
   * this one re-reads the supplier continuously.
   *
   * <p>Runs until interrupted, so it is a natural fit for {@code whileTrue}.
   */
  public static Command trackPosition(ExampleArm arm, Supplier<Angle> positionSupplier) {
    return arm.runRepeatedly(() -> arm.setPosition(positionSupplier.get()))
        .named("ExampleArm_Track");
  }

  /**
   * Default command: hold a safe resting angle.
   *
   * <p>Runs whenever nothing else requires the mechanism, and it must <b>never end</b> — {@code
   * runRepeatedly}, not a one-shot. It runs at {@code LOWEST_PRIORITY}: V3 lets a new command
   * interrupt a running one only at equal or higher priority, so a default at or above {@code
   * DEFAULT_PRIORITY} would block every ordinary command.
   *
   * <p><b>Do not make this a stop.</b> Zero output on a gravity-loaded arm means it falls. The safe
   * idle state for this scaffold is holding a position, which is what closed-loop control gives you
   * for free.
   */
  public static Command holdAt(ExampleArm arm, Angle stowPosition) {
    return arm.runRepeatedly(() -> arm.setPosition(stowPosition))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("ExampleArm_Hold");
  }

  /**
   * Open-loop. Bring-up and characterization only — never bind this to a match control.
   *
   * <p>Bypasses gravity compensation and the closed loop, so the arm falls at zero volts and has
   * nothing but the soft limits protecting it. Useful for confirming direction of travel before
   * anything is tuned; see {@code docs/new-mechanism-bringup.md}.
   */
  public static Command runAtVoltage(ExampleArm arm, Voltage voltage) {
    return arm.run(
            co -> {
              arm.setVoltage(voltage);
              co.park();
            })
        .whenExited(arm::stop)
        .named("ExampleArm_Voltage");
  }

  /**
   * Dashboard setpoint for {@link #tunePosition}, in mechanism degrees. Read nowhere else. Starts
   * at the reverse bound, which is always inside travel.
   */
  private static final LoggedTunableNumber TUNE_POSITION_DEGREES =
      new LoggedTunableNumber(
          ExampleArmConstants.NAME + "/Tune/PositionDeg",
          ExampleArmConstants.REVERSE_SOFT_LIMIT_DEGREES);

  /**
   * Moves to whatever angle the dashboard says, for finding a setpoint with the mechanism in front
   * of you. Edit {@code Tuning/ExampleArm/Tune/PositionDeg} and the arm follows on the next loop.
   * The goal is clamped to travel by {@code setPosition}, and {@code GoalWasClamped} says when.
   *
   * <p>It does <b>not</b> stop when it ends: the arm keeps holding the last angle, because zero
   * output on a gravity-loaded arm lets it fall.
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
  public static Command tunePosition(ExampleArm arm) {
    return arm.run(
            co -> {
              // The loop condition is the guard: it refuses to start outside Utility mode and
              // ends the moment the robot leaves it, so a mode transition applies no last setpoint.
              while (org.wpilib.driverstation.RobotState.isUtilityEnabled()) {
                double degrees = TUNE_POSITION_DEGREES.get();
                Logger.recordOutput(ExampleArmConstants.NAME + "/Tune/PositionDeg", degrees);
                arm.setPosition(Degrees.of(degrees));
                co.yield();
              }
            })
        .named("ExampleArm_Tune");
  }
}
