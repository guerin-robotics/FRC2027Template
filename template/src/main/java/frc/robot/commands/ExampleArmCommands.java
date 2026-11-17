package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.exampleArm.ExampleArm;
import java.util.function.Supplier;

/**
 * Command factory for a rotary position-controlled mechanism.
 *
 * <p>Commands are <b>static factory methods, not classes</b>. See {@code
 * .claude/rules/01-architecture.md}.
 *
 * <p><b>Always call {@code .withName()}.</b> Names appear in AdvantageKit's command log, and bad
 * names make a match impossible to debug. Format: {@code "SubsystemName_ActionVerb_OptionalParam"}.
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
   * <p><b>The timeout is mandatory.</b> A bare {@code waitUntil} hangs forever if the condition
   * never becomes true, and a robot that hangs mid-sequence scores zero. This is a competition
   * safety rule, not a style preference — see {@code .claude/rules/03-commands.md}.
   *
   * <p>It does <b>not</b> stop the mechanism when it ends. That is deliberate on a gravity-loaded
   * arm: the Talon keeps holding the last commanded position, which is what you want between the
   * move and whatever happens next. A {@code finallyDo(stop)} here would let the arm fall the
   * moment the command completed.
   */
  public static Command goToPosition(ExampleArm arm, Angle position) {
    return Commands.sequence(
            Commands.runOnce(() -> arm.setPosition(position), arm),
            Commands.waitUntil(arm::isAtPosition)
                .withTimeout(Constants.Waits.MECHANISM_READY_SECONDS))
        .withName("ExampleArm_GoTo_" + Math.round(position.in(Degrees)) + "deg");
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
    return Commands.run(() -> arm.setPosition(positionSupplier.get()), arm)
        .withName("ExampleArm_Track");
  }

  /**
   * Default command: hold a safe resting angle.
   *
   * <p>Runs whenever nothing else requires the subsystem, and it must <b>never end</b> — {@code
   * Commands.run}, not {@code Commands.runOnce}.
   *
   * <p><b>Do not make this a stop.</b> Zero output on a gravity-loaded arm means it falls. The safe
   * idle state for this scaffold is holding a position, which is what closed-loop control gives you
   * for free.
   */
  public static Command holdAt(ExampleArm arm, Angle stowPosition) {
    return Commands.run(() -> arm.setPosition(stowPosition), arm).withName("ExampleArm_Hold");
  }

  /**
   * Open-loop. Bring-up and characterization only — never bind this to a match control.
   *
   * <p>Bypasses gravity compensation and the closed loop, so the arm falls at zero volts and has
   * nothing but the soft limits protecting it. Useful for confirming direction of travel before
   * anything is tuned; see {@code docs/new-mechanism-bringup.md}.
   */
  public static Command runAtVoltage(ExampleArm arm, Voltage voltage) {
    return Commands.startEnd(() -> arm.setVoltage(voltage), arm::stop, arm)
        .withName("ExampleArm_Voltage");
  }
}
