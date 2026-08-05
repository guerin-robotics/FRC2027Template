package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.example.ExampleSubsystem;
import java.util.function.BooleanSupplier;

/**
 * Command factory for ExampleSubsystem.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Rename "ExampleSubsystem" → your subsystem name 2. All methods are
 * static — never instantiate this class 3. Always call .withName() for AdvantageKit command logging
 * 4. Use Commands.startEnd() for "run while held, stop on release" behavior 5. Use
 * Commands.runOnce() for one-shot actions 6. Use Commands.run() for continuous actions that should
 * repeat each loop 7. Use Commands.sequence() to chain actions
 *
 * <p>PATTERN: If a command needs to wait for spinup then alignment before acting:
 *
 * <p>Commands.sequence( Commands.waitUntil(isSpunUp).withTimeout(spinupTimeout),
 * Commands.waitUntil(isAligned).withTimeout(alignTimeout - spinupTimeout), runAction() )
 *
 * <p>The alignment timeout is intentional — act anyway if alignment never comes. A robot that hangs
 * mid-sequence scores zero points. See the "Ready → Align → Act" section of
 * .claude/rules/03-commands.md.
 *
 * <p>SETPOINTS ARE PARAMETERS. Note that every factory below takes the value it commands as an
 * argument. None of them reads a setpoint, and none hardcodes one. The caller — almost always
 * RobotContainer — supplies it from Constants.Setpoints:
 *
 * <pre>
 * controller.rightBumper()
 *     .whileTrue(ExampleCommands.runAtVelocity(subsystem, Constants.Setpoints.INTAKE_VELOCITY));
 * </pre>
 *
 * <p>Keeping factories parameterized is what lets the same command serve several setpoints, and
 * what keeps every tunable number in one file for the pit. A private constant in RobotContainer is
 * not a substitute — it is still scattered, just less visibly.
 */
public class ExampleCommands {

  // Basic voltage control — runs while command is active, stops on end
  public static Command runAtVoltage(ExampleSubsystem subsystem, Voltage voltage) {
    return Commands.startEnd(() -> subsystem.setVoltage(voltage), () -> subsystem.stop(), subsystem)
        .withName("Example_Voltage_" + voltage.in(Volts) + "V");
  }

  // One-shot velocity set
  public static Command runAtVelocity(ExampleSubsystem subsystem, AngularVelocity velocity) {
    return Commands.runOnce(() -> subsystem.setVelocity(velocity), subsystem)
        .withName("Example_Velocity");
  }

  // Stop immediately
  public static Command stop(ExampleSubsystem subsystem) {
    return Commands.runOnce(() -> subsystem.stop(), subsystem).withName("Example_Stop");
  }

  // Idle (continuous default command pattern — runs forever until interrupted)
  public static Command idle(ExampleSubsystem subsystem) {
    return Commands.run(() -> subsystem.setVoltage(Volts.of(0)), subsystem)
        .withName("Example_Idle");
  }

  // Gated action — waits for spinup, then alignment, then acts
  // Use this pattern when the mechanism must be ready before an action is safe
  public static Command runAfterReady(
      ExampleSubsystem subsystem, AngularVelocity velocity, BooleanSupplier isAligned) {
    return Commands.sequence(
            Commands.waitUntil(
                    () -> subsystem.isAtVelocity()) // TODO: add isAtVelocity() to subsystem
                .withTimeout(Constants.Waits.MECHANISM_READY_SECONDS),
            // Phase 2 gets the REMAINING budget, not a fresh one. This is what
            // guarantees the whole sequence finishes inside the total budget.
            Commands.waitUntil(isAligned)
                .withTimeout(
                    Constants.Waits.TOTAL_TIMEOUT_SECONDS
                        - Constants.Waits.MECHANISM_READY_SECONDS),
            runAtVelocity(subsystem, velocity))
        .withName("Example_AfterReady");
  }
}
