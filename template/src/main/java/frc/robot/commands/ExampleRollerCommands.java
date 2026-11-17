package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.exampleRoller.ExampleRoller;
import java.util.function.BooleanSupplier;

/**
 * Command factory for a velocity-controlled mechanism.
 *
 * <p>Commands are <b>static factory methods, not classes</b>. Named command classes create
 * unnecessary files and hide composition structure; factories are composable, testable, and
 * traceable in AdvantageKit logs. See {@code .claude/rules/01-architecture.md}.
 *
 * <p><b>Always call {@code .withName()}.</b> Names appear in AdvantageKit's command log, and bad
 * names make a match impossible to debug. Format: {@code "SubsystemName_ActionVerb_OptionalParam"}.
 *
 * <h2>SETPOINTS ARE PARAMETERS</h2>
 *
 * <p>Every factory below takes the value it commands as an argument. None reads a setpoint, and
 * none hardcodes one. The caller — almost always {@code RobotContainer} — supplies it from {@code
 * Constants.Setpoints}:
 *
 * <pre>
 * triggers.runIntake()
 *     .whileTrue(ExampleRollerCommands.runAtVelocity(roller, Constants.Setpoints.INTAKE_VELOCITY));
 * </pre>
 *
 * <p>Keeping factories parameterized is what lets one command serve several setpoints, and what
 * keeps every number a driver might want changed in one file for the pit. A private constant in
 * {@code RobotContainer} is not a substitute — it is still scattered, just less visibly. When
 * {@code RobotContainer} is finished it should contain no bare numbers at all.
 *
 * <h2>Choosing a factory</h2>
 *
 * <table>
 *   <tr><td>Run while held, stop on release</td><td>{@code Commands.startEnd}</td></tr>
 *   <tr><td>Do this once and finish</td><td>{@code Commands.runOnce}</td></tr>
 *   <tr><td>Run every loop until interrupted</td><td>{@code Commands.run}</td></tr>
 *   <tr><td>A, then B, then C</td><td>{@code Commands.sequence}</td></tr>
 *   <tr><td>Run both, stop when A finishes</td><td>{@code Commands.deadline}</td></tr>
 * </table>
 */
public class ExampleRollerCommands {

  private ExampleRollerCommands() {}

  /**
   * Spins at a velocity while the command runs, and stops on release.
   *
   * <p>{@code startEnd} rather than {@code runOnce} because a roller left spinning after its button
   * is released is the single most common way a mechanism ends up running through a whole match.
   * The end action is what guarantees it stops, including when the scheduler interrupts.
   */
  public static Command runAtVelocity(ExampleRoller roller, AngularVelocity velocity) {
    return Commands.startEnd(() -> roller.setVelocity(velocity), roller::stop, roller)
        .withName("ExampleRoller_Velocity");
  }

  /** Open-loop. Bring-up and characterization only — match logic should command a velocity. */
  public static Command runAtVoltage(ExampleRoller roller, Voltage voltage) {
    return Commands.startEnd(() -> roller.setVoltage(voltage), roller::stop, roller)
        .withName("ExampleRoller_Voltage_" + voltage.in(Volts) + "V");
  }

  /** One-shot stop. */
  public static Command stop(ExampleRoller roller) {
    return Commands.runOnce(roller::stop, roller).withName("ExampleRoller_Stop");
  }

  /**
   * Default command: hold the mechanism at rest.
   *
   * <p>Runs whenever nothing else requires the subsystem. It must <b>never end</b> — {@code
   * Commands.run}, not {@code Commands.runOnce} — or the subsystem spends the match with no default
   * and a released button leaves it in whatever state it was last commanded to.
   */
  public static Command idle(ExampleRoller roller) {
    return Commands.run(() -> roller.setVoltage(Volts.of(0)), roller)
        .withName("ExampleRoller_Idle");
  }

  /**
   * The Ready to Align to Act pattern: wait for the mechanism, then for alignment, then act.
   *
   * <p>Phase 2 gets the <b>remaining</b> budget, not a fresh one. That is what guarantees the whole
   * sequence finishes inside {@code TOTAL_TIMEOUT_SECONDS} even when alignment never arrives.
   *
   * <p><b>Both timeouts are mandatory.</b> A bare {@code waitUntil} hangs forever if its condition
   * never becomes true, and a robot that hangs mid-sequence scores zero. This is a competition
   * safety rule, not a style preference — see {@code .claude/rules/03-commands.md}.
   *
   * <p>Acting anyway on timeout is deliberate: a shot taken slightly misaligned is worth more than
   * a shot never taken. If that is the wrong tradeoff for a particular action, make the last phase
   * conditional on {@code isAligned} rather than removing the timeout.
   */
  public static Command runWhenReady(
      ExampleRoller roller, AngularVelocity velocity, BooleanSupplier isAligned) {
    return Commands.sequence(
            Commands.runOnce(() -> roller.setVelocity(velocity), roller),
            Commands.waitUntil(roller::isAtVelocity)
                .withTimeout(Constants.Waits.MECHANISM_READY_SECONDS),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    Constants.Waits.TOTAL_TIMEOUT_SECONDS
                        - Constants.Waits.MECHANISM_READY_SECONDS))
        .finallyDo(interrupted -> roller.stop())
        .withName("ExampleRoller_WhenReady");
  }

  // ==========================================================================================
  // JAM RESPONSE — pairs with the JAM DETECTION block in ExampleRoller
  // ==========================================================================================
  //
  // The subsystem owns the SIGNAL; this file owns the POLICY. Reversing to clear a jam can eject
  // a game piece the driver wanted, so it is a strategy decision and belongs where strategy lives.
  //
  //   /** Reverses briefly to clear a jam, then resumes. Bind to a driver button, or to
  //    * roller.isJammed() as a trigger once the thresholds are trusted. */
  //   public static Command unjam(ExampleRoller roller, Voltage reverseVoltage) {
  //     return Commands.sequence(
  //             Commands.runOnce(() -> roller.setVoltage(reverseVoltage), roller),
  //             Commands.waitSeconds(Constants.Waits.UNJAM_SECONDS))
  //         .finallyDo(interrupted -> roller.stop())
  //         .withName("ExampleRoller_Unjam");
  //   }
  //
  // Bind it to the operator first and watch the log before wiring it to isJammed() directly. An
  // automatic unjam driven by an untuned threshold ejects game pieces during normal pickup, which
  // is a worse failure than the jam it was meant to fix.
}
