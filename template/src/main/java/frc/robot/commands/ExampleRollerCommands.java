package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.exampleRoller.ExampleRoller;
import frc.robot.subsystems.exampleRoller.ExampleRollerConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

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
   * <p>Phase 3 is {@code act} — the feeder push, the release, whatever the readiness was for. It
   * runs <b>while the roller is still held at velocity</b>; the roller is only stopped once {@code
   * act} has finished, so a feeder never pushes into a flywheel that is already coasting down.
   * {@code act} must end on its own (bound it with {@code withTimeout} or make it a finite
   * sequence), or this command never ends either.
   *
   * <p>Acting anyway on timeout is deliberate: a shot taken slightly misaligned is worth more than
   * a shot never taken. If that is the wrong tradeoff for a particular action, make {@code act}
   * conditional on {@code isAligned} rather than removing the timeout.
   *
   * @param act the action the readiness gates; must be finite
   */
  public static Command runWhenReady(
      ExampleRoller roller, AngularVelocity velocity, BooleanSupplier isAligned, Command act) {
    return Commands.sequence(
            Commands.runOnce(() -> roller.setVelocity(velocity), roller),
            Commands.waitUntil(roller::isAtVelocity)
                .withTimeout(Constants.Waits.MECHANISM_READY_SECONDS),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    Constants.Waits.TOTAL_TIMEOUT_SECONDS
                        - Constants.Waits.MECHANISM_READY_SECONDS),
            act)
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

  /** Dashboard setpoint for {@link #tuneVelocity}, in mechanism RPM. Read nowhere else. */
  private static final LoggedTunableNumber TUNE_VELOCITY_RPM =
      new LoggedTunableNumber(ExampleRollerConstants.NAME + "/Tune/VelocityRpm", 0.0);

  /**
   * Runs at whatever velocity the dashboard says, for finding a setpoint with the mechanism in
   * front of you. Edit {@code Tuning/ExampleRoller/Tune/VelocityRpm} and the roller follows on the
   * next loop; it stops when the command ends.
   *
   * <p><b>Test mode, with Start held.</b> Bind it with {@code
   * triggers.tuningEnabled.whileTrue(...)}, which is true only while the robot is enabled in test
   * mode AND the operator holds Start — a dead-man switch, so enabling test mode for something else
   * does not move the mechanism. It also refuses to start outside test mode and ends the moment the
   * robot leaves it, and {@code LoggedTunableNumber.get()} returns its default outside test mode
   * anyway, so the dashboard value can never reach the field. At the 2027 port it becomes a
   * {@code @Utility} OpMode; keep the held button.
   *
   * <p>When the number is right, write it into {@code Constants.Setpoints} and commit it. The
   * dashboard value lives only in NetworkTables and is gone at the next boot.
   */
  public static Command tuneVelocity(ExampleRoller roller) {
    return Commands.run(
            () -> {
              // Guard every execution as well as scheduling: the end check runs after execute(),
              // so a mode transition must not apply one last dashboard setpoint.
              if (!DriverStation.isTestEnabled()) {
                return;
              }
              double rpm = TUNE_VELOCITY_RPM.get();
              Logger.recordOutput(ExampleRollerConstants.NAME + "/Tune/VelocityRpm", rpm);
              roller.setVelocity(RPM.of(rpm));
            },
            roller)
        .onlyIf(DriverStation::isTestEnabled)
        .until(() -> !DriverStation.isTestEnabled())
        .finallyDo(interrupted -> roller.stop())
        .withName("ExampleRoller_Tune");
  }
}
