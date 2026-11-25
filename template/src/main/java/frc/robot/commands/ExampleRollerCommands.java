package frc.robot.commands;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.exampleRoller.ExampleRoller;
import frc.robot.subsystems.exampleRoller.ExampleRollerConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

/**
 * Command factory for a velocity-controlled mechanism.
 *
 * <p>Commands are <b>static factory methods, not classes</b>. Named command classes create
 * unnecessary files and hide composition structure; factories are composable, testable, and
 * traceable in AdvantageKit logs. See {@code .claude/rules/01-architecture.md}.
 *
 * <p><b>Every command ends in {@code .named(...)}</b> — Commands V3 will not build one without a
 * name. Names appear in AdvantageKit's command log, and bad names (or {@code withAutomaticName()}
 * on anything bound to a button) make a match impossible to debug. Format: {@code
 * "SubsystemName_ActionVerb_OptionalParam"}.
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
 *   <tr><td>Run while held, stop on release</td><td>{@code mech.run(co -> { start; co.park(); }).whenExited(stop)}</td></tr>
 *   <tr><td>Do this once and finish</td><td>{@code mech.run(co -> action)}</td></tr>
 *   <tr><td>Run every loop until interrupted</td><td>{@code mech.runRepeatedly(action)}</td></tr>
 *   <tr><td>A, then B, then C</td><td>{@code Command.sequence(a, b, c)}</td></tr>
 *   <tr><td>Run both, stop when A finishes</td><td>{@code new ParallelGroupBuilder().requiring(a).optional(b)}</td></tr>
 * </table>
 *
 * <p>A body that loops must {@code co.yield()} every pass. V3 has no preemption: a loop that never
 * yields freezes the whole robot program.
 */
public class ExampleRollerCommands {

  private ExampleRollerCommands() {}

  /**
   * Built once. {@link #idle} is a default command, so it runs every loop of every match, and a
   * {@code Volts.of(0)} written inline would allocate a new object each time.
   */
  private static final Voltage ZERO_VOLTS = Volts.of(0);

  /**
   * Spins at a velocity while the command runs, and stops on release.
   *
   * <p>Parks rather than finishing because a roller left spinning after its button is released is
   * the single most common way a mechanism ends up running through a whole match. {@code
   * whenExited} is what guarantees it stops — on cancel, on interruption, on any exit. (A body that
   * parks can only exit by cancel; {@code whenExited} also covers a body that can finish.)
   */
  public static Command runAtVelocity(ExampleRoller roller, AngularVelocity velocity) {
    return roller
        .run(
            co -> {
              roller.setVelocity(velocity);
              co.park();
            })
        .whenExited(roller::stop)
        .named("ExampleRoller_Velocity");
  }

  /** Open-loop. Bring-up and characterization only — match logic should command a velocity. */
  public static Command runAtVoltage(ExampleRoller roller, Voltage voltage) {
    return roller
        .run(
            co -> {
              roller.setVoltage(voltage);
              co.park();
            })
        .whenExited(roller::stop)
        .named("ExampleRoller_Voltage_" + voltage.in(Volts) + "V");
  }

  /** One-shot stop. */
  public static Command stop(ExampleRoller roller) {
    return roller.run(co -> roller.stop()).named("ExampleRoller_Stop");
  }

  /**
   * Default command: hold the mechanism at rest.
   *
   * <p>Runs whenever nothing else requires the mechanism. It must <b>never end</b> — {@code
   * runRepeatedly}, not a one-shot — or the mechanism spends the match with no default and a
   * released button leaves it in whatever state it was last commanded to.
   *
   * <p>It runs at {@code LOWEST_PRIORITY}: V3 lets a new command interrupt a running one only at
   * equal or higher priority, so a default at {@code DEFAULT_PRIORITY} or above would block every
   * ordinary command. V3 also gives an unclaimed mechanism an {@code idle()} that does nothing, and
   * for a motor that means the last control request keeps running — which is why a motor mechanism
   * still needs this explicit stop/hold default.
   */
  public static Command idle(ExampleRoller roller) {
    return roller
        .runRepeatedly(() -> roller.setVoltage(ZERO_VOLTS))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("ExampleRoller_Idle");
  }

  /**
   * The Ready to Align to Act pattern: wait for the mechanism, then for alignment, then act.
   *
   * <p>Phase 2 gets the <b>remaining</b> budget, not a fresh one. That is what guarantees the whole
   * sequence finishes inside {@code TOTAL_TIMEOUT_SECONDS} even when alignment never arrives.
   *
   * <p><b>Both timeouts are mandatory.</b> An untimed {@code waitUntil} — {@code
   * Command.waitUntil(cond)} or {@code coroutine.waitUntil(cond)} with no timeout — hangs forever
   * if its condition never becomes true, and a robot that hangs mid-sequence scores zero. This is a
   * competition safety rule, not a style preference — see {@code .claude/rules/03-commands.md}. The
   * timed form returns a {@code WaitResult}; this factory logs whether each phase timed out, so a
   * log can tell "ready" from "gave up".
   *
   * <p>Phase 3 is {@code act} — the feeder push, the release, whatever the readiness was for. It
   * runs <b>while the roller is still held at velocity</b>; the roller is only stopped once {@code
   * act} has finished, so a feeder never pushes into a flywheel that is already coasting down.
   * {@code act} must end on its own (bound it with {@code withTimeout} or make it a finite
   * sequence), or this command never ends either, and it should <b>not require the roller</b> —
   * this command owns it for the whole sequence, and a child that shares it has ownership semantics
   * this repo has not verified (see {@code .claude/rules/03-commands.md}).
   *
   * <p>Acting anyway on timeout is deliberate: a shot taken slightly misaligned is worth more than
   * a shot never taken. If that is the wrong tradeoff for a particular action, make {@code act}
   * conditional on {@code isAligned} rather than removing the timeout.
   *
   * @param act the action the readiness gates; must be finite and should not require the roller
   */
  public static Command runWhenReady(
      ExampleRoller roller, AngularVelocity velocity, BooleanSupplier isAligned, Command act) {
    return roller
        .run(
            co -> {
              roller.setVelocity(velocity);
              var ready =
                  co.waitUntil(
                      roller::isAtVelocity, Seconds.of(Constants.Waits.MECHANISM_READY_SECONDS));
              var aligned =
                  co.waitUntil(
                      isAligned,
                      Seconds.of(
                          Constants.Waits.TOTAL_TIMEOUT_SECONDS
                              - Constants.Waits.MECHANISM_READY_SECONDS));
              Logger.recordOutput(
                  ExampleRollerConstants.NAME + "/WhenReady/ReadyTimedOut", ready.timedOut());
              Logger.recordOutput(
                  ExampleRollerConstants.NAME + "/WhenReady/AlignTimedOut", aligned.timedOut());
              co.await(act);
            })
        .whenExited(roller::stop)
        .named("ExampleRoller_WhenReady");
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
  //     return roller
  //         .run(
  //             co -> {
  //               roller.setVoltage(reverseVoltage);
  //               co.wait(Seconds.of(Constants.Waits.UNJAM_SECONDS));
  //             })
  //         .whenExited(roller::stop)
  //         .named("ExampleRoller_Unjam");
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
  public static Command tuneVelocity(ExampleRoller roller) {
    return roller
        .run(
            co -> {
              // The loop condition is the guard: it refuses to start outside Utility mode and
              // ends the moment the robot leaves it, so a mode transition applies no last setpoint.
              while (org.wpilib.driverstation.RobotState.isUtilityEnabled()) {
                double rpm = TUNE_VELOCITY_RPM.get();
                Logger.recordOutput(ExampleRollerConstants.NAME + "/Tune/VelocityRpm", rpm);
                roller.setVelocity(RPM.of(rpm));
                co.yield();
              }
            })
        .whenExited(roller::stop)
        .named("ExampleRoller_Tune");
  }
}
