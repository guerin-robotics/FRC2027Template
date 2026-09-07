package frc.robot.subsystems.exampleRoller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.exampleRoller.io.ExampleRollerIO;
import frc.robot.subsystems.exampleRoller.io.ExampleRollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Velocity-controlled mechanism — roller, flywheel, feeder, transport, intake.
 *
 * <p><b>RULE: this class must behave identically whether the IO is real, sim, or replay.</b> That
 * is the entire point of the abstraction. The moment a {@code TalonFX} import appears here, log
 * replay stops working and you lose the ability to debug a match after the fact.
 *
 * <p>There is deliberately <b>no visualizer</b> for this scaffold. A spinning drum has no position
 * worth drawing, and a ligament turning at 6000 RPM sampled at 50 Hz aliases into a bar that
 * appears to drift slowly backward. The velocity plot beside {@code closedLoopReference} already
 * answers every question a picture would, and answers it better. {@code exampleArm} and {@code
 * exampleLift} both ship one, because for those it earns its place.
 *
 * <p>TEMPLATE INSTRUCTIONS:
 *
 * <ol>
 *   <li>Rename {@code ExampleRoller} to your mechanism name throughout, including the log keys.
 *   <li>Add a public method per control action. Keep them thin — they set a goal and forward to the
 *       IO; anything that decides <i>when</i> belongs in a command factory.
 *   <li>Do NOT add hardware calls here. Do NOT reference another subsystem — read {@code
 *       RobotState} or take a {@code Supplier} in the constructor.
 * </ol>
 */
public class ExampleRoller extends SubsystemBase {

  /**
   * Fraction of the stator limit at which the mechanism counts as saturated.
   *
   * <p>Not 1.0: current limiting is a control loop of its own and rides just under the ceiling
   * rather than pinning to it, so an exact comparison reports saturation far less often than it
   * happens.
   */
  private static final double STATOR_SATURATION_FRACTION = 0.95;

  private final ExampleRollerIO io;
  private final ExampleRollerIOInputsAutoLogged inputs;

  /** Last commanded velocity, so {@link #isAtVelocity()} has something to compare against. */
  private AngularVelocity goalVelocity = RotationsPerSecond.of(0);

  public ExampleRoller(ExampleRollerIO io) {
    this.io = io;
    this.inputs = new ExampleRollerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(ExampleRollerConstants.NAME, inputs); // NEVER remove this line

    // Supply current, not stator — this feeds a battery-power calculation. In 2026 the drivetrain
    // passed stator current here and every power and energy figure from that season is inflated.
    // See .claude/rules/02-hardware.md.
    Robot.batteryLogger.reportCurrentUsage(
        ExampleRollerConstants.NAME, false, inputs.motorSupplyAmps.in(Amps));

    // Log the goal alongside the measurement. Without it a log shows the mechanism at 2000 RPM
    // and cannot say whether that was right.
    Logger.recordOutput(ExampleRollerConstants.NAME + "/GoalRpm", goalVelocity.in(RPM));
    Logger.recordOutput(ExampleRollerConstants.NAME + "/VelocityRpm", inputs.motorVelocity.in(RPM));
    Logger.recordOutput(ExampleRollerConstants.NAME + "/AtVelocity", isAtVelocity());
    Logger.recordOutput(ExampleRollerConstants.NAME + "/AtStatorLimit", isAtStatorLimit());
  }

  // ============================================================================================
  // Control — called by ExampleRollerCommands
  // ============================================================================================

  /** Open-loop. Bring-up and characterization only; match logic should use velocity control. */
  public void setVoltage(Voltage volts) {
    goalVelocity = RotationsPerSecond.of(0);
    io.setVoltage(volts);
  }

  public void setVelocity(AngularVelocity velocity) {
    goalVelocity = velocity;
    io.setVelocity(velocity);
  }

  public void stop() {
    goalVelocity = RotationsPerSecond.of(0);
    io.stop();
  }

  // ============================================================================================
  // State queries — read by Triggers, commands, and FaultMonitor
  // ============================================================================================

  /**
   * True when the mechanism is within tolerance of the requested velocity.
   *
   * <p>Every mechanism a command waits on needs a query like this, and it needs an explicit
   * tolerance rather than an equality check — a real mechanism never sits exactly on its setpoint.
   * The tolerance lives in the constants file so it can be adjusted without touching logic.
   *
   * <p>Compared in RPM, because that is the unit the tolerance is written in and the unit anyone
   * reading the log is thinking in.
   *
   * <p>This is a pure predicate: it is logged from {@link #periodic()}, not from inside here.
   * Recording from a getter means the channel updates only as often as something happens to call
   * it, so a log gap reads as "the robot stopped checking" rather than "nothing asked".
   */
  public boolean isAtVelocity() {
    return Math.abs(inputs.motorVelocity.in(RPM) - goalVelocity.in(RPM))
        < ExampleRollerConstants.VELOCITY_TOLERANCE_RPM.get();
  }

  /** Is the motor answering on the bus? Register with {@code FaultMonitor} in RobotContainer. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /**
   * True while stator current is sitting at the configured ceiling.
   *
   * <p>This is how you tell "the gains are wrong" apart from "the current limit is the constraint",
   * and those need opposite fixes. The scaffolds ship {@code STATOR_CURRENT_LIMIT_AMPS}
   * deliberately low, so this flag is what says the limit has been outgrown — raise it, and {@code
   * PEAK_FORWARD_TORQUE_CURRENT_AMPS} with it, rather than reaching for kP.
   *
   * <p>Read it beside {@code closedLoopReference}: reference tracking the goal, measured lagging
   * behind, and this flag true is saturation. The same lag with this flag false is a gain problem.
   *
   * <p>In simulation this stays false — the limits are not enforced there, by design. See the note
   * in {@code getFXConfig()}.
   */
  public boolean isAtStatorLimit() {
    return inputs.motorStatorAmps.in(Amps)
        > ExampleRollerConstants.STATOR_CURRENT_LIMIT_AMPS * STATOR_SATURATION_FRACTION;
  }

  // ============================================================================================
  // JAM DETECTION — keep for anything that can stall against a game piece
  // ============================================================================================
  //
  // Rollers, feeders, intakes and transports jam. A flywheel spinning in free air does not —
  // delete this block and the matching constants if that is what you are building.
  //
  // 2026 ran its rollers open-loop with no feedback, so jams were SILENT: the mechanism stopped
  // working, and nothing in the log said why. This is the cheapest instrumentation on the list and
  // it is the one that was missing.
  //
  // A JAM IS A CONJUNCTION, never a single signal:
  //
  //   1. we are actually commanding motion         (an idle roller is not jammed)
  //   2. measured velocity is far below commanded  (it is not turning)
  //   3. stator current is high                    (it is trying hard)
  //   4. all three have held for a dwell           (not a transient)
  //
  // Current alone is the classic mistake — it spikes on every static-friction breakaway and every
  // first contact with a game piece, both entirely normal.
  //
  // STATOR, not supply. .claude/rules/02-hardware.md assigns stall indication to stator: it is the
  // winding current. At the low-speed, high-load condition that defines a jam, supply is only a
  // fraction of stator because the controller is chopping, so a supply threshold sits much closer
  // to the noise floor.
  //
  // THE DWELL MUST EXCEED SPIN-UP TIME, or every start reads as a jam: during spin-up the command
  // is high, the measurement is low and the current is high — exactly the jam signature. Check it
  // against ACCELERATION_RPM_PER_SEC, and raise it if that ever drops.
  //
  // DETECT HERE, RESPOND IN A COMMAND. Reversing to clear a jam is a policy decision with a
  // game-strategy answer — it can eject a piece the driver wanted. The subsystem owns the signal;
  // ExampleRollerCommands owns what to do about it.
  //
  //   private final Debouncer jamDebounce =
  //       new Debouncer(ExampleRollerConstants.JAM_DEBOUNCE_SECONDS, kRising);
  //
  //   // Evaluated ONCE per loop in periodic(), never inside the getter. A debouncer advances its
  //   // timer every time it is polled, so calling calculate() from isJammed() would make the
  //   // dwell depend on how many callers happened to ask — a command polling it and a
  //   // FaultMonitor condition reading it in the same loop would trip it in half the time.
  //   private boolean jammed = false;
  //   private int jamCount = 0;
  //   private boolean wasJammed = false;
  //
  //   // ...called from periodic():
  //   private void updateJamDetection() {
  //     jammed = jamDebounce.calculate(isJamConditionPresent());
  //     if (jammed && !wasJammed) {
  //       jamCount++;
  //     }
  //     wasJammed = jammed;
  //     Logger.recordOutput(ExampleRollerConstants.NAME + "/Jammed", jammed);
  //     Logger.recordOutput(ExampleRollerConstants.NAME + "/JamCount", jamCount);
  //     // The raw conjunction too. Comparing it against Jammed in a log is how you tell
  //     // "threshold too low, it keeps flickering" from "dwell too long, it never latches".
  //     Logger.recordOutput(
  //         ExampleRollerConstants.NAME + "/JamConditionRaw", isJamConditionPresent());
  //   }
  //
  //   private boolean isJamConditionPresent() {
  //     double commandedRpm = Math.abs(goalVelocity.in(RPM));
  //     if (commandedRpm < ExampleRollerConstants.JAM_MIN_COMMANDED_RPM) {
  //       return false;   // idle is not jammed
  //     }
  //     boolean notTurning =
  //         Math.abs(inputs.motorVelocity.in(RPM))
  //             < commandedRpm * ExampleRollerConstants.JAM_VELOCITY_FRACTION;
  //     boolean workingHard =
  //         inputs.motorStatorAmps.in(Amps) > ExampleRollerConstants.JAM_STATOR_CURRENT_AMPS;
  //     return notTurning && workingHard;
  //   }
  //
  //   /** Live condition — clears when the jam does. Safe to poll from anywhere. */
  //   public boolean isJammed() {
  //     return jammed;
  //   }
  //
  // stop() and setVoltage() already clear goalVelocity above, which the detector depends on —
  // otherwise it keeps comparing against a setpoint nobody is commanding any more.
  //
  // AND: register it with FaultMonitor in RobotContainer, so a mechanism jamming repeatedly
  // reaches the pit between matches instead of only the log.
}
