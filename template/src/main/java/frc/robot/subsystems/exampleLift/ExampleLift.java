package frc.robot.subsystems.exampleLift;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CommandLogger;
import frc.robot.Robot;
import frc.robot.subsystems.exampleLift.io.ExampleLiftIO;
import frc.robot.subsystems.exampleLift.io.ExampleLiftIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Linear position-controlled mechanism — elevator, lift, extension.
 *
 * <p><b>RULE: this class must behave identically whether the IO is real, sim, or replay.</b> That
 * is the entire point of the abstraction. The moment a {@code TalonFX} import appears here, log
 * replay stops working and you lose the ability to debug a match after the fact.
 *
 * <h2>Inches in, inches out</h2>
 *
 * <p>This class is the unit boundary. The IO layer speaks drum rotations, because that is what the
 * hardware reports and what a log should record; everything this class exposes is in <b>inches</b>,
 * converted through {@link ExampleLiftConstants}. Nothing above this layer should ever see a
 * rotation.
 *
 * <h2>Zeroing</h2>
 *
 * <p>The reported height is <b>meaningless until this mechanism has been zeroed</b> — the motor
 * encoder is relative and reads zero at boot wherever the carriage happens to be sitting. This
 * class holds the state and the primitive; the routine that decides when it is safe to declare zero
 * is {@code ExampleLiftCommands.zero()}, so the scheduler can interrupt it like anything else.
 *
 * <p>{@link #isZeroed()} exists so commands and {@code FaultMonitor} can refuse to trust a height
 * that was never established. Wire it up — an un-zeroed lift running to a setpoint is how a
 * carriage gets driven into a hard stop at full profile speed.
 *
 * <p>TEMPLATE INSTRUCTIONS:
 *
 * <ol>
 *   <li>Rename {@code ExampleLift} to your mechanism name throughout, including the log keys.
 *   <li>Add a public method per control action. Keep them thin.
 *   <li>Do NOT add hardware calls here. Do NOT reference another subsystem — read {@code
 *       RobotState} or take a {@code Supplier} in the constructor.
 * </ol>
 */
public class ExampleLift extends SubsystemBase {

  /**
   * Fraction of the stator limit at which the mechanism counts as saturated.
   *
   * <p>Not 1.0: current limiting is a control loop of its own and rides just under the ceiling
   * rather than pinning to it, so an exact comparison reports saturation far less often than it
   * happens.
   */
  private static final double STATOR_SATURATION_FRACTION = 0.95;

  private final ExampleLiftIO io;
  private final ExampleLiftIOInputsAutoLogged inputs;

  /** Last commanded height in inches, already clamped to travel, for {@link #isAtPosition()}. */
  private double goalInches = ExampleLiftConstants.MIN_TRAVEL_INCHES;

  /**
   * True only while a closed-loop height is being commanded. Open-loop voltage, stop and the
   * zeroing creep clear it, so a goal nobody is holding any more cannot report the lift as arrived.
   */
  private boolean closedLoopActive = false;

  /** False until a zero has been sent AND confirmed. Every height is a lie until then. */
  private boolean zeroed = false;

  /** True between sending a zero and seeing it land (or giving up). See {@link #requestZero()}. */
  private boolean zeroPending = false;

  /** When the pending zero was sent, for the {@code ZERO_CONFIRM_SECONDS} window. */
  private double zeroRequestedAt = 0.0;

  /**
   * Debounces the zeroing stall check.
   *
   * <p>Evaluated ONCE per loop in {@link #periodic()}, never inside the getter. A debouncer
   * advances its timer every time it is polled, so calling {@code calculate()} from {@link
   * #isAtZeroingStall()} would make the dwell depend on how many callers happened to ask — a
   * command polling it and a logger reading it in the same loop would trip it in half the time.
   */
  private final Debouncer zeroingStallDebounce =
      new Debouncer(
          ExampleLiftConstants.ZEROING_STALL_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);

  private boolean atZeroingStall = false;

  /**
   * Draws the mechanism. Diagnostic only — deleting this field and the file changes nothing except
   * what you can see. It earns its place here: see {@link ExampleLiftVisualizer}.
   */
  private final ExampleLiftVisualizer visualizer = new ExampleLiftVisualizer();

  public ExampleLift(ExampleLiftIO io) {
    this.io = io;
    this.inputs = new ExampleLiftIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(ExampleLiftConstants.NAME, inputs); // NEVER remove this line
    // Which command owns this mechanism right now, default included. Answers "what was driving
    // the lift at 47 s?" without reconstructing it from the global command list.
    CommandLogger.recordCurrentCommand(ExampleLiftConstants.NAME, this);

    // Supply current, not stator — this feeds a battery-power calculation. In 2026 the drivetrain
    // passed stator current here and every power and energy figure from that season is inflated.
    // See .claude/rules/02-hardware.md.
    Robot.batteryLogger.reportCurrentUsage(
        ExampleLiftConstants.NAME, false, inputs.motorSupplyAmps.in(Amps));

    // Once per loop, before anything reads it. See the field comment.
    atZeroingStall = zeroingStallDebounce.calculate(isZeroingStallConditionPresent());

    updateZeroConfirmation();

    Logger.recordOutput(ExampleLiftConstants.NAME + "/GoalInches", goalInches);
    Logger.recordOutput(ExampleLiftConstants.NAME + "/PositionInches", getPositionInches());
    Logger.recordOutput(ExampleLiftConstants.NAME + "/AtPosition", isAtPosition());
    Logger.recordOutput(ExampleLiftConstants.NAME + "/Zeroed", zeroed);
    Logger.recordOutput(ExampleLiftConstants.NAME + "/ZeroPending", zeroPending);
    Logger.recordOutput(ExampleLiftConstants.NAME + "/AtZeroingStall", atZeroingStall);
    Logger.recordOutput(ExampleLiftConstants.NAME + "/AtStatorLimit", isAtStatorLimit());

    // Inches in, because the visualizer holds no unit math on purpose — the conversion belongs at
    // the one boundary in ExampleLiftConstants.
    visualizer.update(getPositionInches(), goalInches, isAtPosition());
  }

  // ============================================================================================
  // Control — called by ExampleLiftCommands
  // ============================================================================================

  /**
   * Commands a profiled move to a height, in inches, and holds there.
   *
   * <p>The goal is <b>clamped to travel</b> before it is sent, and {@code GoalWasClamped} is
   * logged. Unclamped, the Talon would stop at the soft limit and {@link #isAtPosition()} would
   * read false forever, so every sequence waiting on it would burn its whole timeout with nothing
   * in the log saying why. The clamp flag is that missing line.
   *
   * <p><b>Only meaningful once {@link #isZeroed()} is true.</b> Check zeroing in the command
   * factory, where refusal is visible in the command log. This method also requires confirmed
   * startup configuration.
   */
  public void setPositionInches(double inches) {
    Logger.recordOutput(
        ExampleLiftConstants.NAME + "/ClosedLoopBlockedByStartupConfig", !inputs.configured);
    if (!inputs.configured) {
      stop();
      return;
    }
    double clamped =
        Math.max(
            ExampleLiftConstants.MIN_TRAVEL_INCHES,
            Math.min(ExampleLiftConstants.MAX_TRAVEL_INCHES, inches));
    Logger.recordOutput(ExampleLiftConstants.NAME + "/GoalWasClamped", clamped != inches);
    goalInches = clamped;
    closedLoopActive = true;
    io.setPosition(Rotations.of(ExampleLiftConstants.inchesToRotations(clamped)));
  }

  /**
   * Open-loop. Used by the zeroing routine, and by bring-up.
   *
   * <p>This bypasses gravity compensation, so zero volts lets the carriage fall. It is not a safe
   * resting state — the default command holds a height instead.
   */
  public void setVoltage(Voltage volts) {
    closedLoopActive = false;
    // Open-loop voltage remains available for zeroing and bring-up.
    io.setVoltage(volts);
  }

  /** Stops commanding output. See the warning on {@link #setVoltage}. */
  public void stop() {
    closedLoopActive = false;
    io.stop();
  }

  /**
   * Switches the reverse soft limit, writing only if the device does not already report that state.
   *
   * <p>Only {@code ExampleLiftCommands.zero()} should call this. The write is the one the zeroing
   * routine waits for — see {@code ExampleLiftIO.setReverseSoftLimitEnabled} — so skipping it when
   * nothing would change is what keeps a first zero after boot to a single waited write.
   */
  public void setReverseSoftLimitEnabled(boolean enabled) {
    if (inputs.reverseSoftLimitEnabled != enabled) {
      io.setReverseSoftLimitEnabled(enabled);
    }
  }

  /**
   * Forgets any stall the detector has already seen. Called by {@code ExampleLiftCommands.zero()}
   * once its creep has settled, so the stall that ends zeroing is one that happened <b>under the
   * creep</b> — not one left over from a carriage that was already jammed when the button was
   * pressed. Without this, {@code zero()} could finish on its first loop and declare zero mid-air.
   */
  public void resetZeroingStallDetection() {
    zeroingStallDebounce.calculate(false);
    atZeroingStall = false;
  }

  /**
   * Sends "the current position is zero" to the device and starts confirming it.
   *
   * <p>Only call this once the carriage is genuinely at the hard stop — {@code
   * ExampleLiftCommands.zero()} is what establishes that. A zero taken at an unknown position is
   * worse than no zero: the mechanism believes it for the rest of the match, soft limits included.
   *
   * <p>The write is sent without waiting, so the robot loop never stalls on it. The lift becomes
   * {@link #isZeroed() zeroed} only once the logged position reads within {@code
   * ZERO_CONFIRM_TOLERANCE_ROTATIONS} of zero, inside {@code ZERO_CONFIRM_SECONDS}; otherwise it
   * stays not zeroed and {@code ZeroFailed} is logged. Declaring zero on a write that never landed
   * is the silent fixed-offset failure this routine exists to prevent. Because the decision is made
   * from logged inputs, replay reaches the same verdict the robot did.
   *
   * <p>The one ambiguous case is harmless: if the write is lost but the position already read near
   * zero, the lift booted on its stop, so its power-on zero was already the right one.
   */
  public void requestZero() {
    zeroed = false; // the frame is about to change; nothing may trust the old one meanwhile
    closedLoopActive = false;
    zeroPending = true;
    zeroRequestedAt = Timer.getFPGATimestamp();
    io.zeroPosition();
  }

  /** Settles a pending zero. Called once per loop from {@link #periodic()}, after the inputs. */
  private void updateZeroConfirmation() {
    if (!zeroPending) {
      return;
    }
    if (Math.abs(inputs.motorPosition.in(Rotations))
        < ExampleLiftConstants.ZERO_CONFIRM_TOLERANCE_ROTATIONS) {
      zeroPending = false;
      zeroed = true;
      goalInches = 0.0;
      Logger.recordOutput(ExampleLiftConstants.NAME + "/ZeroFailed", false);
    } else if (Timer.getFPGATimestamp() - zeroRequestedAt
        > ExampleLiftConstants.ZERO_CONFIRM_SECONDS) {
      zeroPending = false;
      Logger.recordOutput(ExampleLiftConstants.NAME + "/ZeroFailed", true);
      DriverStation.reportError(
          ExampleLiftConstants.NAME
              + " zero failed: the position never read zero after the zero was sent",
          false);
    }
  }

  /** True between sending a zero and its confirmation or failure. */
  public boolean isZeroPending() {
    return zeroPending;
  }

  /** Whether the device has confirmed the reverse soft limit is on. */
  public boolean isReverseSoftLimitEnabled() {
    return inputs.reverseSoftLimitEnabled;
  }

  // ============================================================================================
  // State queries — read by Triggers, commands, and FaultMonitor
  // ============================================================================================

  /** Current carriage height in inches. Meaningless unless {@link #isZeroed()} is true. */
  public double getPositionInches() {
    return ExampleLiftConstants.rotationsToInches(inputs.motorPosition.in(Rotations));
  }

  /**
   * True when the carriage is within tolerance of the requested height.
   *
   * <p>Compared in inches, because that is the unit the tolerance is written in and the unit anyone
   * reading the log is thinking in. A tolerance in degrees would mean nothing here, which is why
   * the linear and rotary scaffolds are separate files.
   *
   * <p>This is a pure predicate: it is logged from {@link #periodic()}, not from inside here.
   */
  public boolean isAtPosition() {
    // False under open-loop or stop: sitting near an old goal is not the same as holding it.
    return closedLoopActive
        && Math.abs(getPositionInches() - goalInches)
            < ExampleLiftConstants.POSITION_TOLERANCE_INCHES;
  }

  /** Has the hard stop been found, and the zero confirmed, this power cycle? */
  public boolean isZeroed() {
    return zeroed;
  }

  /**
   * True once current is high AND velocity is near zero for a sustained dwell.
   *
   * <p>Not current alone: current spikes at the instant the motor breaks static friction, long
   * before the carriage is anywhere near the real hard stop. Requiring both, for a dwell, is what
   * tells a breakaway apart from an arrival.
   *
   * <p>Only meaningful while the zeroing command is actively creeping downward. Reading it at any
   * other time tells you nothing, because a stationary lift holding position also has near-zero
   * velocity.
   */
  public boolean isAtZeroingStall() {
    return atZeroingStall;
  }

  /** Is the motor answering on the bus? Register with {@code FaultMonitor} in RobotContainer. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Were all startup writes confirmed? Register with {@code FaultMonitor} in RobotContainer. */
  public boolean isConfigured() {
    return inputs.configured;
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
   * <p><b>Read it with care in simulation.</b> Sim current comes from WPILib's model, which knows
   * nothing about the Talon's limits, so this can read true at currents the real device would never
   * allow. In sim that means the sim gains are pushing harder than the robot could — not that the
   * limit needs raising.
   */
  public boolean isAtStatorLimit() {
    // Magnitude: Phoenix signs stator current with the direction of output, so a bare comparison
    // only ever reads true while driving up.
    return Math.abs(inputs.motorStatorAmps.in(Amps))
        > ExampleLiftConstants.STATOR_CURRENT_LIMIT_AMPS * STATOR_SATURATION_FRACTION;
  }

  private boolean isZeroingStallConditionPresent() {
    // STATOR, not supply. A stall against the hard stop is near-zero velocity, so the controller
    // is chopping hard and supply is only a small fraction of stator — a supply threshold set to
    // a plausible-looking stall current is one the mechanism never reaches, and the symptom is a
    // zeroing routine that always runs to its timeout and never zeroes. Stator is the winding
    // current, which is what actually rises when the mechanism cannot turn.
    //
    // MAGNITUDE, not the signed value. Phoenix signs stator current with the direction of output,
    // and zeroing drives DOWN, so a signed comparison against a positive threshold can never be
    // true here: the routine would always run to its timeout and never zero.
    boolean highCurrent =
        Math.abs(inputs.motorStatorAmps.in(Amps)) > ExampleLiftConstants.ZEROING_STALL_CURRENT_AMPS;
    boolean nearZeroVelocity =
        Math.abs(inputs.motorVelocity.in(RPM))
            < ExampleLiftConstants.ZEROING_VELOCITY_THRESHOLD_RPM;
    return highCurrent && nearZeroVelocity;
  }
}
