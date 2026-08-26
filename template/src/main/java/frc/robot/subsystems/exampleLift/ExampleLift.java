package frc.robot.subsystems.exampleLift;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final ExampleLiftIO io;
  private final ExampleLiftIOInputsAutoLogged inputs;

  /** Last commanded height in inches, so {@link #isAtPosition()} has something to compare to. */
  private double goalInches = 0.0;

  /** False until the zeroing routine has found the hard stop. Every height is a lie until then. */
  private boolean zeroed = false;

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

    // Supply current, not stator — this feeds a battery-power calculation. In 2026 the drivetrain
    // passed stator current here and every power and energy figure from that season is inflated.
    // See .claude/rules/02-hardware.md.
    Robot.batteryLogger.reportCurrentUsage(
        ExampleLiftConstants.NAME, false, inputs.motorSupplyAmps.in(Amps));

    // Once per loop, before anything reads it. See the field comment.
    atZeroingStall = zeroingStallDebounce.calculate(isZeroingStallConditionPresent());

    Logger.recordOutput(ExampleLiftConstants.NAME + "/GoalInches", goalInches);
    Logger.recordOutput(ExampleLiftConstants.NAME + "/PositionInches", getPositionInches());
    Logger.recordOutput(ExampleLiftConstants.NAME + "/AtPosition", isAtPosition());
    Logger.recordOutput(ExampleLiftConstants.NAME + "/Zeroed", zeroed);
    Logger.recordOutput(ExampleLiftConstants.NAME + "/AtZeroingStall", atZeroingStall);

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
   * <p>Soft limits are enforced on the Talon, so an out-of-range goal is clamped by the device
   * rather than driving into a hard stop. That does mean {@link #isAtPosition()} will read false
   * forever for such a goal, which is the intended tell.
   *
   * <p><b>Only meaningful once {@link #isZeroed()} is true.</b> Nothing here blocks the call — a
   * subsystem that silently refuses commands is harder to debug than one that obeys — so the check
   * belongs in the command factory, where refusing is visible in the command log.
   */
  public void setPositionInches(double inches) {
    goalInches = inches;
    io.setPosition(Rotations.of(ExampleLiftConstants.inchesToRotations(inches)));
  }

  /**
   * Open-loop. Used by the zeroing routine, and by bring-up.
   *
   * <p>This bypasses gravity compensation, so zero volts lets the carriage fall. It is not a safe
   * resting state — the default command holds a height instead.
   */
  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  /** Stops commanding output. See the warning on {@link #setVoltage}. */
  public void stop() {
    io.stop();
  }

  /**
   * Declares the current position to be zero.
   *
   * <p>Only call this once the carriage is genuinely at the hard stop — {@code
   * ExampleLiftCommands.zero()} is what establishes that. A zero taken at an unknown position is
   * worse than no zero: the mechanism believes it for the rest of the match, soft limits included.
   */
  public void zeroAtCurrentPosition() {
    io.zeroPosition();
    zeroed = true;
    goalInches = 0.0;
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
    return Math.abs(getPositionInches() - goalInches)
        < ExampleLiftConstants.POSITION_TOLERANCE_INCHES;
  }

  /** Has the hard stop been found and zero established this power cycle? */
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

  private boolean isZeroingStallConditionPresent() {
    boolean highCurrent =
        inputs.motorSupplyAmps.in(Amps) > ExampleLiftConstants.ZEROING_STALL_CURRENT_AMPS;
    boolean nearZeroVelocity =
        Math.abs(inputs.motorVelocity.in(RPM))
            < ExampleLiftConstants.ZEROING_VELOCITY_THRESHOLD_RPM;
    return highCurrent && nearZeroVelocity;
  }
}
