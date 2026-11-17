package frc.robot.subsystems.exampleArm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CommandLogger;
import frc.robot.Robot;
import frc.robot.subsystems.exampleArm.io.ExampleArmIO;
import frc.robot.subsystems.exampleArm.io.ExampleArmIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Rotary position-controlled mechanism — arm, pivot, hood, wrist, turret.
 *
 * <p><b>RULE: this class must behave identically whether the IO is real, sim, or replay.</b> That
 * is the entire point of the abstraction. The moment a {@code TalonFX} import appears here, log
 * replay stops working and you lose the ability to debug a match after the fact.
 *
 * <p>There is deliberately <b>no zeroing routine</b> in this scaffold. A fused CANcoder knows where
 * the mechanism is at boot, which is exactly why a position mechanism gets one. If this arm's
 * travel exceeds one sensor rotation — see the note in {@link ExampleArmConstants} — the CANcoder
 * cannot say which turn it is on, and you want {@code exampleLift}'s motor-encoder-plus-zeroing
 * shape instead.
 *
 * <p>TEMPLATE INSTRUCTIONS:
 *
 * <ol>
 *   <li>Rename {@code ExampleArm} to your mechanism name throughout, including the log keys.
 *   <li>Add a public method per control action. Keep them thin — they set a goal and forward to the
 *       IO; anything that decides <i>when</i> belongs in a command factory.
 *   <li>Do NOT add hardware calls here. Do NOT reference another subsystem — read {@code
 *       RobotState} or take a {@code Supplier} in the constructor.
 * </ol>
 */
public class ExampleArm extends SubsystemBase {

  /**
   * Fraction of the stator limit at which the mechanism counts as saturated.
   *
   * <p>Not 1.0: current limiting is a control loop of its own and rides just under the ceiling
   * rather than pinning to it, so an exact comparison reports saturation far less often than it
   * happens.
   */
  private static final double STATOR_SATURATION_FRACTION = 0.95;

  private final ExampleArmIO io;
  private final ExampleArmIOInputsAutoLogged inputs;

  /**
   * Last commanded position, already clamped to travel, so {@link #isAtPosition()} has something to
   * compare against. Starts at the reverse bound rather than zero, because zero is not necessarily
   * inside travel.
   */
  private Angle goalPosition = Degrees.of(ExampleArmConstants.REVERSE_SOFT_LIMIT_DEGREES);

  /**
   * True only while a closed-loop position is being commanded. Open-loop voltage and stop clear it,
   * so a goal nobody is holding any more cannot report the arm as arrived.
   */
  private boolean closedLoopActive = false;

  /**
   * Draws the mechanism. Diagnostic only — deleting this field and the file changes nothing except
   * what you can see. It earns its place on a position mechanism: see {@link ExampleArmVisualizer}.
   */
  private final ExampleArmVisualizer visualizer = new ExampleArmVisualizer();

  public ExampleArm(ExampleArmIO io) {
    this.io = io;
    this.inputs = new ExampleArmIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(ExampleArmConstants.NAME, inputs); // NEVER remove this line
    // Which command owns this mechanism right now, default included. Answers "what was driving
    // the arm at 47 s?" without reconstructing it from the global command list.
    CommandLogger.recordCurrentCommand(ExampleArmConstants.NAME, this);

    // Supply current, not stator — this feeds a battery-power calculation. In 2026 the drivetrain
    // passed stator current here and every power and energy figure from that season is inflated.
    // See .claude/rules/02-hardware.md.
    Robot.batteryLogger.reportCurrentUsage(
        ExampleArmConstants.NAME, false, inputs.motorSupplyAmps.in(Amps));

    // Log the goal alongside the measurement. Without it a log shows the arm at 47 degrees and
    // cannot say whether that was right.
    Logger.recordOutput(ExampleArmConstants.NAME + "/GoalDegrees", goalPosition.in(Degrees));
    Logger.recordOutput(
        ExampleArmConstants.NAME + "/PositionDegrees", inputs.motorPosition.in(Degrees));
    Logger.recordOutput(ExampleArmConstants.NAME + "/AtPosition", isAtPosition());
    Logger.recordOutput(ExampleArmConstants.NAME + "/AtStatorLimit", isAtStatorLimit());

    // Degrees in, because the visualizer holds no unit math on purpose — the conversion belongs at
    // the one boundary in ExampleArmConstants, and a second copy is a second place for a factor to
    // get in.
    visualizer.update(inputs.motorPosition.in(Degrees), goalPosition.in(Degrees), isAtPosition());
  }

  // ============================================================================================
  // Control — called by ExampleArmCommands
  // ============================================================================================

  /**
   * Commands a profiled move to an angle, and holds there.
   *
   * <p>The goal is <b>clamped to the soft limits</b> before it is sent, and {@code GoalWasClamped}
   * is logged. Unclamped, the Talon would stop at the bound and {@link #isAtPosition()} would read
   * false forever, so every sequence waiting on it would burn its whole timeout with nothing in the
   * log saying why. The clamp flag is that missing line.
   */
  public void setPosition(Angle position) {
    double requested = position.in(Degrees);
    double clamped =
        Math.max(
            ExampleArmConstants.REVERSE_SOFT_LIMIT_DEGREES,
            Math.min(ExampleArmConstants.FORWARD_SOFT_LIMIT_DEGREES, requested));
    Logger.recordOutput(ExampleArmConstants.NAME + "/GoalWasClamped", clamped != requested);
    goalPosition = Degrees.of(clamped);
    closedLoopActive = true;
    io.setPosition(goalPosition);
  }

  /**
   * Open-loop. Bring-up and characterization only.
   *
   * <p>This bypasses gravity compensation, so zero volts lets the arm fall. It is not a safe
   * resting state — the default command holds a position instead.
   */
  public void setVoltage(Voltage volts) {
    closedLoopActive = false;
    io.setVoltage(volts);
  }

  /** Stops commanding output. See the warning on {@link #setVoltage}. */
  public void stop() {
    closedLoopActive = false;
    io.stop();
  }

  // ============================================================================================
  // State queries — read by Triggers, commands, and FaultMonitor
  // ============================================================================================

  /**
   * True when the mechanism is within tolerance of the requested position.
   *
   * <p>Compared in degrees, because that is the unit the tolerance is written in and the unit
   * anyone reading the log is thinking in. An explicit tolerance rather than an equality check,
   * because a real mechanism never sits exactly on its setpoint.
   *
   * <p>This is a pure predicate: it is logged from {@link #periodic()}, not from inside here.
   * Recording from a getter means the channel updates only as often as something happens to call
   * it, so a log gap reads as "the robot stopped checking" rather than "nothing asked".
   */
  public boolean isAtPosition() {
    // False under open-loop or stop: sitting near an old goal is not the same as holding it.
    return closedLoopActive
        && Math.abs(inputs.motorPosition.in(Degrees) - goalPosition.in(Degrees))
            < ExampleArmConstants.POSITION_TOLERANCE_DEGREES;
  }

  /** Current mechanism angle. Useful for interpolation tables and for logging call sites. */
  public Angle getPosition() {
    return inputs.motorPosition;
  }

  /** Is the motor answering on the bus? Register with {@code FaultMonitor} in RobotContainer. */
  public boolean isMotorConnected() {
    return inputs.motorConnected;
  }

  /**
   * Is the CANcoder answering?
   *
   * <p>Separate from {@link #isMotorConnected()} on purpose. An encoder can drop out while its
   * motor stays perfectly healthy, and on a fused-CANcoder mechanism that is the failure that
   * matters — the position loop starts fighting a sensor that is no longer reporting. Register this
   * with {@code FaultMonitor} as its own condition.
   */
  public boolean isEncoderConnected() {
    return inputs.encoderConnected;
  }

  /**
   * True when the CANcoder reports a red magnet while it is connected.
   *
   * <p>Register with {@code FaultMonitor} beside {@link #isEncoderConnected()}. A red magnet is the
   * worse of the two failures: the encoder keeps answering, with readings that look plausible and
   * are wrong.
   */
  public boolean isEncoderMagnetRed() {
    return inputs.encoderConnected && inputs.encoderMagnetHealth == 1;
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
    // only ever reads true while driving forward.
    return Math.abs(inputs.motorStatorAmps.in(Amps))
        > ExampleArmConstants.STATOR_CURRENT_LIMIT_AMPS * STATOR_SATURATION_FRACTION;
  }
}
