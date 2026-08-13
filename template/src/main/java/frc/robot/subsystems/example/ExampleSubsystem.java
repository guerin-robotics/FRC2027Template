package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.io.ExampleSubsystemIO;
import frc.robot.subsystems.example.io.ExampleSubsystemIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Template subsystem following the Guerin Robotics AdvantageKit IO pattern.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Rename "ExampleSubsystem" → your subsystem name 2. Update the
 * Logger.processInputs() key to match your subsystem name 3. Update the batteryLogger key to match
 * your subsystem name 4. Add public methods for every control action 5. Do NOT add hardware calls
 * here — all hardware goes in the IO implementation 6. Do NOT reference other subsystems — use
 * RobotState or Supplier callbacks
 *
 * <p>RULE: This class must compile and run identically whether the IO is real or sim. That's the
 * whole point of the IO abstraction.
 */
public class ExampleSubsystem extends SubsystemBase {

  private final ExampleSubsystemIO io;
  private final ExampleSubsystemIOInputsAutoLogged inputs;

  /** Last commanded velocity, so {@link #isAtVelocity()} has something to compare against. */
  private AngularVelocity goalVelocity = RotationsPerSecond.of(0);

  /** Last commanded position, so {@link #isAtPosition()} has something to compare against. */
  private Angle goalPosition = Rotations.of(0);

  /**
   * Draws the mechanism. Diagnostic only — delete this field, the file and the constants block if
   * this mechanism is velocity-controlled or you do not want the loop cost.
   */
  private final ExampleSubsystemVisualizer visualizer = new ExampleSubsystemVisualizer();

  public ExampleSubsystem(ExampleSubsystemIO io) {
    this.io = io;
    this.inputs = new ExampleSubsystemIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ExampleSubsystem", inputs); // TODO: rename key

    // Report current to battery logger for brownout diagnosis
    Robot.batteryLogger.reportCurrentUsage(
        "ExampleSubsystem", // TODO: rename
        false,
        inputs.motorSupplyAmps != null ? inputs.motorSupplyAmps.in(Amps) : 0.0);

    // Draw it. Diagnostic only — the visualizer no-ops when Visualization.ENABLED is false.
    // For a LINEAR mechanism, swap to visualizer.updateLinear() and convert both sides with
    // ExampleSubsystemConstants.rotationsToInches().
    visualizer.updateRotation(
        inputs.motorPosition.in(Degrees), goalPosition.in(Degrees), isAtPosition());
  }

  // --- Public control methods (called by commands) ---

  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  public void setVelocity(AngularVelocity velocity) {
    goalVelocity = velocity;
    io.setVelocity(velocity);
  }

  public void setPosition(Angle position) {
    goalPosition = position;
    io.setPosition(position);
  }

  public void stop() {
    io.stop();
  }

  // --- State queries (used by Triggers or commands) ---

  /**
   * True when the mechanism is within tolerance of the requested velocity.
   *
   * <p>Every mechanism a command waits on needs a query like this, and it needs an explicit
   * tolerance rather than an equality check — a real mechanism never sits exactly on its setpoint.
   * The tolerance lives in the constants file so it can be adjusted between matches without
   * touching logic.
   *
   * <p><b>Log it.</b> A readiness condition you cannot see flipping in a match log is one you
   * cannot debug afterward, and "the robot just never fired" is almost always this returning false
   * for a reason nobody recorded.
   */
  public boolean isAtVelocity() {
    // Compared in RPM, because that is the unit the tolerance is written in and the unit
    // anyone reading the log will be thinking in.
    boolean atVelocity =
        Math.abs(inputs.motorVelocity.in(RPM) - goalVelocity.in(RPM))
            < ExampleSubsystemConstants.VELOCITY_TOLERANCE_RPM;
    Logger.recordOutput("ExampleSubsystem/AtVelocity", atVelocity);
    Logger.recordOutput("ExampleSubsystem/VelocityRpm", inputs.motorVelocity.in(RPM));
    return atVelocity;
  }

  /**
   * True when the mechanism is within tolerance of the requested position.
   *
   * <p>The position-control counterpart to {@link #isAtVelocity()}. Delete whichever one this
   * mechanism does not use — a readiness query that is never true, because nothing ever sets its
   * goal, is worse than no query at all.
   */
  public boolean isAtPosition() {
    // Degrees for a rotating mechanism. For a linear one, convert both sides to inches with
    // ExampleSubsystemConstants.rotationsToInches() and compare against
    // POSITION_TOLERANCE_INCHES instead — a tolerance in degrees means nothing on a lift.
    boolean atPosition =
        Math.abs(inputs.motorPosition.in(Degrees) - goalPosition.in(Degrees))
            < ExampleSubsystemConstants.POSITION_TOLERANCE_DEGREES;
    Logger.recordOutput("ExampleSubsystem/AtPosition", atPosition);
    Logger.recordOutput("ExampleSubsystem/PositionDegrees", inputs.motorPosition.in(Degrees));
    return atPosition;
  }

  // ============================================================================================
  // JAM DETECTION — any mechanism that can stall against a game piece. Rollers, feeders,
  // intakes, transports. Delete for anything that physically cannot jam.
  // ============================================================================================
  //
  // 2026 ran its rollers open-loop with no feedback, so jams were SILENT: the mechanism stopped
  // working, and nothing in the log said why. This is the cheapest instrumentation on this list
  // and it is the one that was missing.
  //
  // A jam is a CONJUNCTION, never a single signal:
  //
  //   1. we are actually commanding motion          (an idle roller is not jammed)
  //   2. measured velocity is far below commanded   (it is not turning)
  //   3. stator current is high                     (it is trying hard)
  //   4. all three have held for a dwell            (not a transient)
  //
  // Current alone is the classic mistake — it spikes on every static-friction breakaway and
  // every first contact with a game piece, both normal. Same lesson as the ZEROING block below.
  //
  // STATOR, not supply. .claude/rules/02-hardware.md assigns stall indication to stator: it is
  // the winding current. At the low-speed, high-load condition that defines a jam, supply is
  // only a fraction of stator because the controller is chopping, so a supply threshold sits
  // much closer to the noise floor. (GUIDE.md section D.3 says "supply-current monitoring" —
  // it predates this block and is the looser phrasing.)
  //
  // THE DWELL MUST EXCEED SPIN-UP TIME, or every start reads as a jam: during spin-up the
  // command is high, the measurement is low and the current is high, which is exactly the jam
  // signature. Check it against ACCELERATION_RPM_PER_SEC, and raise it if that ever drops.
  //
  // DETECT HERE, RESPOND IN A COMMAND. Reversing to clear a jam is a policy decision with a
  // game-strategy answer — it can eject a piece the driver wanted. The subsystem owns the
  // signal; ExampleCommands owns what to do about it. Same split as ZEROING.
  //
  //   private final Debouncer jamDebounce =
  //       new Debouncer(ExampleSubsystemConstants.JAM_DEBOUNCE_SECONDS, kRising);
  //
  //   // Evaluated ONCE per loop in periodic(), never inside the getter. A debouncer advances
  //   // its timer every time it is polled, so calling calculate() from isJammed() would make
  //   // the dwell depend on how many callers happened to ask — a command polling it and a
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
  //     Logger.recordOutput("ExampleSubsystem/Jammed", jammed);
  //     Logger.recordOutput("ExampleSubsystem/JamCount", jamCount);
  //     // The raw conjunction too. Comparing it against Jammed in a log is how you tell
  //     // "threshold too low, it keeps flickering" from "dwell too long, it never latches".
  //     Logger.recordOutput("ExampleSubsystem/JamConditionRaw", isJamConditionPresent());
  //   }
  //
  //   private boolean isJamConditionPresent() {
  //     double commandedRpm = Math.abs(goalVelocity.in(RPM));
  //     if (commandedRpm < ExampleSubsystemConstants.JAM_MIN_COMMANDED_RPM) {
  //       return false;   // idle is not jammed
  //     }
  //     boolean notTurning =
  //         Math.abs(inputs.motorVelocity.in(RPM))
  //             < commandedRpm * ExampleSubsystemConstants.JAM_VELOCITY_FRACTION;
  //     boolean workingHard =
  //         inputs.motorStatorAmps.in(Amps) > ExampleSubsystemConstants.JAM_STATOR_CURRENT_AMPS;
  //     return notTurning && workingHard;
  //   }
  //
  //   /** Live condition — clears when the jam does. Safe to poll from anywhere. */
  //   public boolean isJammed() {
  //     return jammed;
  //   }
  //
  // ALSO: clear goalVelocity in stop() and setVoltage(), or the detector keeps comparing
  // against a setpoint nobody is commanding any more.
  //
  // AND: register it with FaultMonitor in RobotContainer, so a mechanism jamming repeatedly
  // reaches the pit between matches instead of only the log.

  // ============================================================================================
  // ZEROING — relative encoders only. Delete if this mechanism has an absolute encoder that
  // fits within one turn. See ExampleSubsystemIOReal and ExampleCommands' commented ZEROING
  // blocks, and docs/new-mechanism-bringup.md Phase 1 step 6.
  // ============================================================================================
  //
  // The subsystem holds only the state and the primitive. The routine that decides WHEN to
  // call zeroAtCurrentPosition() — drive down, watch for stall, refuse on timeout — is a
  // COMMAND, not a subsystem method, so the scheduler can interrupt it like anything else.
  //
  //   private boolean zeroed = false;
  //   private final Debouncer zeroingStallDebounce =
  //       new Debouncer(ExampleSubsystemConstants.ZEROING_STALL_DEBOUNCE_SECONDS, kRising);
  //
  //   public void zeroAtCurrentPosition() {
  //     io.zeroPosition();
  //     zeroed = true;
  //   }
  //
  //   public boolean isZeroed() {
  //     return zeroed;
  //   }
  //
  //   /**
  //    * True once BOTH current is high and velocity is near zero for a sustained dwell — not
  //    * current alone, which spikes momentarily on static friction breakaway even far from the
  //    * real hard stop. Only meaningful while the zeroing command is actively driving down.
  //    */
  //   public boolean isAtZeroingStall() {
  //     boolean highCurrent =
  //         inputs.motorSupplyAmps.in(Amps) > ExampleSubsystemConstants.ZEROING_STALL_CURRENT_AMPS;
  //     boolean nearZeroVelocity =
  //         Math.abs(inputs.motorVelocity.in(RPM))
  //             < ExampleSubsystemConstants.ZEROING_VELOCITY_THRESHOLD_RPM;
  //     return zeroingStallDebounce.calculate(highCurrent && nearZeroVelocity);
  //   }
}
