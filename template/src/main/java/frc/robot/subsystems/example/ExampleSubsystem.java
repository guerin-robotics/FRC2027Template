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
