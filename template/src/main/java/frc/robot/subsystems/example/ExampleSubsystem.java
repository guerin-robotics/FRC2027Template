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
    boolean atVelocity =
        Math.abs(inputs.motorVelocity.in(RotationsPerSecond) - goalVelocity.in(RotationsPerSecond))
            < ExampleSubsystemConstants.VELOCITY_TOLERANCE_ROTATIONS_PER_SEC;
    Logger.recordOutput("ExampleSubsystem/AtVelocity", atVelocity);
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
    boolean atPosition =
        Math.abs(inputs.motorPosition.in(Rotations) - goalPosition.in(Rotations))
            < ExampleSubsystemConstants.POSITION_TOLERANCE_ROTATIONS;
    Logger.recordOutput("ExampleSubsystem/AtPosition", atPosition);
    return atPosition;
  }
}
