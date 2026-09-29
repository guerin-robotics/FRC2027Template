package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import java.util.function.BooleanSupplier;

/**
 * The 2026 feeder verbs, ported from Rebuilt2026 {@code FeederCommands}.
 *
 * <p>These keep the 2026 semantics rather than using {@code RollerCommands}: the velocity factories
 * are {@code runOnce} (the feeder keeps running after the command ends), and "stop" commands a
 * velocity of zero — an active, ramped stop — rather than neutral. The shooting bindings rely on
 * both.
 */
public final class FeederCommands {

  private FeederCommands() {}

  // ============================================================================================
  // Upper feeder
  // ============================================================================================

  /** Runs at a voltage while held, 0 V on release. */
  public static Command setUpperFeederVoltage(UpperFeeder feeder, Voltage voltage) {
    return Commands.startEnd(
            () -> feeder.setVoltage(voltage), () -> feeder.setVoltage(Volts.of(0)), feeder)
        .withName("UpperFeeder_Voltage_" + voltage.in(Volts) + "V");
  }

  public static Command stopUpper(UpperFeeder feeder) {
    return Commands.runOnce(() -> feeder.setVelocity(RotationsPerSecond.of(0)), feeder)
        .withName("UpperFeeder_Stop");
  }

  /** Sets a velocity and finishes immediately; the feeder keeps running. */
  public static Command setUpperFeederVelocity(UpperFeeder feeder, AngularVelocity velocity) {
    return Commands.runOnce(() -> feeder.setVelocity(velocity), feeder)
        .withName("UpperFeeder_Velocity");
  }

  /**
   * Waits for spin-up, then for alignment within the remaining budget, then sets a velocity.
   *
   * <p>Total wait is capped at {@link Constants.Waits#ALIGNMENT_TIMEOUT_SECONDS} from the start.
   */
  public static Command setUpperVelocityAfterWait(
      UpperFeeder feeder, AngularVelocity velocity, BooleanSupplier isAligned) {
    return Commands.sequence(
            Commands.waitSeconds(Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    Constants.Waits.ALIGNMENT_TIMEOUT_SECONDS
                        - Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            setUpperFeederVelocity(feeder, velocity))
        .withName("UpperFeeder_VelocityAfterWait");
  }

  public static Command setUpperVelocityAfterWait(UpperFeeder feeder, AngularVelocity velocity) {
    return setUpperVelocityAfterWait(feeder, velocity, () -> true)
        .withName("UpperFeeder_VelocityAfterWaitNoAlign");
  }

  // ============================================================================================
  // Lower feeder
  // ============================================================================================

  /** Runs at a voltage while held, 0 V on release. */
  public static Command setLowerFeederVoltage(LowerFeeder feeder, Voltage voltage) {
    return Commands.startEnd(
            () -> feeder.setVoltage(voltage), () -> feeder.setVoltage(Volts.of(0)), feeder)
        .withName("LowerFeeder_Voltage_" + voltage.in(Volts) + "V");
  }

  public static Command stopLower(LowerFeeder feeder) {
    return Commands.runOnce(() -> feeder.setVelocity(RotationsPerSecond.of(0)), feeder)
        .withName("LowerFeeder_Stop");
  }

  /** Sets a velocity and finishes immediately; the feeder keeps running. */
  public static Command setLowerFeederVelocity(LowerFeeder feeder, AngularVelocity velocity) {
    return Commands.runOnce(() -> feeder.setVelocity(velocity), feeder)
        .withName("LowerFeeder_Velocity");
  }

  /**
   * Waits for spin-up, then for alignment within the remaining budget, then sets a velocity.
   *
   * <p>Total wait is capped at {@link Constants.Waits#ALIGNMENT_TIMEOUT_SECONDS} from the start.
   */
  public static Command setLowerVelocityAfterWait(
      LowerFeeder feeder, AngularVelocity velocity, BooleanSupplier isAligned) {
    return Commands.sequence(
            Commands.waitSeconds(Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    Constants.Waits.ALIGNMENT_TIMEOUT_SECONDS
                        - Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            setLowerFeederVelocity(feeder, velocity))
        .withName("LowerFeeder_VelocityAfterWait");
  }

  public static Command setLowerVelocityAfterWait(LowerFeeder feeder, AngularVelocity velocity) {
    return setLowerVelocityAfterWait(feeder, velocity, () -> true)
        .withName("LowerFeeder_VelocityAfterWaitNoAlign");
  }
}
