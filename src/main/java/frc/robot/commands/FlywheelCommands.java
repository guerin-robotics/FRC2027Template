package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheel.Flywheel;

/**
 * The 2026 flywheel verbs, ported from Rebuilt2026 {@code FlywheelCommands}.
 *
 * <p>These keep the 2026 semantics rather than using {@code RollerCommands}. Unlike the feeders,
 * the flywheel velocity factories are {@code run}, not {@code runOnce} — they hold the subsystem
 * until interrupted — and "stop" commands a velocity of zero, an active ramped stop.
 */
public final class FlywheelCommands {

  private FlywheelCommands() {}

  /** Runs at a voltage while held, 0 V on release. */
  public static Command setFlywheelVoltage(Flywheel flywheel, Voltage voltage) {
    return Commands.startEnd(
            () -> flywheel.setVoltage(voltage), () -> flywheel.setVoltage(Volts.of(0)), flywheel)
        .withName("Flywheel_Voltage_" + voltage.in(Volts) + "V");
  }

  /** Holds a velocity until interrupted. Does not stop on end. */
  public static Command setFlywheelVelocity(Flywheel flywheel, AngularVelocity velocity) {
    return Commands.run(() -> flywheel.setVelocity(velocity), flywheel)
        .withName("Flywheel_Velocity_" + velocity.in(RotationsPerSecond) + "rps");
  }

  /** The 2026 default command: holds the idle speed. */
  public static Command flywheelIdle(Flywheel flywheel, AngularVelocity idleVelocity) {
    return Commands.run(() -> flywheel.setVelocity(idleVelocity), flywheel)
        .withName("Flywheel_Idle");
  }

  /** Unbound in 2026. */
  public static Command flywheelHighIdle(Flywheel flywheel, AngularVelocity highIdleVelocity) {
    return Commands.run(() -> flywheel.setVelocity(highIdleVelocity), flywheel)
        .withName("Flywheel_HighIdle");
  }

  /**
   * Unbound in 2026. The pass target is read once, when this command is <i>built</i>, not when it
   * runs — as it was in 2026.
   */
  public static Command setPassVelocity(Flywheel flywheel) {
    return setVelocityForTarget(flywheel, RobotState.getInstance().getPassTarget())
        .withName("Flywheel_PassVelocity");
  }

  /** Tracks the hub-distance speed every loop until interrupted. */
  public static Command setVelocityForHub(Flywheel flywheel) {
    return Commands.run(flywheel::setSpeedForHub, flywheel).withName("Flywheel_VelocityForHub");
  }

  public static Command setVelocityForTarget(Flywheel flywheel, Translation3d target) {
    return Commands.startEnd(
            () -> flywheel.setSpeedForTarget(target),
            () -> flywheel.setVoltage(Volts.of(0)),
            flywheel)
        .withName("Flywheel_VelocityForTarget");
  }

  public static Command setVelocityForDistance(Flywheel flywheel, Distance distance) {
    return Commands.startEnd(
            () -> flywheel.setSpeedForDistance(distance),
            () -> flywheel.setVoltage(Volts.of(0)),
            flywheel)
        .withName("Flywheel_VelocityForDistance");
  }

  /** Tracks the pass-distance speed every loop until interrupted. */
  public static Command setVelocityForPassing(Flywheel flywheel) {
    return Commands.run(flywheel::setSpeedForPassing, flywheel)
        .withName("Flywheel_VelocityForPassing");
  }

  public static Command stop(Flywheel flywheel) {
    return Commands.runOnce(() -> flywheel.setVelocity(RotationsPerSecond.of(0)), flywheel)
        .withName("Flywheel_Stop");
  }

  /**
   * Shoot-on-the-move (sim binding only). See {@link Flywheel#shootDynamic} for why its output is
   * wrong.
   *
   * <p>Declares <b>no requirement on the flywheel</b>, exactly as 2026 did, so the flywheel's
   * default command keeps running and both write a setpoint every loop. Left as found.
   */
  public static Command shootOnTheMove(Flywheel flywheel, double hoodDegrees) {
    return Commands.run(() -> flywheel.shootDynamic(hoodDegrees))
        .withName("Flywheel_ShootOnTheMove");
  }
}
