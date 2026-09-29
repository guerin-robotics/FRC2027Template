package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.prestage.Prestage;

/**
 * The 2026 prestage verbs, ported from Rebuilt2026 {@code PrestageCommands}.
 *
 * <p>These keep the 2026 semantics rather than using {@code RollerCommands}: the velocity factory
 * is {@code runOnce} (the prestage keeps running after the command ends), and "stop" commands a
 * velocity of zero rather than neutral.
 */
public final class PrestageCommands {

  private PrestageCommands() {}

  /** Runs at a voltage while held, 0 V on release. */
  public static Command setPrestageVoltage(Prestage prestage, Voltage voltage) {
    return Commands.startEnd(
            () -> prestage.setVoltage(voltage), () -> prestage.setVoltage(Volts.of(0)), prestage)
        .withName("Prestage_Voltage_" + voltage.in(Volts) + "V");
  }

  public static Command stop(Prestage prestage) {
    return Commands.runOnce(() -> prestage.setVelocity(RotationsPerSecond.of(0)), prestage)
        .withName("Prestage_Stop");
  }

  /** Sets a velocity and finishes immediately; the prestage keeps running. */
  public static Command setPrestageVelocity(Prestage prestage, AngularVelocity velocity) {
    return Commands.runOnce(() -> prestage.setVelocity(velocity), prestage)
        .withName("Prestage_Velocity");
  }

  /**
   * Holds an idle velocity. Never ends, so it can be a default command.
   *
   * <p>Unbound in 2026 — the default-command line was commented out — and unbound here.
   */
  public static Command prestageIdle(Prestage prestage, AngularVelocity idleVelocity) {
    return Commands.run(() -> prestage.setVelocity(idleVelocity), prestage)
        .withName("Prestage_Idle");
  }
}
