package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import java.util.function.BooleanSupplier;

/**
 * The 2026 intake roller verbs, ported from Rebuilt2026 {@code intakeRollerCommands}.
 *
 * <p>These keep the 2026 semantics rather than using {@code RollerCommands}. Note the asymmetry
 * that shipped: the voltage command's own end sets 0 V, but {@link #stopIntakeRoller} commands a
 * velocity of zero — an active, ramped stop — and that is what the bindings schedule on release.
 */
public final class IntakeRollerCommands {

  private IntakeRollerCommands() {}

  /** Runs at a voltage while held, 0 V on release. */
  public static Command setRollerVoltage(IntakeRoller intakeRoller, Voltage voltage) {
    return Commands.startEnd(
            () -> intakeRoller.setVoltage(voltage),
            () -> intakeRoller.setVoltage(Volts.of(0)),
            intakeRoller)
        .withName("IntakeRoller_Voltage_" + voltage.in(Volts) + "V");
  }

  /** Sets a velocity and finishes immediately; the roller keeps running. */
  public static Command setRollerVelocity(IntakeRoller intakeRoller, AngularVelocity velocity) {
    return Commands.runOnce(() -> intakeRoller.setVelocity(velocity), intakeRoller)
        .withName("IntakeRoller_Velocity");
  }

  /** Holds the agitate voltage. Never ends; this is the 2026 default command. */
  public static Command intakeRollerIdle(IntakeRoller intakeRoller, Voltage agitateVoltage) {
    return Commands.run(() -> intakeRoller.setVoltage(agitateVoltage), intakeRoller)
        .withName("IntakeRoller_Idle");
  }

  public static Command stopIntakeRoller(IntakeRoller intakeRoller) {
    return Commands.runOnce(() -> intakeRoller.setVelocity(RotationsPerSecond.of(0)), intakeRoller)
        .withName("IntakeRoller_Stop");
  }

  /**
   * Waits for spin-up, then for alignment within the remaining budget, then runs at a voltage while
   * held.
   */
  public static Command setVoltageAfterWait(
      IntakeRoller intakeRoller, Voltage voltage, BooleanSupplier isAligned) {
    return Commands.sequence(
            Commands.waitSeconds(Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    Constants.Waits.ALIGNMENT_TIMEOUT_SECONDS
                        - Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            setRollerVoltage(intakeRoller, voltage))
        .withName("IntakeRoller_VoltageAfterWait");
  }
}
