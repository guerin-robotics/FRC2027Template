package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.transport.Transport;
import java.util.function.BooleanSupplier;

/**
 * The 2026 transport verbs, ported from Rebuilt2026 {@code TransportCommands}.
 *
 * <p>These deliberately keep the 2026 semantics rather than using {@code RollerCommands}: the
 * velocity factories are {@code runOnce} (the belt keeps running after the command ends), and
 * "stop" commands 0 V rather than neutral. The shooting bindings rely on both.
 */
public final class TransportCommands {

  private TransportCommands() {}

  /** Runs at a voltage while held, 0 V on release. */
  public static Command setTransportVoltage(Transport transport, Voltage voltage) {
    return Commands.startEnd(
            () -> transport.setVoltage(voltage), () -> transport.setVoltage(Volts.of(0)), transport)
        .withName("Transport_Voltage_" + voltage.in(Volts) + "V");
  }

  public static Command stop(Transport transport) {
    return Commands.runOnce(() -> transport.setVoltage(Volts.of(0)), transport)
        .withName("Transport_Stop");
  }

  /** Sets a velocity and finishes immediately; the belt keeps running. */
  public static Command setTransportVelocity(Transport transport, AngularVelocity velocity) {
    return Commands.runOnce(() -> transport.setVelocity(velocity), transport)
        .withName("Transport_Velocity");
  }

  public static Command setVoltageAfterWait(Transport transport, Voltage voltage) {
    return Commands.sequence(
            Commands.waitSeconds(Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            setTransportVoltage(transport, voltage))
        .withName("Transport_VoltageAfterWait");
  }

  /**
   * Waits for spin-up, then for alignment within the remaining budget, then sets a velocity.
   *
   * <p>Total wait is capped at {@link Constants.Waits#ALIGNMENT_TIMEOUT_SECONDS} from the start.
   */
  public static Command setVelocityAfterWait(
      Transport transport, AngularVelocity velocity, BooleanSupplier isAligned) {
    return Commands.sequence(
            Commands.waitSeconds(Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            Commands.waitUntil(isAligned)
                .withTimeout(
                    Constants.Waits.ALIGNMENT_TIMEOUT_SECONDS
                        - Constants.Waits.FLYWHEEL_SPINUP_SECONDS),
            setTransportVelocity(transport, velocity))
        .withName("Transport_VelocityAfterWait");
  }

  public static Command setVelocityAfterWait(Transport transport, AngularVelocity velocity) {
    return setVelocityAfterWait(transport, velocity, () -> true)
        .withName("Transport_VelocityAfterWaitNoAlign");
  }
}
