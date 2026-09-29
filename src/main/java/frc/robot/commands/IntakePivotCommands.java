package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.ContinuousConditionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intakePivot.IntakePivot;
import java.util.function.BooleanSupplier;

/**
 * The 2026 intake pivot verbs, ported from Rebuilt2026 {@code IntakePivotCommands}.
 *
 * <p>"Compress" is the 2026 hopper squeeze: lift the intake part way so its linkage pushes fuel
 * toward the feeders while shooting. The single compress lifts once after a wait; the double
 * compress lifts, drops, and lifts again, for a full hopper.
 *
 * <p>These keep the 2026 semantics rather than using {@code RotaryCommands}: {@link
 * #setPivotPosition} is {@code runOnce}, so the device holds the goal after the command ends.
 */
public final class IntakePivotCommands {

  private IntakePivotCommands() {}

  /** The three positions a compress moves through. From {@code Constants.Setpoints}. */
  public record CompressPositions(Angle down, Angle jostleFirst, Angle jostleUp) {}

  /** Runs at a voltage while held, 0 V on release. */
  public static Command setPivotVoltage(IntakePivot intakePivot, Voltage voltage) {
    return Commands.startEnd(
            () -> intakePivot.setVoltage(voltage),
            () -> intakePivot.setVoltage(Volts.of(0)),
            intakePivot)
        .withName("IntakePivot_Voltage_" + voltage.in(Volts) + "V");
  }

  public static Command stopPivot(IntakePivot intakePivot) {
    return Commands.runOnce(() -> intakePivot.setVoltage(Volts.of(0)), intakePivot)
        .withName("IntakePivot_Stop");
  }

  /** Sets a goal and finishes immediately; the device keeps holding it. */
  public static Command setPivotPosition(IntakePivot intakePivot, Angle position) {
    return Commands.runOnce(() -> intakePivot.setPosition(position), intakePivot)
        .withName("IntakePivot_SetPos");
  }

  /**
   * Compress the hopper, single or double depending on {@code doubleCompress} — re-evaluated every
   * loop, so the operator can switch modes mid-compress.
   */
  public static Command compressPivot(
      IntakePivot intakePivot, CompressPositions positions, BooleanSupplier doubleCompress) {
    // Branch A: single compress
    Command oneCompressBranch =
        Commands.sequence(
                Commands.waitSeconds(Constants.Waits.WAIT_TO_COMPRESS_SECONDS),
                setPivotPosition(intakePivot, positions.jostleUp()))
            .withName("IntakePivot_Compress_HalfHopper");

    // Branch B: double compress (more conservative, can be manually defaulted to)
    Command twoCompressBranch =
        Commands.sequence(
                setPivotPosition(intakePivot, positions.jostleFirst()),
                Commands.waitSeconds(Constants.Waits.WAIT_TO_DROP_SECONDS),
                setPivotPosition(intakePivot, positions.down()),
                Commands.waitSeconds(Constants.Waits.WAIT_BETWEEN_COMPRESS_SECONDS),
                setPivotPosition(intakePivot, positions.jostleUp()))
            .withName("IntakePivot_Compress_FullHopper");

    return new ContinuousConditionalCommand(twoCompressBranch, oneCompressBranch, doubleCompress)
        .withName("IntakePivot_Compress");
  }

  /** Single compress, as the shoot button runs it. */
  public static Command compressPivot(IntakePivot intakePivot, CompressPositions positions) {
    return compressPivot(intakePivot, positions, () -> false);
  }

  /** Double compress. */
  public static Command manualPivotCompress(IntakePivot intakePivot, CompressPositions positions) {
    return compressPivot(intakePivot, positions, () -> true);
  }

  /** Auto: a single compress after the auto-specific wait. */
  public static Command autoPivotCompress(IntakePivot intakePivot, Angle jostleUp) {
    return Commands.sequence(
            Commands.waitSeconds(Constants.Waits.AUTO_WAIT_TO_COMPRESS_SECONDS),
            setPivotPosition(intakePivot, jostleUp))
        .withName("IntakePivot_AutoCompress");
  }
}
