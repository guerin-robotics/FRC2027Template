package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;

/**
 * The 2026 hood verbs, ported from Rebuilt2026 {@code HoodCommands}.
 *
 * <p>These keep the 2026 semantics rather than using {@code RotaryCommands}: the aiming commands
 * stow the hood when they end, and the default command drives to the down position rather than
 * holding wherever the hood happens to be.
 */
public final class HoodCommands {

  private HoodCommands() {}

  /** Goes to an angle while held, stows on release. */
  public static Command setHoodPos(Hood hood, Angle position) {
    return Commands.startEnd(() -> hood.setPosition(position), hood::stowHood, hood)
        .withName("Hood_SetPos_" + position.in(Degrees) + "deg");
  }

  /** Tracks the hub-distance angle every loop while held, stows on release. */
  public static Command setHoodPosForHub(Hood hood) {
    return Commands.runEnd(hood::setHoodPosForHub, hood::stowHood, hood).withName("Hood_PosForHub");
  }

  /** Tracks the pass-distance angle every loop. Does not stow on end. */
  public static Command setPosForPassing(Hood hood) {
    return Commands.run(hood::setHoodPosForPass, hood).withName("Hood_PosForPassing");
  }

  public static Command stowHood(Hood hood) {
    return Commands.runOnce(hood::stowHood, hood).withName("Hood_Stow");
  }

  /** The 2026 default command: holds the down position. */
  public static Command hoodIdle(Hood hood, Angle downPosition) {
    return Commands.run(() -> hood.setPosition(downPosition), hood).withName("Hood_Idle");
  }

  /** Unbound in 2026. */
  public static Command incrementHoodPos(Hood hood) {
    return Commands.runOnce(hood::incrementHoodPos, hood).withName("Hood_Increment");
  }
}
