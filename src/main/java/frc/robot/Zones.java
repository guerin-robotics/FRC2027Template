package frc.robot;

import frc.lib.util.AllianceFlipUtil;

/**
 * The 2026 field zone vocabulary, ported from Rebuilt2026 {@code HardwareConstants.Zones}.
 *
 * <p>{@link RobotState} classifies the robot's pose into these. Only {@link BroadZone} drives robot
 * behavior (the shoot/pass split in {@code Triggers}); the others are ported for completeness and
 * are read only by the sim bindings or not at all.
 */
public final class Zones {

  private Zones() {}

  public enum BroadZone {
    ALLIANCE_ZONE,
    ALLIANCE_TRENCH,
    NEUTRAL,
    OPPOSING_TRENCH,
    OPPOSING_ZONE
  }

  public enum SpecificZone {
    ALLIANCE_TOWER,
    ALLIANCE_TRENCH_NEAR,
    ALLIANCE_BUMP_NEAR,
    ALLIANCE_HUB,
    ALLIANCE_BUMP_FAR,
    ALLIANCE_TRENCH_FAR,
    OPPOSING_TRENCH_NEAR,
    OPPOSING_BUMP_NEAR,
    OPPOSING_HUB,
    OPPOSING_BUMP_FAR,
    OPPOSING_TRENCH_FAR,
    OPPOSING_TOWER,
    NEUTRAL
  }

  public enum ApproachingZoneX {
    APPROACHING_ALLIANCE_TOWER,
    APPROACHING_ALLIANCE_TRENCH,
    APPROACHING_OPPOSING_TRENCH,
    APPROACHING_OPPOSING_TOWER,
    NEUTRAL
  }

  public enum ApproachingZoneY {
    APPROACHING_ALLIANCE_TOWER,
    APPROACHING_TRENCH,
    APPROACHING_BUMP,
    APPROACHING_OPPOSING_TOWER,
    APPROACHING_WALL
  }

  public enum ApproachingZoneComposite {
    APPROACHING_ALLIANCE_TOWER,
    APPROACHING_ALLIANCE_TRENCH,
    APPROACHING_ALLIANCE_BUMP,
    APPROACHING_OPPOSING_BUMP,
    APPROACHING_OPPOSING_TRENCH,
    APPROACHING_OPPOSING_TOWER
  }

  // KNOWN 2026 BUG, carried over unchanged: these are static finals that call AllianceFlipUtil,
  // so they are evaluated ONCE, whenever this class first loads, against whatever alliance was
  // cached at that moment. Loaded while blue or unknown they are the small offsets they look
  // like; loaded while red they become the mirrored X — about 16 m — and every "approaching"
  // classification that uses them is wrong for the rest of the run. Only the sim bindings read
  // them. Kept as-is so this port behaves like the code it came from.

  public static final double ZONE_OFFSET = AllianceFlipUtil.applyX(0.5);
  public static final double APPROACHING_X_OFFSET = AllianceFlipUtil.applyX(1);
  public static final double APPROACHING_Y_OFFSET = AllianceFlipUtil.applyY(0.5);
}
