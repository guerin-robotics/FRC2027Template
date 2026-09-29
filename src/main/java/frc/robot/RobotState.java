package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.FieldConstants;
import frc.robot.Zones.ApproachingZoneComposite;
import frc.robot.Zones.ApproachingZoneX;
import frc.robot.Zones.ApproachingZoneY;
import frc.robot.Zones.BroadZone;
import frc.robot.Zones.SpecificZone;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Centralized robot state container that tracks the robot's position and velocity on the field.
 *
 * <p>This is a singleton that provides a single source of truth for:
 *
 * <ul>
 *   <li>Robot pose (position and rotation on the field)
 *   <li>Robot velocity (field-relative and robot-relative)
 *   <li>Distance and bearing to arbitrary field points
 * </ul>
 *
 * <p><b>Why use a centralized RobotState?</b>
 *
 * <ul>
 *   <li>Decouples subsystems — a shooter doesn't need a reference to Drive
 *   <li>Single source of truth — all subsystems see the same robot state
 *   <li>Easier testing — robot state can be mocked for unit tests
 *   <li>Cleaner architecture — subsystems don't need pose/speed suppliers threaded through
 * </ul>
 *
 * <p><b>Usage:</b>
 *
 * <pre>
 * RobotState state = RobotState.getInstance();
 * Pose2d pose = state.getEstimatedPose();
 * Distance d = state.getDistanceToPoint(someTarget);
 * </pre>
 *
 * <p><b>Adding game-specific state:</b> This class is intentionally game-agnostic. Add scoring
 * targets, alignment checks and zone classification here as the 2027 game is understood — that is
 * what this class is for, and keeping it here is what prevents subsystems from referencing each
 * other. Two rules from the 2026 season worth keeping:
 *
 * <ol>
 *   <li>Anything annotated {@code @AutoLogOutput} is called by AdvantageKit every loop whether or
 *       not your code uses it. Do not annotate expensive methods, and delete annotations on methods
 *       that lose their callers — that was a measurable chunk of loop time in 2026.
 *   <li>Never call {@code DriverStation.getAlliance()} here. Use {@link
 *       frc.lib.util.AllianceFlipUtil#shouldFlip()}, which is cached once per loop.
 * </ol>
 *
 * <p><b>Odometry updates:</b> Drive calls {@link #updateModuleStates} during its periodic loop.
 * RobotState does NOT own a pose estimator — see {@link #setPoseSupplier}.
 */
public class RobotState {

  // ==================== SINGLETON PATTERN ====================

  private static RobotState instance;

  /**
   * Returns the singleton instance of RobotState, creating it on first call.
   *
   * @return The RobotState singleton
   */
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  // ==================== POSE ESTIMATION ====================

  /**
   * Supplier that provides the current estimated pose from Drive's pose estimator.
   *
   * <p>RobotState does NOT maintain its own pose estimator. It delegates to Drive's single {@code
   * SwerveDrivePoseEstimator} via this supplier. This eliminates the dual-estimator divergence bug
   * where two independent estimators drift apart and cause pose jumps when vision is lost. Do not
   * add a second estimator here.
   */
  private Supplier<Pose2d> poseSupplier = Pose2d::new;

  /** Kinematics for converting between chassis speeds and module states. */
  private final SwerveDriveKinematics kinematics;

  /** Current module states for velocity calculation. */
  private SwerveModuleState[] currentModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  // ==================== CONSTRUCTOR ====================

  /** Private constructor — use {@link #getInstance()}. */
  private RobotState() {
    // Kinematics is needed for velocity calculations (toChassisSpeeds).
    // Drive.getModuleTranslations() reads from TunerConstants — the single source of truth.
    kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
  }

  /**
   * Sets the pose supplier that RobotState uses to get the current estimated pose.
   *
   * <p>Called once during initialization from Drive's constructor to wire RobotState to Drive's
   * single pose estimator.
   *
   * @param poseSupplier A supplier that returns the current estimated pose (e.g. {@code
   *     drive::getPose})
   */
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  // ==================== POSE GETTERS ====================

  /**
   * Returns the current estimated robot pose on the field.
   *
   * <p>The origin (0, 0) is at the blue alliance corner, with positive X toward the red alliance
   * and positive Y to the left when looking from the blue alliance.
   *
   * @return The robot's current estimated pose
   */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseSupplier.get();
  }

  /**
   * Returns the robot's current rotation (heading) on the field.
   *
   * @return The robot's current rotation
   */
  public Rotation2d getRotation() {
    return getEstimatedPose().getRotation();
  }

  // ==================== VELOCITY GETTERS ====================

  /**
   * Returns the robot's velocity in field-relative coordinates.
   *
   * <ul>
   *   <li>vx = velocity toward the red alliance (positive X direction)
   *   <li>vy = velocity toward the left side of the field (positive Y direction)
   *   <li>omega = rotational velocity (counterclockwise positive)
   * </ul>
   *
   * <p>Used by Vision to reject observations while the robot is spinning, and by any
   * shoot-on-the-move style compensation.
   *
   * @return Field-relative chassis speeds
   */
  @AutoLogOutput(key = "RobotState/FieldRelativeVelocity")
  public ChassisSpeeds getFieldRelativeVelocity() {
    ChassisSpeeds robotRelative = getRobotRelativeVelocity();
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getRotation());
  }

  /**
   * Returns the robot's velocity in robot-relative coordinates.
   *
   * <ul>
   *   <li>vx = forward velocity (positive = driving forward)
   *   <li>vy = left velocity (positive = strafing left)
   *   <li>omega = rotational velocity (counterclockwise positive)
   * </ul>
   *
   * @return Robot-relative chassis speeds
   */
  @AutoLogOutput(key = "RobotState/RobotRelativeVelocity")
  public ChassisSpeeds getRobotRelativeVelocity() {
    return kinematics.toChassisSpeeds(currentModuleStates);
  }

  // ==================== FIELD GEOMETRY HELPERS ====================

  /**
   * Returns the 2D distance from the robot to an arbitrary point on the field.
   *
   * @param point The target point (2D field coordinates, blue-origin)
   * @return Distance to the point
   */
  public Distance getDistanceToPoint(Translation2d point) {
    Translation2d robotPosition = getEstimatedPose().getTranslation();
    return Meters.of(robotPosition.getDistance(point));
  }

  /**
   * Returns the heading the robot must face for its FRONT to point at the given field point.
   *
   * <p>Feed the result straight into {@code DriveCommands.joystickDriveAtAngle}, whose
   * ProfiledPIDController has continuous input enabled, so it automatically takes the shortest
   * rotation path (at 170° with a target of -170° it turns 20°, not 340°).
   *
   * <p><b>Mechanism offsets:</b> if the mechanism doing the aiming faces the BACK of the robot (the
   * 2026 shooter did), add half a turn at the call site:
   *
   * <pre>
   * RobotState.getInstance().getAngleToTarget(target).plus(Rotation2d.kPi)
   * </pre>
   *
   * Keeping the offset at the call site rather than baked in here means this method stays correct
   * for front-facing mechanisms too.
   *
   * @param target The point to aim at (2D field coordinates, blue-origin)
   * @return The heading the robot should face
   */
  public Rotation2d getAngleToTarget(Translation2d target) {
    Translation2d robotToTarget = target.minus(getEstimatedPose().getTranslation());
    return new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
  }

  // ==================== MODULE STATE UPDATES ====================

  /**
   * Updates the current module states used for velocity calculation.
   *
   * <p>Called once per cycle from {@code Drive.periodic()} after the high-frequency odometry loop.
   * Only the module states (velocity + angle) are needed — pose estimation is handled entirely by
   * Drive's single {@code SwerveDrivePoseEstimator}.
   *
   * @param moduleStates The current module states (velocity and angle for each module)
   */
  public void updateModuleStates(SwerveModuleState[] moduleStates) {
    currentModuleStates = moduleStates;
  }

  // ============================================================================================
  // 2026 REBUILT GAME STATE — ported from Rebuilt2026 RobotState
  // ============================================================================================
  //
  // The 2026 shooter faces the BACK of the robot. 2026 baked that half turn into
  // getAngleToAllianceHub() and getAngleToTarget(); the template's getAngleToTarget() returns the
  // true bearing, so the offset lives in getShooterAngleToTarget() below and every 2026 call site
  // that relied on it now calls that instead. getAngleToAllianceHub() keeps its 2026 meaning.

  /** Half a turn: the shooter points out the back of the robot. */
  private static final Rotation2d SHOOTER_HEADING_OFFSET = Rotation2d.kPi;

  /** Robot-relative velocity rotated into the hub-facing frame, for shoot-on-the-move. */
  @AutoLogOutput(key = "RobotState/HubRelativeVelocity")
  public ChassisSpeeds getHubRelativeVelocity() {
    ChassisSpeeds robotRelative = getRobotRelativeVelocity();
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getAngleToAllianceHub());
  }

  @AutoLogOutput(key = "RobotState/DistanceToAllianceHub_m")
  public Distance getDistanceToAllianceHub() {
    return getDistanceToPoint(getAllianceHubTarget().toTranslation2d());
  }

  /** The top center of our hub, flipped for red. */
  public Translation3d getAllianceHubTarget() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
  }

  /** The heading that points the (rear-facing) SHOOTER at our hub. */
  @AutoLogOutput(key = "RobotState/AngleToAllianceHub")
  public Rotation2d getAngleToAllianceHub() {
    return getShooterAngleToTarget(getAllianceHubTarget().toTranslation2d());
  }

  /**
   * The heading that points the rear-facing SHOOTER at a field point. This is what Rebuilt2026
   * called {@code getAngleToTarget}.
   */
  public Rotation2d getShooterAngleToTarget(Translation2d target) {
    return getAngleToTarget(target).plus(SHOOTER_HEADING_OFFSET);
  }

  // ---- Shooting alignment ----

  @AutoLogOutput(key = "RobotState/IsAlignedToHub")
  public boolean isAlignedToHub() {
    // Rotation2d.minus() handles wrap-around (179 - (-179) = 2 degrees, not 358).
    double errorDegrees =
        Math.abs(getAngleToAllianceHub().minus(getEstimatedPose().getRotation()).getDegrees());
    return errorDegrees < Constants.Thresholds.HUB_ALIGNMENT_TOLERANCE_DEGREES;
  }

  /** Looser tolerance, for staying "aligned" once shooting has started. */
  public boolean isAlignedToHubLoose() {
    double errorDegrees =
        Math.abs(getAngleToAllianceHub().minus(getEstimatedPose().getRotation()).getDegrees());
    return errorDegrees < Constants.Thresholds.HUB_LOOSE_ALIGNMENT_TOLERANCE_DEGREES;
  }

  @AutoLogOutput(key = "RobotState/IsAlignedToPass")
  public boolean isAlignedToPass() {
    double errorDegrees =
        Math.abs(
            getShooterAngleToTarget(getPassTarget().toTranslation2d())
                .minus(getEstimatedPose().getRotation())
                .getDegrees());
    return errorDegrees < Constants.Thresholds.PASS_ALIGNMENT_TOLERANCE_DEGREES;
  }

  public boolean isAlignedToPassLoose() {
    double errorDegrees =
        Math.abs(
            getShooterAngleToTarget(getPassTarget().toTranslation2d())
                .minus(getEstimatedPose().getRotation())
                .getDegrees());
    return errorDegrees < Constants.Thresholds.PASS_LOOSE_ALIGNMENT_TOLERANCE_DEGREES;
  }

  /** Where to pass to: our side of the field, on whichever half (left/right) the robot is on. */
  public Translation3d getPassTarget() {
    double poseY = getEstimatedPose().getY();
    if (AllianceFlipUtil.shouldFlip()) {
      // Red alliance
      if (poseY > (FieldConstants.fieldWidth / 2)) {
        return new Translation3d(12.0, 6.1, 0);
      } else {
        return new Translation3d(12.0, 2.3, 0);
      }
    } else {
      // Blue alliance (or unknown — defaults to blue)
      if (poseY < (FieldConstants.fieldWidth / 2)) {
        return new Translation3d(4.5, 2.3, 0);
      } else {
        return new Translation3d(4.5, 6.1, 0);
      }
    }
  }

  // ---- Zone classification ----

  /** Alliance zone, alliance trench, neutral, opposing trench or opposing zone. */
  public BroadZone getBroadZone() {
    double poseX = AllianceFlipUtil.applyX(getEstimatedPose().getX());
    if (poseX < FieldConstants.LinesVertical.allianceZone) {
      return BroadZone.ALLIANCE_ZONE;
    } else if (poseX < FieldConstants.LinesVertical.neutralZoneNear) {
      return BroadZone.ALLIANCE_TRENCH;
    } else if (poseX < FieldConstants.LinesVertical.neutralZoneFar) {
      return BroadZone.NEUTRAL;
    } else if (poseX < FieldConstants.LinesVertical.oppAllianceZone) {
      return BroadZone.OPPOSING_TRENCH;
    } else {
      return BroadZone.OPPOSING_ZONE;
    }
  }

  /**
   * Alliance/opposing tower, trench or bump, near or far. NEUTRAL when in none of them, including
   * the middle of the alliance zone. Unused by 2026 robot logic.
   */
  public SpecificZone getSpecificZone(Pose2d pose) {
    double poseX = AllianceFlipUtil.applyX(pose.getX());
    double poseY = AllianceFlipUtil.applyY(pose.getY());
    BroadZone broadZone = getBroadZone();
    if ((broadZone == BroadZone.ALLIANCE_ZONE)
        && (poseX < FieldConstants.Tower.frontFaceX)
        && (poseY < FieldConstants.Tower.leftUpright.getY())
        && poseY > FieldConstants.Tower.rightUpright.getY()) {
      return SpecificZone.ALLIANCE_TOWER;
    } else if (broadZone == BroadZone.ALLIANCE_TRENCH) {
      if (poseY < FieldConstants.RightTrench.openingTopLeft.getY()) {
        return SpecificZone.ALLIANCE_TRENCH_NEAR;
      } else if (poseY < FieldConstants.Hub.farRightCorner.getY()) {
        return SpecificZone.ALLIANCE_BUMP_NEAR;
      } else if (poseY < FieldConstants.Hub.farLeftCorner.getY()) {
        return SpecificZone.ALLIANCE_HUB;
      } else if (poseY < FieldConstants.LeftTrench.openingTopRight.getY()) {
        return SpecificZone.ALLIANCE_BUMP_FAR;
      } else {
        return SpecificZone.ALLIANCE_TRENCH_FAR;
      }
    } else if (broadZone == BroadZone.OPPOSING_TRENCH) {
      if (poseY < FieldConstants.RightTrench.openingTopLeft.getY()) {
        return SpecificZone.OPPOSING_TRENCH_NEAR;
      } else if (poseY < FieldConstants.RightBump.nearLeftCorner.getY()) {
        return SpecificZone.OPPOSING_BUMP_NEAR;
      } else if (poseY < FieldConstants.LeftBump.farRightCorner.getY()) {
        return SpecificZone.OPPOSING_HUB;
      } else if (poseY < FieldConstants.LeftTrench.openingTopRight.getY()) {
        return SpecificZone.OPPOSING_BUMP_FAR;
      } else {
        return SpecificZone.OPPOSING_TRENCH_FAR;
      }
    } else if ((broadZone == BroadZone.OPPOSING_ZONE)
        && (poseX < FieldConstants.Tower.oppLeftUpright.getX())
        && (poseY < FieldConstants.Tower.oppLeftUpright.getY())
        && (poseY > FieldConstants.Tower.oppRightUpright.getY())) {
      return SpecificZone.OPPOSING_TOWER;
    } else {
      return SpecificZone.NEUTRAL;
    }
  }

  public ApproachingZoneX getApproachingZoneX(Pose2d pose) {
    double poseX = AllianceFlipUtil.applyX(pose.getX());
    if (poseX < (FieldConstants.Tower.leftUpright.getX() + Zones.APPROACHING_X_OFFSET)) {
      return ApproachingZoneX.APPROACHING_ALLIANCE_TOWER;
    } else if ((poseX > FieldConstants.LinesVertical.allianceZone - Zones.APPROACHING_X_OFFSET)
        && (poseX < FieldConstants.LinesVertical.neutralZoneNear + Zones.APPROACHING_X_OFFSET)) {
      return ApproachingZoneX.APPROACHING_ALLIANCE_TRENCH;
    } else if ((poseX > FieldConstants.LinesVertical.neutralZoneFar - Zones.APPROACHING_X_OFFSET)
        && (poseX < FieldConstants.LinesVertical.oppAllianceZone + Zones.APPROACHING_X_OFFSET)) {
      return ApproachingZoneX.APPROACHING_OPPOSING_TRENCH;
    } else if (poseX > FieldConstants.Tower.oppLeftUpright.getX() - Zones.APPROACHING_X_OFFSET) {
      return ApproachingZoneX.APPROACHING_OPPOSING_TOWER;
    } else {
      return ApproachingZoneX.NEUTRAL;
    }
  }

  /** Unused by 2026 robot logic. Returns null when no case matches, as 2026 did. */
  public ApproachingZoneY getApproachingZoneY(Pose2d pose) {
    double poseY = AllianceFlipUtil.applyY(pose.getY());
    BroadZone broadZone = getBroadZone();
    if ((broadZone == BroadZone.ALLIANCE_ZONE)
        && ((poseY > (FieldConstants.Tower.leftUpright.getY() - Zones.APPROACHING_Y_OFFSET))
            || (poseY < (FieldConstants.Tower.rightUpright.getY() + Zones.APPROACHING_Y_OFFSET)))) {
      return ApproachingZoneY.APPROACHING_ALLIANCE_TOWER;
    } else if ((broadZone == BroadZone.OPPOSING_ZONE)
        && ((poseY > (FieldConstants.Tower.oppLeftUpright.getY() - Zones.APPROACHING_Y_OFFSET))
            || (poseY
                < (FieldConstants.Tower.oppRightUpright.getY() + Zones.APPROACHING_Y_OFFSET)))) {
      return ApproachingZoneY.APPROACHING_OPPOSING_TOWER;
    } else if ((poseY < FieldConstants.RightBump.farLeftCorner.getY())
        || (poseY > FieldConstants.LeftBump.farRightCorner.getY())) {
      return ApproachingZoneY.APPROACHING_BUMP;
    } else if ((poseY > FieldConstants.RightBump.farLeftCorner.getY())
        || (poseY < FieldConstants.LeftBump.farRightCorner.getY())) {
      return ApproachingZoneY.APPROACHING_TRENCH;
    } else {
      return null;
    }
  }

  /** Unused by 2026 robot logic. Returns null when no case matches, as 2026 did. */
  public ApproachingZoneComposite getApproachingZone(Pose2d pose) {
    ApproachingZoneX zoneX = getApproachingZoneX(pose);
    ApproachingZoneY zoneY = getApproachingZoneY(pose);
    if ((zoneX == ApproachingZoneX.APPROACHING_ALLIANCE_TRENCH)
        && (zoneY == ApproachingZoneY.APPROACHING_TRENCH)) {
      return ApproachingZoneComposite.APPROACHING_ALLIANCE_TRENCH;
    } else if ((zoneX == ApproachingZoneX.APPROACHING_ALLIANCE_TRENCH)
        && zoneY == ApproachingZoneY.APPROACHING_BUMP) {
      return ApproachingZoneComposite.APPROACHING_ALLIANCE_BUMP;
    } else if ((zoneX == ApproachingZoneX.APPROACHING_OPPOSING_TRENCH)
        && zoneY == ApproachingZoneY.APPROACHING_TRENCH) {
      return ApproachingZoneComposite.APPROACHING_OPPOSING_TRENCH;
    } else if ((zoneX == ApproachingZoneX.APPROACHING_OPPOSING_TRENCH)
        && zoneY == ApproachingZoneY.APPROACHING_BUMP) {
      return ApproachingZoneComposite.APPROACHING_OPPOSING_BUMP;
    } else if ((zoneX == ApproachingZoneX.APPROACHING_ALLIANCE_TOWER)
        && (zoneY == ApproachingZoneY.APPROACHING_ALLIANCE_TOWER)) {
      return ApproachingZoneComposite.APPROACHING_ALLIANCE_TOWER;
    } else if ((zoneX == ApproachingZoneX.APPROACHING_OPPOSING_TOWER)
        && (zoneY == ApproachingZoneY.APPROACHING_OPPOSING_TOWER)) {
      return ApproachingZoneComposite.APPROACHING_OPPOSING_TOWER;
    } else {
      return null;
    }
  }
}
