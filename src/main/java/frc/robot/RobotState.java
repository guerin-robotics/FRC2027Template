package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
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
}
