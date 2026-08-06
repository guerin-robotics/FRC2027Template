package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link RobotState}'s field-geometry helpers.
 *
 * <p>Pure math over {@code Pose2d}/{@code Translation2d} — no HAL, no sim, no subsystem. These are
 * the two calls every future alignment command builds on ({@code
 * DriveCommands.joystickDriveAtAngle} takes a heading straight from {@link
 * RobotState#getAngleToTarget}, and {@code driveToPose} could use {@link
 * RobotState#getDistanceToPoint} the same way), so a sign or axis regression here would be silent
 * everywhere downstream until a robot pointed or drove the wrong way.
 *
 * <p>{@link RobotState} is a singleton — each test sets its own pose supplier rather than relying
 * on whatever a previous test (or the real robot) left behind.
 */
class RobotStateGeometryTest {

  private static final double EPSILON = 1e-9;

  @Test
  void angleToTargetDueEastIsZero() {
    RobotState.getInstance().setPoseSupplier(() -> Pose2d.kZero);
    Rotation2d angle = RobotState.getInstance().getAngleToTarget(new Translation2d(5.0, 0.0));
    assertEquals(0.0, angle.getRadians(), EPSILON);
  }

  @Test
  void angleToTargetDueNorthIsNinetyDegrees() {
    RobotState.getInstance().setPoseSupplier(() -> Pose2d.kZero);
    Rotation2d angle = RobotState.getInstance().getAngleToTarget(new Translation2d(0.0, 5.0));
    assertEquals(90.0, angle.getDegrees(), EPSILON);
  }

  @Test
  void angleToTargetIsRelativeToRobotPosition() {
    // Robot away from the origin: the target bearing must come from the robot's position, not
    // the field origin. Robot at (1, 1), target at (-2, 5) -> delta (-3, 4) -> atan2(4, -3).
    RobotState.getInstance().setPoseSupplier(() -> new Pose2d(1.0, 1.0, Rotation2d.kZero));
    Rotation2d angle = RobotState.getInstance().getAngleToTarget(new Translation2d(-2.0, 5.0));
    assertEquals(Math.atan2(4.0, -3.0), angle.getRadians(), EPSILON);
  }

  @Test
  void angleToTargetIgnoresCurrentHeading() {
    // getAngleToTarget returns a field-absolute bearing, not one relative to the robot's own
    // rotation — the robot facing away from the target should not change the answer.
    RobotState.getInstance()
        .setPoseSupplier(() -> new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
    Rotation2d angle = RobotState.getInstance().getAngleToTarget(new Translation2d(5.0, 0.0));
    assertEquals(0.0, angle.getRadians(), EPSILON);
  }

  @Test
  void distanceToPointIsAThreeFourFiveTriangle() {
    RobotState.getInstance().setPoseSupplier(() -> Pose2d.kZero);
    double distanceMeters =
        RobotState.getInstance().getDistanceToPoint(new Translation2d(3.0, 4.0)).in(Meters);
    assertEquals(5.0, distanceMeters, EPSILON);
  }

  @Test
  void distanceToPointIsZeroAtTheRobotsOwnPosition() {
    Pose2d pose = new Pose2d(7.5, -2.5, Rotation2d.fromDegrees(33.0));
    RobotState.getInstance().setPoseSupplier(() -> pose);
    double distanceMeters =
        RobotState.getInstance().getDistanceToPoint(pose.getTranslation()).in(Meters);
    assertEquals(0.0, distanceMeters, EPSILON);
  }
}
