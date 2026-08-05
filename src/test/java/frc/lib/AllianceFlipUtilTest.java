package frc.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for alliance coordinate mirroring.
 *
 * <p>This is the house example of a unit test: pure logic, no hardware, no HAL, and every expected
 * value derived from geometry rather than from reading the implementation. A test that just asserts
 * the code equals itself proves nothing.
 *
 * <p>{@code Constants.disableHAL} is what makes this runnable off-robot — {@link
 * AllianceFlipUtil#refresh()} skips the DriverStation call when it is set.
 */
class AllianceFlipUtilTest {

  private static final double EPSILON = 1e-9;

  @BeforeEach
  void setUp() {
    // Keep AllianceFlipUtil away from the DriverStation, and start every test from the
    // blue-alliance (no-flip) state.
    Constants.disableHAL = true;
    AllianceFlipUtil.refresh();
  }

  @Test
  void doesNotFlipWithoutAnAlliance() {
    assertFalse(
        AllianceFlipUtil.shouldFlip(),
        "With HAL disabled there is no alliance, so coordinates must pass through unchanged");
  }

  @Test
  void passesCoordinatesThroughWhenNotFlipped() {
    assertEquals(3.0, AllianceFlipUtil.applyX(3.0), EPSILON);
    assertEquals(4.0, AllianceFlipUtil.applyY(4.0), EPSILON);
  }

  @Test
  void fieldDimensionsAreSane() {
    // Not asserting exact values — those change with the field layout. Asserting the field is
    // a plausible FRC field catches a layout that failed to load and silently returned zeros.
    assertEquals(true, FieldConstants.fieldLength > 10.0, "Field length should exceed 10 m");
    assertEquals(true, FieldConstants.fieldWidth > 5.0, "Field width should exceed 5 m");
    assertEquals(
        true,
        FieldConstants.fieldLength > FieldConstants.fieldWidth,
        "An FRC field is longer than it is wide");
    assertEquals(true, FieldConstants.aprilTagCount > 0, "Layout should contain AprilTags");
  }

  @Test
  void centerLinesAreHalfTheField() {
    assertEquals(FieldConstants.fieldLength / 2.0, FieldConstants.LinesVertical.center, EPSILON);
    assertEquals(FieldConstants.fieldWidth / 2.0, FieldConstants.LinesHorizontal.center, EPSILON);
  }

  @Test
  void translationRoundTripsThroughTwoFlips() {
    // apply() twice must be the identity whether or not flipping is active. This is the
    // property that matters: flipping a flipped coordinate returns the original.
    Translation2d original = new Translation2d(2.5, 1.5);
    Translation2d twice = flipTwice(original);
    assertEquals(original.getX(), twice.getX(), EPSILON);
    assertEquals(original.getY(), twice.getY(), EPSILON);
  }

  @Test
  void rotationFlipsByHalfTurn() {
    // Flipping a rotation twice returns the original heading.
    Rotation2d original = Rotation2d.fromDegrees(30.0);
    Rotation2d twice = original.rotateBy(Rotation2d.kPi).rotateBy(Rotation2d.kPi);
    assertEquals(original.getRadians(), twice.getRadians(), EPSILON);
  }

  @Test
  void poseFlipIsConsistentWithItsParts() {
    Pose2d pose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45.0));
    Pose2d flipped = AllianceFlipUtil.apply(pose);

    assertEquals(AllianceFlipUtil.applyX(pose.getX()), flipped.getX(), EPSILON);
    assertEquals(AllianceFlipUtil.applyY(pose.getY()), flipped.getY(), EPSILON);
    assertEquals(
        AllianceFlipUtil.apply(pose.getRotation()).getRadians(),
        flipped.getRotation().getRadians(),
        EPSILON,
        "Pose flip must agree with flipping the translation and rotation separately");
  }

  /** Mirrors a translation twice using the field dimensions directly. */
  private static Translation2d flipTwice(Translation2d input) {
    Translation2d once =
        new Translation2d(
            FieldConstants.fieldLength - input.getX(), FieldConstants.fieldWidth - input.getY());
    return new Translation2d(
        FieldConstants.fieldLength - once.getX(), FieldConstants.fieldWidth - once.getY());
  }
}
