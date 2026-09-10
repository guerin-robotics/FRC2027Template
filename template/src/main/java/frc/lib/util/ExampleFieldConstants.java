package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Worked example of the pattern 6328 (Mechanical Advantage) rebuilds every season in their real
 * {@code FieldConstants.java} — see their 2025 ({@code Reef}/{@code CoralStation}/{@code
 * branchPositions}) and 2026 repos for the actual thing this is modeled on. {@code
 * frc.lib.util.FieldConstants} already tells you to follow this shape in its start-of-season
 * checklist; this file is what "the 2026 pattern" in that comment means, worked out in full.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Rename {@code ExampleFieldConstants} → whatever this year's
 * structure is called (2025 called it {@code Reef}, 2024 called it {@code Speaker}) 2. Replace
 * {@link #EXAMPLE_TAG_ID} and the four fictional face headings with real tag IDs and count from the
 * official layout 3. Replace {@link #SIDE_OFFSET_METERS} / {@link #STANDOFF_METERS} with measured
 * values off the CAD or the field drawing 4. Delete this instructions block 5. Move the result out
 * of {@code template/} and into {@code src/main/java/frc/lib/} — it does not belong here once it
 * describes a real game
 *
 * <h2>The pattern, in three rules</h2>
 *
 * <ol>
 *   <li><b>Derive from tag poses, never hardcode field coordinates by hand.</b> {@link
 *       FieldConstants#aprilTagLayout} is the official layout; a scoring location defined as an
 *       offset from a tag pose stays correct if the layout is ever corrected, and a hand-measured
 *       coordinate does not.
 *   <li><b>Everything is blue-alliance origin.</b> Every {@code Pose2d} in this file, and
 *       everywhere else in {@code FieldConstants}, is blue-alliance-relative. Never bake a
 *       red-alliance mirror into a constant. Flip at the call site with {@link
 *       AllianceFlipUtil#apply(Pose2d)} — {@code driveToPose}'s javadoc says exactly this for the
 *       same reason: one flip point is auditable, N flip points is N chances to flip twice or not
 *       at all.
 *   <li><b>Return a {@code Pose2d}, not a {@code Translation2d}.</b> {@code
 *       DriveCommands.driveToPose} drives translation AND heading — a location that does not also
 *       say which way the robot should face when it gets there is half a target.
 * </ol>
 *
 * <h2>How this plugs into driveToPose</h2>
 *
 * <pre>{@code
 * Pose2d target = AllianceFlipUtil.apply(ExampleFieldConstants.faceScoringPose(2, Side.LEFT));
 * controller.a().whileTrue(DriveCommands.driveToPose(drive, () -> target));
 * }</pre>
 *
 * A dynamic target (nearest face rather than a fixed one) works the same way — put the "which face"
 * decision inside the supplier instead of capturing a fixed {@code Pose2d}:
 *
 * <pre>{@code
 * DriveCommands.driveToPose(drive,
 *     () -> AllianceFlipUtil.apply(
 *         ExampleFieldConstants.faceScoringPose(
 *             ExampleFieldConstants.nearestFace(RobotState.getInstance().getEstimatedPose()),
 *             Side.LEFT)));
 * }</pre>
 */
public class ExampleFieldConstants {

  private ExampleFieldConstants() {}

  /**
   * FICTIONAL tag ID standing in for "the first tag on the structure you're scoring against." Real
   * code reads every relevant tag ID off {@link FieldConstants#aprilTagLayout} — see rule 1 above —
   * and there are usually several (6328's 2025 reef used six, one per face).
   */
  private static final int EXAMPLE_TAG_ID = 1;

  /** How many faces the structure has. 6328's 2025 reef was hexagonal; adjust per game. */
  private static final int FACE_COUNT = 4;

  /** Lateral offset from a face's center to its left/right scoring position, in meters. */
  private static final double SIDE_OFFSET_METERS = Units.inchesToMeters(6.5);

  /** How far in front of the face the robot's scoring pose sits, in meters. */
  private static final double STANDOFF_METERS = Units.inchesToMeters(18.0);

  /** Which side of a face to score on — mirrors 6328's left/right branch convention. */
  public enum Side {
    LEFT,
    RIGHT
  }

  /**
   * The center-face pose for each side of the structure, derived from tag poses.
   *
   * <p>Real code builds this the way 6328's {@code Reef.centerFaces} does: one {@code getTagPose}
   * call per known tag ID, assembled in a fixed clockwise (or counterclockwise) order so {@code
   * faceIndex} means the same face in code as it does when your drive team calls it out over the
   * radio.
   */
  private static final Pose2d[] centerFaces = buildCenterFaces();

  private static Pose2d[] buildCenterFaces() {
    Pose2d[] faces = new Pose2d[FACE_COUNT];
    for (int i = 0; i < FACE_COUNT; i++) {
      // FICTIONAL: real code calls FieldConstants.aprilTagLayout.getTagPose(realTagId).get()
      // once per face, the way 6328's Reef static initializer does for tags 17-22. Substituting
      // arithmetic here only stands in for "N faces around a center" so this file compiles
      // without a real layout to point at.
      faces[i] =
          new Pose2d(
              FieldConstants.fieldLength / 2.0 + Math.cos(Math.toRadians(90.0 * i)),
              FieldConstants.fieldWidth / 2.0 + Math.sin(Math.toRadians(90.0 * i)),
              Rotation2d.fromDegrees(180.0 + 90.0 * i));
    }
    return faces;
  }

  /**
   * The pose the robot should be at to score on the given face and side — the direct input to
   * {@code DriveCommands.driveToPose}.
   *
   * <p>Built the same way 6328 builds a branch pose: start at the face's pose (which already faces
   * outward, away from the structure, because {@code AprilTagFieldLayout} tag poses do), then
   * {@link Pose2d#transformBy} a lateral offset for left/right and a standoff so the robot parks
   * facing the structure rather than embedded in it.
   *
   * @param faceIndex Which face, in the same clockwise order {@link #centerFaces} was built in
   * @param side Left or right scoring position on that face
   * @return The blue-alliance-origin pose to drive to. Flip with {@link AllianceFlipUtil} at the
   *     call site before handing it to {@code driveToPose} — see the class javadoc.
   */
  public static Pose2d faceScoringPose(int faceIndex, Side side) {
    double lateral = side == Side.LEFT ? SIDE_OFFSET_METERS : -SIDE_OFFSET_METERS;
    return centerFaces[faceIndex].transformBy(
        new Transform2d(new Translation2d(-STANDOFF_METERS, lateral), Rotation2d.kZero));
  }

  /**
   * Which face is closest to a given robot pose — the building block for an "auto-align to
   * whichever face I'm nearest" command, as opposed to a driver picking one explicitly.
   *
   * <p>Compare against {@link frc.robot.RobotState#getAngleToTarget}: that returns a bearing for
   * {@code joystickDriveAtAngle} to hold, this returns an index used to look up a full pose for
   * {@code driveToPose}. Same idea, one level apart, matching the two commands themselves.
   *
   * @param robotPose Current robot pose, blue-alliance origin (e.g. {@code
   *     RobotState.getInstance().getEstimatedPose()})
   * @return Index into {@link #centerFaces} / {@link #faceScoringPose}
   */
  public static int nearestFace(Pose2d robotPose) {
    int nearest = 0;
    double nearestDistanceMeters = Double.POSITIVE_INFINITY;
    for (int i = 0; i < centerFaces.length; i++) {
      double distanceMeters =
          robotPose.getTranslation().getDistance(centerFaces[i].getTranslation());
      if (distanceMeters < nearestDistanceMeters) {
        nearestDistanceMeters = distanceMeters;
        nearest = i;
      }
    }
    return nearest;
  }
}
