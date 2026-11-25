package frc.robot.autos;

import static edu.wpi.first.math.util.Units.metersToInches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.BLine.Path;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * "Auto Preview" field widget: draws the selected auto's BLine paths exactly as they will run
 * (alliance flip, then Left/Right mirror), checks the robot is standing on the start pose, and
 * while the auto runs shows the active leg and the closest point on it.
 *
 * <p>The preview reloads whenever the selected auto, the alliance <b>or</b> the Left/Right side
 * changes. The alliance usually arrives late, from the FMS, after the drive team has already picked
 * the auto, so keying on the auto alone would show the wrong side of the field.
 *
 * <p>Only BLine routines registered with {@code BLineAutos.add(...)} have paths to draw. Others
 * show an empty preview.
 *
 * <p>Pass this to {@code BLineAutos.createBuilder}: BLine's logging consumers feed the live markers
 * through {@link #showActivePath} and {@link #showClosestPoint}.
 */
public final class AutoPreview {

  private final Field2d field = new Field2d();

  /** What the drawn preview was built from. Any difference means redraw. */
  private List<String> loadedNames = null;

  private boolean loadedFlip;
  private boolean loadedMirror;

  /** Start pose of the first leg, as it will run. Null when nothing valid is selected. */
  private Pose2d startPose = null;

  /**
   * False when the first path has no rotation target. FollowPath then holds the robot's live
   * starting heading, so any heading is a valid start and the heading check must not fail.
   */
  private boolean startHeadingConstrained = true;

  public AutoPreview() {
    SmartDashboard.putData("Auto Preview", field);
  }

  /** The leg FollowPath is running, as it transformed it (the path the robot actually follows). */
  public void showActivePath(Translation2d[] translations) {
    field.getObject("activePath").setPoses(toPoses(translations));
  }

  /** The closest point on the active segment to the robot. */
  public void showClosestPoint(Pose2d pose) {
    field.getObject("closestPoint").setPose(pose);
  }

  /**
   * Call every loop while disabled. Redraws if the selection changed, then logs how far the robot
   * is from the start pose.
   *
   * @param pathNames the selected routine's path files in order (empty if none)
   * @param flip whether the path will be flipped for red
   * @param mirror whether the path will be mirrored for the right side
   */
  public void update(List<String> pathNames, boolean flip, boolean mirror, Pose2d robotPose) {
    field.getObject("activePath").setPoses();
    field.getObject("closestPoint").setPoses();

    // Field compare, not a built string: this runs every disabled loop and must not allocate.
    if (!pathNames.equals(loadedNames) || flip != loadedFlip || mirror != loadedMirror) {
      loadedNames = pathNames;
      loadedFlip = flip;
      loadedMirror = mirror;
      load(pathNames, flip, mirror);
    }

    field.setRobotPose(robotPose);
    checkStartPose(robotPose);
  }

  /** Call every loop during auto so the robot follows the path on the widget. */
  public void showRobot(Pose2d robotPose) {
    field.setRobotPose(robotPose);
  }

  private void load(List<String> pathNames, boolean flip, boolean mirror) {
    field.getObject("path").setPoses();
    Logger.recordOutput("Auto/PreviewPath", new Pose2d[0]);
    startPose = null;
    if (pathNames.isEmpty()) {
      Logger.recordOutput("Auto/PreviewStatus", "No BLine path for the selected auto");
      return;
    }
    try {
      List<Pose2d> poses = new ArrayList<>();
      for (int i = 0; i < pathNames.size(); i++) {
        // Fresh copies, transformed in the same order FollowPath uses (flip, then mirror), so
        // the ones the follower runs are never touched.
        Path path = new Path(pathNames.get(i));
        if (flip) {
          path.flip();
        }
        if (mirror) {
          path.mirror();
        }
        poses.addAll(toPoses(path.getTranslations().toArray(Translation2d[]::new)));
        if (i == 0) {
          startPose = path.getStartPose();
          startHeadingConstrained =
              path.getPathElements().stream()
                  .anyMatch(e -> e instanceof Path.Waypoint || e instanceof Path.RotationTarget);
        }
      }
      field.getObject("path").setPoses(poses);
      Logger.recordOutput("Auto/PreviewPath", poses.toArray(Pose2d[]::new));
      Logger.recordOutput("Auto/StartPose", startPose);
      Logger.recordOutput("Auto/PreviewStatus", "Loaded " + pathNames.size() + " paths");
    } catch (RuntimeException e) {
      startPose = null;
      Logger.recordOutput("Auto/PreviewStatus", "Error loading " + pathNames + ": " + e);
    }
  }

  private void checkStartPose(Pose2d robotPose) {
    if (startPose == null) {
      Logger.recordOutput("Auto/StartCheck/PositionOK", false);
      Logger.recordOutput("Auto/StartCheck/RotationOK", false);
      return;
    }
    double distanceInches =
        metersToInches(robotPose.getTranslation().getDistance(startPose.getTranslation()));
    double rotationDegrees =
        Math.abs(robotPose.getRotation().minus(startPose.getRotation()).getDegrees());
    Logger.recordOutput("Auto/StartCheck/DistanceInches", distanceInches);
    Logger.recordOutput("Auto/StartCheck/RotationDiffDegrees", rotationDegrees);
    Logger.recordOutput(
        "Auto/StartCheck/PositionOK",
        distanceInches <= Constants.Thresholds.START_POSE_POSITION_TOLERANCE_INCHES);
    Logger.recordOutput(
        "Auto/StartCheck/RotationOK",
        !startHeadingConstrained
            || rotationDegrees <= Constants.Thresholds.START_POSE_HEADING_TOLERANCE_DEGREES);
  }

  private static List<Pose2d> toPoses(Translation2d[] translations) {
    List<Pose2d> poses = new ArrayList<>();
    for (Translation2d t : translations) {
      poses.add(new Pose2d(t, Rotation2d.kZero));
    }
    return poses;
  }
}
