package frc.robot;

import static edu.wpi.first.math.util.Units.metersToInches;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The pre-match auto check, ported from Rebuilt2026 {@code RobotContainer}.
 *
 * <p>While disabled, draws the selected auto's paths on the "Auto Preview" field and reports how
 * far the robot is from the auto's starting pose, so the drive team can confirm placement before
 * the match. During auto the same field shows the robot following the path.
 *
 * <p>In 2026 this lived in {@code RobotContainer}. It is logic, not wiring, so it has its own class
 * here; the behavior and every dashboard and log key are unchanged.
 */
public class AutoPreview {

  private final Field2d field = new Field2d();
  private final Supplier<Command> selectedAuto;

  private String lastAutoName = "";
  private Pose2d autoStartPose = new Pose2d();

  public AutoPreview(Supplier<Command> selectedAuto) {
    this.selectedAuto = selectedAuto;
    SmartDashboard.putData("Auto Preview", field);
  }

  /** Redraws the selected auto's paths when the selection changes. Call while disabled. */
  public void updatePathPreview() {
    Command auto = selectedAuto.get();
    if (auto == null) {
      return;
    }

    String autoName = auto.getName();
    if (autoName.equals(lastAutoName)) {
      return;
    }
    lastAutoName = autoName;

    Logger.recordOutput("Auto/SelectedAuto", autoName);
    field.getObject("path").setPoses();

    try {
      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      if (paths.isEmpty()) {
        Logger.recordOutput("Auto/PreviewStatus", "No paths found for: " + autoName);
        autoStartPose = new Pose2d();
        return;
      }

      List<Pose2d> allPoses = new ArrayList<>();
      for (PathPlannerPath path : paths) {
        PathPlannerPath displayPath = AllianceFlipUtil.shouldFlip() ? path.flipPath() : path;
        allPoses.addAll(displayPath.getPathPoses());
      }
      field.getObject("path").setPoses(allPoses);

      autoStartPose = paths.get(0).getStartingHolonomicPose().orElse(new Pose2d());
      if (AllianceFlipUtil.shouldFlip()) {
        autoStartPose = AllianceFlipUtil.apply(autoStartPose);
      }

      Logger.recordOutput("Auto/PreviewStatus", "Loaded " + paths.size() + " paths");
      Logger.recordOutput("Auto/StartPose", autoStartPose);
    } catch (Exception e) {
      Logger.recordOutput("Auto/PreviewStatus", "Error loading: " + e.getMessage());
      autoStartPose = new Pose2d();
    }
  }

  /** Publishes distance and heading error to the auto's start pose. Call while disabled. */
  public void checkStartPose() {
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    field.setRobotPose(currentPose);

    if (autoStartPose.equals(new Pose2d())) {
      Logger.recordOutput("Auto/StartCheck/PositionOK", false);
      Logger.recordOutput("Auto/StartCheck/RotationOK", false);
      return;
    }

    double distanceInches =
        metersToInches(currentPose.getTranslation().getDistance(autoStartPose.getTranslation()));
    double rotationDifferenceDegrees =
        Math.abs(currentPose.getRotation().minus(autoStartPose.getRotation()).getDegrees());

    boolean positionOK = distanceInches <= Constants.Thresholds.STARTING_POSE_DISTANCE_INCHES;
    boolean rotationOK =
        rotationDifferenceDegrees <= Constants.Thresholds.STARTING_POSE_ROTATION_DEGREES;

    Logger.recordOutput("Auto/StartCheck/DistanceInches", distanceInches);
    Logger.recordOutput("Auto/StartCheck/RotationDiffDegrees", rotationDifferenceDegrees);
    Logger.recordOutput("Auto/StartCheck/PositionOK", positionOK);
    Logger.recordOutput("Auto/StartCheck/RotationOK", rotationOK);
  }

  /** Shows the robot on the preview field. Call during auto. */
  public void showRobotPose() {
    field.setRobotPose(RobotState.getInstance().getEstimatedPose());
  }
}
