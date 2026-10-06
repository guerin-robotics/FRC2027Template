package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auto.Waypoint;
import frc.lib.auto.WaypointAuto;
import frc.lib.command.SteppableCommandGroup;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Point-to-point autos: drive straight from waypoint to waypoint with {@link
 * DriveCommands#driveToWaypoint}, no paths and no PathPlanner GUI.
 *
 * <p>These sit <b>beside</b> PathPlanner, not instead of it. Reach for one when an auto is a few
 * fixed stops with nothing in between to avoid; reach for PathPlanner when the route itself matters
 * (curves, speed through a gap, event markers mid-path). Both end up in the same auto chooser.
 *
 * <p>An auto is data — a {@link WaypointAuto} — and the factories here turn it into a command. Add
 * one by writing a method like {@link #example()} and listing it in {@link #all()}.
 *
 * <p><b>Time budget.</b> Every waypoint has a timeout, so the auto always ends, but the sum of the
 * timeouts can exceed the auto period. Time each one in sim before an event, per {@code
 * .claude/rules/03-commands.md}.
 */
public final class PointToPointAutos {

  private PointToPointAutos() {}

  /** Every point-to-point auto that should appear in the chooser. */
  public static List<WaypointAuto> all() {
    return List.of(example());
  }

  /**
   * A placeholder square near the blue alliance wall — replace with real 2027 autos. Coordinates
   * are blue-origin meters on whatever field {@code FieldConstants} currently describes.
   */
  public static WaypointAuto example() {
    return new WaypointAuto(
        "P2P Example",
        List.of(
            waypoint(2.0, 4.0, 0.0),
            waypoint(3.0, 4.0, 0.0),
            waypoint(3.0, 5.0, 90.0),
            waypoint(2.0, 5.0, 180.0)));
  }

  /**
   * The auto as one command: reset odometry to the (alliance-flipped) first waypoint, then drive to
   * each in turn. Logs the waypoints to {@code Autos/<name>/Waypoints} for an AdvantageScope
   * preview.
   */
  public static Command build(Drive drive, WaypointAuto auto) {
    Logger.recordOutput("Autos/" + auto.name() + "/Waypoints", auto.poses());
    return Commands.sequence(steps(drive, auto).toArray(new Command[0]))
        .withName("Auto_" + auto.name().replace(' ', '_'));
  }

  /**
   * The same auto, one waypoint per press of {@code forward} — for walking it through in the pit.
   * Pass trigger accessors from {@code Triggers.java}.
   */
  public static Command stepThrough(
      Drive drive, WaypointAuto auto, BooleanSupplier forward, BooleanSupplier back) {
    return new SteppableCommandGroup(forward, back, steps(drive, auto))
        .withName("AutoStep_" + auto.name().replace(' ', '_'));
  }

  private static List<Command> steps(Drive drive, WaypointAuto auto) {
    List<Command> steps = new ArrayList<>();
    steps.add(
        Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(auto.startingPose())), drive)
            .withName("Drive_ResetToAutoStart"));
    for (Waypoint waypoint : auto.waypoints()) {
      steps.add(DriveCommands.driveToWaypoint(drive, waypoint));
    }
    return steps;
  }

  /** A waypoint with the default tolerances and timeout from {@code Constants}. */
  private static Waypoint waypoint(double xMeters, double yMeters, double headingDegrees) {
    return new Waypoint(
        new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees)),
        Meters.of(Constants.Thresholds.WAYPOINT_POSITION_TOLERANCE_METERS),
        Degrees.of(Constants.Thresholds.WAYPOINT_HEADING_TOLERANCE_DEGREES),
        Seconds.of(Constants.Waits.WAYPOINT_TIMEOUT_SECONDS));
  }
}
