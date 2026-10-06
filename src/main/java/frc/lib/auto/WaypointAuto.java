package frc.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

/**
 * A point-to-point auto as data: a name and the waypoints it drives through, in order.
 *
 * <p>The data lives in the library; turning it into a command does not, because that needs the
 * drivetrain and {@code frc.lib} may not depend on {@code frc.robot}. See {@code
 * frc.robot.autos.PointToPointAutos}.
 *
 * <p>The first waypoint is also the starting pose: the auto resets odometry to it before driving,
 * so place the robot there.
 *
 * @param name Shown in the auto chooser and used as the log key
 * @param waypoints Blue-origin, in driving order. At least one.
 */
public record WaypointAuto(String name, List<Waypoint> waypoints) {

  public WaypointAuto {
    if (waypoints.isEmpty()) {
      throw new IllegalArgumentException("WaypointAuto '" + name + "' needs at least one waypoint");
    }
    waypoints = List.copyOf(waypoints);
  }

  /** Blue-origin starting pose — the first waypoint. */
  public Pose2d startingPose() {
    return waypoints.get(0).pose();
  }

  /** Blue-origin poses in order, for a dashboard or AdvantageScope preview. */
  public Pose2d[] poses() {
    return waypoints.stream().map(Waypoint::pose).toArray(Pose2d[]::new);
  }
}
