package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Choreo autos: trajectories drawn in the Choreo GUI and saved to {@code src/main/deploy/choreo/},
 * followed by {@link Drive#followTrajectory}, and composed into routines here.
 *
 * <p><b>Three layers, one {@code driveToPose}.</b> A routine can drive a trajectory, drive a
 * trajectory and then finish on a PID-to-pose ({@link #thenAlign}), or skip trajectories entirely
 * and go waypoint to waypoint ({@code PointToPointAutos}). All three end in the same {@code
 * DriveCommands.driveToPose}, so leaning toward one style later needs no re-plumbing. See {@code
 * docs/autos.md}.
 *
 * <p><b>Event markers replace PathPlanner named commands.</b> A marker placed in the GUI fires the
 * command bound to its name with {@link #bind}. Bind in {@code RobotContainer}, before any routine
 * is built — a routine only sees bindings that existed when it was created. A bound command runs
 * alongside the trajectory, so it must <b>not</b> require the drive, or it interrupts the path —
 * the same rule PathPlanner event triggers had.
 */
public final class ChoreoAutos {

  private ChoreoAutos() {}

  /** Every event-marker name bound so far, for {@code RobotContainerSmokeTest}. */
  private static final Set<String> boundEvents = new LinkedHashSet<>();

  /**
   * The one {@link AutoFactory}. Built once in {@code RobotContainer}: it reads the pose from the
   * drive's estimator, resets odometry through {@code Drive.setPose}, follows samples with {@link
   * Drive#followTrajectory}, flips for the red alliance, and logs each trajectory to {@code
   * Autos/Trajectory} while it runs.
   */
  public static AutoFactory createFactory(Drive drive) {
    return new AutoFactory(
        drive::getPose,
        drive::setPose,
        drive::followTrajectory,
        true,
        drive,
        (trajectory, starting) ->
            Logger.recordOutput(
                "Autos/Trajectory", starting ? trajectory.getPoses() : new Pose2d[0]));
  }

  /**
   * Binds a command to every event marker called {@code eventName}. Use this instead of {@code
   * factory.bind} so the smoke test can check every marker in a {@code .traj} has a binding — an
   * unbound marker does nothing on the field, silently.
   */
  public static void bind(AutoFactory factory, String eventName, Command command) {
    boundEvents.add(eventName);
    factory.bind(eventName, command);
  }

  /** Read-only view of {@link #boundEvents}. */
  public static Set<String> boundEvents() {
    return Collections.unmodifiableSet(boundEvents);
  }

  /**
   * Every Choreo routine for the auto chooser, by name. Each value builds its routine only when
   * called — at auto start, for the selected one — so unselected trajectories are never loaded.
   *
   * <p>Empty until the team draws trajectories. Add an entry per routine:
   *
   * <pre>{@code
   * routines.put("Two Piece Left", () -> twoPieceLeft(factory, drive).cmd());
   * }</pre>
   */
  public static Map<String, Supplier<Command>> all(AutoFactory factory, Drive drive) {
    Map<String, Supplier<Command>> routines = new LinkedHashMap<>();
    return routines;
  }

  /**
   * The layered PID finish: when {@code trajectory} ends, drive to its final pose with {@code
   * DriveCommands.driveToPoseWithin}, so the robot settles on the exact pose instead of wherever
   * the follower left it. The 6328 and 1678 pattern — a trajectory for the route, PID for the last
   * few centimetres.
   *
   * <p>The final pose is already alliance-flipped by Choreo. Tolerances and timeout default from
   * {@code Constants}, the same ones point-to-point waypoints use.
   *
   * @return a trigger-free command; schedule it from {@code trajectory.done().onTrue(...)}
   */
  public static Command thenAlign(Drive drive, AutoTrajectory trajectory) {
    return DriveCommands.driveToPoseWithin(
            drive,
            () -> trajectory.getFinalPose().orElseGet(drive::getPose),
            Meters.of(Constants.Thresholds.WAYPOINT_POSITION_TOLERANCE_METERS),
            Degrees.of(Constants.Thresholds.WAYPOINT_HEADING_TOLERANCE_DEGREES),
            Seconds.of(Constants.Waits.WAYPOINT_TIMEOUT_SECONDS))
        .withName("Drive_AlignToTrajectoryEnd");
  }

  // ==============================================================================================
  // Example routine — copy this shape once a trajectory named "ExamplePath" exists in the GUI.
  // ==============================================================================================
  //
  //   public static AutoRoutine example(AutoFactory factory, Drive drive) {
  //     AutoRoutine routine = factory.newRoutine("Example");
  //     AutoTrajectory path = routine.trajectory("ExamplePath");
  //
  //     // Reset odometry to the trajectory's start, then drive it.
  //     routine.active().onTrue(Commands.sequence(path.resetOdometry(), path.cmd()));
  //
  //     // Optional layer two: finish on a PID-to-pose.
  //     path.done().onTrue(ChoreoAutos.thenAlign(drive, path));
  //
  //     // Mechanism actions go on event markers (bind) or on trajectory triggers:
  //     //   path.atTime("Intake").onTrue(IntakeCommands.deploy(intake));
  //     return routine;
  //   }

}
