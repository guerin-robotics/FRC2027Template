package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.drive.Drive;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * BLine autos: polyline paths drawn in BLine Web, exported to {@code src/main/deploy/autos/}, and
 * followed by BLine-Lib's {@link FollowPath}. Routines are composed here.
 *
 * <p><b>Event triggers.</b> An event drawn in BLine Web fires the action registered under its
 * {@code lib_key} with {@link #bind}. Bind in {@code RobotContainer}, before any routine is built.
 * A bound command runs alongside the path, so it must <b>not</b> require the drive, or it
 * interrupts the path.
 *
 * <p><b>No time budget in the follower.</b> BLine follows physical progress, not a clock: a blocked
 * robot never finishes the path. Every path command in an auto needs {@code .withTimeout(...)},
 * with the value in {@code Constants.Waits}.
 */
public final class BLineAutos {

  private BLineAutos() {}

  // Follower gains: BLine's documented first-path starting values (docs.bline.run, "Tune Your
  // Robot"), deliberately left at the library defaults and not tuned on any robot. They are not the
  // old PathPlanner/Choreo gains (10 / 7.1) and do not mean the same thing: BLine's translation
  // output is the whole speed command, speed = kP x remaining path distance, so kP must stay
  // <= 2 x max acceleration / max velocity (2 x 8 / 4.0 = 4.0 for config.json) or the robot
  // overshoots its stop. Tune with /bline-auto; changing these is a High-risk change.
  private static final double TRANSLATION_KP = 2.0;
  private static final double ROTATION_KP = 1.0;
  private static final double CROSS_TRACK_KP = 0.2;

  /** Every event key bound so far, for {@code RobotContainerSmokeTest}. */
  private static final Set<String> boundEvents = new LinkedHashSet<>();

  /**
   * The one {@link FollowPath.Builder}. Built once in {@code RobotContainer}. It reads pose and
   * measured speeds from the drive, drives it with robot-relative speeds through {@code
   * Drive.runVelocity}, flips paths for the red alliance (from the Driver Station alliance, so it
   * follows the FMS station), mirrors them when {@code mirror} is true, and logs follower internals
   * to {@code FollowPath/} through AdvantageKit (all four BLine logging consumers are set here).
   *
   * <p><b>Author blue-left only.</b> Flip is rotational (red); mirror reflects across the field
   * centerline (right). BLine applies flip, then mirror, once each time a path command starts, so
   * the Left/Right choice made before the match is read at auto start. Check mechanism symmetry
   * before trusting a mirrored path: a side-mounted intake or shooter faces the wrong way.
   *
   * <p>The builder holds its PID objects and must not run two path commands at once. Autos run one
   * path at a time, so one builder is enough.
   *
   * <p>Pose reset is deliberately not set here. Reset odometry on the first path of a routine only:
   * {@code builder.withPoseReset(drive::setPose).build(first)}, then {@code
   * builder.withPoseReset(ignored -> {})} for the rest.
   */
  public static FollowPath.Builder createBuilder(
      Drive drive, Supplier<Boolean> mirror, AutoPreview preview) {
    FollowPath.setDoubleLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setBooleanLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    // BLine publishes through four process-wide consumers, all no-ops until set. Everything goes to
    // AdvantageKit under FollowPath/; the two geometry keys also feed the live markers on the Auto
    // Preview widget. Set here and nowhere else, so construction order cannot lose one.
    FollowPath.setPoseLoggingConsumer(
        value -> {
          Logger.recordOutput(value.getFirst(), value.getSecond());
          if (value.getFirst().equals("FollowPath/closestPoint")) {
            preview.showClosestPoint(value.getSecond());
          }
        });
    FollowPath.setTranslationListLoggingConsumer(
        value -> {
          Logger.recordOutput(value.getFirst(), value.getSecond());
          if (value.getFirst().equals("FollowPath/pathTranslations")) {
            preview.showActivePath(value.getSecond());
          }
        });

    return new FollowPath.Builder(
            drive,
            drive::getPose,
            drive::getChassisSpeeds,
            drive::runVelocity,
            new PIDController(TRANSLATION_KP, 0.0, 0.0),
            new PIDController(ROTATION_KP, 0.0, 0.0),
            new PIDController(CROSS_TRACK_KP, 0.0, 0.0))
        .withDefaultShouldFlip()
        .withShouldMirror(mirror)
        .withTRatioBasedTranslationHandoffs(true);
  }

  /**
   * Registers a command to run when any path crosses an event trigger with {@code key}. Use this
   * instead of {@code FollowPath.registerEventTrigger} so the smoke test can check every {@code
   * lib_key} in {@code deploy/autos/paths} has a registration — an unregistered key only warns on
   * the field, and the auto drives its path on schedule without the action.
   */
  public static void bind(String key, Command command) {
    boundEvents.add(key);
    FollowPath.registerEventTrigger(key, command);
  }

  /** Read-only view of {@link #boundEvents}. */
  public static Set<String> boundEvents() {
    return Collections.unmodifiableSet(boundEvents);
  }

  /**
   * Every BLine routine for the auto chooser, by name. Each value builds its routine only when
   * called — at auto start, for the selected one — so unselected paths are never loaded.
   *
   * <p>Empty until the team draws paths. Add an entry per routine with {@link #add}, listing its
   * path files in order so {@link AutoPreview} can draw it:
   *
   * <pre>{@code
   * add(routines, "Two Piece Left", () -> twoPieceLeft(builder, drive),
   *     "left-to-first", "first-to-score");
   * }</pre>
   */
  public static Map<String, Supplier<Command>> all(FollowPath.Builder builder, Drive drive) {
    Map<String, Supplier<Command>> routines = new LinkedHashMap<>();
    return routines;
  }

  /** Routine -> its path files in order, for {@link AutoPreview}. Keyed by the supplier itself. */
  private static final Map<Supplier<Command>, List<String>> pathsByRoutine =
      new IdentityHashMap<>();

  /** Adds {@code routine} to {@code routines} and records its path files for the preview. */
  private static void add(
      Map<String, Supplier<Command>> routines,
      String name,
      Supplier<Command> routine,
      String... pathFiles) {
    routines.put(name, routine);
    pathsByRoutine.put(routine, List.of(pathFiles));
  }

  /** The path files of a chooser selection, in order; empty for any non-BLine routine. */
  public static List<String> pathsOf(Supplier<Command> routine) {
    return pathsByRoutine.getOrDefault(routine, List.of());
  }

  // ==============================================================================================
  // Example routine — copy this shape once a path named "example" exists in BLine Web.
  // ==============================================================================================
  //
  //   public static Command example(FollowPath.Builder builder, Drive drive) {
  //     Path path = new Path("example"); // loads deploy/autos/paths/example.json
  //
  //     // Reset odometry to the path's start on the first path only, then clear the reset.
  //     Command follow =
  //         builder.withPoseReset(drive::setPose).build(path)
  //             .withTimeout(Constants.Waits.PATH_TIMEOUT_SECONDS);
  //     builder.withPoseReset(ignored -> {});
  //
  //     // Mechanism actions go on event triggers (bind). Compose with BLineCommands.sequence /
  //     // deadline rather than Commands.* when an event command shares a requirement with a
  //     // later step in the same group, or scheduling it cancels the whole auto.
  //     return follow.withName("Auto_Example");
  //   }

}
