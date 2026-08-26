package frc.robot.subsystems.exampleLift;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Draws the lift, so a wrong number is visible as a wrong picture.
 *
 * <p>Everything here is diagnostic. Nothing in this class affects robot behavior, and deleting it
 * changes nothing except what you can see. That is exactly why it earns its place — the failures it
 * catches are the ones where every logged number looks perfectly plausible.
 *
 * <h2>What it actually catches</h2>
 *
 * <p><b>This is the view that catches a wrong {@code STAGE_COUNT}</b>, and that alone justifies the
 * file. A two-stage cascade rigged in code as one-stage draws a carriage that shoots off the top of
 * the canvas at half the commanded height — unmistakable. The same error in a position plot is just
 * a number that seems a bit off, and it stays "a bit off" for weeks.
 *
 * <ul>
 *   <li><b>Wrong stage count or drum diameter.</b> The carriage travels an exact integer (or
 *       near-integer) multiple of its real travel.
 *   <li><b>Inverted sense.</b> Commanded up, drawn down. The plot rises either way.
 *   <li><b>Wrong zero.</b> A carriage drawn below the base after a zeroing run that stalled early.
 *   <li><b>Goal never reached.</b> Measured and goal drawn together makes a saturated profile or a
 *       clamping soft limit obvious without opening a log.
 *   <li><b>Goal outside the travel bounds.</b> The bound marker below makes this a five-second
 *       diagnosis instead of a tuning session spent on a gain that was never the problem.
 * </ul>
 *
 * <p>Use it during bring-up, before the mechanism is trusted — see {@code
 * docs/new-mechanism-bringup.md}. It is most valuable in simulation, where it is the only way to
 * see what the model is doing at all.
 *
 * <h2>Cost, and the flag</h2>
 *
 * <p>This runs every loop and publishes to NetworkTables every loop. That is not free, and the 2026
 * robot spent its season closer to 30 Hz than 50 Hz. {@code Visualization.ENABLED} exists so the
 * whole thing can be switched off in one place. Turn it off for competition unless you have checked
 * {@code LoopTiming/} with it on and the budget is intact.
 *
 * <p>{@code SmartDashboard.putData} is called <b>once</b>, in the constructor. The 2026 {@code
 * IntakePivotVisualizer} called it inside its update method, re-publishing the Sendable fifty times
 * a second — the dashboard holds a reference and reads through it, so once is all it ever needed.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename throughout, then fill in the {@code Visualization} block in
 * {@link ExampleLiftConstants}. The display geometry is cosmetic and can be nudged while watching
 * AdvantageScope; the {@code Pose3d} offset comes off the CAD.
 */
public class ExampleLiftVisualizer {

  // Cached, because a Color8Bit built inside update() is garbage generated fifty times a second
  // for no reason. Same argument as caching StatusSignals — see .claude/rules/02-hardware.md.
  private static final Color8Bit COLOR_MEASURED = new Color8Bit(Color.kGreen);
  private static final Color8Bit COLOR_GOAL = new Color8Bit(Color.kYellow);
  private static final Color8Bit COLOR_BOUND = new Color8Bit(Color.kWhite);
  private static final Color8Bit COLOR_BACKGROUND = new Color8Bit(Color.kBlack);
  private static final Color8Bit COLOR_BACKGROUND_AT_GOAL = new Color8Bit(Color.kDarkGreen);

  /** Drawn thicker than the goal so it reads on top where the two overlap. */
  private static final double LINE_WIDTH_MEASURED = 6.0;

  private static final double LINE_WIDTH_GOAL = 4.0;
  private static final double LINE_WIDTH_BOUND = 2.0;

  /** Straight up. A lift's ligament LENGTH is the mechanism state, not its angle. */
  private static final double VERTICAL_DEGREES = 90.0;

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d measured;
  private final LoggedMechanismLigament2d goal;

  public ExampleLiftVisualizer() {
    mechanism =
        new LoggedMechanism2d(
            ExampleLiftConstants.Visualization.CANVAS_WIDTH_METERS,
            ExampleLiftConstants.Visualization.CANVAS_HEIGHT_METERS,
            COLOR_BACKGROUND);

    // The root is the fixed point the mechanism travels from: the bottom of the travel.
    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            ExampleLiftConstants.NAME + "_Root",
            ExampleLiftConstants.Visualization.ROOT_X_METERS,
            ExampleLiftConstants.Visualization.ROOT_Y_METERS);

    // Top of travel, drawn from the SOFT LIMIT rather than duplicated here. A display bound that
    // disagrees with the enforced bound draws a reassuring picture of a lie.
    root.append(
        new LoggedMechanismLigament2d(
            ExampleLiftConstants.NAME + "_MaxBound",
            canvasLength(ExampleLiftConstants.MAX_TRAVEL_INCHES),
            VERTICAL_DEGREES,
            LINE_WIDTH_BOUND,
            COLOR_BOUND));

    // Goal appended before measured so measured draws over it.
    goal =
        new LoggedMechanismLigament2d(
            ExampleLiftConstants.NAME + "_Goal",
            ExampleLiftConstants.Visualization.MIN_LENGTH_METERS,
            VERTICAL_DEGREES,
            LINE_WIDTH_GOAL,
            COLOR_GOAL);

    measured =
        new LoggedMechanismLigament2d(
            ExampleLiftConstants.NAME + "_Measured",
            ExampleLiftConstants.Visualization.MIN_LENGTH_METERS,
            VERTICAL_DEGREES,
            LINE_WIDTH_MEASURED,
            COLOR_MEASURED);

    root.append(goal);
    root.append(measured);

    // ONCE, not every loop. The dashboard keeps a reference and reads through it.
    SmartDashboard.putData(ExampleLiftConstants.NAME + "/Visualizer", mechanism);
  }

  /**
   * Draws the current state. Called from {@code ExampleLift.periodic()}.
   *
   * <p>Both values arrive already converted to inches. This class holds no unit math on purpose:
   * the conversion belongs at the one boundary in {@link ExampleLiftConstants}, and a second copy
   * here is a second place for a factor to get in.
   *
   * @param measuredInches current carriage height
   * @param goalInches commanded carriage height
   * @param atGoal whether the mechanism is within tolerance — tints the canvas
   */
  public void update(double measuredInches, double goalInches, boolean atGoal) {
    if (!ExampleLiftConstants.Visualization.ENABLED) {
      return;
    }

    measured.setLength(canvasLength(measuredInches));
    goal.setLength(canvasLength(goalInches));
    mechanism.setBackgroundColor(atGoal ? COLOR_BACKGROUND_AT_GOAL : COLOR_BACKGROUND);

    Logger.recordOutput(ExampleLiftConstants.NAME + "/Visualizer/Mechanism2d", mechanism);

    // --- 3D component pose, for the AdvantageScope robot model ---
    //
    // A lift TRANSLATES rather than rotates, so the carriage's travel goes into the pose's Z and
    // its rotation stays fixed. That is the whole difference from the arm visualizer's Pose3d.
    Logger.recordOutput(
        ExampleLiftConstants.NAME + "/Visualizer/Pose3d",
        new Pose3d(
            ExampleLiftConstants.Visualization.CARRIAGE_OFFSET.plus(
                new Translation3d(0.0, 0.0, Units.inchesToMeters(measuredInches))),
            ExampleLiftConstants.Visualization.CARRIAGE_ROTATION));
  }

  /**
   * Real inches to canvas meters, floored so a retracted carriage is still drawn.
   *
   * <p>A ligament of zero length is not drawn at all, so without the floor a fully retracted
   * carriage silently disappears — which reads as a broken visualizer rather than a retracted
   * mechanism.
   */
  private static double canvasLength(double inches) {
    return ExampleLiftConstants.Visualization.MIN_LENGTH_METERS
        + inches * ExampleLiftConstants.Visualization.INCHES_TO_CANVAS_METERS;
  }

  // ============================================================================================
  // MULTIPLE MECHANISMS — build the shared component-pose publisher instead
  // ============================================================================================
  //
  // The Pose3d channel above is one per mechanism, which is right for the first one and stops
  // scaling around the third. AdvantageScope's 3D robot model wants a SINGLE Pose3d[] whose element
  // order matches the components list in the model's config.json.
  //
  // 2026 solved this with a RobotModelVisualizer that took a Supplier per articulated component and
  // published Pose3d[] at RobotModel/ComponentPoses from robotPeriodic(). It is not carried into
  // this template because it described the 2026 robot's four components exactly.
  //
  // Rebuild it in frc/lib once there are two or more articulated mechanisms. Suppliers rather than
  // subsystem references — a visualizer holding a subsystem would be the cross-subsystem coupling
  // .claude/rules/01-architecture.md forbids.
}
