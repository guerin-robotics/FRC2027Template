package frc.robot.subsystems.exampleArm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Draws the arm, so a wrong number is visible as a wrong picture.
 *
 * <p>Everything here is diagnostic. Nothing in this class affects robot behavior, and deleting it
 * changes nothing except what you can see. That is exactly why it earns its place — the failures it
 * catches are the ones where every logged number looks perfectly plausible.
 *
 * <h2>What it actually catches</h2>
 *
 * <p>A position plot tells you the mechanism reported 47 degrees. It does not tell you that 47
 * degrees points into the floor.
 *
 * <ul>
 *   <li><b>Inverted sense.</b> Commanded up, drawn down. The plot rises either way.
 *   <li><b>Wrong gear ratio.</b> The arm sweeps three times the travel it physically has. A
 *       position plot just shows larger numbers.
 *   <li><b>Wrong zero.</b> Stowed reads 90 degrees, so every setpoint in {@code
 *       Constants.Setpoints} is offset by a constant nobody wrote down.
 *   <li><b>Goal never reached.</b> Measured and goal drawn together makes a saturated profile or a
 *       clamping soft limit obvious without opening a log.
 *   <li><b>Goal outside the travel bounds.</b> The bound markers below make this a five-second
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
 * {@link ExampleArmConstants}. The display geometry is cosmetic and can be nudged while watching
 * AdvantageScope; the {@code Pose3d} offsets come off the CAD.
 */
public class ExampleArmVisualizer {

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

  /** Bound markers are drawn shorter than the arm so they frame it rather than hide it. */
  private static final double BOUND_LENGTH_FRACTION = 0.5;

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d measured;
  private final LoggedMechanismLigament2d goal;

  public ExampleArmVisualizer() {
    mechanism =
        new LoggedMechanism2d(
            ExampleArmConstants.Visualization.CANVAS_WIDTH_METERS,
            ExampleArmConstants.Visualization.CANVAS_HEIGHT_METERS,
            COLOR_BACKGROUND);

    // The root is the fixed point the mechanism turns about: the pivot shaft.
    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            ExampleArmConstants.NAME + "_Root",
            ExampleArmConstants.Visualization.ROOT_X_METERS,
            ExampleArmConstants.Visualization.ROOT_Y_METERS);

    // Travel bounds, drawn from the SOFT LIMITS rather than duplicated here. A display bound that
    // disagrees with the enforced bound draws a reassuring picture of a lie.
    root.append(
        new LoggedMechanismLigament2d(
            ExampleArmConstants.NAME + "_MinBound",
            ExampleArmConstants.Visualization.LENGTH_METERS * BOUND_LENGTH_FRACTION,
            ExampleArmConstants.REVERSE_SOFT_LIMIT_DEGREES,
            LINE_WIDTH_BOUND,
            COLOR_BOUND));

    root.append(
        new LoggedMechanismLigament2d(
            ExampleArmConstants.NAME + "_MaxBound",
            ExampleArmConstants.Visualization.LENGTH_METERS * BOUND_LENGTH_FRACTION,
            ExampleArmConstants.FORWARD_SOFT_LIMIT_DEGREES,
            LINE_WIDTH_BOUND,
            COLOR_BOUND));

    // Goal appended before measured so measured draws over it.
    goal =
        new LoggedMechanismLigament2d(
            ExampleArmConstants.NAME + "_Goal",
            ExampleArmConstants.Visualization.LENGTH_METERS,
            0.0,
            LINE_WIDTH_GOAL,
            COLOR_GOAL);

    measured =
        new LoggedMechanismLigament2d(
            ExampleArmConstants.NAME + "_Measured",
            ExampleArmConstants.Visualization.LENGTH_METERS,
            0.0,
            LINE_WIDTH_MEASURED,
            COLOR_MEASURED);

    root.append(goal);
    root.append(measured);

    // ONCE, not every loop. The dashboard keeps a reference and reads through it.
    SmartDashboard.putData(ExampleArmConstants.NAME + "/Visualizer", mechanism);
  }

  /**
   * Draws the current state. Called from {@code ExampleArm.periodic()}.
   *
   * <p>Both values arrive already converted to degrees. This class holds no unit math on purpose:
   * the conversion belongs at the one boundary in {@link ExampleArmConstants}, and a second copy
   * here is a second place for a factor to get in.
   *
   * @param measuredDegrees current mechanism position
   * @param goalDegrees commanded mechanism position
   * @param atGoal whether the mechanism is within tolerance — tints the canvas
   */
  public void update(double measuredDegrees, double goalDegrees, boolean atGoal) {
    if (!ExampleArmConstants.Visualization.ENABLED) {
      return;
    }

    measured.setAngle(measuredDegrees);
    goal.setAngle(goalDegrees);
    mechanism.setBackgroundColor(atGoal ? COLOR_BACKGROUND_AT_GOAL : COLOR_BACKGROUND);

    Logger.recordOutput(ExampleArmConstants.NAME + "/Visualizer/Mechanism2d", mechanism);

    // --- 3D component pose, for the AdvantageScope robot model ---
    //
    // The pose of the mechanism's PIVOT, not its tip: AdvantageScope rotates the whole component
    // mesh about the pose it is given, so the pose has to sit on the real axis of rotation.
    //
    // PITCH_SIGN carries the direction the mesh has to turn as the measured angle increases.
    // Whether it is +1 or -1 depends on which way the mechanism faces in the robot frame, and
    // guessing is fine — flip it while watching AdvantageScope. It is cosmetic and cannot affect
    // the robot.
    Logger.recordOutput(
        ExampleArmConstants.NAME + "/Visualizer/Pose3d",
        new Pose3d(
            ExampleArmConstants.Visualization.PIVOT_OFFSET,
            ExampleArmConstants.Visualization.PIVOT_ROTATION.plus(
                new Rotation3d(
                    0.0,
                    ExampleArmConstants.Visualization.PITCH_SIGN
                        * Units.degreesToRadians(measuredDegrees),
                    0.0))));
  }

  // ============================================================================================
  // MULTIPLE MECHANISMS — build the shared component-pose publisher instead
  // ============================================================================================
  //
  // The Pose3d channel above is one per mechanism, which is right for the first one and stops
  // scaling around the third. AdvantageScope's 3D robot model wants a SINGLE Pose3d[] whose element
  // order matches the components list in the model's config.json — drag one field onto the robot
  // object rather than wiring up four.
  //
  // 2026 solved this with a RobotModelVisualizer that took a Supplier<Angle> per articulated
  // component and published Pose3d[] at RobotModel/ComponentPoses from robotPeriodic(). It is not
  // carried into this template because it described the 2026 robot's four components exactly and
  // none of those numbers transfer.
  //
  // Rebuild it in frc/lib once there are two or more articulated mechanisms and a CAD export to
  // measure against. Suppliers rather than subsystem references — a visualizer holding a subsystem
  // would be the cross-subsystem coupling .claude/rules/01-architecture.md forbids.
}
