package frc.robot.subsystems.example;

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
 * Optional seventh file: draws the mechanism, so a wrong number is visible as a wrong picture.
 *
 * <p>Everything here is diagnostic. Nothing in this class affects robot behavior, and deleting it
 * changes nothing except what you can see. That is exactly why it earns its place — the failures it
 * catches are the ones where every logged number looks plausible.
 *
 * <h2>What it actually catches</h2>
 *
 * <p>A position plot tells you the mechanism reported 47°. It does not tell you that 47° points
 * into the floor. These are the specific mistakes that show up instantly as a picture and hide for
 * weeks as a number:
 *
 * <ul>
 *   <li><b>Inverted sense.</b> Commanded up, drawn down. The plot rises either way.
 *   <li><b>Wrong gear ratio.</b> The arm sweeps three times the travel it physically has. A
 *       position plot just shows larger numbers.
 *   <li><b>Wrong zero.</b> Stowed reads 90°, so every setpoint in {@code Constants.Setpoints} is
 *       offset by a constant nobody wrote down.
 *   <li><b>Goal never reached.</b> Measured and goal drawn together makes a saturated profile or a
 *       clamping soft limit obvious without opening a log.
 *   <li><b>Linear: wrong stage count.</b> The carriage travels an exact integer multiple of its
 *       real travel — the tell for a missed cascade multiplier.
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
 * <p>The 2026 {@code IntakePivotVisualizer} called {@code SmartDashboard.putData} inside its update
 * method, re-publishing the Sendable fifty times a second. It is called once in the constructor
 * here — the dashboard holds a reference to the object and reads it, so once is all it ever needed.
 *
 * <h2>TEMPLATE INSTRUCTIONS</h2>
 *
 * <ol>
 *   <li>Rename {@code ExampleSubsystem} → your subsystem name throughout.
 *   <li>Keep <b>one</b> of {@link #updateRotation} / {@link #updateLinear}, rename it {@code
 *       update()}, delete the other. For a velocity mechanism see the note on {@link
 *       #updateRotation} — you probably want to delete this whole file.
 *   <li>Fill in the {@code Visualization} block in {@code ExampleSubsystemConstants}. The display
 *       geometry is cosmetic and can be nudged while watching AdvantageScope; the Pose3d offsets
 *       come off the CAD.
 *   <li>Uncomment the soft-limit bound markers below once the soft limits exist.
 * </ol>
 */
public class ExampleSubsystemVisualizer {

  // Cached, because a Color8Bit built inside update() is garbage generated fifty times a second
  // for no reason. Same argument as caching StatusSignals — see .claude/rules/02-hardware.md.
  private static final Color8Bit COLOR_MEASURED = new Color8Bit(Color.kGreen);
  private static final Color8Bit COLOR_GOAL = new Color8Bit(Color.kYellow);
  private static final Color8Bit COLOR_BACKGROUND = new Color8Bit(Color.kBlack);
  private static final Color8Bit COLOR_BACKGROUND_AT_GOAL = new Color8Bit(Color.kDarkGreen);

  /** Drawn thicker than the goal so it reads on top where the two overlap. */
  private static final double LINE_WIDTH_MEASURED = 6.0;

  private static final double LINE_WIDTH_GOAL = 4.0;

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d measured;
  private final LoggedMechanismLigament2d goal;

  public ExampleSubsystemVisualizer() {
    mechanism =
        new LoggedMechanism2d(
            ExampleSubsystemConstants.Visualization.CANVAS_WIDTH_METERS,
            ExampleSubsystemConstants.Visualization.CANVAS_HEIGHT_METERS,
            COLOR_BACKGROUND);

    // The root is the fixed point the mechanism moves about: the pivot shaft for an arm, the
    // bottom of the travel for a lift.
    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            ExampleSubsystemConstants.NAME + "_Root",
            ExampleSubsystemConstants.Visualization.ROOT_X_METERS,
            ExampleSubsystemConstants.Visualization.ROOT_Y_METERS);

    // Goal appended first so measured draws over it.
    goal =
        new LoggedMechanismLigament2d(
            ExampleSubsystemConstants.NAME + "_Goal",
            ExampleSubsystemConstants.Visualization.LENGTH_METERS,
            0.0,
            LINE_WIDTH_GOAL,
            COLOR_GOAL);

    measured =
        new LoggedMechanismLigament2d(
            ExampleSubsystemConstants.NAME + "_Measured",
            ExampleSubsystemConstants.Visualization.LENGTH_METERS,
            0.0,
            LINE_WIDTH_MEASURED,
            COLOR_MEASURED);

    root.append(goal);
    root.append(measured);

    // ==========================================================================================
    // TRAVEL BOUNDS — uncomment once FORWARD/REVERSE_SOFT_LIMIT_ROTATIONS exist
    // ==========================================================================================
    //
    // Static markers at the soft limits. Worth having: a goal drawn past a bound is the picture
    // of a setpoint the mechanism will never reach, and that is a five-second diagnosis instead
    // of a tuning session spent on a gain that was never the problem.
    //
    // Read from the soft limits rather than duplicated here on purpose. A display bound that
    // disagrees with the enforced bound draws a reassuring picture of a lie.
    //
    //   private static final Color8Bit COLOR_BOUND = new Color8Bit(Color.kWhite);   // with the
    //                                                                               // colors above
    //
    //   root.append(
    //       new LoggedMechanismLigament2d(
    //           ExampleSubsystemConstants.NAME + "_MinBound",
    //           ExampleSubsystemConstants.Visualization.LENGTH_METERS * 0.5,
    //           ExampleSubsystemConstants.rotationsToDegrees(
    //               ExampleSubsystemConstants.REVERSE_SOFT_LIMIT_ROTATIONS),
    //           2.0,
    //           COLOR_BOUND));
    //
    //   root.append(
    //       new LoggedMechanismLigament2d(
    //           ExampleSubsystemConstants.NAME + "_MaxBound",
    //           ExampleSubsystemConstants.Visualization.LENGTH_METERS * 0.5,
    //           ExampleSubsystemConstants.rotationsToDegrees(
    //               ExampleSubsystemConstants.FORWARD_SOFT_LIMIT_ROTATIONS),
    //           2.0,
    //           COLOR_BOUND));

    // ONCE, not every loop. The dashboard keeps a reference and reads through it.
    SmartDashboard.putData(ExampleSubsystemConstants.NAME + "/Visualizer", mechanism);
  }

  /**
   * Rotating mechanism — arm, pivot, hood, turret. Angles in <b>mechanism degrees</b>, matching the
   * unit convention in {@code template/GUIDE.md}.
   *
   * <p>Both values arrive already converted. The visualizer holds no unit math on purpose: the
   * conversion belongs at the one boundary in {@code ExampleSubsystemConstants}, and a second copy
   * here is a second place for a factor of sixty to get in.
   *
   * <p><b>Velocity mechanisms — rollers, flywheels, feeders — should delete this whole file.</b> A
   * spinning drum has no position worth drawing, and a ligament rotating at 6000 RPM sampled at 50
   * Hz aliases into a bar that appears to drift slowly backward. The velocity plot beside {@code
   * closedLoopReference} already answers every question a picture would, and answers it better. The
   * 2026 {@code FlywheelVisualizer} was not this at all — it projected a shot trajectory, which is
   * game logic and belongs with the sequences, not in the scaffold.
   *
   * @param measuredDegrees current mechanism position
   * @param goalDegrees commanded mechanism position
   * @param atGoal whether the mechanism is within tolerance — tints the canvas
   */
  public void updateRotation(double measuredDegrees, double goalDegrees, boolean atGoal) {
    if (!ExampleSubsystemConstants.Visualization.ENABLED) {
      return;
    }

    measured.setAngle(measuredDegrees);
    goal.setAngle(goalDegrees);
    mechanism.setBackgroundColor(atGoal ? COLOR_BACKGROUND_AT_GOAL : COLOR_BACKGROUND);

    Logger.recordOutput(ExampleSubsystemConstants.NAME + "/Visualizer/Mechanism2d", mechanism);

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
        ExampleSubsystemConstants.NAME + "/Visualizer/Pose3d",
        new Pose3d(
            ExampleSubsystemConstants.Visualization.PIVOT_OFFSET,
            ExampleSubsystemConstants.Visualization.PIVOT_ROTATION.plus(
                new Rotation3d(
                    0.0,
                    ExampleSubsystemConstants.Visualization.PITCH_SIGN
                        * Units.degreesToRadians(measuredDegrees),
                    0.0))));
  }

  /**
   * Linear mechanism — elevator, lift, extension. Heights in <b>inches</b>, per the unit
   * convention.
   *
   * <p>Drawn as a bar that grows upward from the root rather than one that rotates: the ligament's
   * <i>length</i> is the mechanism state. {@code INCHES_TO_CANVAS_METERS} scales real travel onto
   * the canvas and is purely cosmetic — pick it so full travel roughly fills the canvas height.
   *
   * <p>This is the view that catches a wrong {@code STAGE_COUNT}. A two-stage cascade rigged as
   * one-stage draws a carriage that shoots off the top of the canvas at half the commanded height,
   * which is unmistakable. The same error in a position plot is just a number that seems a bit off.
   *
   * @param measuredInches current carriage height
   * @param goalInches commanded carriage height
   * @param atGoal whether the mechanism is within tolerance — tints the canvas
   */
  public void updateLinear(double measuredInches, double goalInches, boolean atGoal) {
    if (!ExampleSubsystemConstants.Visualization.ENABLED) {
      return;
    }

    // Straight up. A ligament of zero length is not drawn at all, so the minimum keeps the
    // carriage visible when the mechanism is fully retracted.
    measured.setAngle(90.0);
    goal.setAngle(90.0);
    measured.setLength(canvasLength(measuredInches));
    goal.setLength(canvasLength(goalInches));
    mechanism.setBackgroundColor(atGoal ? COLOR_BACKGROUND_AT_GOAL : COLOR_BACKGROUND);

    Logger.recordOutput(ExampleSubsystemConstants.NAME + "/Visualizer/Mechanism2d", mechanism);

    // 3D component pose: a lift TRANSLATES rather than rotates, so the carriage's travel goes
    // into the pose's Z and its rotation stays fixed.
    Logger.recordOutput(
        ExampleSubsystemConstants.NAME + "/Visualizer/Pose3d",
        new Pose3d(
            ExampleSubsystemConstants.Visualization.PIVOT_OFFSET.plus(
                new edu.wpi.first.math.geometry.Translation3d(
                    0.0, 0.0, Units.inchesToMeters(measuredInches))),
            ExampleSubsystemConstants.Visualization.PIVOT_ROTATION));
  }

  /** Real inches to canvas meters, floored so a retracted carriage is still drawn. */
  private static double canvasLength(double inches) {
    return ExampleSubsystemConstants.Visualization.MIN_LENGTH_METERS
        + inches * ExampleSubsystemConstants.Visualization.INCHES_TO_CANVAS_METERS;
  }

  // ============================================================================================
  // MULTIPLE MECHANISMS — build the shared component-pose publisher instead
  // ============================================================================================
  //
  // The Pose3d channels above are one per mechanism, which is right for the first one and stops
  // scaling around the third. AdvantageScope's 3D robot model wants a SINGLE Pose3d[] whose
  // element order matches the components list in the model's config.json — drag one field onto
  // the robot object rather than wiring up four.
  //
  // 2026 solved this with a RobotModelVisualizer in frc/robot/util that took a Supplier<Angle>
  // per articulated component and published Pose3d[] at RobotModel/ComponentPoses from
  // robotPeriodic(). It is not carried into this template because it described the 2026 robot's
  // four components exactly and none of those numbers transfer.
  //
  // Rebuild it in frc/lib once there are two or more articulated mechanisms and a CAD export to
  // measure against. Suppliers rather than subsystem references — a visualizer holding a
  // subsystem would be the cross-subsystem coupling .claude/rules/01-architecture.md forbids.
}
