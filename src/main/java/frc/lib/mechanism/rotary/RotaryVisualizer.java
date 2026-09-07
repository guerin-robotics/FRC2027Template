package frc.lib.mechanism.rotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.mechanism.MechanismVisualization;
import frc.lib.mechanism.MotorConfig.SoftLimits;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Draws a rotary mechanism, so a wrong number is visible as a wrong picture.
 *
 * <p>Everything here is diagnostic. Nothing in this class affects robot behaviour, and deleting it
 * would change only what you can see — which is exactly why it earns its place, because the
 * failures it catches are the ones where every logged number looks perfectly plausible.
 *
 * <h2>What it actually catches</h2>
 *
 * <p>A position plot tells you the mechanism reported 47 degrees. It does not tell you that 47
 * degrees points into the floor.
 *
 * <ul>
 *   <li><b>Inverted sense.</b> Commanded up, drawn down. The plot rises either way.
 *   <li><b>Wrong gear ratio.</b> The arm sweeps three times the travel it physically has. A plot
 *       just shows larger numbers.
 *   <li><b>Wrong zero.</b> Stowed reads 90 degrees, so every setpoint is offset by a constant
 *       nobody wrote down.
 *   <li><b>Goal outside the travel bounds.</b> The bound markers make this a five-second diagnosis
 *       instead of a tuning session spent on a gain that was never the problem.
 * </ul>
 *
 * <p>The geometry comes from the mechanism's own soft limits and arm length rather than from a
 * block of display constants. That is deliberate: a visualizer configured separately from the
 * mechanism drifts out of step with it, and a picture that lies is worse than no picture.
 *
 * <p>{@code SmartDashboard.putData} is called <b>once</b>, in the constructor. The 2026 {@code
 * IntakePivotVisualizer} called it inside its update method, republishing the Sendable fifty times
 * a second — the dashboard holds a reference and reads through it, so once is all it ever needed.
 */
class RotaryVisualizer {

  // Cached. A Color8Bit built inside update() is garbage generated fifty times a second for no
  // reason — the same argument that applies to caching StatusSignals.
  private static final Color8Bit COLOR_MEASURED = new Color8Bit(Color.kGreen);
  private static final Color8Bit COLOR_GOAL = new Color8Bit(Color.kYellow);
  private static final Color8Bit COLOR_BOUND = new Color8Bit(Color.kWhite);
  private static final Color8Bit COLOR_BACKGROUND = new Color8Bit(Color.kBlack);

  /** Drawn thicker than the goal so it reads on top where the two overlap. */
  private static final double LINE_WIDTH_MEASURED = 6.0;

  private static final double LINE_WIDTH_GOAL = 4.0;
  private static final double LINE_WIDTH_BOUND = 2.0;

  /** Bound markers are drawn shorter than the arm so they frame it rather than hide it. */
  private static final double BOUND_LENGTH_FRACTION = 0.5;

  private final String name;
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d measured;
  private final LoggedMechanismLigament2d goal;

  RotaryVisualizer(String name, RotarySettings settings, SoftLimits limits) {
    this.name = name;

    double armLength = settings.armLength().in(Meters);
    // The canvas is sized from the arm so it always frames the mechanism, whatever its scale.
    double canvas = armLength * 2.5;

    mechanism = new LoggedMechanism2d(canvas, canvas, COLOR_BACKGROUND);
    LoggedMechanismRoot2d root = mechanism.getRoot(name + "_Pivot", canvas / 2.0, canvas / 2.0);

    // Bounds first, so the live ligaments draw over them.
    root.append(
        new LoggedMechanismLigament2d(
            name + "_ReverseBound",
            armLength * BOUND_LENGTH_FRACTION,
            Rotations.of(limits.reverseRotations()).in(Degrees),
            LINE_WIDTH_BOUND,
            COLOR_BOUND));
    root.append(
        new LoggedMechanismLigament2d(
            name + "_ForwardBound",
            armLength * BOUND_LENGTH_FRACTION,
            Rotations.of(limits.forwardRotations()).in(Degrees),
            LINE_WIDTH_BOUND,
            COLOR_BOUND));

    goal =
        root.append(
            new LoggedMechanismLigament2d(
                name + "_Goal", armLength, 0.0, LINE_WIDTH_GOAL, COLOR_GOAL));
    measured =
        root.append(
            new LoggedMechanismLigament2d(
                name + "_Measured", armLength, 0.0, LINE_WIDTH_MEASURED, COLOR_MEASURED));

    if (MechanismVisualization.ENABLED) {
      SmartDashboard.putData(name + "/Visualizer", mechanism);
    }
  }

  void update(Angle position, Angle goalPosition, boolean atGoal) {
    if (!MechanismVisualization.ENABLED) {
      return;
    }
    measured.setAngle(position.in(Degrees));
    goal.setAngle(goalPosition.in(Degrees));
    measured.setColor(atGoal ? COLOR_MEASURED : COLOR_GOAL);
    Logger.recordOutput(name + "/Visualizer", mechanism);
  }
}
