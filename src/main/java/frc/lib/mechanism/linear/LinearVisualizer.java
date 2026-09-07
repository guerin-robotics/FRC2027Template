package frc.lib.mechanism.linear;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Distance;
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
 * Draws a linear mechanism as a vertical bar, measured against goal.
 *
 * <p>Diagnostic only — nothing here affects robot behaviour. What it catches is the class of
 * failure where every logged number looks plausible:
 *
 * <ul>
 *   <li><b>Wrong drum diameter or stage count.</b> The carriage sweeps twice the travel the
 *       mechanism physically has. A height plot just shows larger numbers, and both values scale
 *       every height the mechanism reports.
 *   <li><b>Inverted sense.</b> Commanded up, drawn down.
 *   <li><b>Wrong zero.</b> The carriage sits at the bottom reporting 14 inches, so every setpoint
 *       is offset by a constant nobody wrote down. On an elevator this is the common one, because
 *       the internal encoder reads zero wherever the carriage happened to be at boot.
 *   <li><b>Goal outside the travel bounds.</b> The bar is drawn against a full-travel frame, so a
 *       goal off the end of it is obvious.
 * </ul>
 *
 * <p>Geometry comes from the mechanism's own soft limits and {@link LinearGeometry}, so the picture
 * and the mechanism cannot disagree about how tall the elevator is.
 */
class LinearVisualizer {

  private static final Color8Bit COLOR_MEASURED = new Color8Bit(Color.kGreen);
  private static final Color8Bit COLOR_GOAL = new Color8Bit(Color.kYellow);
  private static final Color8Bit COLOR_FRAME = new Color8Bit(Color.kWhite);
  private static final Color8Bit COLOR_BACKGROUND = new Color8Bit(Color.kBlack);

  private static final double LINE_WIDTH_MEASURED = 8.0;
  private static final double LINE_WIDTH_GOAL = 4.0;
  private static final double LINE_WIDTH_FRAME = 2.0;

  /** Vertical, so the picture matches the mechanism. */
  private static final double STRAIGHT_UP_DEGREES = 90.0;

  private final String name;
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d measured;
  private final LoggedMechanismLigament2d goal;

  /**
   * Ligaments cannot have zero length without warnings, and a carriage at the bottom of travel has
   * exactly zero height. A hair of length keeps the drawing valid at the one position an elevator
   * spends most of its time in.
   */
  private static final double MIN_DRAWN_METERS = 0.001;

  LinearVisualizer(String name, LinearGeometry geometry, SoftLimits limits) {
    this.name = name;

    double minHeight = geometry.distanceFor(Rotations.of(limits.reverseRotations())).in(Meters);
    double maxHeight = geometry.distanceFor(Rotations.of(limits.forwardRotations())).in(Meters);
    double travel = maxHeight - minHeight;

    // Canvas sized from the travel, so it frames any elevator from a 6-inch extension upward.
    double canvasHeight = maxHeight * 1.2;
    double canvasWidth = Math.max(travel * 0.5, 0.2);

    mechanism = new LoggedMechanism2d(canvasWidth, canvasHeight, COLOR_BACKGROUND);
    LoggedMechanismRoot2d root = mechanism.getRoot(name + "_Base", canvasWidth / 2.0, minHeight);

    // The frame is full travel, drawn once. A bar shorter than the frame is a carriage partway up;
    // a goal marker past the top of the frame is a setpoint outside the world.
    root.append(
        new LoggedMechanismLigament2d(
            name + "_Frame",
            Math.max(travel, MIN_DRAWN_METERS),
            STRAIGHT_UP_DEGREES,
            LINE_WIDTH_FRAME,
            COLOR_FRAME));

    goal =
        root.append(
            new LoggedMechanismLigament2d(
                name + "_Goal",
                MIN_DRAWN_METERS,
                STRAIGHT_UP_DEGREES,
                LINE_WIDTH_GOAL,
                COLOR_GOAL));
    measured =
        root.append(
            new LoggedMechanismLigament2d(
                name + "_Measured",
                MIN_DRAWN_METERS,
                STRAIGHT_UP_DEGREES,
                LINE_WIDTH_MEASURED,
                COLOR_MEASURED));

    if (MechanismVisualization.ENABLED) {
      // Once, in the constructor. The dashboard holds a reference and reads through it; the 2026
      // IntakePivotVisualizer republished its Sendable fifty times a second for nothing.
      SmartDashboard.putData(name + "/Visualizer", mechanism);
    }
  }

  void update(Distance height, Distance goalHeight, boolean atGoal) {
    if (!MechanismVisualization.ENABLED) {
      return;
    }
    measured.setLength(Math.max(height.in(Meters), MIN_DRAWN_METERS));
    goal.setLength(Math.max(goalHeight.in(Meters), MIN_DRAWN_METERS));
    measured.setColor(atGoal ? COLOR_MEASURED : COLOR_GOAL);
    Logger.recordOutput(name + "/Visualizer", mechanism);
  }
}
