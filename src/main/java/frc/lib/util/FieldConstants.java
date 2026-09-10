// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

/**
 * Field dimensions and the AprilTag layout, defined from the blue alliance perspective.
 *
 * <p>This is the single source of truth for the field. {@link AllianceFlipUtil} mirrors coordinates
 * using {@link #fieldLength} / {@link #fieldWidth}, and {@code VisionConstants} reads {@link
 * #aprilTagLayout} rather than loading its own copy.
 *
 * <p><b>Start-of-season checklist:</b>
 *
 * <ol>
 *   <li>Update {@link #aprilTagLayout} to the new season's {@link AprilTagFields} value. Everything
 *       else here derives from it.
 *   <li>Add game-element geometry (goals, scoring zones, staging marks) as nested static classes
 *       below, following the 2026 pattern: dimensions first, then reference points derived from tag
 *       poses so the numbers stay tied to the official layout.
 * </ol>
 *
 * <p>The 2026 version of this file carried Hub, Tower, Trench, Bump, Depot and Outpost geometry
 * plus vertical/horizontal field lines. All of it was removed for the template — see git history in
 * the Rebuilt2026 repo if you want the shape of those definitions as a reference.
 */
public class FieldConstants {
  /**
   * The official AprilTag layout for the current season.
   *
   * <p>TODO(2027): change this to the 2027 layout as soon as WPILib ships it. Until then this loads
   * the 2026 Rebuilt welded field, so any pose estimate produced by this template is against the
   * 2026 field.
   */
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /** Number of tags in the layout. */
  public static final int aprilTagCount = aprilTagLayout.getTags().size();

  /** Printed width of one AprilTag, in meters. */
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);

  /** Field length along the X axis (blue wall to red wall), in meters. */
  public static final double fieldLength = aprilTagLayout.getFieldLength();

  /** Field width along the Y axis (scoring table to opposite wall), in meters. */
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  /** Useful reference lines. Extend with game-specific lines as the field is revealed. */
  public static class LinesVertical {
    /** Midfield, along the X axis. */
    public static final double center = fieldLength / 2.0;
  }

  /** Useful reference lines. Extend with game-specific lines as the field is revealed. */
  public static class LinesHorizontal {
    /** Midfield, along the Y axis. */
    public static final double center = fieldWidth / 2.0;
  }
}
