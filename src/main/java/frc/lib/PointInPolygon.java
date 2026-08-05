package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Tests whether a field point lies inside an arbitrary polygon.
 *
 * <p><b>Why this exists.</b> The 2026 season classified field zones with roughly 120 lines of
 * nested if-statements over x and y bounds. It worked, but nobody could read it, reviewing a
 * boundary change meant tracing every branch, and it could only express axis-aligned rectangles. A
 * zone written as a list of corners is checkable against a field drawing at a glance.
 *
 * <pre>
 * private static final List&lt;Translation2d&gt; SCORING_ZONE =
 *     List.of(
 *         new Translation2d(0.0, 0.0),
 *         new Translation2d(3.0, 0.0),
 *         new Translation2d(3.0, 2.0),
 *         new Translation2d(0.0, 2.0));
 *
 * boolean inZone = PointInPolygon.contains(robotPose.getTranslation(), SCORING_ZONE);
 * </pre>
 *
 * <p>Define zones from the blue-alliance perspective and mirror the robot's position with {@link
 * AllianceFlipUtil} before testing, rather than defining each zone twice.
 */
public class PointInPolygon {

  private PointInPolygon() {}

  /**
   * Returns true if {@code point} lies inside {@code polygon}.
   *
   * <p>Uses the ray-casting method: count how many polygon edges a ray from the point crosses. Odd
   * means inside. The polygon is treated as closed — do not repeat the first vertex at the end.
   * Vertex order does not matter, and the polygon may be concave.
   *
   * <p>Behavior exactly on an edge is not defined; a point on a boundary may test either way. Zones
   * that must not overlap should be defined with a small gap rather than sharing an edge.
   *
   * @param point The point to test
   * @param polygon Vertices in order, at least 3
   * @return true if the point is inside
   */
  public static boolean contains(Translation2d point, List<Translation2d> polygon) {
    if (polygon == null || polygon.size() < 3) {
      return false;
    }

    double x = point.getX();
    double y = point.getY();
    boolean inside = false;

    // Walk each edge from vertex j to vertex i, wrapping at the end.
    for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      double xi = polygon.get(i).getX();
      double yi = polygon.get(i).getY();
      double xj = polygon.get(j).getX();
      double yj = polygon.get(j).getY();

      // Does a horizontal ray at height y cross this edge, and does it cross to the right?
      boolean straddles = (yi > y) != (yj > y);
      if (straddles && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
        inside = !inside;
      }
    }

    return inside;
  }
}
