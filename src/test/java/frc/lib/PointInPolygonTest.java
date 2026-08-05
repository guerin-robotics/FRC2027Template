package frc.lib;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Tests for polygon zone containment.
 *
 * <p>Expected values come from geometry, not from running the implementation. The concave case
 * matters most — an axis-aligned bounding-box check would pass every other test here and still get
 * that one wrong, which is exactly the failure mode this class exists to avoid.
 */
class PointInPolygonTest {

  /** Unit square with corners at (0,0) and (2,2). */
  private static final List<Translation2d> SQUARE =
      List.of(
          new Translation2d(0.0, 0.0),
          new Translation2d(2.0, 0.0),
          new Translation2d(2.0, 2.0),
          new Translation2d(0.0, 2.0));

  /**
   * An L shape occupying the bottom and left of a 3x3 box. The notch is the top-right quadrant,
   * from (1,1) to (3,3).
   */
  private static final List<Translation2d> L_SHAPE =
      List.of(
          new Translation2d(0.0, 0.0),
          new Translation2d(3.0, 0.0),
          new Translation2d(3.0, 1.0),
          new Translation2d(1.0, 1.0),
          new Translation2d(1.0, 3.0),
          new Translation2d(0.0, 3.0));

  @Test
  void pointInsideSquare() {
    assertTrue(PointInPolygon.contains(new Translation2d(1.0, 1.0), SQUARE));
  }

  @Test
  void pointsOutsideSquareOnEverySide() {
    assertFalse(PointInPolygon.contains(new Translation2d(-1.0, 1.0), SQUARE), "left");
    assertFalse(PointInPolygon.contains(new Translation2d(3.0, 1.0), SQUARE), "right");
    assertFalse(PointInPolygon.contains(new Translation2d(1.0, -1.0), SQUARE), "below");
    assertFalse(PointInPolygon.contains(new Translation2d(1.0, 3.0), SQUARE), "above");
  }

  @Test
  void pointDiagonallyOutsideIsNotInside() {
    // Catches an implementation that tests x and y independently instead of as a region.
    assertFalse(PointInPolygon.contains(new Translation2d(-1.0, -1.0), SQUARE));
  }

  @Test
  void concaveNotchIsNotInside() {
    // (2, 2) sits inside the L's bounding box but inside the notch, so it is OUTSIDE the shape.
    // A bounding-box check would wrongly return true here.
    assertFalse(
        PointInPolygon.contains(new Translation2d(2.0, 2.0), L_SHAPE),
        "Point in the concave notch is outside the polygon");
  }

  @Test
  void concaveArmsAreInside() {
    assertTrue(PointInPolygon.contains(new Translation2d(2.0, 0.5), L_SHAPE), "bottom arm");
    assertTrue(PointInPolygon.contains(new Translation2d(0.5, 2.0), L_SHAPE), "left arm");
  }

  @Test
  void vertexOrderDoesNotMatter() {
    // Clockwise instead of counter-clockwise. Ray casting is winding-agnostic; an
    // implementation that relied on signed area would fail this.
    List<Translation2d> reversed = new ArrayList<>(SQUARE);
    Collections.reverse(reversed);
    assertTrue(PointInPolygon.contains(new Translation2d(1.0, 1.0), reversed));
    assertFalse(PointInPolygon.contains(new Translation2d(5.0, 5.0), reversed));
  }

  @Test
  void degeneratePolygonsAreNotContaining() {
    assertFalse(PointInPolygon.contains(new Translation2d(0.0, 0.0), List.of()));
    assertFalse(
        PointInPolygon.contains(
            new Translation2d(0.0, 0.0),
            List.of(new Translation2d(0.0, 0.0), new Translation2d(1.0, 1.0))));
  }
}
