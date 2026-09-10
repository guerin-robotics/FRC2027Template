package frc.lib.mechanism.linear;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import edu.wpi.first.units.measure.Distance;
import org.junit.jupiter.api.Test;

/**
 * The drum-and-rigging arithmetic, pinned.
 *
 * <p>Worth its own test because it is the one piece of the linear mechanism that is wrong
 * <i>silently</i>. A wrong drum diameter or stage count scales every height the mechanism reports,
 * so the subsystem compiles, runs, logs plausible numbers, and is wrong everywhere at once — which
 * is far harder to notice than a mechanism that refuses to move.
 *
 * <p>Four consumers share this formula: the constants file converting travel bounds into mechanism
 * rotations, the mechanism reporting heights, the visualizer drawing them, and the simulation
 * sizing its effective drum radius. They agree because there is only one of it.
 */
class LinearGeometryTest {

  /** A 1.5-inch pitch diameter drum on a two-stage cascade. */
  private static final LinearGeometry GEOMETRY = new LinearGeometry(Inches.of(1.5), 2);

  @Test
  void travelPerRotationIsCircumferenceTimesStages() {
    // pi * 1.5 * 2
    assertEquals(9.4248, GEOMETRY.travelPerRotation().in(Inches), 1e-3);
  }

  @Test
  void aSingleStageIsJustTheCircumference() {
    assertEquals(
        4.7124, new LinearGeometry(Inches.of(1.5), 1).travelPerRotation().in(Inches), 1e-3);
  }

  @Test
  void theConversionRoundTrips() {
    // Every height a command asks for goes through rotationsFor on the way to the device and back
    // through distanceFor on the way to a log. A conversion that does not round-trip shows up as a
    // mechanism that reports a height slightly different from the one it was told to go to, which
    // reads as a tolerance problem rather than an arithmetic one.
    Distance height = Inches.of(23.75);
    assertEquals(
        height.in(Inches), GEOMETRY.distanceFor(GEOMETRY.rotationsFor(height)).in(Inches), 1e-9);
  }

  @Test
  void rotationsForConvertsAKnownHeight() {
    // One full travel-per-rotation of height is exactly one rotation, by definition.
    assertEquals(1.0, GEOMETRY.rotationsFor(GEOMETRY.travelPerRotation()).in(Rotations), 1e-9);
    assertEquals(0.0, GEOMETRY.rotationsFor(Inches.of(0)).in(Rotations), 1e-9);
  }

  @Test
  void theStageCountReachesTheSimulationRadius() {
    // ElevatorSim knows nothing about rigging, so the multiplier is folded into the radius. That
    // is not a fudge: a cascade moves the carriage `stages` times the rope travel and applies
    // 1/stages of the force, which is exactly what a larger effective radius does to both.
    assertEquals(1.5, GEOMETRY.simEffectiveRadius().in(Inches), 1e-9);
    assertEquals(0.75, new LinearGeometry(Inches.of(1.5), 1).simEffectiveRadius().in(Inches), 1e-9);
  }

  @Test
  void impossibleGeometryIsRejected() {
    assertThrows(IllegalArgumentException.class, () -> new LinearGeometry(Inches.of(0), 1));
    assertThrows(IllegalArgumentException.class, () -> new LinearGeometry(Inches.of(-1), 1));

    IllegalArgumentException thrown =
        assertThrows(IllegalArgumentException.class, () -> new LinearGeometry(Inches.of(1.5), 0));
    assertEquals(true, thrown.getMessage().contains("at least 1"), thrown.getMessage());
  }
}
