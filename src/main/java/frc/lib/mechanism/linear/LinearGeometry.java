package frc.lib.mechanism.linear;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * How one rotation of the drum becomes travel at the carriage.
 *
 * <p>Two numbers, and <b>neither can be guessed</b>. Both scale every height the mechanism reports,
 * so a wrong value produces a subsystem that compiles, runs, logs plausible numbers and is wrong
 * everywhere at once. That is much harder to notice than a mechanism that refuses to move.
 *
 * <p>This record is the single place the conversion lives. The constants file uses it to turn
 * travel bounds in inches into the mechanism rotations {@code MotorConfig} wants, the mechanism
 * uses it to report heights, the visualizer uses it to draw, and the simulation uses it to size its
 * effective drum radius. Four consumers, one formula — which is the whole point, because in 2026
 * this arithmetic was written out separately wherever it was needed.
 *
 * @param drumPitchDiameter Where the rope, belt or chain actually rides. <b>Not</b> the nominal
 *     diameter of the drum: a rope wraps at its own centreline, so the pitch diameter is the drum
 *     plus one rope thickness. On a belt it is the pitch diameter of the pulley, which is not the
 *     number stamped on it either
 * @param stages The rigging multiplier. 1 for a directly driven carriage, 2 for a two-stage cascade
 *     where the carriage moves twice the rope travel, and so on. Count the stages on the physical
 *     mechanism rather than trusting the CAD name — "two-stage" sometimes means two tubes and a 1:1
 *     rigging
 */
public record LinearGeometry(Distance drumPitchDiameter, int stages) {

  public LinearGeometry {
    if (drumPitchDiameter.in(Meters) <= 0.0) {
      throw new IllegalArgumentException("Drum pitch diameter must be greater than zero.");
    }
    if (stages < 1) {
      throw new IllegalArgumentException(
          "Stage count must be at least 1; got "
              + stages
              + ". A zero-stage elevator has no travel.");
    }
  }

  /** How far the carriage moves for one full rotation of the drum. */
  public Distance travelPerRotation() {
    return drumPitchDiameter.times(Math.PI).times(stages);
  }

  /**
   * Converts a carriage height into mechanism rotations.
   *
   * <p>This is what turns travel bounds in inches into the rotations {@code MotorConfig.softLimits}
   * expects.
   */
  public Angle rotationsFor(Distance distance) {
    return Rotations.of(distance.in(Meters) / travelPerRotation().in(Meters));
  }

  /** Converts mechanism rotations into a carriage height. */
  public Distance distanceFor(Angle angle) {
    return travelPerRotation().times(angle.in(Rotations));
  }

  /**
   * The radius the simulation should use.
   *
   * <p>Drum radius times the stage count. {@code ElevatorSim} knows nothing about rigging, so
   * folding the multiplier into the radius is what makes carriage travel and carriage force both
   * come out right: a cascade moves the carriage {@code stages} times the rope travel and applies
   * {@code 1/stages} of the force, which is exactly what a larger effective radius does.
   */
  Distance simEffectiveRadius() {
    return drumPitchDiameter.div(2.0).times(stages);
  }
}
