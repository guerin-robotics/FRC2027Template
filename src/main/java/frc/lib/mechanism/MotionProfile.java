package frc.lib.mechanism;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.LoggedTunableNumber;

/**
 * The Motion Magic profile a mechanism moves along: how fast it may travel, how hard it may
 * accelerate, and how sharply it may change that acceleration.
 *
 * <p>All three are in <b>mechanism</b> rotations, matching {@code SensorToMechanismRatio}. Doubles
 * rather than {@code Measure} types because jerk has no WPILib measure type and a record with two
 * typed fields and one untyped one is worse than three consistent ones; the unit is in each name.
 * {@link #of} exists for callers who would rather write the typed form.
 *
 * <h2>Cruise velocity is a ceiling, not a target</h2>
 *
 * <p>Ask for more than the motor can physically deliver and nothing fails loudly — the profile
 * simply saturates and stops being followed, which shows up in a log as the reference running away
 * from the measurement. {@code MotorSpecs.maxMechanismRpm(ratio)} is the number to stay under, and
 * 80% of it is an optimistic working figure for a lightly loaded mechanism.
 *
 * <h2>Jerk</h2>
 *
 * <p>Zero means "no jerk limit", which is Phoenix's default and is the right starting point. A
 * non-zero jerk turns the trapezoidal profile into an S-curve: gentler on belts and chain, at the
 * cost of a slower move. Reach for it when a mechanism is audibly slamming into motion, not before.
 *
 * @param cruiseVelocityRps Maximum profile velocity, mechanism rotations per second
 * @param accelerationRpsSq Maximum profile acceleration, mechanism rotations per second squared
 * @param jerkRpsCubed Maximum profile jerk, mechanism rotations per second cubed; 0 disables
 */
public record MotionProfile(
    double cruiseVelocityRps, double accelerationRpsSq, double jerkRpsCubed) {

  /** Typed form, for callers who would rather not do the unit conversion by hand. */
  public static MotionProfile of(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
    return new MotionProfile(
        cruiseVelocity.in(RotationsPerSecond), acceleration.in(RotationsPerSecondPerSecond), 0.0);
  }

  /** Typed form including a jerk limit. */
  public static MotionProfile of(
      AngularVelocity cruiseVelocity, AngularAcceleration acceleration, double jerkRpsCubed) {
    return new MotionProfile(
        cruiseVelocity.in(RotationsPerSecond),
        acceleration.in(RotationsPerSecondPerSecond),
        jerkRpsCubed);
  }

  /**
   * A profile for a mechanism that only ever runs velocity control.
   *
   * <p>Cruise velocity is meaningless under {@code MotionMagicVelocity} — the commanded velocity is
   * the target, and the profile only governs how quickly it ramps there. Setting it to zero is
   * Phoenix's own "unlimited", and stating it that way says the omission was deliberate rather than
   * forgotten.
   */
  public static MotionProfile rampOnly(AngularAcceleration acceleration) {
    return new MotionProfile(0.0, acceleration.in(RotationsPerSecondPerSecond), 0.0);
  }

  /** Reads the current value out of a bank of tunables. Order matches {@link #tunables}. */
  static MotionProfile fromTunables(LoggedTunableNumber[] tunables) {
    return new MotionProfile(tunables[0].get(), tunables[1].get(), tunables[2].get());
  }

  /**
   * Builds the dashboard-editable copy of this profile, under {@code Tuning/<name>/}.
   *
   * <p>The profile is tunable alongside the gains and for the same reason: a mechanism that will
   * not reach its setpoint might have weak gains or might have a profile too slow to ask for the
   * motion in the first place, and those look similar until you can move both.
   */
  LoggedTunableNumber[] tunables(String name) {
    return new LoggedTunableNumber[] {
      new LoggedTunableNumber(name + "/Profile/CruiseVelocityRps", cruiseVelocityRps),
      new LoggedTunableNumber(name + "/Profile/AccelerationRpsSq", accelerationRpsSq),
      new LoggedTunableNumber(name + "/Profile/JerkRpsCubed", jerkRpsCubed),
    };
  }
}
