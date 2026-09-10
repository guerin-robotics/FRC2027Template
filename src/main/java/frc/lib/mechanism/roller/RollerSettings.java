package frc.lib.mechanism.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;

/**
 * The behavioural properties of a velocity-controlled mechanism: how close counts as "at speed",
 * and what a jam looks like.
 *
 * <p>These belong with the mechanism rather than in {@code Constants.Setpoints}, because they
 * describe how the mechanism is built rather than what it is being asked to do. The test from
 * {@code .claude/rules/03-commands.md} is whether a driver might ask you to change it between
 * matches: "run the intake a bit faster" is a setpoint, "call it at speed within 100 RPM" is not.
 *
 * @param velocityTolerance How far from the commanded velocity still counts as being there. A real
 *     mechanism never sits exactly on its setpoint, so a command that waits on one needs an
 *     explicit tolerance rather than an equality check
 * @param jam How to recognise a jam, or empty for a mechanism that cannot jam
 */
public record RollerSettings(AngularVelocity velocityTolerance, Optional<JamDetection> jam) {

  /** A mechanism that cannot jam — a flywheel spinning in free air. */
  public static RollerSettings of(AngularVelocity velocityTolerance) {
    return new RollerSettings(velocityTolerance, Optional.empty());
  }

  /** Adds jam detection. Rollers, feeders, intakes and transports all want it. */
  public RollerSettings withJamDetection(JamDetection detection) {
    return new RollerSettings(velocityTolerance, Optional.of(detection));
  }

  /**
   * What a jam looks like on this mechanism.
   *
   * <p><b>A jam is a conjunction, never a single signal.</b> All four of these must hold:
   *
   * <ol>
   *   <li>we are actually commanding motion — an idle roller is not jammed
   *   <li>measured velocity is far below commanded — it is not turning
   *   <li>stator current is high — it is trying hard
   *   <li>all three have held for a dwell — it is not a transient
   * </ol>
   *
   * <p>Current alone is the classic mistake. It spikes on every static-friction breakaway and every
   * first contact with a game piece, both entirely normal.
   *
   * <p>2026 ran its rollers open-loop with no feedback, so jams were <b>silent</b>: the mechanism
   * stopped working and nothing in the log said why. This is the cheapest instrumentation on the
   * list and it is the one that was missing.
   *
   * @param minCommandedVelocity Below this, the mechanism is idle and cannot be jammed
   * @param velocityFraction Fraction of the commanded velocity below which it counts as "not
   *     turning". 0.2 is a reasonable starting point
   * @param statorThreshold Stator current above which it counts as "trying hard". <b>Stator, not
   *     supply</b> — at the low-speed, high-load condition that defines a jam, supply is only a
   *     fraction of stator because the controller is chopping, so a supply threshold sits much
   *     closer to the noise floor
   * @param dwell How long all three must hold. <b>This must exceed spin-up time</b>, or every start
   *     reads as a jam: during spin-up the command is high, the measurement is low and the current
   *     is high, which is exactly the jam signature. Check it against the profile's acceleration,
   *     and raise it if that ever drops
   */
  public record JamDetection(
      AngularVelocity minCommandedVelocity,
      double velocityFraction,
      Current statorThreshold,
      Time dwell) {}
}
