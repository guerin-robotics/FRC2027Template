package frc.lib.mechanism.roller;

import edu.wpi.first.units.measure.AngularVelocity;

/**
 * The behavioural properties of a velocity-controlled mechanism: how close counts as "at speed".
 *
 * <p>These belong with the mechanism rather than in {@code Constants.Setpoints}, because they
 * describe how the mechanism is built rather than what it is being asked to do. The test from
 * {@code .claude/rules/03-commands.md} is whether a driver might ask you to change it between
 * matches: "run the intake a bit faster" is a setpoint, "call it at speed within 100 RPM" is not.
 *
 * @param velocityTolerance How far from the commanded velocity still counts as being there. A real
 *     mechanism never sits exactly on its setpoint, so a command that waits on one needs an
 *     explicit tolerance rather than an equality check
 */
public record RollerSettings(AngularVelocity velocityTolerance) {

  public static RollerSettings of(AngularVelocity velocityTolerance) {
    return new RollerSettings(velocityTolerance);
  }
}
