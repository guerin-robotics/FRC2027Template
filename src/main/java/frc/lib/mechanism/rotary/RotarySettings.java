package frc.lib.mechanism.rotary;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The behavioural properties of a rotary position mechanism.
 *
 * <p>Both of these describe how the mechanism is built, not what it is asked to do, so neither
 * belongs in {@code Constants.Setpoints}. The angles it is commanded to — stow, deploy, score — do.
 *
 * @param positionTolerance How close counts as being there. A real mechanism never sits exactly on
 *     its setpoint, so anything that waits on one needs an explicit tolerance. Too tight and a
 *     sequence stalls waiting for a precision the mechanism does not have; too loose and it acts
 *     before the arm has arrived
 * @param armLength Distance from the pivot to the end of the arm. Used only for drawing — the
 *     visualizer needs a length to scale to, and the simulation needs it for gravity torque. A
 *     rough number is fine for the picture; the simulation wants a real one
 */
public record RotarySettings(Angle positionTolerance, Distance armLength) {}
