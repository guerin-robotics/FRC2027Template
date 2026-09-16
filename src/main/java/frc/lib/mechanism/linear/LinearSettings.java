package frc.lib.mechanism.linear;

import edu.wpi.first.units.measure.Distance;

/**
 * The behavioural properties of a linear position mechanism.
 *
 * <p>Both describe how the mechanism is built rather than what it is asked to do, so neither
 * belongs in {@code Constants.Setpoints}. The heights it is commanded to — stow, L2, L3 — do.
 *
 * @param positionTolerance How close counts as being there. Too tight and a sequence stalls waiting
 *     for a precision the mechanism does not have; too loose and it acts before the carriage has
 *     arrived, which on an elevator usually means acting while it is still moving
 * @param geometry How a drum rotation becomes carriage travel
 */
public record LinearSettings(Distance positionTolerance, LinearGeometry geometry) {}
