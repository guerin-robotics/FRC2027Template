package frc.lib.mechanism.linear;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/**
 * The physics a simulated linear mechanism obeys.
 *
 * <p>The drum geometry is not here — it lives in {@link LinearGeometry}, shared with the mechanism,
 * the visualizer and the constants file that wrote the travel bounds. Duplicating it here is how
 * simulation and hardware end up disagreeing about how far a rotation goes.
 *
 * @param gearbox The motors driving this mechanism, sized for leader plus followers. {@code
 *     MotorSpecs.KRAKEN_X60_FOC.gearbox(config.motorCount())} — a two-motor elevator simulated as
 *     one reaches half the acceleration, and every gain found against it is wrong
 * @param carriageMass Everything the motors have to lift: carriage, stages above it, and whatever
 *     it is holding. Worth a CAD number, because {@code kG} is most of what an elevator's loop is
 *     doing and {@code kG} is proportional to this
 * @param startingHeight Where the carriage sits at boot. Usually the bottom hard stop
 * @param simulateGravity False only for a horizontal extension. A horizontal mechanism with gravity
 *     simulated learns a {@code kG} that will fight it on the real robot
 */
public record LinearSimModel(
    DCMotor gearbox, Mass carriageMass, Distance startingHeight, boolean simulateGravity) {}
