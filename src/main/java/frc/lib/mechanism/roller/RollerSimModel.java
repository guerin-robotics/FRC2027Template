package frc.lib.mechanism.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * The physics a simulated roller obeys.
 *
 * <p>Two numbers, both of which want to come from somewhere real. Build the gearbox from {@code
 * MotorSpecs} rather than calling {@code DCMotor} directly, so the motor driving the simulation is
 * the same one the speed arithmetic in the constants file used and the two cannot disagree.
 *
 * @param gearbox The motors driving this mechanism, sized for leader plus followers. {@code
 *     MotorSpecs.KRAKEN_X60_FOC.gearbox(config.motorCount())}
 * @param momentOfInertia Rotational inertia at the mechanism, in kg·m². A CAD mass-properties
 *     number if you have one. If you do not, a guess makes the simulation spin up at the wrong rate
 *     and nothing else — which is fine for checking command logic and useless for tuning, so know
 *     which one you are doing
 */
public record RollerSimModel(DCMotor gearbox, MomentOfInertia momentOfInertia) {}
