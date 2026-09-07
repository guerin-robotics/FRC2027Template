package frc.lib.mechanism.rotary;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;

/**
 * The physics a simulated rotary mechanism obeys.
 *
 * <p>{@code SingleJointedArmSim} needs a <b>length and a mass</b> rather than a bare moment of
 * inertia, because gravity torque depends on where the mass actually sits. Both are worth measuring
 * in CAD: a guessed centre of mass makes every simulated conclusion about {@code kG} meaningless,
 * and {@code kG} is most of what an arm's control loop is fighting.
 *
 * <p>The arm length comes from {@link RotarySettings}, so the picture and the physics cannot
 * disagree about how long the arm is.
 *
 * @param gearbox The motors driving this mechanism, sized for leader plus followers. {@code
 *     MotorSpecs.KRAKEN_X60_FOC.gearbox(config.motorCount())}
 * @param mass Mass of the arm
 * @param startingAngle Where the mechanism sits at boot. For an arm with no absolute encoder this
 *     is usually the hard stop it rests against
 * @param simulateGravity False for a mechanism that turns in a horizontal plane — a turret has an
 *     angle but no gravity load, and simulating one would teach a {@code kG} that does not exist
 */
public record RotarySimModel(
    DCMotor gearbox, Mass mass, Angle startingAngle, boolean simulateGravity) {}
