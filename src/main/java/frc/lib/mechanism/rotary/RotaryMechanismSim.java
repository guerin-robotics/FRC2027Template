package frc.lib.mechanism.rotary;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorIOTalonFXSim;

/**
 * A rotary mechanism in simulation, with the real gains running on a simulated Talon.
 *
 * <p>{@code SingleJointedArmSim} is the right model: it carries an angle, enforces travel limits,
 * and applies gravity torque that varies with the cosine of that angle. The last part is what makes
 * simulation useful here at all.
 *
 * <p>The travel limits are the mechanism's own soft limits, taken from the same config the Talon
 * was given. Keeping them in step matters — a simulated arm that can swing past a bound the real
 * one cannot is a simulation that hides the bug the bounds exist for.
 *
 * <p>Unlike the 2026 scaffold, the loop closes on the simulated <i>device</i> rather than in a
 * roboRIO-side {@code PIDController}. There is no second set of sim gains to keep in step with the
 * real ones, and no quiet lesson about the wrong {@code kG}.
 */
public class RotaryMechanismSim extends RotaryMechanism {

  /** Matches the real 50 Hz loop. See {@code RollerMechanismSim} for why this is fixed. */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final MotorIOTalonFXSim simIO;
  private final SingleJointedArmSim sim;

  RotaryMechanismSim(MotorConfig config, RotarySettings settings, RotarySimModel model) {
    super(config, settings, new MotorIOTalonFXSim(config));
    this.simIO = (MotorIOTalonFXSim) io;

    double armLengthMeters = settings.armLength().in(Meters);
    double gearing = config.rotorToMechanismRatio();
    double momentOfInertia =
        SingleJointedArmSim.estimateMOI(armLengthMeters, model.mass().in(Kilograms));

    sim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(model.gearbox(), momentOfInertia, gearing),
            model.gearbox(),
            gearing,
            armLengthMeters,
            Rotations.of(limits().reverseRotations()).in(Radians),
            Rotations.of(limits().forwardRotations()).in(Radians),
            model.simulateGravity(),
            model.startingAngle().in(Radians));

    // Seed the device with where the mechanism actually starts. Without this the Talon reads zero
    // while the physics model sits at the hard stop, and the first commanded move is wrong by the
    // whole starting offset.
    simIO.setMechanismState(model.startingAngle(), RadiansPerSecond.zero());
  }

  @Override
  protected void simulationPeriodic() {
    // The device needs a supply voltage before it reports an applied output, and that output is
    // the input to the physics.
    simIO.updateSupplyVoltage();
    sim.setInputVoltage(simIO.getSimVoltage().in(Volts));
    sim.update(LOOP_PERIOD_SECONDS);

    // SingleJointedArmSim carries position directly, so unlike the roller there is nothing to
    // integrate and no accumulated integration error to worry about.
    simIO.setMechanismState(
        Radians.of(sim.getAngleRads()), RadiansPerSecond.of(sim.getVelocityRadPerSec()));
  }
}
