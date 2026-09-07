package frc.lib.mechanism.linear;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorIOTalonFXSim;

/**
 * A linear mechanism in simulation, with the real gains running on a simulated Talon.
 *
 * <p>{@code ElevatorSim} carries a height, enforces travel limits and applies a constant gravity
 * force — which is the whole reason simulation is useful here, since gravity is most of what an
 * elevator's loop is fighting.
 *
 * <p>It knows nothing about rigging, so the stage multiplier is folded into the drum radius by
 * {@link LinearGeometry#simEffectiveRadius()}. That is not a fudge: a cascade moves the carriage
 * {@code stages} times the rope travel and applies {@code 1/stages} of the force, which is exactly
 * what a larger effective radius does to both.
 *
 * <p>Travel limits come from the mechanism's own soft limits, so a simulated carriage cannot travel
 * somewhere the real one is forbidden from — a simulation that can is one that hides the bug the
 * bounds exist for.
 */
public class LinearMechanismSim extends LinearMechanism {

  /** Matches the real 50 Hz loop. See {@code RollerMechanismSim} for why this is fixed. */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final MotorIOTalonFXSim simIO;
  private final ElevatorSim sim;
  private final LinearGeometry geometry;

  LinearMechanismSim(MotorConfig config, LinearSettings settings, LinearSimModel model) {
    super(config, settings, new MotorIOTalonFXSim(config));
    this.simIO = (MotorIOTalonFXSim) io;
    this.geometry = settings.geometry();

    double radius = geometry.simEffectiveRadius().in(Meters);
    double massKg = model.carriageMass().in(Kilograms);
    double gearing = config.rotorToMechanismRatio();

    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(model.gearbox(), massKg, radius, gearing),
            model.gearbox(),
            getMinHeight().in(Meters),
            getMaxHeight().in(Meters),
            model.simulateGravity(),
            model.startingHeight().in(Meters));

    // Seed the device with where the carriage actually starts. Without this the Talon reads zero
    // while the physics model sits partway up, and the first commanded move is wrong by the whole
    // starting offset — which is also precisely the real-robot failure that zeroing exists to fix.
    seedDevice(model.startingHeight(), MetersPerSecond.zero());
  }

  @Override
  protected void simulationPeriodic() {
    simIO.updateSupplyVoltage();
    sim.setInputVoltage(simIO.getSimVoltage().in(Volts));
    sim.update(LOOP_PERIOD_SECONDS);

    seedDevice(
        Meters.of(sim.getPositionMeters()), MetersPerSecond.of(sim.getVelocityMetersPerSecond()));
  }

  /** Converts carriage motion into the mechanism rotations the device works in. */
  private void seedDevice(Distance height, edu.wpi.first.units.measure.LinearVelocity velocity) {
    simIO.setMechanismState(
        geometry.rotationsFor(height),
        RotationsPerSecond.of(
            velocity.in(MetersPerSecond) / geometry.travelPerRotation().in(Meters)));
  }
}
