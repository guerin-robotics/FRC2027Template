package frc.lib.mechanism.roller;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorIOTalonFXSim;

/**
 * A roller in simulation, with the real gains running on a simulated Talon.
 *
 * <p>The physics lives here and the device lives in {@link MotorIOTalonFXSim}, which is what makes
 * a simulated tuning session worth anything: the closed loop, the Motion Magic profile, the current
 * limits and the torque-current clamps are the configured ones, so a gain that works here has a
 * fighting chance of working on the robot.
 *
 * <p>What simulation will not tell you is whether the inertia is right, whether the mechanism
 * binds, or how it behaves with a game piece in it. Those still need a real robot.
 */
public class RollerMechanismSim extends RollerMechanism {

  /**
   * The step the physics advances by, in seconds.
   *
   * <p>Fixed rather than measured off the FPGA clock, matching {@code ModuleIOSim}. A measured step
   * ties the simulation to wall-clock time, which makes a sim-backed unit test either slow (it has
   * to sleep) or wrong (a tight loop measures a step near zero and the mechanism never moves). At
   * 20 ms this is also what the real loop actually runs at.
   */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final MotorIOTalonFXSim simIO;
  private final FlywheelSim sim;

  private Angle simPosition = Radians.zero();
  private AngularVelocity lastVelocity = RadiansPerSecond.zero();

  RollerMechanismSim(MotorConfig config, RollerSettings settings, RollerSimModel model) {
    super(config, settings, new MotorIOTalonFXSim(config));
    this.simIO = (MotorIOTalonFXSim) io;

    if (model.momentOfInertia().in(KilogramSquareMeters) <= 0.0) {
      throw new IllegalArgumentException(
          "Moment of inertia for '"
              + config.name()
              + "' must be greater than zero. A massless flywheel reaches any speed instantly,"
              + " which makes every gain look perfect.");
    }

    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                model.gearbox(),
                model.momentOfInertia().in(KilogramSquareMeters),
                config.rotorToMechanismRatio()),
            model.gearbox());
  }

  @Override
  protected void simulationPeriodic() {
    // Order matters. The device needs a supply voltage before it will report an applied output,
    // and the applied output is the input to the physics.
    simIO.updateSupplyVoltage();
    sim.setInputVoltage(simIO.getSimVoltage().in(Volts));
    sim.update(LOOP_PERIOD_SECONDS);

    // FlywheelSim models velocity, not position. Integrating trapezoidally rather than
    // rectangularly matters here: a roller is usually accelerating when anyone is looking at it,
    // and rectangular integration accumulates a visible position error over a spin-up.
    AngularVelocity velocity = sim.getAngularVelocity();
    simPosition =
        simPosition.plus(
            lastVelocity.plus(velocity).times(Seconds.of(LOOP_PERIOD_SECONDS)).times(0.5));
    lastVelocity = velocity;

    simIO.setMechanismState(simPosition, velocity);
  }
}
