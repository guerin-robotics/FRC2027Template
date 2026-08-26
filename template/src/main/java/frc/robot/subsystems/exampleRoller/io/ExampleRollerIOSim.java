package frc.robot.subsystems.exampleRoller.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.exampleRoller.ExampleRollerConstants;

/**
 * Simulation implementation for a velocity-controlled mechanism.
 *
 * <p>{@code FlywheelSim} is the right model for anything that spins: a rotational inertia driven by
 * a gearbox, with no position state and no travel limits. That is exactly what a roller is.
 *
 * <p><b>This closes a real velocity loop rather than faking one.</b> An earlier version of this
 * scaffold approximated velocity control as {@code appliedVolts = target * 0.1}, which meant the
 * {@code Sim} gains in the constants file were declared and never used, and sim never showed
 * anything about how the mechanism would actually behave. The loop below runs on the roboRIO side
 * at 50 Hz — the real robot's loop runs on the Talon at 1 kHz — so sim will look slightly softer
 * than hardware. That is a known and acceptable difference; the point of sim is to exercise command
 * logic and catch unit errors, not to predict gains.
 *
 * <p>Sim gains are separate from the real ones on purpose. This model has no backlash, no belt
 * stretch, no friction beyond what it declares, and an inertia that is usually a guess. See the
 * {@code Sim} block in {@link ExampleRollerConstants}.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename throughout, then set {@code Sim.MOI} to something defensible. It
 * sets how fast the simulated mechanism accelerates, so everything you conclude from sim depends on
 * it.
 */
public class ExampleRollerIOSim implements ExampleRollerIO {

  /** Sim runs at the robot loop period. */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              ExampleRollerConstants.Sim.MOTOR,
              ExampleRollerConstants.Sim.MOI,
              ExampleRollerConstants.GEAR_RATIO),
          ExampleRollerConstants.Sim.MOTOR);

  private final PIDController controller =
      new PIDController(
          ExampleRollerConstants.Sim.KP,
          ExampleRollerConstants.Sim.KI,
          ExampleRollerConstants.Sim.KD);

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          ExampleRollerConstants.Sim.KS,
          ExampleRollerConstants.Sim.KV,
          ExampleRollerConstants.Sim.KA);

  /** True while a closed-loop velocity is commanded; false under open-loop voltage or stop. */
  private boolean closedLoop = false;

  /** Goal in mechanism rotations per second, matching the units the controller works in. */
  private double goalRotationsPerSec = 0.0;

  private double appliedVolts = 0.0;

  /** Integrated by hand — FlywheelSim carries no position state. */
  private double positionRotations = 0.0;

  @Override
  public void updateInputs(ExampleRollerIOInputs inputs) {
    double measuredRotationsPerSec = sim.getAngularVelocityRPM() / 60.0;

    if (closedLoop) {
      appliedVolts =
          feedforward.calculate(goalRotationsPerSec)
              + controller.calculate(measuredRotationsPerSec, goalRotationsPerSec);
      appliedVolts =
          Math.max(
              ExampleRollerConstants.PEAK_REVERSE_VOLTAGE,
              Math.min(ExampleRollerConstants.PEAK_FORWARD_VOLTAGE, appliedVolts));
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(LOOP_PERIOD_SECONDS);

    measuredRotationsPerSec = sim.getAngularVelocityRPM() / 60.0;
    positionRotations += measuredRotationsPerSec * LOOP_PERIOD_SECONDS;

    inputs.connected = true;
    inputs.motorVoltage = Volts.of(appliedVolts);
    inputs.motorVelocity = RotationsPerSecond.of(measuredRotationsPerSec);
    inputs.motorPosition = Rotations.of(positionRotations);
    inputs.motorStatorAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.motorSupplyAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.motorTemperature = Celsius.of(25.0);

    inputs.closedLoopReference = closedLoop ? goalRotationsPerSec : 0.0;
    inputs.closedLoopError = closedLoop ? goalRotationsPerSec - measuredRotationsPerSec : 0.0;

    // Leave motorTorqueCurrentAmps at zero. WPILib sim models total current draw, not the
    // torque-producing component a FOC controller commands, so anything written here would be
    // fiction that looks real in a log. Do not "helpfully" populate it — the field still exists in
    // the shared AutoLog schema, so the real and sim schemas match.
    //
    // Sticky faults stay false for the same reason: sim has no devices to boot, brown out or
    // overheat, and a fabricated fault would be indistinguishable from a real one in replay.
  }

  @Override
  public void setVoltage(Voltage volts) {
    closedLoop = false;
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    closedLoop = true;
    goalRotationsPerSec = velocity.in(RotationsPerSecond);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
    goalRotationsPerSec = 0.0;
  }
}
