package frc.robot.subsystems.exampleArm.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.exampleArm.ExampleArmConstants;

/**
 * Simulation implementation for a rotary position-controlled mechanism.
 *
 * <p>{@code SingleJointedArmSim} is the right model: it carries an angle, enforces travel limits,
 * and — critically — applies gravity torque that varies with the cosine of the angle. That last
 * part is what makes sim useful here at all, because gravity is the thing an arm's control loop
 * mostly fights.
 *
 * <p>It needs a <b>length and a mass</b> rather than a bare moment of inertia, because gravity
 * torque depends on where the mass actually sits. Both come from {@code ExampleArmConstants.Sim},
 * and both are worth measuring in CAD — a guessed center of mass makes every sim conclusion about
 * kG meaningless.
 *
 * <p><b>Sim closes its loop on the roboRIO side at 50 Hz</b>, whereas the real Talon runs Motion
 * Magic at 1 kHz on the device. Sim will therefore look slightly softer than hardware. That is a
 * known and acceptable difference: the point of sim is to exercise command logic, catch unit errors
 * and see the mechanism move, not to predict gains.
 *
 * <p><b>The travel limits here are the sim's own,</b> derived from the same soft-limit constants
 * the Talon is configured with. Keeping them in step matters — a sim arm that can swing past a
 * bound the real one cannot is a sim that hides the bug you built the bounds for.
 */
public class ExampleArmIOSim implements ExampleArmIO {

  /** Sim runs at the robot loop period. */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              ExampleArmConstants.Sim.MOTOR,
              SingleJointedArmSim.estimateMOI(
                  ExampleArmConstants.Sim.ARM_LENGTH_METERS, ExampleArmConstants.Sim.ARM_MASS_KG),
              ExampleArmConstants.GEAR_RATIO),
          ExampleArmConstants.Sim.MOTOR,
          ExampleArmConstants.GEAR_RATIO,
          ExampleArmConstants.Sim.ARM_LENGTH_METERS,
          Units.degreesToRadians(ExampleArmConstants.REVERSE_SOFT_LIMIT_DEGREES),
          Units.degreesToRadians(ExampleArmConstants.FORWARD_SOFT_LIMIT_DEGREES),
          ExampleArmConstants.Sim.SIMULATE_GRAVITY,
          Units.degreesToRadians(ExampleArmConstants.Sim.STARTING_ANGLE_DEGREES));

  private final PIDController controller =
      new PIDController(
          ExampleArmConstants.Sim.KP, ExampleArmConstants.Sim.KI, ExampleArmConstants.Sim.KD);

  /**
   * Gravity-aware feedforward.
   *
   * <p>{@code ArmFeedforward}, not {@code SimpleMotorFeedforward}: it scales kG by the cosine of
   * the angle, matching what {@code GravityTypeValue.Arm_Cosine} does on the real Talon. Using the
   * simple one here would make sim behave like an elevator and quietly teach you the wrong kG.
   */
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ExampleArmConstants.Sim.KS,
          ExampleArmConstants.Sim.KG,
          ExampleArmConstants.Sim.KV,
          ExampleArmConstants.Sim.KA);

  /** True while a closed-loop position is commanded; false under open-loop voltage or stop. */
  private boolean closedLoop = false;

  /** Goal in mechanism radians, matching the units the sim and the feedforward work in. */
  private double goalRadians =
      Units.degreesToRadians(ExampleArmConstants.Sim.STARTING_ANGLE_DEGREES);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ExampleArmIOInputs inputs) {
    if (closedLoop) {
      // The feedforward's angle argument is measured from HORIZONTAL, which is what makes the
      // cosine term correct. GRAVITY_HORIZONTAL_OFFSET_DEGREES carries the same correction the
      // real config applies via GravityArmPositionOffset — keeping the two in step is what stops
      // sim and hardware disagreeing about where gravity peaks.
      double angleFromHorizontalRadians =
          sim.getAngleRads()
              + Units.degreesToRadians(ExampleArmConstants.GRAVITY_HORIZONTAL_OFFSET_DEGREES);

      appliedVolts =
          feedforward.calculate(angleFromHorizontalRadians, 0.0)
              + controller.calculate(sim.getAngleRads(), goalRadians);
      appliedVolts =
          Math.max(
              ExampleArmConstants.PEAK_REVERSE_VOLTAGE,
              Math.min(ExampleArmConstants.PEAK_FORWARD_VOLTAGE, appliedVolts));
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(LOOP_PERIOD_SECONDS);

    inputs.motorConnected = true;
    inputs.encoderConnected = true;
    inputs.motorVoltage = Volts.of(appliedVolts);
    inputs.motorPosition = Radians.of(sim.getAngleRads());
    inputs.motorVelocity = RadiansPerSecond.of(sim.getVelocityRadPerSec());
    inputs.encoderAbsolutePosition = Radians.of(sim.getAngleRads());
    inputs.motorStatorAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.motorSupplyAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.motorTemperature = Celsius.of(25.0);

    inputs.closedLoopReference = closedLoop ? Units.radiansToRotations(goalRadians) : 0.0;
    inputs.closedLoopError =
        closedLoop ? Units.radiansToRotations(goalRadians - sim.getAngleRads()) : 0.0;

    // Leave motorTorqueCurrentAmps at zero. WPILib sim models total current draw, not the
    // torque-producing component a FOC controller commands, so anything written here would be
    // fiction that looks real in a log. The field still exists in the shared AutoLog schema, so the
    // real and sim schemas match.
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
  public void setPosition(Angle position) {
    closedLoop = true;
    goalRadians = position.in(Radians);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  /**
   * Convenience for tests and for reading the sim state in mechanism rotations.
   *
   * <p>Not part of {@link ExampleArmIO} — nothing in the robot code may call it, because it would
   * not exist on hardware.
   */
  public Angle getSimulatedPosition() {
    return Rotations.of(Units.radiansToRotations(sim.getAngleRads()));
  }
}
