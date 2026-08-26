package frc.robot.subsystems.exampleLift.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.exampleLift.ExampleLiftConstants;

/**
 * Simulation implementation for a linear position-controlled mechanism.
 *
 * <p>{@code ElevatorSim} is the right model: it carries a height in meters, enforces travel limits,
 * and applies a constant gravity force. That last part is what makes sim useful here — gravity is
 * what a lift's control loop mostly fights, and unlike an arm it does not vary with position.
 *
 * <h2>The unit boundary, which is the interesting part of this file</h2>
 *
 * <p>Three unit systems meet here and they must not be confused:
 *
 * <ul>
 *   <li>{@code ElevatorSim} works in <b>meters</b> of carriage travel.
 *   <li>The IO contract is in <b>drum rotations</b>, because that is what the hardware reports.
 *   <li>The team unit is <b>inches</b>, applied by the subsystem.
 * </ul>
 *
 * <p>The conversions below all route through {@code ExampleLiftConstants.inchesToRotations} and its
 * inverse, so {@code DRUM_PITCH_DIAMETER} and {@code STAGE_COUNT} are honoured in sim exactly as
 * they are on hardware. That matters more than it sounds: a sim that used its own drum radius would
 * happily validate a wrong {@code STAGE_COUNT}, which is the single most common way a lift ends up
 * off by an exact integer factor.
 *
 * <h2>Zeroing in sim</h2>
 *
 * <p>{@link #zeroPosition()} works, and the zeroing command runs against this model — the carriage
 * creeps down, hits the bottom of the sim's travel, velocity goes to zero, current rises, and the
 * stall detection fires. That is worth exercising, because it is the routine you least want to
 * debug for the first time with a real carriage in the air.
 *
 * <p>But note what sim cannot tell you: it always <b>starts</b> at zero, so it can never reproduce
 * the failure that a real relative encoder has at boot. Sim proves the routine runs; it cannot
 * prove you needed it.
 */
public class ExampleLiftIOSim implements ExampleLiftIO {

  /** Sim runs at the robot loop period. */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  /**
   * Effective drum radius in meters, as {@code ElevatorSim} understands it.
   *
   * <p>Derived from {@code TRAVEL_PER_ROTATION} rather than from {@code DRUM_PITCH_DIAMETER}
   * directly, so the rigging {@code STAGE_COUNT} is folded in. A cascade that travels twice per
   * drum rotation behaves, as far as the model is concerned, like a drum of twice the radius.
   */
  private static final double EFFECTIVE_DRUM_RADIUS_METERS =
      Units.inchesToMeters(ExampleLiftConstants.TRAVEL_PER_ROTATION.in(Inches)) / (2.0 * Math.PI);

  private final ElevatorSim sim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              ExampleLiftConstants.Sim.MOTOR,
              ExampleLiftConstants.Sim.CARRIAGE_MASS_KG,
              EFFECTIVE_DRUM_RADIUS_METERS,
              ExampleLiftConstants.GEAR_RATIO),
          ExampleLiftConstants.Sim.MOTOR,
          Units.inchesToMeters(ExampleLiftConstants.MIN_TRAVEL_INCHES),
          Units.inchesToMeters(ExampleLiftConstants.MAX_TRAVEL_INCHES),
          ExampleLiftConstants.Sim.SIMULATE_GRAVITY,
          Units.inchesToMeters(ExampleLiftConstants.MIN_TRAVEL_INCHES));

  private final PIDController controller =
      new PIDController(
          ExampleLiftConstants.Sim.KP, ExampleLiftConstants.Sim.KI, ExampleLiftConstants.Sim.KD);

  /**
   * Gravity-aware feedforward.
   *
   * <p>{@code ElevatorFeedforward}, whose kG term is constant — matching what {@code
   * GravityTypeValue.Elevator_Static} does on the real Talon. An {@code ArmFeedforward} here would
   * scale kG by a cosine and quietly teach you a gain that is wrong everywhere except one height.
   */
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ExampleLiftConstants.Sim.KS,
          ExampleLiftConstants.Sim.KG,
          ExampleLiftConstants.Sim.KV,
          ExampleLiftConstants.Sim.KA);

  /** True while a closed-loop position is commanded; false under open-loop voltage or stop. */
  private boolean closedLoop = false;

  /** Goal in carriage meters, matching the units the sim and the feedforward work in. */
  private double goalMeters = Units.inchesToMeters(ExampleLiftConstants.MIN_TRAVEL_INCHES);

  private double appliedVolts = 0.0;

  /**
   * Offset applied to the reported position, in meters.
   *
   * <p>This is what makes {@link #zeroPosition()} mean something in sim. The model's own height is
   * absolute and cannot be reassigned, so instead the offset records where "zero" was declared and
   * every reported position is measured from there — which is exactly what {@code setPosition(0)}
   * does to a real Talon's internal accumulator.
   */
  private double zeroOffsetMeters = 0.0;

  @Override
  public void updateInputs(ExampleLiftIOInputs inputs) {
    if (closedLoop) {
      // Velocity setpoint of zero: this is a position hold, and Motion Magic on the real robot
      // generates its own velocity feedforward from the profile. Matching that here keeps sim and
      // hardware asking the feedforward for the same thing.
      appliedVolts =
          feedforward.calculate(0.0) + controller.calculate(reportedMeters(), goalMeters);
      appliedVolts =
          Math.max(
              ExampleLiftConstants.PEAK_REVERSE_VOLTAGE,
              Math.min(ExampleLiftConstants.PEAK_FORWARD_VOLTAGE, appliedVolts));
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(LOOP_PERIOD_SECONDS);

    inputs.connected = true;
    inputs.motorVoltage = Volts.of(appliedVolts);
    inputs.motorPosition = Rotations.of(metersToRotations(reportedMeters()));
    inputs.motorVelocity =
        RotationsPerSecond.of(metersToRotations(sim.getVelocityMetersPerSecond()));
    inputs.motorStatorAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.motorSupplyAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.motorTemperature = Celsius.of(25.0);

    inputs.closedLoopReference = closedLoop ? metersToRotations(goalMeters) : 0.0;
    inputs.closedLoopError = closedLoop ? metersToRotations(goalMeters - reportedMeters()) : 0.0;

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
    // REPORTED frame, not the sim's absolute frame. Everything the controller compares — the
    // measurement, the goal, the logged reference and error — is measured from wherever zero was
    // last declared, so the offset must be applied in exactly one place: reportedMeters().
    goalMeters = rotationsToMeters(position.in(Rotations));
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void zeroPosition() {
    zeroOffsetMeters = sim.getPositionMeters();
  }

  /** Height as the mechanism believes it, measured from wherever zero was last declared. */
  private double reportedMeters() {
    return sim.getPositionMeters() - zeroOffsetMeters;
  }

  /** Carriage meters to drum rotations, honouring drum diameter and stage count. */
  private static double metersToRotations(double meters) {
    return ExampleLiftConstants.inchesToRotations(Units.metersToInches(meters));
  }

  /** Drum rotations back to carriage meters. */
  private static double rotationsToMeters(double rotations) {
    return Units.inchesToMeters(ExampleLiftConstants.rotationsToInches(rotations));
  }
}
