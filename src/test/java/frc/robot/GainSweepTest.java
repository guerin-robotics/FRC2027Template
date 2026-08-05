package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Reference harness for tuning gains in simulation instead of on robot time.
 *
 * <p>This is the pattern {@code /pid-tune} assumes exists. It sweeps a grid of gains against a sim
 * model, scores each run against numeric criteria, and reports the best — so a tuning session
 * produces evidence rather than a feeling.
 *
 * <p><b>How to adapt this for a real mechanism:</b>
 *
 * <ol>
 *   <li>Replace {@link #createSim()} with your mechanism's sim model, or drive its {@code IOSim}
 *       directly so you are testing the real IO layer
 *   <li>Replace the criteria constants with the numbers you actually need
 *   <li>Widen or narrow the sweep grid
 *   <li>Keep the metrics — rise time, overshoot and steady-state error are what turn "feels better"
 *       into a decision you can defend in a review
 * </ol>
 *
 * <p><b>What this does not do:</b> the model here is a generic flywheel with invented inertia, so
 * the winning gains are meaningless for any real mechanism. Sim-validated gains are always a
 * proposal — first hardware test at reduced output, per {@code .claude/rules/00-safety.md}.
 *
 * <p><b>The HAL requirement.</b> Note the {@link BeforeAll} below. WPILib's sim classes clamp their
 * input to the current battery voltage, which reads through {@code RobotController} into the HAL —
 * so a sim-backed test without {@code HAL.initialize()} does not fail, it hard-crashes the JVM with
 * an access violation in {@code wpiHal.dll}. Any test you write that touches a sim class or an
 * {@code IOSim} needs this same setup.
 */
class GainSweepTest {

  private static final double DT = 0.02;
  private static final double SETPOINT_RAD_PER_SEC = 300.0;
  private static final double SIM_DURATION_SECONDS = 3.0;

  // Success criteria. Numbers, not adjectives.
  private static final double MAX_STEADY_STATE_ERROR = 15.0; // rad/s
  private static final double MAX_OVERSHOOT_FRACTION = 0.15; // 15%
  private static final double MAX_RISE_TIME_SECONDS = 1.5;

  @BeforeAll
  static void initializeHal() {
    // Required before touching any WPILib sim class — see the class javadoc.
    assert HAL.initialize(500, 0) : "HAL failed to initialize; sim-backed tests cannot run";

    // Sim classes clamp their input to battery voltage. Pin it so results do not depend on
    // whatever the simulated rail happens to default to.
    RoboRioSim.setVInVoltage(12.0);
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  @Test
  void sweepFindsGainsMeetingCriteria() {
    List<Result> passing = new ArrayList<>();

    for (double kP = 0.005; kP <= 0.10; kP += 0.005) {
      Result result = simulate(kP);
      if (result.meetsCriteria()) {
        passing.add(result);
      }
    }

    assertTrue(
        !passing.isEmpty(),
        "No gain in the swept range met the criteria. Either the criteria are impossible for "
            + "this plant, or the sweep range is wrong — both are useful findings, but the "
            + "sweep should not silently pass.");

    Result best =
        passing.stream()
            .min((a, b) -> Double.compare(a.riseTimeSeconds, b.riseTimeSeconds))
            .orElseThrow();

    System.out.println("********** Gain Sweep Results **********");
    System.out.printf("\tCandidates meeting criteria: %d%n", passing.size());
    System.out.printf("\tBest kP: %.3f%n", best.kP);
    System.out.printf("\tRise time: %.3f s%n", best.riseTimeSeconds);
    System.out.printf("\tOvershoot: %.1f%%%n", best.overshootFraction * 100.0);
    System.out.printf("\tSteady-state error: %.2f rad/s%n", best.steadyStateError);
  }

  /**
   * Velocity feedforward for this plant, derived from the motor rather than guessed: volts per
   * rad/s is nominal voltage over free speed.
   */
  private static final double KV_VOLTS_PER_RAD_PER_SEC =
      DCMotor.getKrakenX60Foc(1).nominalVoltageVolts
          / DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec;

  private static FlywheelSim createSim() {
    return new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.004, 1.0),
        DCMotor.getKrakenX60Foc(1));
  }

  private Result simulate(double kP) {
    FlywheelSim sim = createSim();

    // try-with-resources: PIDController is AutoCloseable because it registers itself with
    // SendableRegistry. The sweep builds one per iteration, so without closing them the
    // registry accumulates a controller per gain tried and holds them for the life of the
    // JVM. Harmless in a short test, a genuine leak in a long-running sweep or on the robot.
    try (PIDController controller = new PIDController(kP, 0.0, 0.0)) {
      double peak = 0.0;
      double riseTime = Double.POSITIVE_INFINITY;
      double velocity = 0.0;

      int steps = (int) (SIM_DURATION_SECONDS / DT);
      for (int i = 0; i < steps; i++) {
        velocity = sim.getAngularVelocityRadPerSec();

        // Feedforward carries the steady state; PID only corrects what it misses. This ordering
        // is the whole doctrine — see docs/characterization-and-tuning.md Part 3. Sweeping kP
        // with no feedforward cannot meet a tight steady-state criterion at any gain, because
        // a pure proportional controller needs standing error to produce standing output.
        double feedforward = KV_VOLTS_PER_RAD_PER_SEC * SETPOINT_RAD_PER_SEC;
        double output = feedforward + controller.calculate(velocity, SETPOINT_RAD_PER_SEC);
        sim.setInputVoltage(clamp(output, -12.0, 12.0));
        sim.update(DT);

        if (velocity > peak) {
          peak = velocity;
        }
        // Rise time: first reach of 95% of setpoint.
        if (riseTime == Double.POSITIVE_INFINITY && velocity >= 0.95 * SETPOINT_RAD_PER_SEC) {
          riseTime = i * DT;
        }
      }

      double steadyStateError = Math.abs(SETPOINT_RAD_PER_SEC - velocity);
      double overshoot = Math.max(0.0, (peak - SETPOINT_RAD_PER_SEC) / SETPOINT_RAD_PER_SEC);

      return new Result(kP, riseTime, overshoot, steadyStateError);
    }
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  /** One sweep sample and its metrics. */
  private record Result(
      double kP, double riseTimeSeconds, double overshootFraction, double steadyStateError) {

    boolean meetsCriteria() {
      return steadyStateError <= MAX_STEADY_STATE_ERROR
          && overshootFraction <= MAX_OVERSHOOT_FRACTION
          && riseTimeSeconds <= MAX_RISE_TIME_SECONDS;
    }
  }
}
