package frc.robot.scaffold;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.RotationsPerSecond;

import frc.robot.subsystems.exampleRoller.io.ExampleRollerIO.ExampleRollerIOInputs;
import frc.robot.subsystems.exampleRoller.io.ExampleRollerIOSim;
import java.util.Arrays;
import java.util.Comparator;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.controller.PIDController;
import org.wpilib.simulation.RoboRioSim;

/** Step-response harness adapted from the 2026 gain sweep. */
class GainSweepTest {
  private static final double DT = 0.02;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    RoboRioSim.setVInVoltage(12.0);
  }

  @Test
  void sweepRollerProportionalGain() {
    double target = 3.0;
    double[] candidates = {0.05, 0.10, 0.15, 0.20, 0.25, 0.30};
    Result best =
        sweep(kP -> rollerStep(kP, target), target, 3.0, candidates)
            .filter(
                result ->
                    result.riseTimeSeconds() <= 1.5
                        && result.overshootFraction() <= 0.15
                        && result.steadyStateError() <= 0.05)
            .min(Comparator.comparingDouble(Result::riseTimeSeconds))
            .orElseThrow(() -> new AssertionError("No candidate met the response criteria"));
    System.out.printf("Best qualifying kP: %.3f%n", best.kP());
  }

  private static Step rollerStep(double kP, double target) {
    var sim = new ExampleRollerIOSim(new PIDController(kP, 4.0, 0.0));
    var inputs = new ExampleRollerIOInputs();
    sim.updateInputs(inputs);
    sim.setVelocity(RotationsPerSecond.of(target));
    return new Step(
        inputs.motorVelocity.in(RotationsPerSecond),
        () -> {
          sim.updateInputs(inputs);
          return inputs.motorVelocity.in(RotationsPerSecond);
        });
  }

  /**
   * Each candidate supplies a fresh, commanded IOSim with its initial measurement and a sampler
   * that advances it one update per call. Measurements use the same units as the target.
   */
  private static Stream<Result> sweep(
      DoubleFunction<Step> createStep, double target, double duration, double[] gains) {
    return Arrays.stream(gains)
        .mapToObj(
            kP -> {
              Result result = simulate(kP, createStep.apply(kP), target, duration);
              System.out.printf(
                  "kP %.3f | rise (95%%) %.3f s | overshoot %.1f%% | final error %.3f%n",
                  kP,
                  result.riseTimeSeconds(),
                  result.overshootFraction() * 100,
                  result.steadyStateError());
              return result;
            });
  }

  private static Result simulate(double kP, Step response, double target, double duration) {
    double initial = response.initial();
    double step = target - initial;
    assertTrue(Double.isFinite(step) && step != 0.0, "Expected a finite, nonzero step");
    double riseTime = Double.POSITIVE_INFINITY;
    double peak = 0.0;
    double measurement = initial;
    int steps = (int) Math.round(duration / DT);
    for (int i = 1; i <= steps; i++) {
      measurement = response.tick().getAsDouble();
      assertTrue(Double.isFinite(measurement), "Non-finite measurement for kP " + kP);
      double progress = (measurement - initial) / step;
      peak = Math.max(peak, progress);
      if (riseTime == Double.POSITIVE_INFINITY && progress >= 0.95) {
        riseTime = i * DT;
      }
    }
    return new Result(kP, riseTime, Math.max(0.0, peak - 1.0), Math.abs(target - measurement));
  }

  private record Step(double initial, DoubleSupplier tick) {}

  private record Result(
      double kP, double riseTimeSeconds, double overshootFraction, double steadyStateError) {}
}
