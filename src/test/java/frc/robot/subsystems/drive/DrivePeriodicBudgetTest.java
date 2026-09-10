package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import java.util.Arrays;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * Guards {@code Drive.periodic()} against becoming expensive enough to threaten the 20 ms loop
 * budget.
 *
 * <p>Drive and Vision were the two dominators of {@code robotPeriodic} on the 2026 robot, which ran
 * at roughly 30 Hz against a 20 ms budget for an entire season. {@link
 * frc.lib.util.LoopTimeMonitorTest} pins down the watchdog that reports that condition; this test
 * attacks the other end, keeping the largest known contributor from quietly growing in the first
 * place.
 *
 * <p><b>What this can and cannot tell you.</b> It measures wall-clock time on whatever machine runs
 * the test — a dev laptop or CI runner, not a roboRIO. A RIO is far slower, so passing here is
 * necessary, not sufficient. What it reliably catches is a <i>structural</i> regression: an
 * accidental O(n²), a blocking or retrying call added to the hot path, a per-loop allocation of
 * something large, or a heavyweight object rebuilt every cycle instead of once in the constructor.
 * Those show up as multiples, not percentages, which is why the threshold below can afford to be
 * generous enough never to flake.
 *
 * <p><b>The real check still happens on the robot.</b> Watch {@code LoopTiming/AverageMs} from the
 * first day the 2027 robot drives, per {@code .claude/rules/00-safety.md}. This test is an early
 * warning, not a substitute.
 *
 * <p>Timings are printed on every run, so a reviewer can see the actual margin rather than only
 * whether an assertion passed.
 */
class DrivePeriodicBudgetTest {

  /** Discarded, so JIT compilation and first-call setup do not land in the measurement. */
  private static final int WARMUP_ITERATIONS = 200;

  private static final int MEASURED_ITERATIONS = 500;

  /**
   * Ceiling for the median loop, in milliseconds.
   *
   * <p>Measured on a dev machine when this test was written: ~0.10 ms median running alone, ~0.34
   * ms median running inside the full suite — the same code, 3x apart, purely from JVM and machine
   * state. That spread is the argument for the ~15-50x headroom here. A threshold set close to the
   * baseline would flake, and a flaky test gets deleted rather than investigated. Anything that
   * actually trips this is a change in the <i>shape</i> of the work, not a slow afternoon.
   *
   * <p>It is still only a quarter of the full 20 ms budget, so a Drive that trips this has become
   * expensive enough to matter even before the RIO's slowdown factor is applied.
   */
  private static final double MEDIAN_BUDGET_MS = 5.0;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);

    // Drive.periodic() takes a different (cheaper) path while disabled — measure the enabled one.
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // See DriveToPoseSimTest for why the robot-base check has to be disabled here.
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  @AfterEach
  void cancelScheduledCommands() {
    // CommandScheduler is a JVM-wide singleton shared with every other test class in this Gradle
    // run — see DriveToPoseSimTest's javadoc. A command left running here would land directly in
    // another test's timing measurement.
    CommandScheduler.getInstance().cancelAll();
  }

  @Test
  void drivePeriodicStaysWellInsideTheLoopBudget() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

    for (int i = 0; i < WARMUP_ITERATIONS; i++) {
      CommandScheduler.getInstance().run();
    }

    double[] samplesMs = new double[MEASURED_ITERATIONS];
    for (int i = 0; i < MEASURED_ITERATIONS; i++) {
      long startNanos = System.nanoTime();
      CommandScheduler.getInstance().run();
      samplesMs[i] = (System.nanoTime() - startNanos) / 1_000_000.0;
    }

    double meanMs = Arrays.stream(samplesMs).average().orElseThrow();
    Arrays.sort(samplesMs);
    double medianMs = samplesMs[MEASURED_ITERATIONS / 2];
    double p95Ms = samplesMs[(int) (MEASURED_ITERATIONS * 0.95)];
    double maxMs = samplesMs[MEASURED_ITERATIONS - 1];

    System.out.println("********** Drive.periodic() Loop Budget **********");
    System.out.printf("\tmedian: %.4f ms  (budget %.1f ms)%n", medianMs, MEDIAN_BUDGET_MS);
    System.out.printf("\tmean:   %.4f ms%n", meanMs);
    System.out.printf("\tp95:    %.4f ms%n", p95Ms);
    System.out.printf("\tmax:    %.4f ms  (GC pauses land here)%n", maxMs);

    // Median rather than mean or max: a single GC pause mid-run should not fail the build, and
    // the question being asked is "is the steady-state cost still small", not "was any one loop
    // ever slow". Pause behaviour is a JVM-flag concern, covered in .claude/rules/04-build.md.
    assertTrue(
        medianMs < MEDIAN_BUDGET_MS,
        "Drive.periodic() median rose to "
            + medianMs
            + " ms (ceiling "
            + MEDIAN_BUDGET_MS
            + " ms). That is more than an order of magnitude above the measured baseline, so it "
            + "indicates a structural change in the hot path — an added blocking call, a per-loop "
            + "allocation, or work that scales with something it did not before. On a roboRIO "
            + "this is several times worse.");
  }
}
