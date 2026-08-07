package frc.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * Tests for the 50 Hz loop-time watchdog.
 *
 * <p>{@link LoopTimeMonitor} is the guard against the 2026 season's defining bug: the robot ran at
 * roughly 30 Hz against a 20 ms budget for the entire season and nothing surfaced it until match
 * logs were analyzed months later. A watchdog that silently stops detecting overruns puts the team
 * straight back in that position, so its behavior is worth pinning down.
 *
 * <p><b>Timing is controlled, not measured.</b> {@link SimHooks#pauseTiming()} freezes the FPGA
 * clock and {@link SimHooks#stepTiming} advances it by an exact amount, so a "17 ms loop" here is
 * exactly 17 ms rather than however long the test machine happened to take. That makes these
 * assertions deterministic — this test says nothing about how fast any real code runs, which is
 * {@link frc.robot.subsystems.drive.DrivePeriodicBudgetTest}'s job.
 */
class LoopTimeMonitorTest {

  /** {@code LoopTimeMonitor.WINDOW} — the rolling average is over this many samples. */
  private static final int WINDOW = 50;

  private LoopTimeMonitor monitor;

  @BeforeAll
  static void initializeHal() {
    // LoopTimeMonitor reads RobotController.getFPGATime(), and Alert construction touches HAL.
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  @BeforeEach
  void freezeClock() {
    SimHooks.pauseTiming();
    SimHooks.restartTiming();
    monitor = new LoopTimeMonitor();
  }

  @AfterEach
  void unfreezeClock() {
    SimHooks.resumeTiming();
  }

  /** Simulates {@code count} loops that each take exactly {@code loopMs} milliseconds. */
  private void runLoops(int count, double loopMs) {
    for (int i = 0; i < count; i++) {
      monitor.begin();
      SimHooks.stepTiming(loopMs / 1000.0);
      monitor.end();
    }
  }

  @Test
  void averageReflectsActualLoopDuration() {
    runLoops(WINDOW, 10.0);
    assertEquals(
        10.0,
        monitor.getAverageMs(),
        0.5,
        "A steady 10 ms loop should average 10 ms; if this drifts, the rolling-average math is "
            + "wrong and every LoopTiming/AverageMs number in every log is wrong with it");
  }

  @Test
  void aHealthyLoopIsNotOverBudget() {
    runLoops(WINDOW, 10.0);
    assertFalse(monitor.isOverBudget(), "A 10 ms loop is comfortably inside the 20 ms budget");
  }

  @Test
  void aSustainedSlowLoopIsReportedOverBudget() {
    // 33 ms is roughly the 30 Hz the 2026 robot actually ran at. This is the case the whole
    // class exists to catch.
    runLoops(WINDOW, 33.0);
    assertTrue(
        monitor.isOverBudget(),
        "A sustained 33 ms loop (~30 Hz) must be reported over budget — this is precisely the "
            + "2026 failure that went undetected for a season");
  }

  @Test
  void startupSpikeDoesNotTripTheAlert() {
    // Deliberate design decision in LoopTimeMonitor: the alert is suppressed until the rolling
    // window has filled, so the slow first loops at startup and first enable do not cry wolf.
    runLoops(WINDOW - 1, 100.0);
    assertFalse(
        monitor.isOverBudget(),
        "The alert must stay suppressed until the window fills, or every boot raises it");
  }

  @Test
  void alertThresholdSitsAboveTheBudgetOnPurpose() {
    // BUDGET_MS is 20 and ALERT_THRESHOLD_MS is 25. A loop in that gap is over budget but not
    // yet alert-worthy, so a robot hovering at 22 ms does not spam the driver station. If
    // someone collapses the two constants, this is what tells them it was deliberate.
    runLoops(WINDOW, 22.0);
    assertFalse(
        monitor.isOverBudget(),
        "22 ms is over the 20 ms budget but under the 25 ms alert threshold — the gap between "
            + "those two constants is intentional");
  }

  @Test
  void recoveringFromSlowLoopsClearsTheAlert() {
    runLoops(WINDOW, 33.0);
    assertTrue(monitor.isOverBudget(), "precondition: sustained slow loops raise the alert");

    // A full window of healthy loops must push the slow samples back out of the average.
    runLoops(WINDOW, 8.0);
    assertFalse(
        monitor.isOverBudget(),
        "Once the robot recovers, the alert must clear — a watchdog that latches on forever is "
            + "one the drive team learns to ignore");
  }

  @Test
  void mixedLoopTimesAverageOutRatherThanTrackingThePeak() {
    // Half the window fast, half slow. The average, not the worst sample, is what drives the
    // alert — a single 100 ms hiccup mid-match should not be reported as a sustained problem.
    runLoops(WINDOW / 2, 5.0);
    runLoops(WINDOW / 2, 15.0);
    assertEquals(
        10.0,
        monitor.getAverageMs(),
        0.5,
        "The rolling average should be the mean of the window, not weighted toward the peak");
    assertFalse(monitor.isOverBudget(), "A 10 ms average is healthy regardless of the spread");
  }
}
