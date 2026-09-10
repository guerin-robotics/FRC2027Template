package frc.lib.mechanism.linear;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.util.MotorSpecs;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * An elevator against constant gravity, with the configured gains running on a simulated Talon.
 *
 * <p>What is worth pinning here is what an elevator gets wrong quietly: it arrives at the commanded
 * height, it <i>stays</i> there once it has (the {@code kG} claim, not the {@code kP} one), it
 * starts where the model says rather than at zero, a setpoint past the top of travel is clamped,
 * and the zeroing routine actually drives the carriage to the stop and detects the stall.
 *
 * <h2>What simulation cannot tell you here</h2>
 *
 * <p>The zeroing routine's <i>answer</i> — that {@code zeroAt} correctly re-datums the encoder — is
 * not testable in simulation, and no test below claims it is. The simulated device's position is
 * overwritten from the physics model every loop, and the physics model is always right about where
 * the carriage is, so there is no offset to correct. On the real robot that offset is the entire
 * point of the routine. What simulation does verify is the part with the moving parts: that the
 * carriage travels down, that the stall is detected, and that the command finishes rather than
 * running out its timeout.
 *
 * <p>Like the other sim tests, this sleeps — Phoenix's device simulation runs on wall-clock time.
 * See {@code RollerMechanismSimTest.run}.
 */
class LinearMechanismSimTest {

  private static final CANBus BUS = new CANBus("rio");

  /** A 1.5-inch drum on a two-stage cascade, 12:1 to the motor. */
  private static final LinearGeometry GEOMETRY = new LinearGeometry(Inches.of(1.5), 2);

  private static final Distance MIN_HEIGHT = Inches.of(0);
  private static final Distance MAX_HEIGHT = Inches.of(40);
  private static final Distance STARTING_HEIGHT = Inches.of(0);
  private static final Distance TARGET = Inches.of(24);
  private static final Distance TOLERANCE = Inches.of(0.5);

  private static final long LOOP_PERIOD_MILLIS = 20;
  /** Generous cap. Settling normally takes well under half of this. */
  private static final int MAX_SETTLE_LOOPS = 300;

  /** How many consecutive loops at goal count as settled rather than passing through. */
  private static final int HOLD_LOOPS = 10;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();

    // WPILib's Timer reads RobotController.getTime(), which a running robot program advances once
    // per loop from IterativeRobotBase. There is no robot base in a unit test, so that clock is
    // frozen and every WaitCommand and every withTimeout waits forever — a zeroing routine sits at
    // its settle step for as long as you are willing to watch. Pointing the time source at the FPGA
    // clock is what makes command timing work here at all.
    RobotController.setTimeSource(RobotController::getFPGATime);
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  private static MotorConfig config(int canId) {
    return MotorConfig.builder("SimLift" + canId, MechanismKind.LINEAR)
        .canId(canId, BUS)
        .sensorToMechanismRatio(12.0)
        .softLimits(
            GEOMETRY.rotationsFor(MIN_HEIGHT).in(edu.wpi.first.units.Units.Rotations),
            GEOMETRY.rotationsFor(MAX_HEIGHT).in(edu.wpi.first.units.Units.Rotations))
        .supplyCurrentLimit(40.0)
        .statorCurrentLimit(60.0)
        // Amps. kG is the current that holds the carriage against gravity, constant at every
        // height — which is what Elevator_Static compensation means.
        .gains(new Gains(500.0, 0.0, 30.0, 0.5, 0.0, 0.0, 11.0))
        .motionProfile(
            MotionProfile.of(RotationsPerSecond.of(3.0), RotationsPerSecondPerSecond.of(10.0)))
        .build();
  }

  private static LinearSettings settings() {
    return new LinearSettings(TOLERANCE, GEOMETRY);
  }

  private static LinearMechanismSim newLift(int canId, Distance startingHeight) {
    return LinearMechanism.sim(
        config(canId),
        settings(),
        new LinearSimModel(
            MotorSpecs.KRAKEN_X60_FOC.gearbox(1), Pounds.of(15), startingHeight, true));
  }

  private static void sleepOneLoop() {
    try {
      Thread.sleep(LOOP_PERIOD_MILLIS);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("interrupted while stepping the simulation", e);
    }
  }

  private static void run(LinearMechanismSim lift, int loops) {
    for (int i = 0; i < loops; i++) {
      lift.periodic();
      sleepOneLoop();
    }
  }

  /**
   * Runs until the carriage has been at its goal for {@link #HOLD_LOOPS} consecutive loops.
   *
   * <p>Waiting for a condition rather than running a fixed number of loops is not just tidier here,
   * it is necessary. The physics steps a fixed 20 ms per call while Phoenix's device simulation
   * advances on wall-clock time, so how much control the device gets per physics step depends on
   * how loaded the machine is. A fixed loop count that passes on an idle laptop fails on a busy CI
   * runner, and the failure looks like a mechanism that cannot reach its setpoint.
   *
   * <p>Requiring the condition to hold for several consecutive loops is what makes this an arrival
   * test rather than a pass-through test — a carriage overshooting its goal is momentarily "at" it.
   *
   * @return true if it settled within the budget
   */
  private static boolean settles(LinearMechanismSim lift) {
    int consecutive = 0;
    for (int i = 0; i < MAX_SETTLE_LOOPS; i++) {
      lift.periodic();
      sleepOneLoop();
      consecutive = lift.isAtPosition() ? consecutive + 1 : 0;
      if (consecutive >= HOLD_LOOPS) {
        return true;
      }
    }
    return false;
  }

  @Test
  void theCarriageStartsWhereTheModelSaysItStarts() {
    LinearMechanismSim lift = newLift(30, Inches.of(12));
    run(lift, 3);
    assertEquals(
        12.0,
        lift.getHeight().in(Inches),
        1.0,
        "the simulated device should be seeded with the model's starting height");
  }

  @Test
  void aCommandedHeightIsReachedAndHeld() {
    LinearMechanismSim lift = newLift(31, STARTING_HEIGHT);
    lift.setPosition(TARGET);
    boolean settled = settles(lift);

    System.out.println("********** Linear Sim: move to 24 in **********");
    System.out.println("\tmeasured " + lift.getHeight().in(Inches) + " in");
    System.out.println("\ttorque   " + lift.getTorqueCurrent());

    assertTrue(
        settled,
        "lift should reach " + TARGET.in(Inches) + " in; measured " + lift.getHeight().in(Inches));

    // Still there a second later. An elevator with a working position loop but no gravity
    // compensation arrives and then sinks, which a short test would miss entirely.
    run(lift, 50);
    assertTrue(
        lift.isAtPosition(),
        "lift should still be holding "
            + TARGET.in(Inches)
            + " in; measured "
            + lift.getHeight().in(Inches));
  }

  @Test
  void aGoalAboveTheTopOfTravelIsClamped() {
    LinearMechanismSim lift = newLift(32, STARTING_HEIGHT);
    lift.setPosition(MAX_HEIGHT.plus(Inches.of(15)));

    assertEquals(
        MAX_HEIGHT.in(Inches),
        lift.getGoalPosition().in(Inches),
        1e-6,
        "a goal above the top of travel should be clamped to it");

    settles(lift);
    assertTrue(
        lift.getHeight().in(Inches) < MAX_HEIGHT.in(Inches) + 0.5,
        "the carriage must not travel past the top; measured " + lift.getHeight().in(Inches));
  }

  @Test
  void comingBackDownWorksToo() {
    // Gravity helps downward, which is the direction an over-large kG makes overshoot.
    LinearMechanismSim lift = newLift(33, STARTING_HEIGHT);
    lift.setPosition(Inches.of(30));
    assertTrue(settles(lift), "precondition: should have reached 30 in");

    lift.setPosition(Inches.of(6));
    assertTrue(
        settles(lift),
        "lift should come back down to 6 in; measured " + lift.getHeight().in(Inches));
  }

  @Test
  void theZeroingRoutineDrivesToTheStopAndDetectsTheStall() {
    // Not a test of the offset zeroAt applies — see the class javadoc for why simulation cannot
    // check that. This is a test of the moving parts: it goes down, it notices it has stopped, and
    // it finishes rather than running out its timeout.
    LinearMechanismSim lift = newLift(34, Inches.of(20));
    LinearSubsystem subsystem = new LinearSubsystem(lift);

    Command zeroing =
        LinearCommands.zeroAtHardStop(
            subsystem,
            Amps.of(-12),
            Seconds.of(0.3),
            InchesPerSecond.of(0.5),
            Seconds.of(4.0),
            MIN_HEIGHT);

    CommandScheduler scheduler = CommandScheduler.getInstance();

    scheduler.schedule(zeroing);

    int loops = 0;
    while (zeroing.isScheduled() && loops < 300) {
      scheduler.run();
      sleepOneLoop();
      loops++;
    }

    System.out.println("********** Linear Sim: zeroing **********");
    System.out.println("\tfinished after " + loops + " loops");
    System.out.println("\theight " + lift.getHeight().in(Inches) + " in");

    assertTrue(
        loops < 300, "zeroing should finish on its own; it ran " + loops + " loops without ending");
    assertEquals(
        MIN_HEIGHT.in(Inches),
        lift.getHeight().in(Inches),
        1.0,
        "the carriage should have travelled down to the stop");

    scheduler.cancelAll();
  }
}
