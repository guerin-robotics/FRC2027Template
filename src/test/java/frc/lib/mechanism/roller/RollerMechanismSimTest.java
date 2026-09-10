package frc.lib.mechanism.roller;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
 * Proves the simulation loop is actually closed.
 *
 * <p>This is the test that would have caught the whole framework being wired up backwards. The
 * simulated Talon runs the configured gains and the configured Motion Magic profile against
 * WPILib's flywheel physics; if the ratio conversion in {@code MotorIOTalonFXSim} were wrong, or
 * the physics stepped before the applied voltage were read, the mechanism would sit at zero or run
 * away, and both show up here.
 *
 * <p>It is deliberately not a claim that these gains are good. It is a claim that a gain has an
 * effect, in the right direction, with the right magnitude of authority — which is the property a
 * tuning session depends on and the one a hand-written sim IO does not give you.
 */
class RollerMechanismSimTest {

  private static final CANBus BUS = new CANBus("rio");
  private static final AngularVelocity TARGET = RPM.of(3000);
  private static final AngularVelocity TOLERANCE = RPM.of(120);

  /** Matches the fixed physics step in {@link RollerMechanismSim} and the real 50 Hz loop. */
  private static final long LOOP_PERIOD_MILLIS = 20;

  /** Generous cap. Spin-up normally takes well under half of this. */
  private static final int MAX_SPIN_UP_LOOPS = 250;

  /** How many consecutive loops at speed count as settled rather than passing through. */
  private static final int HOLD_LOOPS = 10;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);
    // Phoenix will not apply output from a simulated device while the robot is disabled, exactly as
    // on the real robot. Without this every mechanism sits at zero and every test here fails in a
    // way that looks like broken physics.
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  private static MotorConfig config(int canId) {
    return MotorConfig.builder("SimRoller" + canId, MechanismKind.ROLLER)
        .canId(canId, BUS)
        .sensorToMechanismRatio(1.0)
        .supplyCurrentLimit(60.0)
        .statorCurrentLimit(80.0)
        .gains(new Gains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(400.0)))
        .build();
  }

  private static RollerMechanismSim newRoller(int canId) {
    return RollerMechanism.sim(
        config(canId),
        RollerSettings.of(TOLERANCE),
        new RollerSimModel(MotorSpecs.KRAKEN_X60_FOC.gearbox(1), KilogramSquareMeters.of(0.004)));
  }

  /**
   * Runs the mechanism for {@code loops} cycles of the robot loop.
   *
   * <p><b>This sleeps, and it has to.</b> Phoenix's device simulation advances on wall-clock time,
   * not on the FPGA simulation clock — {@code SimHooks.stepTiming} does nothing for it, and a tight
   * loop with no delay leaves the simulated Talon's internal control loop never ticking, so the
   * mechanism sits at exactly zero in a way that looks identical to broken physics. That cost about
   * an hour to find once; it is written down here so it costs nobody else one.
   *
   * <p>This is the price of running the real gains against a real device model. {@code ModuleIOSim}
   * needs no sleeping because it models everything in Java and never touches a Phoenix device.
   */
  private static void run(RollerMechanismSim roller, int loops) {
    for (int i = 0; i < loops; i++) {
      roller.periodic();
      sleepOneLoop();
    }
  }

  private static void sleepOneLoop() {
    try {
      Thread.sleep(LOOP_PERIOD_MILLIS);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("interrupted while stepping the simulation", e);
    }
  }

  /**
   * Runs until the mechanism has been at its goal for {@link #HOLD_LOOPS} consecutive loops.
   *
   * <p>Waiting on a condition rather than running a fixed number of loops is necessary, not tidier.
   * The physics steps a fixed 20 ms per call while Phoenix's device simulation advances on
   * wall-clock time, so how much control the device gets per physics step depends on how loaded the
   * machine is. A fixed loop count that passes on an idle laptop fails on a busy CI runner, and the
   * failure reads as a mechanism that cannot reach its setpoint.
   *
   * <p>Requiring several consecutive loops is what makes this an arrival test rather than a
   * pass-through test, since a mechanism overshooting is momentarily at its goal on the way past.
   *
   * @return true if it settled within the budget
   */
  private static boolean settles(RollerMechanismSim roller) {
    int consecutive = 0;
    for (int i = 0; i < MAX_SPIN_UP_LOOPS; i++) {
      roller.periodic();
      sleepOneLoop();
      consecutive = roller.isAtVelocity() ? consecutive + 1 : 0;
      if (consecutive >= HOLD_LOOPS) {
        return true;
      }
    }
    return false;
  }

  @Test
  void aCommandedVelocityIsReached() {
    RollerMechanismSim roller = newRoller(40);
    roller.setVelocity(TARGET);
    boolean settled = settles(roller);

    System.out.println("********** Roller Sim: spin-up **********");
    System.out.println("\tgoal " + TARGET.in(RPM) + " RPM");
    System.out.println("\tmeasured " + roller.getVelocity().in(RPM) + " RPM");
    System.out.println("\tstator " + roller.getStatorCurrent());

    assertTrue(
        settled,
        "Roller should reach "
            + TARGET.in(RPM)
            + " RPM within "
            + TOLERANCE.in(RPM)
            + "; measured "
            + roller.getVelocity().in(RPM));
  }

  @Test
  void itStartsStoppedAndReportsSo() {
    RollerMechanismSim roller = newRoller(41);
    run(roller, 5);

    // isAtVelocity compares against the goal, so a stopped mechanism with a zero goal is "there".
    // Worth pinning: a naive implementation that compares against a fixed target instead would
    // report true here for the wrong reason and false everywhere else.
    assertTrue(roller.isAtVelocity(), "a stopped roller with no goal is at its goal");
    assertTrue(Math.abs(roller.getVelocity().in(RPM)) < 1.0);
  }

  @Test
  void stoppingClearsTheGoalAndSpinsDown() {
    RollerMechanismSim roller = newRoller(42);
    roller.setVelocity(TARGET);
    assertTrue(settles(roller), "precondition: should have reached speed");

    roller.stop();
    assertTrue(
        roller.getGoalVelocity().in(RPM) == 0.0,
        "stop() must clear the goal, or the jam detector keeps comparing against a setpoint"
            + " nobody is commanding");

    run(roller, 5);
    assertFalse(
        roller.isAtVelocity(),
        "immediately after a stop the mechanism is still spinning, so it is not at its new goal");
  }

  @Test
  void aReversedCommandProducesReversedMotion() {
    // Sign errors are the cheapest thing to get wrong and among the most expensive to find on a
    // real robot, where the first symptom is a game piece going the wrong way.
    RollerMechanismSim roller = newRoller(43);
    roller.setVelocity(TARGET.unaryMinus());
    settles(roller);

    assertTrue(
        roller.getVelocity().in(RPM) < -1000.0,
        "commanding -3000 RPM should produce clearly negative motion; got "
            + roller.getVelocity().in(RPM));
  }

  @Test
  void aMechanismWithNoJamDetectionNeverReportsAJam() {
    RollerMechanismSim roller = newRoller(44);
    roller.setVelocity(TARGET);
    settles(roller);

    // RollerSettings.of() configures no jam detection, and "this cannot jam" has to mean the
    // detector is absent rather than merely quiet.
    assertFalse(roller.isJammed());
    assertTrue(roller.getJamCount() == 0);
  }
}
