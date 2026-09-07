package frc.lib.mechanism.rotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.MotorSpecs;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * A rotary mechanism against real gravity, with the configured gains running on a simulated Talon.
 *
 * <p>The properties worth pinning here are the ones an arm gets wrong in ways a position plot does
 * not reveal: it arrives at the commanded angle, it <i>stays</i> there once it has (which is a
 * claim about {@code kG}, not about {@code kP}), it starts where the model says it starts rather
 * than at zero, and a setpoint outside the travel bounds is clamped rather than silently ignored.
 *
 * <p>Like the roller test, this sleeps — Phoenix's device simulation runs on wall-clock time. See
 * {@code RollerMechanismSimTest.run} for the full explanation.
 */
class RotaryMechanismSimTest {

  private static final CANBus BUS = new CANBus("rio");

  private static final Angle REVERSE_LIMIT = Degrees.of(-10);
  private static final Angle FORWARD_LIMIT = Degrees.of(100);
  private static final Angle STARTING_ANGLE = Degrees.of(0);
  private static final Angle TARGET = Degrees.of(60);
  private static final Angle TOLERANCE = Degrees.of(2);

  private static final long LOOP_PERIOD_MILLIS = 20;
  private static final int SETTLE_LOOPS = 120;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);
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
    return MotorConfig.builder("SimArm" + canId, MechanismKind.ROTARY)
        .canId(canId, BUS)
        .sensorToMechanismRatio(60.0)
        .softLimits(REVERSE_LIMIT, FORWARD_LIMIT)
        .supplyCurrentLimit(40.0)
        .statorCurrentLimit(60.0)
        // Amps, not volts. kG is the current needed to hold the arm horizontal; kP is amps per
        // mechanism rotation of error, which is why it is in the thousands and still reasonable.
        .gains(new Gains(2500.0, 0.0, 120.0, 0.5, 0.0, 0.0, 6.5))
        .motionProfile(
            MotionProfile.of(RotationsPerSecond.of(0.8), RotationsPerSecondPerSecond.of(2.0)))
        .build();
  }

  private static RotarySettings settings() {
    return new RotarySettings(TOLERANCE, Inches.of(20));
  }

  private static RotarySimModel model() {
    return new RotarySimModel(
        MotorSpecs.KRAKEN_X60_FOC.gearbox(1), Pounds.of(6.5), STARTING_ANGLE, true);
  }

  private static RotaryMechanismSim newArm(int canId) {
    return RotaryMechanism.sim(config(canId), settings(), model());
  }

  private static void run(RotaryMechanismSim arm, int loops) {
    for (int i = 0; i < loops; i++) {
      arm.periodic();
      try {
        Thread.sleep(LOOP_PERIOD_MILLIS);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new IllegalStateException("interrupted while stepping the simulation", e);
      }
    }
  }

  @Test
  void theArmStartsWhereTheModelSaysItStarts() {
    // Not at zero because the encoder happens to boot at zero. An arm seeded wrong is wrong by a
    // constant offset in every setpoint, and nothing in the log says so.
    RotaryMechanismSim arm = newArm(21);
    run(arm, 3);
    assertEquals(
        STARTING_ANGLE.in(Degrees),
        arm.getPosition().in(Degrees),
        2.0,
        "the simulated device should be seeded with the model's starting angle");
  }

  @Test
  void aCommandedAngleIsReachedAndHeld() {
    RotaryMechanismSim arm = newArm(22);
    arm.setPosition(TARGET);
    run(arm, SETTLE_LOOPS);

    System.out.println("********** Rotary Sim: move to 60 deg **********");
    System.out.println("\tmeasured " + arm.getPosition().in(Degrees) + " deg");
    System.out.println("\ttorque   " + arm.getTorqueCurrent());

    assertTrue(
        arm.isAtPosition(),
        "arm should reach "
            + TARGET.in(Degrees)
            + " deg; measured "
            + arm.getPosition().in(Degrees));

    // And still there a second later. This is the kG claim: an arm with a working position loop but
    // no gravity compensation reaches its setpoint and then sags, which a short test would miss.
    run(arm, 50);
    assertTrue(
        arm.isAtPosition(),
        "arm should still be holding "
            + TARGET.in(Degrees)
            + " deg a second later; measured "
            + arm.getPosition().in(Degrees));
  }

  @Test
  void aGoalBeyondTheTravelBoundsIsClamped() {
    // The Talon would refuse to travel past its soft limit anyway, but silently. Clamping in the
    // mechanism means the goal that is logged is the goal that can actually be reached, so
    // "stopped short at a bound" stops looking like "gains too weak".
    RotaryMechanismSim arm = newArm(23);
    arm.setPosition(FORWARD_LIMIT.plus(Degrees.of(45)));

    assertEquals(
        FORWARD_LIMIT.in(Degrees),
        arm.getGoalPosition().in(Degrees),
        1e-6,
        "a goal past the forward bound should be clamped to it");

    run(arm, SETTLE_LOOPS);
    assertTrue(
        arm.getPosition().in(Degrees) < FORWARD_LIMIT.in(Degrees) + 2.0,
        "the arm must not travel past its forward bound; measured "
            + arm.getPosition().in(Degrees));
  }

  @Test
  void movingDownwardWorksToo() {
    // Gravity helps in this direction, which is a different failure mode: an arm with too much kG
    // overshoots on the way down and can oscillate. Worth exercising both directions.
    RotaryMechanismSim arm = newArm(24);
    arm.setPosition(Degrees.of(80));
    run(arm, SETTLE_LOOPS);
    assertTrue(arm.isAtPosition(), "precondition: should have reached 80 deg");

    arm.setPosition(Degrees.of(10));
    run(arm, SETTLE_LOOPS);
    assertTrue(
        arm.isAtPosition(),
        "arm should come back down to 10 deg; measured " + arm.getPosition().in(Degrees));
  }

  @Test
  void aRollerConfigIsRejectedByARotaryMechanism() {
    // The kind decides soft-limit requirements, the gravity model and the static-feedforward sign.
    // A mismatch is not cosmetic, and it is exactly the kind of thing a copy-paste introduces.
    MotorConfig rollerConfig =
        MotorConfig.builder("WrongKind", MechanismKind.ROLLER)
            .canId(25, BUS)
            .sensorToMechanismRatio(1.0)
            .supplyCurrentLimit(40.0)
            .statorCurrentLimit(40.0)
            .gains(Gains.zero())
            .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(10.0)))
            .build();

    assertThrows(
        IllegalArgumentException.class, () -> RotaryMechanism.replay(rollerConfig, settings()));
  }

  @Test
  void theSimModelAcceptsMassInAnyUnit() {
    // Guarding the units boundary rather than the physics: Pounds and Kilograms both have to reach
    // SingleJointedArmSim as kilograms, and a mass off by 2.2x makes every kG conclusion wrong.
    assertEquals(
        2.948,
        new RotarySimModel(
                MotorSpecs.KRAKEN_X60_FOC.gearbox(1), Pounds.of(6.5), STARTING_ANGLE, true)
            .mass()
            .in(Kilograms),
        0.001);
  }
}
