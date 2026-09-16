package frc.lib.mechanism.rotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
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
 * That simulated gravity peaks where the arm is actually level.
 *
 * <p>{@code SingleJointedArmSim} applies gravity torque as the cosine of its own angle, so its
 * gravity peaks at its zero. The Talon computes {@code cos(position + GravityArmPositionOffset)},
 * so its compensation peaks at the level position. Those coincide only when the mechanism's zero
 * happens to be level — and the arm scaffold actively invites a stow-referenced zero, which is the
 * case where they do not.
 *
 * <p>The consequence of getting it wrong is not a simulation that looks broken. It is a simulation
 * that looks fine and teaches a {@code kG} that is wrong on the robot, reported as nothing at all.
 *
 * <p>The probe is an arm hanging straight down with no output commanded. Straight down is where
 * gravity torque is zero and the equilibrium is stable, so a correctly framed arm stays put. An arm
 * whose sim thinks level is somewhere else has real torque there and swings away from it.
 */
class RotaryGravityFrameTest {

  private static final CANBus BUS = new CANBus("rio");

  /** The arm is horizontal at +20 degrees, so it hangs straight down at -70. */
  private static final Angle LEVEL_AT = Degrees.of(20);

  private static final Angle HANGING = Degrees.of(-70);

  private static final long LOOP_PERIOD_MILLIS = 20;
  private static final int LOOPS = 100;

  /**
   * Correctly framed, the arm drifts about 0.05 degrees over this run. Mis-framed, it drifts about
   * 4.5. One degree sits an order of magnitude above the noise and well under the signal.
   */
  private static final double TOLERANCE_DEGREES = 1.0;

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
    return MotorConfig.builder("GravityArm" + canId, MechanismKind.ROTARY)
        .canId(canId, BUS)
        .sensorToMechanismRatio(60.0)
        .softLimits(Degrees.of(-120), Degrees.of(120))
        .supplyCurrentLimit(40.0)
        .statorCurrentLimit(60.0)
        .gains(Gains.zero())
        .gravityOffset(LEVEL_AT)
        .motionProfile(
            MotionProfile.of(RotationsPerSecond.of(0.8), RotationsPerSecondPerSecond.of(2.0)))
        .build();
  }

  @Test
  void anArmHangingStraightDownStaysThereWithNoOutput() {
    RotaryMechanismSim arm =
        RotaryMechanism.sim(
            config(46),
            new RotarySettings(Degrees.of(2), Inches.of(20)),
            new RotarySimModel(
                MotorSpecs.KRAKEN_X60_FOC.gearbox(1), Pounds.of(6.5), HANGING, true));

    // Nothing commanded: the only thing acting on the arm is gravity, which is the point.
    arm.stop();

    for (int i = 0; i < LOOPS; i++) {
      arm.periodic();
      try {
        Thread.sleep(LOOP_PERIOD_MILLIS);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new IllegalStateException("interrupted while stepping the simulation", e);
      }
    }

    double drifted = Math.abs(arm.getPosition().in(Degrees) - HANGING.in(Degrees));
    System.out.println("********** Rotary gravity frame **********");
    System.out.println(
        "\tstarted "
            + HANGING.in(Degrees)
            + " deg, ended "
            + arm.getPosition().in(Degrees)
            + " deg, drift "
            + drifted
            + " deg");

    assertTrue(
        drifted < TOLERANCE_DEGREES,
        "an arm hanging straight down has no gravity torque on it and should stay put; it moved "
            + drifted
            + " degrees, which means simulated gravity peaks somewhere other than the level"
            + " position");
  }
}
