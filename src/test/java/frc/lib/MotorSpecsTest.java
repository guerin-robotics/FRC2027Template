package frc.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.Test;

/**
 * Tests for the motor free-speed table and the mechanism top-speed arithmetic.
 *
 * <p>This class is load-bearing in a way that is easy to miss: every mechanism computes {@code
 * MAX_SPEED_RPM} from it, and a Motion Magic cruise velocity above the real top speed does not fail
 * loudly — the motor saturates and the profile silently stops being followed. A wrong number here
 * surfaces as bad tuning on a practice field, not as an error.
 *
 * <p>Expected values are the published Kraken specifications, asserted independently of {@link
 * DCMotor} so that a WPILib revision to either motor shows up here as a failure rather than
 * silently changing every mechanism's speed ceiling.
 */
class MotorSpecsTest {

  private static final double EPSILON = 1.0;

  @Test
  void freeSpeedsMatchPublishedSpecs() {
    assertEquals(5800.0, MotorSpecs.KRAKEN_X60_FOC.freeSpeedRpm, EPSILON);
    assertEquals(7368.0, MotorSpecs.KRAKEN_X44_FOC.freeSpeedRpm, EPSILON);
  }

  @Test
  void freeSpeedAgreesWithTheSimModel() {
    // The whole point of sourcing from DCMotor is that the speed math and the sim plant cannot
    // drift apart. If these ever disagree, one of them is lying to somebody.
    for (MotorSpecs spec : MotorSpecs.values()) {
      double fromModel =
          edu.wpi.first.math.util.Units.radiansPerSecondToRotationsPerMinute(
              spec.single.freeSpeedRadPerSec);
      assertEquals(spec.freeSpeedRpm, fromModel, 1e-9, spec + " free speed disagrees with DCMotor");
    }
  }

  @Test
  void gearRatioDividesFreeSpeed() {
    // 5800 / 15 = 386.67 — the elevator case.
    assertEquals(386.67, MotorSpecs.KRAKEN_X60_FOC.maxMechanismRpm(15.0), 0.01);

    // 7368 / 45 = 163.73 — the intake pivot case.
    assertEquals(163.73, MotorSpecs.KRAKEN_X44_FOC.maxMechanismRpm(45.0), 0.01);

    // 5800 / 2 = 2900 — the intake roller case.
    assertEquals(2900.0, MotorSpecs.KRAKEN_X60_FOC.maxMechanismRpm(2.0), 0.01);
  }

  @Test
  void directDriveReturnsFreeSpeed() {
    assertEquals(
        MotorSpecs.KRAKEN_X60_FOC.freeSpeedRpm,
        MotorSpecs.KRAKEN_X60_FOC.maxMechanismRpm(1.0),
        0.0);
  }

  @Test
  void reductionAlwaysSlowsTheMechanism() {
    // Guards against the ratio being inverted — a 15:1 reduction must make the mechanism turn
    // slower than the motor, never faster. Inverting it would overstate top speed by 225x here
    // and every derived cruise velocity with it.
    for (MotorSpecs spec : MotorSpecs.values()) {
      assertTrue(
          spec.maxMechanismRpm(15.0) < spec.freeSpeedRpm,
          spec + " reported a mechanism faster than its own motor");
    }
  }

  @Test
  void motorCountDoesNotChangeFreeSpeed() {
    // Two motors on one shaft double available torque; they do not spin any faster. This is why
    // maxMechanismRpm() takes no count, and why a mechanism that misses its predicted speed
    // UNLOADED has a gear ratio problem rather than a motor-count problem.
    for (MotorSpecs spec : MotorSpecs.values()) {
      assertEquals(
          spec.single.freeSpeedRadPerSec,
          spec.gearbox(4).freeSpeedRadPerSec,
          1e-9,
          spec + " free speed changed with motor count");
    }
  }

  @Test
  void gearboxScalesTorqueWithCount() {
    for (MotorSpecs spec : MotorSpecs.values()) {
      DCMotor four = spec.gearbox(4);
      assertEquals(
          4.0 * spec.single.stallTorqueNewtonMeters,
          four.stallTorqueNewtonMeters,
          1e-9,
          spec + " stall torque did not scale with motor count");
      assertEquals(
          4.0 * spec.single.stallCurrentAmps,
          four.stallCurrentAmps,
          1e-9,
          spec + " stall current did not scale with motor count");
    }
  }

  @Test
  void gearboxIsNotAReduction() {
    // The bug this method was written to avoid: DCMotor.withReduction() models a GEARBOX,
    // trading speed for torque, and reads plausibly as "n motors" at a glance. Using it here
    // would have cut sim free speed by the motor count.
    DCMotor twoMotors = MotorSpecs.KRAKEN_X60_FOC.gearbox(2);
    DCMotor throughA2To1 = MotorSpecs.KRAKEN_X60_FOC.single.withReduction(2.0);

    assertEquals(
        MotorSpecs.KRAKEN_X60_FOC.single.freeSpeedRadPerSec,
        twoMotors.freeSpeedRadPerSec,
        1e-9,
        "two motors should share the single-motor free speed");
    assertTrue(
        throughA2To1.freeSpeedRadPerSec < twoMotors.freeSpeedRadPerSec,
        "a 2:1 reduction must be slower than two motors on one shaft");
  }

  @Test
  void singleIsExactlyOneMotor() {
    for (MotorSpecs spec : MotorSpecs.values()) {
      assertEquals(
          spec.single.stallTorqueNewtonMeters,
          spec.gearbox(1).stallTorqueNewtonMeters,
          1e-9,
          spec + " single did not match gearbox(1)");
    }
  }
}
