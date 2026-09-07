package frc.lib.mechanism.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.MotorIO;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * Jam detection, driven against a fake IO rather than a physics model.
 *
 * <p>A jam is defined by a conjunction of three conditions holding for a dwell, and the point of
 * testing it here is that the conjunction is the whole design: current alone spikes on every
 * static-friction breakaway and every first contact with a game piece, both entirely normal, and a
 * detector built on current alone cries wolf until somebody turns it off.
 *
 * <p>Driving the inputs directly is what makes the corner cases reachable. Getting a simulated
 * flywheel to actually jam would mean modelling a jam, which is a lot of machinery to test four
 * boolean terms.
 *
 * <p>This one steps the FPGA clock rather than sleeping, because nothing here touches a Phoenix
 * device — the debouncer reads WPILib's timestamp, which {@code SimHooks} controls.
 */
class RollerJamDetectionTest {

  private static final CANBus BUS = new CANBus("rio");

  private static final AngularVelocity COMMANDED = RPM.of(2000);
  private static final AngularVelocity JAM_MIN_COMMANDED = RPM.of(500);
  private static final double JAM_VELOCITY_FRACTION = 0.2;
  private static final Current JAM_STATOR = Amps.of(40);
  private static final double JAM_DWELL_SECONDS = 0.5;

  private FakeMotorIO fake;
  private RollerMechanism roller;

  /** A MotorIO whose inputs the test writes directly. */
  private static final class FakeMotorIO implements MotorIO {
    private AngularVelocity velocity = RPM.of(0);
    private Current stator = Amps.of(0);

    @Override
    public void updateInputs(MotorInputs inputs) {
      inputs.connected = true;
      inputs.velocity = velocity;
      inputs.statorAmps = stator;
    }
  }

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
    SimHooks.pauseTiming();
  }

  @AfterAll
  static void shutdown() {
    SimHooks.resumeTiming();
    HAL.shutdown();
  }

  @BeforeEach
  void newMechanism() {
    MotorConfig config =
        MotorConfig.builder("JamTest", MechanismKind.ROLLER)
            .canId(60, BUS)
            .sensorToMechanismRatio(1.0)
            .supplyCurrentLimit(40.0)
            .statorCurrentLimit(80.0)
            .gains(Gains.zero())
            .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(100.0)))
            .build();

    RollerSettings settings =
        RollerSettings.of(RPM.of(100))
            .withJamDetection(
                new RollerSettings.JamDetection(
                    JAM_MIN_COMMANDED,
                    JAM_VELOCITY_FRACTION,
                    JAM_STATOR,
                    Seconds.of(JAM_DWELL_SECONDS)));

    fake = new FakeMotorIO();
    roller = new RollerMechanism(config, settings, fake);
  }

  /** One robot loop, with the clock advanced so the debouncer sees time pass. */
  private void step(int loops) {
    for (int i = 0; i < loops; i++) {
      roller.periodic();
      SimHooks.stepTiming(0.02);
    }
  }

  private void jamConditionPresent() {
    fake.velocity = RPM.of(0);
    fake.stator = JAM_STATOR.times(1.5);
  }

  @Test
  void aJamMustHoldForTheDwellBeforeItIsReported() {
    roller.setVelocity(COMMANDED);
    jamConditionPresent();

    // The dwell has to exceed spin-up time. During a normal spin-up the command is high, the
    // measurement is low and the current is high — exactly the jam signature — so a detector that
    // latched immediately would fire on every single start.
    step(10); // 0.2 s, well inside the 0.5 s dwell
    assertFalse(roller.isJammed(), "a jam shorter than the dwell is a transient, not a jam");

    step(25); // now past 0.5 s total
    assertTrue(roller.isJammed(), "the condition held past the dwell and should have latched");
    assertEquals(1, roller.getJamCount());
  }

  @Test
  void anIdleMechanismIsNotJammedNoMatterWhatTheCurrentSays() {
    // No command means no jam, by definition. Without this term a mechanism sitting still while
    // something else on the robot loads the bus reads as permanently jammed.
    roller.setVelocity(JAM_MIN_COMMANDED.div(2.0));
    jamConditionPresent();
    step(60);
    assertFalse(roller.isJammed());
    assertEquals(0, roller.getJamCount());
  }

  @Test
  void highCurrentAloneIsNotAJam() {
    // The classic mistake. Current spikes on every static-friction breakaway and on first contact
    // with a game piece; if the mechanism is still turning, it is working, not jammed.
    roller.setVelocity(COMMANDED);
    fake.velocity = COMMANDED; // turning fine
    fake.stator = JAM_STATOR.times(2.0); // and pulling hard
    step(60);
    assertFalse(
        roller.isJammed(), "a mechanism that is turning is not jammed, however hard it pulls");
  }

  @Test
  void aStalledMechanismDrawingLittleCurrentIsNotAJam() {
    // The mirror image: not turning and not trying. That is a broken belt or a dead motor, which
    // needs a different response than reversing to clear a jam.
    roller.setVelocity(COMMANDED);
    fake.velocity = RPM.of(0);
    fake.stator = JAM_STATOR.div(4.0);
    step(60);
    assertFalse(roller.isJammed());
  }

  @Test
  void theJamClearsWhenTheConditionDoes() {
    roller.setVelocity(COMMANDED);
    jamConditionPresent();
    step(40);
    assertTrue(roller.isJammed(), "precondition: should be jammed");

    fake.velocity = COMMANDED;
    fake.stator = Amps.of(5);
    step(5);
    assertFalse(
        roller.isJammed(), "a rising debouncer releases as soon as the condition goes away");
    assertEquals(1, roller.getJamCount(), "clearing a jam does not count as a new one");
  }

  @Test
  void stoppingClearsTheGoalSoAJamCannotBeReportedAgainstIt() {
    roller.setVelocity(COMMANDED);
    jamConditionPresent();
    step(40);
    assertTrue(roller.isJammed(), "precondition: should be jammed");

    // Without stop() clearing the goal, the detector keeps comparing against a setpoint nobody is
    // commanding, and a mechanism coasting to a halt reads as jammed indefinitely.
    roller.stop();
    step(5);
    assertFalse(roller.isJammed());
  }

  @Test
  void separateJamsAreCountedSeparately() {
    // The count is what turns "it jammed once, annoying" into "it has jammed four times this match,
    // go look at the mechanism" — which is a pit decision, not a driver one.
    roller.setVelocity(COMMANDED);

    jamConditionPresent();
    step(40);
    fake.velocity = COMMANDED;
    fake.stator = Amps.of(5);
    step(5);

    jamConditionPresent();
    step(40);

    assertTrue(roller.isJammed());
    assertEquals(2, roller.getJamCount());
  }
}
