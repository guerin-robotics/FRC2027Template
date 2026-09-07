package frc.lib.mechanism;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.lib.mechanism.MotorConfig.Feedback;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import org.junit.jupiter.api.Test;

/**
 * The builder's job is to make a config that is missing a safety value impossible to construct, and
 * to produce the same Talon configuration every time from the same inputs.
 *
 * <p>Both halves matter. The first is what stops a mechanism shipping with a current limit nobody
 * chose; the second is what lets a reviewer trust that two mechanisms differ only where their
 * builder calls differ.
 */
class MotorConfigTest {

  private static final CANBus BUS = new CANBus("rio");

  /** A config with everything a roller needs, for tests that want to vary one thing. */
  private static MotorConfig.Builder validRoller() {
    return MotorConfig.builder("TestRoller", MechanismKind.ROLLER)
        .canId(20, BUS)
        .sensorToMechanismRatio(4.0)
        .supplyCurrentLimit(40.0)
        .statorCurrentLimit(80.0)
        .gains(Gains.p(5.0))
        .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(50.0)));
  }

  @Test
  void buildRejectsEveryMissingSafetyValueAtOnce() {
    IllegalStateException thrown =
        assertThrows(
            IllegalStateException.class,
            () -> MotorConfig.builder("Bare", MechanismKind.ROLLER).build());

    String message = thrown.getMessage();
    assertAll(
        // All at once, not one per rebuild. Discovering six missing values through six
        // edit-compile-run cycles is how people start guessing at them.
        () -> assertTrue(message.contains("canId"), message),
        () -> assertTrue(message.contains("sensorToMechanismRatio"), message),
        () -> assertTrue(message.contains("supplyCurrentLimit"), message),
        () -> assertTrue(message.contains("statorCurrentLimit"), message),
        () -> assertTrue(message.contains("gains"), message),
        () -> assertTrue(message.contains("motionProfile"), message),
        () -> assertTrue(message.contains("Bare"), "the message should name the mechanism"));
  }

  @Test
  void aPositionMechanismCannotBeBuiltWithoutTravelBounds() {
    // The failure this prevents is not a mechanism that does not work. It is a mechanism that
    // drives into its own hard stop at whatever the current limit allows, and holds there.
    IllegalStateException thrown =
        assertThrows(
            IllegalStateException.class,
            () ->
                MotorConfig.builder("Arm", MechanismKind.ROTARY)
                    .canId(21, BUS)
                    .sensorToMechanismRatio(60.0)
                    .supplyCurrentLimit(40.0)
                    .statorCurrentLimit(40.0)
                    .gains(Gains.zero())
                    .motionProfile(
                        MotionProfile.of(
                            RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)))
                    .build());

    assertTrue(thrown.getMessage().contains("softLimits"), thrown.getMessage());
  }

  @Test
  void aRollerNeedsNoTravelBounds() {
    // A roller has nowhere to slam. Requiring soft limits on one would be noise, and noise in a
    // required-value list is how the list stops being read.
    assertEquals(MechanismKind.ROLLER, validRoller().build().kind());
  }

  @Test
  void zeroGainsSatisfyTheGainRequirementButOmittingThemDoesNot() {
    // "We have not characterized this yet" is a legitimate state. It just has to be written down,
    // because it reads very differently from having forgotten.
    assertEquals(0.0, validRoller().gains(Gains.zero()).build().gains().kP());

    IllegalStateException thrown =
        assertThrows(
            IllegalStateException.class,
            () ->
                MotorConfig.builder("NoGains", MechanismKind.ROLLER)
                    .canId(20, BUS)
                    .sensorToMechanismRatio(1.0)
                    .supplyCurrentLimit(40.0)
                    .statorCurrentLimit(40.0)
                    .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(10.0)))
                    .build());
    assertTrue(thrown.getMessage().contains("gains"), thrown.getMessage());
  }

  @Test
  void torqueCurrentClampsFollowTheStatorLimitUnlessOverridden() {
    // Derived rather than defaulted, so the closed loop can never be allowed to request current
    // the windings are not allowed to carry. Two independent numbers would drift apart.
    var config = validRoller().statorCurrentLimit(60.0).build().toTalonFXConfiguration();
    assertEquals(60.0, config.TorqueCurrent.PeakForwardTorqueCurrent);
    assertEquals(-60.0, config.TorqueCurrent.PeakReverseTorqueCurrent);

    var overridden =
        validRoller()
            .statorCurrentLimit(60.0)
            .peakTorqueCurrent(30.0, -10.0)
            .build()
            .toTalonFXConfiguration();
    assertEquals(30.0, overridden.TorqueCurrent.PeakForwardTorqueCurrent);
    assertEquals(-10.0, overridden.TorqueCurrent.PeakReverseTorqueCurrent);
  }

  @Test
  void currentLimitsAreEnabledNotJustSet() {
    // Phoenix will happily hold a limit value with enforcement switched off, which looks correct in
    // a config dump and does nothing on the robot.
    var config = validRoller().build().toTalonFXConfiguration();
    assertAll(
        () -> assertTrue(config.CurrentLimits.SupplyCurrentLimitEnable),
        () -> assertTrue(config.CurrentLimits.StatorCurrentLimitEnable),
        () -> assertEquals(40.0, config.CurrentLimits.SupplyCurrentLimit),
        () -> assertEquals(80.0, config.CurrentLimits.StatorCurrentLimit));
  }

  @Test
  void theMechanismRatioReachesTheFeedbackBlock() {
    // Phoenix defaults SensorToMechanismRatio to 1.0. A config that silently skipped it would
    // report motor rotations while every setpoint and tolerance assumed mechanism rotations.
    var config = validRoller().sensorToMechanismRatio(7.5).build().toTalonFXConfiguration();
    assertEquals(7.5, config.Feedback.SensorToMechanismRatio);
    assertEquals(FeedbackSensorSourceValue.RotorSensor, config.Feedback.FeedbackSensorSource);
  }

  @Test
  void aFusedEncoderCarriesBothRatiosAndTheRemoteId() {
    var config =
        MotorConfig.builder("Arm", MechanismKind.ROTARY)
            .canId(21, BUS)
            .encoder(Feedback.FUSED_CANCODER, 31, 0.25, 3.0)
            .sensorToMechanismRatio(2.0)
            .softLimits(Degrees.of(-10), Degrees.of(95))
            .supplyCurrentLimit(40.0)
            .statorCurrentLimit(40.0)
            .gains(Gains.zero())
            .motionProfile(
                MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)))
            .build();

    var fx = config.toTalonFXConfiguration();
    assertAll(
        () ->
            assertEquals(FeedbackSensorSourceValue.FusedCANcoder, fx.Feedback.FeedbackSensorSource),
        () -> assertEquals(31, fx.Feedback.FeedbackRemoteSensorID),
        // Under fusion the rotor ratio is in the position path: wrong here and the fused reading is
        // off by the whole reduction while every config call still reports success.
        () -> assertEquals(3.0, fx.Feedback.RotorToSensorRatio),
        () -> assertEquals(2.0, fx.Feedback.SensorToMechanismRatio),
        () -> assertEquals(6.0, config.rotorToMechanismRatio()),
        () -> assertEquals(0.25, config.toCANcoderConfiguration().MagnetSensor.MagnetOffset));
  }

  @Test
  void softLimitsAreEnabledAndConvertedFromAngles() {
    var config =
        MotorConfig.builder("Arm", MechanismKind.ROTARY)
            .canId(21, BUS)
            .sensorToMechanismRatio(60.0)
            .softLimits(Degrees.of(-90), Degrees.of(180))
            .supplyCurrentLimit(40.0)
            .statorCurrentLimit(40.0)
            .gains(Gains.zero())
            .motionProfile(
                MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)))
            .build()
            .toTalonFXConfiguration();

    assertAll(
        () -> assertTrue(config.SoftwareLimitSwitch.ForwardSoftLimitEnable),
        () -> assertTrue(config.SoftwareLimitSwitch.ReverseSoftLimitEnable),
        () -> assertEquals(0.5, config.SoftwareLimitSwitch.ForwardSoftLimitThreshold, 1e-9),
        () -> assertEquals(-0.25, config.SoftwareLimitSwitch.ReverseSoftLimitThreshold, 1e-9));
  }

  @Test
  void swappedSoftLimitsAreRejected() {
    // A swapped pair leaves no legal travel at all, and Phoenix does not object — the mechanism
    // simply refuses to move and nothing says why.
    assertThrows(IllegalArgumentException.class, () -> new MotorConfig.SoftLimits(1.0, -1.0));
  }

  @Test
  void theMechanismKindPicksTheGravityModelAndStaticSign() {
    var roller = validRoller().build().toTalonFXConfiguration();
    var arm =
        MotorConfig.builder("Arm", MechanismKind.ROTARY)
            .canId(21, BUS)
            .sensorToMechanismRatio(60.0)
            .softLimits(Degrees.of(0), Degrees.of(90))
            .supplyCurrentLimit(40.0)
            .statorCurrentLimit(40.0)
            .gains(Gains.zero())
            .motionProfile(
                MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)))
            .build()
            .toTalonFXConfiguration();

    assertAll(
        () -> assertEquals(GravityTypeValue.Elevator_Static, roller.Slot0.GravityType),
        () -> assertEquals(GravityTypeValue.Arm_Cosine, arm.Slot0.GravityType),
        // A velocity loop knows which way it is going. A position loop holding against friction has
        // zero velocity and non-zero error, so kS has to follow the error or it disappears exactly
        // when it is needed.
        () ->
            assertEquals(
                StaticFeedforwardSignValue.UseVelocitySign, roller.Slot0.StaticFeedforwardSign),
        () ->
            assertEquals(
                StaticFeedforwardSignValue.UseClosedLoopSign, arm.Slot0.StaticFeedforwardSign));
  }

  @Test
  void brakeIsTheDefaultNeutralMode() {
    assertEquals(
        NeutralModeValue.Brake,
        validRoller().build().toTalonFXConfiguration().MotorOutput.NeutralMode);
  }

  @Test
  void motorCountIncludesFollowers() {
    // The number a simulation gearbox has to be sized for. A two-motor mechanism simulated as one
    // reaches half the acceleration and every gain found against it is wrong.
    assertEquals(3, validRoller().follower(21, false).follower(22, true).build().motorCount());
  }

  @Test
  void anOutOfRangeCanIdIsRejectedWhereItIsWritten() {
    // Phoenix throws for this too, but from inside a device constructor and without naming the
    // mechanism — so a robot with a dozen of them reports only that some device somewhere is bad.
    // Catching it here names the file to open.
    IllegalArgumentException thrown =
        assertThrows(IllegalArgumentException.class, () -> validRoller().canId(70, BUS));
    assertTrue(thrown.getMessage().contains("70"), thrown.getMessage());
    assertTrue(thrown.getMessage().contains("TestRoller"), thrown.getMessage());

    assertThrows(IllegalArgumentException.class, () -> validRoller().follower(99, false));
    assertThrows(
        IllegalArgumentException.class,
        () -> validRoller().encoder(Feedback.FUSED_CANCODER, 63, 0.0, 1.0));
  }

  @Test
  void requestingAnEncoderWithTheInternalSourceIsRejected() {
    assertThrows(
        IllegalArgumentException.class,
        () -> validRoller().encoder(Feedback.INTERNAL, 31, 0.0, 1.0));
  }
}
