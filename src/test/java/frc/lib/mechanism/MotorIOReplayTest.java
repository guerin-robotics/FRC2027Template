package frc.lib.mechanism;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import frc.lib.mechanism.MotorConfig.Feedback;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import org.junit.jupiter.api.Test;

/**
 * That the replay IO agrees with the real one about the shape of the motor group.
 *
 * <p>Replay reads nothing and writes nothing — AdvantageKit feeds the inputs classes from the log.
 * But {@code Mechanism} decides which input groups exist, and therefore which logged channels get
 * read back at all, from {@code followerCount()} and {@code hasAbsoluteEncoder()}. A bare {@code
 * new MotorIO() {}} answers zero and false to those, so replaying a two-motor lift never feeds the
 * follower's channels back: {@code isConnected()} returns true in replay for a follower that was
 * dead on the field, and anything gated on it takes the branch it did not take during the match.
 *
 * <p>Which is the one thing replay exists to prevent, so it is worth a test that does not need a
 * log, a device or the HAL.
 */
class MotorIOReplayTest {

  private static final CANBus BUS = new CANBus("rio");

  private static MotorConfig.Builder arm(int canId) {
    return MotorConfig.builder("ReplayArm" + canId, MechanismKind.ROTARY)
        .canId(canId, BUS)
        .sensorToMechanismRatio(60.0)
        .softLimits(Degrees.of(-10), Degrees.of(95))
        .supplyCurrentLimit(40.0)
        .statorCurrentLimit(60.0)
        .gains(Gains.zero())
        .motionProfile(
            MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)));
  }

  @Test
  void replayReportsTheFollowersAndTheEncoderTheConfigDeclares() {
    MotorConfig config =
        arm(47).follower(48, true).encoder(Feedback.FUSED_CANCODER, 49, 0.25, 25.0).build();

    MotorIO io = MotorIO.replay(config);

    assertAll(
        () -> assertEquals(1, io.followerCount(), "the follower's channels have to be replayed"),
        () -> assertTrue(io.hasAbsoluteEncoder(), "so do the encoder's"));
  }

  @Test
  void replayReportsNothingExtraForABareMechanism() {
    // The other direction matters too: a single-motor mechanism with no encoder must not allocate
    // input groups that the log has no channels for.
    MotorIO io = MotorIO.replay(arm(50).build());

    assertAll(
        () -> assertEquals(0, io.followerCount()), () -> assertFalse(io.hasAbsoluteEncoder()));
  }
}
