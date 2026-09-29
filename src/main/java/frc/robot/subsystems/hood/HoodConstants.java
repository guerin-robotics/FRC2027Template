package frc.robot.subsystems.hood;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.Feedback;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.rotary.RotarySettings;
import frc.lib.mechanism.rotary.RotarySimModel;
import frc.lib.util.MotorSpecs;
import frc.robot.Constants;

/**
 * How the 2026 shooter hood is built, and the distance-to-angle maps it aims from.
 *
 * <p>Every hardware value is carried over from Rebuilt2026 {@code HoodConstants} and {@code
 * HoodIOReal}. Gains are in <b>amps</b> — the loop runs {@code MotionMagicTorqueCurrentFOC}.
 *
 * <p>The CANcoder sits on an intermediate shaft: 5.33:1 from the motor to the encoder, then
 * 122T:12T from the encoder to the hood. It is fused with the rotor.
 */
public final class HoodConstants {

  private HoodConstants() {}

  public static final String NAME = "Hood";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** Motor rotations per encoder rotation. */
  public static final double ROTOR_TO_SENSOR_RATIO = 5.33;

  /** Encoder rotations per hood rotation (12T driving 122T). */
  public static final double SENSOR_TO_MECHANISM_RATIO = 122.0 / 12.0;

  /** CANcoder calibration from the 2026 robot. */
  public static final double MAGNET_OFFSET_ROTATIONS = -0.16;

  /** 0.75 / 2 + 0.25, as 2026 wrote it. */
  public static final double SENSOR_DISCONTINUITY_POINT = 0.625;

  public static final Angle REVERSE_SOFT_LIMIT = Degrees.of(0);
  public static final Angle FORWARD_SOFT_LIMIT = Degrees.of(62);

  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROTARY)
          .canId(Constants.CanIds.HOOD_MOTOR, Constants.CanIds.RIO_BUS)
          .encoder(
              Feedback.FUSED_CANCODER,
              Constants.CanIds.HOOD_ENCODER,
              MAGNET_OFFSET_ROTATIONS,
              ROTOR_TO_SENSOR_RATIO)
          .encoderDirection(SensorDirectionValue.Clockwise_Positive)
          .sensorDiscontinuityPoint(SENSOR_DISCONTINUITY_POINT)
          .sensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO)
          .softLimits(REVERSE_SOFT_LIMIT, FORWARD_SOFT_LIMIT)
          .supplyCurrentLimit(40.0)
          .supplyCurrentLowerLimit(35.0, Seconds.of(1))
          .statorCurrentLimit(20.0)
          .neutralMode(NeutralModeValue.Brake)
          .inverted(false)
          // kP, kI, kD, kS, kV, kA, kG
          .gains(new Gains(4000.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0))
          // 2026 left StaticFeedforwardSign at Phoenix's default, UseVelocitySign. The library
          // default for a ROTARY mechanism is UseClosedLoopSign; with a 9 A kS that difference is
          // real at rest, so the 2026 behavior is stated explicitly here.
          .staticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
          .motionProfile(
              MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(10.0)))
          .build();

  /**
   * 2026 had no position tolerance for the hood; 2° is the scaffold default and nothing in the 2026
   * logic reads it. The arm length is a placeholder for the visualizer and the sim.
   */
  public static final RotarySettings SETTINGS = new RotarySettings(Degrees.of(2.0), Inches.of(8));

  /**
   * PLACEHOLDER sim model. The 2026 hood sim had no physics — it jumped straight to each setpoint —
   * so there is no 2026 number to carry. Gravity is off because the 2026 gains carry no kG.
   */
  public static final RotarySimModel SIM =
      new RotarySimModel(
          MOTOR.gearbox(CONFIG.motorCount()), Pounds.of(2), REVERSE_SOFT_LIMIT, false);

  /** Upper clamp on every angle the shot maps produce. 234°, far above the 62° soft limit. */
  public static final Angle MAP_MAX = Degrees.of(234);

  public static final Angle MAP_MIN = Degrees.of(0);

  /**
   * Distance (meters from hub or pass target) to hood angle (degrees), measured on the 2026 field.
   */
  public static final class HoodMap {
    private HoodMap() {}

    public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap PASSING_ANGLE_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      ANGLE_MAP.put(inchesToMeters(75.0), 1.0); // Auto shot 1
      ANGLE_MAP.put(inchesToMeters(85.0), 1.5); // Auto shot 2
      ANGLE_MAP.put(inchesToMeters(110.0), 2.5); // Tower shot
      ANGLE_MAP.put(inchesToMeters(130.0), 3.5);
      ANGLE_MAP.put(inchesToMeters(145.0), 5.25);
      ANGLE_MAP.put(inchesToMeters(160.0), 11.0);
      ANGLE_MAP.put(inchesToMeters(175.0), 11.5);
      ANGLE_MAP.put(inchesToMeters(180.0), 12.25);
      ANGLE_MAP.put(inchesToMeters(190.0), 12.0);
    }

    static {
      PASSING_ANGLE_MAP.put(inchesToMeters(70.0), 20.0);
      PASSING_ANGLE_MAP.put(inchesToMeters(140.0), 25.0);
      PASSING_ANGLE_MAP.put(inchesToMeters(225.0), 28.0);
      PASSING_ANGLE_MAP.put(inchesToMeters(410.0), 35.0);
    }
  }
}
