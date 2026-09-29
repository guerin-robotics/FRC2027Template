package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
 * How the 2026 intake pivot is built.
 *
 * <p>Carried over from Rebuilt2026 {@code IntakePivotConstants} and {@code IntakePivotIOReal}, as
 * the device was <i>actually configured</i>, which is not quite what that file says. Gains are in
 * <b>amps</b> — the loop runs {@code MotionMagicTorqueCurrentFOC}.
 *
 * <h2>What 2026 really ran</h2>
 *
 * <p>{@code IntakePivotIOReal.configurePivotMotor} applied three configs, the last a bare {@code
 * FeedbackConfigs}. That reset {@code RotorToSensorRatio} from 45 to 1.0 and selected {@code
 * RemoteCANcoder}, and its soft limits were written <b>disabled</b>. Under {@code RemoteCANcoder}
 * the position comes straight off the encoder, so the lost ratio never affected control — which is
 * why it survived a season (see .claude/rules/02-hardware.md).
 *
 * <ul>
 *   <li><b>Ratio:</b> 45 is set here, the value 2026 meant. It is not in the position path under
 *       {@code REMOTE_CANCODER}, so control is unchanged; it does make the simulation gearbox
 *       right.
 *   <li><b>Soft limits:</b> the library will not build a rotary mechanism without them. To keep
 *       2026 behavior they are set to the encoder's own wrap bounds (discontinuity 0.625, so -0.375
 *       to 0.625 rotations), which the position can never pass, so they never engage. The 0.0 to
 *       0.4 window 2026 wrote down but never enabled is {@link #DESIGN_LOWER_LIMIT} / {@link
 *       #DESIGN_UPPER_LIMIT}. Enforcing it would put the reverse limit exactly on the deployed
 *       position, cutting reverse output there — a behavior change worth testing before adopting.
 * </ul>
 *
 * <p>Position 0 is deployed (down); increasing position stows the intake upward.
 */
public final class IntakePivotConstants {

  private IntakePivotConstants() {}

  public static final String NAME = "IntakePivot";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** Motor rotations per encoder rotation. The encoder is on the pivot shaft. */
  public static final double ROTOR_TO_SENSOR_RATIO = 45.0;

  public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

  /** CANcoder calibration from the 2026 robot. */
  public static final double MAGNET_OFFSET_ROTATIONS = -0.58;

  public static final double SENSOR_DISCONTINUITY_POINT = 0.625;

  /** The travel window 2026 wrote down but never enabled. See the class javadoc. */
  public static final Angle DESIGN_LOWER_LIMIT = Rotations.of(0.0);

  public static final Angle DESIGN_UPPER_LIMIT = Rotations.of(0.4);

  /** The encoder's wrap bounds — effectively no soft limit, as 2026 ran. */
  public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(SENSOR_DISCONTINUITY_POINT - 1.0);

  public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(SENSOR_DISCONTINUITY_POINT);

  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROTARY)
          .canId(Constants.CanIds.INTAKE_PIVOT_MOTOR, Constants.CanIds.RIO_BUS)
          .encoder(
              Feedback.REMOTE_CANCODER,
              Constants.CanIds.INTAKE_PIVOT_ENCODER,
              MAGNET_OFFSET_ROTATIONS,
              ROTOR_TO_SENSOR_RATIO)
          .encoderDirection(SensorDirectionValue.Clockwise_Positive)
          .sensorDiscontinuityPoint(SENSOR_DISCONTINUITY_POINT)
          .sensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO)
          .softLimits(REVERSE_SOFT_LIMIT, FORWARD_SOFT_LIMIT)
          .supplyCurrentLimit(50.0)
          .supplyCurrentLowerLimit(45.0, Seconds.of(1))
          .statorCurrentLimit(80.0)
          .neutralMode(NeutralModeValue.Brake)
          .inverted(false)
          // kP, kI, kD, kS, kV, kA, kG — Arm_Cosine, level at position 0, as in 2026.
          .gains(new Gains(750.0, 0.0, 5.0, 0.0, 0.0, 0.0, 8.0))
          .motionProfile(
              MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)))
          .build();

  /**
   * 0.005 rotations (1.8°) is the tolerance the 2026 visualizer drew "at goal" with. 0.3 m is the
   * 2026 sim arm length.
   */
  public static final RotarySettings SETTINGS =
      new RotarySettings(Rotations.of(0.005), Meters.of(0.3));

  /**
   * 2026 simulated the pivot as a gravity-free DCMotorSim with MOI 0.01 kg·m² on a 0.3 m arm. The
   * library sim takes a mass instead; 0.333 kg on 0.3 m gives that same MOI (m·L²/3). Gravity stays
   * off to match what 2026 simulated — the real 8 A kG would otherwise lift a 0.33 kg arm.
   */
  public static final RotarySimModel SIM =
      new RotarySimModel(
          MOTOR.gearbox(CONFIG.motorCount()), Kilograms.of(0.333), DESIGN_LOWER_LIMIT, false);
}
