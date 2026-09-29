package frc.robot.subsystems.flywheel;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.roller.RollerSettings;
import frc.lib.mechanism.roller.RollerSimModel;
import frc.lib.util.MotorSpecs;
import frc.robot.Constants;

/**
 * How the 2026 flywheel is built, and the distance-to-speed maps it shoots from.
 *
 * <p>Every value is carried over from Rebuilt2026 {@code FlywheelConstants} and {@code
 * FlywheelIOPhoenix6}. Gains are in <b>amps</b> — the loop runs {@code
 * MotionMagicVelocityTorqueCurrentFOC}.
 *
 * <p>Five Krakens: 30 leads, 31 and 32 follow aligned, 33 and 34 follow opposed. 2026 registered
 * every follower signal at 50 Hz alongside the leader's, so {@code followerSignalHz} keeps that
 * rate rather than the library's 4 Hz default.
 */
public final class FlywheelConstants {

  private FlywheelConstants() {}

  public static final String NAME = "Flywheel";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** Motor rotations per flywheel rotation (36:24). */
  public static final double GEAR_RATIO = 36.0 / 24.0;

  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROLLER)
          .canId(Constants.CanIds.FLYWHEEL_LEADER, Constants.CanIds.RIO_BUS)
          .follower(Constants.CanIds.FLYWHEEL_FOLLOWER_1, false)
          .follower(Constants.CanIds.FLYWHEEL_FOLLOWER_2, false)
          .follower(Constants.CanIds.FLYWHEEL_FOLLOWER_3, true)
          .follower(Constants.CanIds.FLYWHEEL_FOLLOWER_4, true)
          .followerSignalHz(50.0)
          .sensorToMechanismRatio(GEAR_RATIO)
          .supplyCurrentLimit(40.0)
          .supplyCurrentLowerLimit(35.0, Seconds.of(1))
          .statorCurrentLimit(45.0)
          .neutralMode(NeutralModeValue.Coast)
          .inverted(true)
          // kP, kI, kD, kS, kV, kA, kG
          .gains(new Gains(15.0, 0.0, 0.0, 8.0, 0.12, 0.0, 0.0))
          .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(100.0)))
          .build();

  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(CONFIG.rotorToMechanismRatio());

  /**
   * "Spun up" is within 200 RPM of the goal — Rebuilt2026 {@code
   * Thresholds.flywheelSpinupThreshold}. {@code isAtVelocity()} uses the same strict less-than 2026
   * {@code isSpunUp()} did. No jam detection: a flywheel in free air has no jam signature.
   */
  public static final RollerSettings SETTINGS = RollerSettings.of(RPM.of(200));

  /** 2026 sim moment of inertia. A guess, not a CAD number. */
  public static final RollerSimModel SIM =
      new RollerSimModel(MOTOR.gearbox(CONFIG.motorCount()), KilogramSquareMeters.of(0.01));

  /** Clamp on every speed the shot maps produce. */
  public static final class Limits {
    private Limits() {}

    public static final AngularVelocity MIN_SPEED = RPM.of(100);
    public static final AngularVelocity MAX_SPEED = RPM.of(5600);
  }

  /** Flywheel surface geometry, for shoot-on-the-move. */
  public static final class Mechanical {
    private Mechanical() {}

    public static final double FLYWHEEL_METERS_PER_ROTATION = inchesToMeters(Math.PI * 4);
    public static final double FLYWHEEL_ROTATIONS_PER_METER = 1 / FLYWHEEL_METERS_PER_ROTATION;
  }

  /**
   * Distance (meters from hub or pass target) to flywheel speed (RPM), measured on the 2026 field.
   * These are structure, not pit knobs, which is why they live here and not in {@code Constants}.
   */
  public static final class DistanceMap {
    private DistanceMap() {}

    public static final InterpolatingDoubleTreeMap SPEED_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap PASSING_SPEED_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      SPEED_MAP.put(inchesToMeters(75.0), 1450.0); // Auto shot 1
      SPEED_MAP.put(inchesToMeters(85.0), 1525.0); // Auto shot 2
      SPEED_MAP.put(inchesToMeters(110.0), 1625.0); // Tower shot
      SPEED_MAP.put(inchesToMeters(130.0), 1700.0);
      SPEED_MAP.put(inchesToMeters(145.0), 1750.0);
      SPEED_MAP.put(inchesToMeters(160.0), 1875.0);
      SPEED_MAP.put(inchesToMeters(175.0), 1925.0);
      SPEED_MAP.put(inchesToMeters(180.0), 1950.0);
      SPEED_MAP.put(inchesToMeters(190.0), 1975.0);
    }

    static {
      PASSING_SPEED_MAP.put(inchesToMeters(70.0), 1600.0);
      PASSING_SPEED_MAP.put(inchesToMeters(140.0), 1700.0);
      PASSING_SPEED_MAP.put(inchesToMeters(225.0), 2050.0);
      PASSING_SPEED_MAP.put(inchesToMeters(410.0), 2700.0);
    }
  }

  /** Display-only parameters for {@link FlywheelVisualizer}. Nothing here affects the robot. */
  public static final class TrajectoryVisualization {
    private TrajectoryVisualization() {}

    public static final int TRAJECTORY_POINTS = 50;
    public static final double TRAJECTORY_TIME_SPAN = 2.0;
    public static final double LAUNCH_HEIGHT_METERS = inchesToMeters(20);
    public static final double DRUM_RADIUS_METERS = inchesToMeters(1.5);
    public static final double VELOCITY_FUDGE_FACTOR = 0.8;
    public static final double MIN_RPM_FOR_TRAJECTORY = 50.0;
    public static final double SHOOTER_EXIT_X_METERS = inchesToMeters(-6); // behind center
    public static final double SHOOTER_EXIT_Y_METERS = 0.0; // centered left-right
  }
}
