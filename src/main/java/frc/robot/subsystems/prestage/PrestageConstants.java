package frc.robot.subsystems.prestage;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.roller.RollerSettings;
import frc.lib.mechanism.roller.RollerSimModel;
import frc.lib.util.MotorSpecs;
import frc.robot.Constants;

/**
 * How the 2026 prestage is built.
 *
 * <p>Every value is carried over from Rebuilt2026 {@code PrestageConstants}. Gains are in
 * <b>amps</b> — the loop runs {@code MotionMagicVelocityTorqueCurrentFOC}.
 *
 * <p>Two motors: the right (37) follows the left (38), opposed. 2026 registered the follower's
 * signals at 50 Hz alongside the leader's, so {@code followerSignalHz} keeps that rate rather than
 * the library's 4 Hz default.
 */
public final class PrestageConstants {

  private PrestageConstants() {}

  public static final String NAME = "Prestage";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** Total reduction, motor to mechanism. */
  public static final double GEAR_RATIO = 24.0 / 11.0;

  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROLLER)
          .canId(Constants.CanIds.PRESTAGE_LEADER, Constants.CanIds.RIO_BUS)
          .follower(Constants.CanIds.PRESTAGE_FOLLOWER, true)
          .followerSignalHz(50.0)
          .sensorToMechanismRatio(GEAR_RATIO)
          .supplyCurrentLimit(40.0)
          .supplyCurrentLowerLimit(35.0, Seconds.of(1))
          .statorCurrentLimit(45.0)
          .neutralMode(NeutralModeValue.Coast)
          .inverted(true)
          // kP, kI, kD, kS, kV, kA, kG
          .gains(new Gains(8.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0))
          .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(100.0)))
          .build();

  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(CONFIG.rotorToMechanismRatio());

  /**
   * 2026 had no velocity tolerance for the prestage and no jam detection; the tolerance here is the
   * scaffold default and nothing in the 2026 logic reads it.
   */
  public static final RollerSettings SETTINGS = RollerSettings.of(RPM.of(100));

  /** 2026 sim moment of inertia. A guess, not a CAD number. */
  public static final RollerSimModel SIM =
      new RollerSimModel(MOTOR.gearbox(CONFIG.motorCount()), KilogramSquareMeters.of(0.001));
}
