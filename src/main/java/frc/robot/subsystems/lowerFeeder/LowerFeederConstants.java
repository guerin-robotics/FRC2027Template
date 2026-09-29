package frc.robot.subsystems.lowerFeeder;

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
 * How the 2026 lower feeder is built.
 *
 * <p>Every value is carried over from Rebuilt2026 {@code LowerFeederConstants}. Gains are in
 * <b>amps</b> — the loop runs {@code MotionMagicVelocityTorqueCurrentFOC}.
 *
 * <p>Brake, unlike every other 2026 roller. The 2026 file marked the ratio, inversion and gains
 * "needs updated"; they are carried over as they ran, not as they were meant to be.
 */
public final class LowerFeederConstants {

  private LowerFeederConstants() {}

  public static final String NAME = "LowerFeeder";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** Total reduction, motor to mechanism. */
  public static final double GEAR_RATIO = 24.0 / 11.0;

  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROLLER)
          .canId(Constants.CanIds.LOWER_FEEDER_MOTOR, Constants.CanIds.RIO_BUS)
          .sensorToMechanismRatio(GEAR_RATIO)
          .supplyCurrentLimit(40.0)
          .supplyCurrentLowerLimit(35.0, Seconds.of(1))
          .statorCurrentLimit(40.0)
          .neutralMode(NeutralModeValue.Brake)
          .inverted(false)
          // kP, kI, kD, kS, kV, kA, kG
          .gains(new Gains(16.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0))
          // As of quals 27 IRI: reverted to 120 from 150.
          .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(120.0)))
          .build();

  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(CONFIG.rotorToMechanismRatio());

  /**
   * 2026 had no velocity tolerance for the feeders and no jam detection; the tolerance here is the
   * scaffold default and nothing in the 2026 logic reads it.
   */
  public static final RollerSettings SETTINGS = RollerSettings.of(RPM.of(100));

  /** 2026 sim moment of inertia. A guess, not a CAD number. */
  public static final RollerSimModel SIM =
      new RollerSimModel(MOTOR.gearbox(CONFIG.motorCount()), KilogramSquareMeters.of(0.001));
}
