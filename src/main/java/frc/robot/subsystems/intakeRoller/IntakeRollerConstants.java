package frc.robot.subsystems.intakeRoller;

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
 * How the 2026 intake roller is built.
 *
 * <p>Every value is carried over from Rebuilt2026 {@code intakeRollerConstants}. Gains are in
 * <b>amps</b> — the loop runs {@code MotionMagicVelocityTorqueCurrentFOC}.
 *
 * <p>Two motors: 43 follows 42, opposed. 2026 never registered the follower's signals, so they sat
 * at the 4 Hz optimize floor; the library default is that same 4 Hz, now on purpose.
 *
 * <p>The 100 A stator limit is the 2026 value (60, then 40 before IRI, then 55, then 100). It is
 * high for a roller; see .claude/rules/00-safety.md before touching it in either direction.
 */
public final class IntakeRollerConstants {

  private IntakeRollerConstants() {}

  public static final String NAME = "IntakeRoller";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** Total reduction, motor to mechanism. */
  public static final double GEAR_RATIO = 24.0 / 11.0;

  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROLLER)
          .canId(Constants.CanIds.INTAKE_ROLLER_LEADER, Constants.CanIds.RIO_BUS)
          .follower(Constants.CanIds.INTAKE_ROLLER_FOLLOWER, true)
          .sensorToMechanismRatio(GEAR_RATIO)
          .supplyCurrentLimit(40.0)
          .supplyCurrentLowerLimit(35.0, Seconds.of(1))
          .statorCurrentLimit(100.0)
          .neutralMode(NeutralModeValue.Coast)
          .inverted(false)
          // kP, kI, kD, kS, kV, kA, kG
          .gains(new Gains(0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0))
          .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(100.0)))
          .build();

  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(CONFIG.rotorToMechanismRatio());

  /**
   * 2026 had no velocity tolerance for the intake roller and no jam detection; the tolerance here
   * is the scaffold default and nothing in the 2026 logic reads it.
   */
  public static final RollerSettings SETTINGS = RollerSettings.of(RPM.of(100));

  /** 2026 sim moment of inertia. A guess, not a CAD number. */
  public static final RollerSimModel SIM =
      new RollerSimModel(MOTOR.gearbox(CONFIG.motorCount()), KilogramSquareMeters.of(0.001));
}
