package frc.robot.subsystems.transport;

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
 * How the 2026 transport (hopper floor belt) is built.
 *
 * <p>Every value is carried over from Rebuilt2026 {@code TransportConstants} and {@code
 * TransportIOReal}. Gains are in <b>amps</b> — the loop runs {@code
 * MotionMagicVelocityTorqueCurrentFOC}.
 */
public final class TransportConstants {

  private TransportConstants() {}

  public static final String NAME = "Transport";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** Total reduction, motor to belt roller. */
  public static final double GEAR_RATIO = 33.0 / 11.0;

  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROLLER)
          .canId(Constants.CanIds.TRANSPORT_MOTOR, Constants.CanIds.RIO_BUS)
          .sensorToMechanismRatio(GEAR_RATIO)
          .supplyCurrentLimit(40.0)
          .supplyCurrentLowerLimit(35.0, Seconds.of(1))
          .statorCurrentLimit(40.0)
          .neutralMode(NeutralModeValue.Coast)
          .inverted(false)
          // kP, kI, kD, kS, kV, kA, kG
          .gains(new Gains(4.2, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0))
          // As of quals 27 IRI: reverted to 120 from 150.
          .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(120.0)))
          .build();

  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(CONFIG.rotorToMechanismRatio());

  /**
   * 2026 had no velocity tolerance for the transport and no jam detection; the tolerance here is
   * the scaffold default and nothing in the 2026 logic reads it.
   */
  public static final RollerSettings SETTINGS = RollerSettings.of(RPM.of(100));

  /** 2026 sim moment of inertia. A guess, not a CAD number. */
  public static final RollerSimModel SIM =
      new RollerSimModel(MOTOR.gearbox(CONFIG.motorCount()), KilogramSquareMeters.of(0.001));
}
