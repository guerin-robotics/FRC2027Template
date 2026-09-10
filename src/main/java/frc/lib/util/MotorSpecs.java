package frc.lib.util;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.function.IntFunction;

/**
 * The motors this team uses, and the arithmetic for turning one into a mechanism's theoretical top
 * speed.
 *
 * <p>Every mechanism should compute its own {@code MAX_SPEED_RPM} rather than guessing one:
 *
 * <pre>{@code
 * public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;
 * public static final double GEAR_RATIO = 15.0;
 * public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(GEAR_RATIO);
 *
 * // ...and the same enum builds the sim model, so the two cannot disagree:
 * public static final DCMotor SIM_MOTOR = MOTOR.gearbox(2);
 * }</pre>
 *
 * <p>Knowing the number matters for two reasons. It is the ceiling a Motion Magic cruise velocity
 * has to sit under — asking for more than the motor can physically deliver does not fail loudly, it
 * just saturates and the profile silently stops being followed. And it is the sanity check on a
 * gear ratio: if the computed top speed is nowhere near what the mechanism needs to do, the ratio
 * is wrong, and finding that in a constants file beats finding it on the practice field.
 *
 * <h2>Values come from WPILib</h2>
 *
 * <p>Free speeds are read from {@link DCMotor} rather than typed in here, so the number driving the
 * speed math is the same number driving the sim model. Hand-entered specs drift from WPILib's when
 * a vendor revises them, and the resulting disagreement between "what sim does" and "what the
 * constants claim" is miserable to track down.
 *
 * <p>Only the FOC variants are listed. Every closed loop in this codebase runs {@code
 * TorqueCurrentFOC}, which requires FOC; the non-FOC numbers would be the wrong ceiling.
 *
 * <h2>These are free speeds, not achievable speeds</h2>
 *
 * <p>Free speed is the motor spinning with nothing attached. A real mechanism carries load,
 * friction and a current limit, so it will not reach this. Treat it as a hard ceiling, not a target
 * — 80% is an optimistic working figure for a lightly loaded mechanism, and a heavily loaded one
 * can sit far below.
 *
 * <p>The drivetrain is the standing example: the 2026 robot was voltage-saturated around 3.8 m/s,
 * well under what the free-speed arithmetic predicted.
 */
public enum MotorSpecs {

  /** Kraken X60 with FOC. 5800 RPM free speed. */
  KRAKEN_X60_FOC(DCMotor::getKrakenX60Foc),

  /** Kraken X44 with FOC. 7368 RPM free speed. */
  KRAKEN_X44_FOC(DCMotor::getKrakenX44Foc);

  private final IntFunction<DCMotor> factory;

  /** WPILib's model of a single one of these motors. */
  public final DCMotor single;

  /** Free speed of a single motor, in RPM, at the rotor. */
  public final double freeSpeedRpm;

  MotorSpecs(IntFunction<DCMotor> factory) {
    this.factory = factory;
    this.single = factory.apply(1);
    this.freeSpeedRpm = Units.radiansPerSecondToRotationsPerMinute(single.freeSpeedRadPerSec);
  }

  /**
   * Theoretical top speed at the mechanism, in RPM.
   *
   * <p>Motor count does not appear here on purpose. Adding a second motor doubles available torque,
   * not free speed — two motors on one shaft reach the same no-load RPM as one. If a mechanism
   * cannot reach its predicted speed under load, more motors help; if it cannot reach it unloaded,
   * the gear ratio is wrong.
   *
   * @param gearRatio Motor rotations per mechanism rotation. A 15:1 reduction is {@code 15.0}
   */
  public double maxMechanismRpm(double gearRatio) {
    return freeSpeedRpm / gearRatio;
  }

  /**
   * The sim model for {@code count} of these motors on one shaft.
   *
   * <p>This scales stall torque and current by the count and leaves free speed alone, which is what
   * happens physically. It is not the same as {@code DCMotor.withReduction()} — that models a
   * gearbox, trading speed for torque.
   *
   * @param count How many motors drive the mechanism, leader and followers together
   */
  public DCMotor gearbox(int count) {
    return factory.apply(count);
  }
}
