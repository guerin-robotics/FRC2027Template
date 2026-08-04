package frc.robot.subsystems.drive;

/**
 * Drive tuning values that are not part of the Tuner X generated {@code TunerConstants}.
 *
 * <p>Module positions, gear ratios, gains and CAN IDs all live in {@code TunerConstants} — that is
 * the single source of truth, and {@code Drive.getModuleTranslations()} reads from it. Do not
 * redeclare drivetrain geometry here; the 2026 version of this file carried a duplicate hardcoded
 * 11×11 inch module layout that had to be kept in sync by hand.
 */
public class DriveConstants {
  /**
   * Maximum linear speed (m/s) used by {@code DriveCommands.joystickDriveLimited}.
   *
   * <p>In 2026 this was the "precision mode" speed used while the intake was extended.
   */
  public static double limitedVelo = 0.1;

  /**
   * Exponent applied to the rotation joystick axis before scaling to angular velocity.
   *
   * <p>1.0 is linear; higher values give finer control near center at the cost of responsiveness.
   * This is mutable so a dashboard driver-preset chooser can set it at {@code teleopInit} — the
   * 2026 drivers ran 1.35 and 2.0.
   */
  public static double rotationExponent = 2.0;
}
