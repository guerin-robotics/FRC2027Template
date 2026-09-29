package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * The 2026 shot-tuning and demo modes, ported from Rebuilt2026 {@code
 * HardwareConstants.TuningConstants}.
 *
 * <p>Not to be confused with {@link Constants#tuningMode}, which enables dashboard-tunable gains.
 * These two change what the <i>shoot button does</i>:
 *
 * <ul>
 *   <li><b>Shot tuning</b> — the shoot button fires at a fixed speed and hood angle ({@code
 *       Constants.Setpoints.FLYWHEEL_TUNING_VELOCITY} / {@code HOOD_TUNING_POSITION}) instead of
 *       the distance maps, for building those maps.
 *   <li><b>Demo</b> — the demo button fires the fixed shot, the hub-shift timer is ignored, and the
 *       pass-align binding is suppressed.
 * </ul>
 *
 * <p>The requested values are compile-time in {@link Constants.ShotModeRequests}. {@link #update()}
 * runs every loop and, when {@code AT_COMP} is set, forces both off while the FMS is attached — so
 * a mode left on by accident cannot reach a real match.
 */
public final class ShotModes {

  private ShotModes() {}

  private static boolean shotTuning = false;
  private static boolean demo = false;

  /** Effective shot-tuning mode this loop. */
  public static boolean isShotTuning() {
    return shotTuning;
  }

  /** Effective demo mode this loop. */
  public static boolean isDemo() {
    return demo;
  }

  /** Call once per loop from {@code Robot.robotPeriodic()}. */
  public static void update() {
    if (Constants.ShotModeRequests.AT_COMP && DriverStation.isFMSAttached()) {
      shotTuning = false;
      demo = false;
    } else {
      shotTuning = Constants.ShotModeRequests.SHOT_TUNING;
      demo = Constants.ShotModeRequests.DEMO;
    }
  }
}
