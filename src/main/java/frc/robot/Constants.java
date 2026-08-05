// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /**
   * Set true to skip DriverStation/HAL access in unit tests. Read by {@link
   * frc.lib.AllianceFlipUtil#refresh()}.
   */
  public static boolean disableHAL = false;

  /**
   * Enables dashboard-adjustable constants ({@code LoggedTunableNumber} / {@code
   * LoggedTunableBoolean}).
   *
   * <p><b>MUST BE FALSE FOR COMPETITION.</b> With it on, every tunable publishes to NetworkTables
   * and reads back from it, which puts dashboard traffic in the 20 ms loop and leaves the robot one
   * stray edit away from a different gain set. With it off, tunables return their compiled-in
   * defaults and cost nothing.
   *
   * <p>Turning this on is a deliberate act at the start of a tuning session, and turning it off
   * again is part of the pre-competition checklist. Values discovered while it is on live only in
   * NetworkTables — write them into the constants and commit them, or they vanish on reboot.
   */
  public static final boolean tuningMode = false;
}
