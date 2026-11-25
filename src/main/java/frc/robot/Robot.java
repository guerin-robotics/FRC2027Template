// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.BatteryLogger;
import frc.lib.util.CANBusMonitor;
import frc.lib.util.CommandLogger;
import frc.lib.util.FaultMonitor;
import frc.lib.util.LoggedTracer;
import frc.lib.util.LoopTimeMonitor;
import frc.lib.util.MatchMetadataLogger;
import frc.lib.util.PhoenixSignalLogger;
import frc.robot.generated.TunerConstants;
import frc.robot.opmodes.AutoOpMode;
import frc.robot.opmodes.OpModes;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedOpModeRobot {
  private final RobotContainer robotContainer;
  private AutoOpMode runningAuto;

  /** Shared battery/energy logger — subsystems call {@code reportCurrentUsage()} each loop. */
  public static final BatteryLogger batteryLogger = new BatteryLogger();

  /** Loop-time watchdog. 2026 ran ~30 Hz all season without anyone noticing. */
  private final LoopTimeMonitor loopMonitor = new LoopTimeMonitor();

  /** CAN health for the swerve bus — bandwidth problems look like sensor faults without this. */
  private final CANBusMonitor canBusMonitor = new CANBusMonitor(TunerConstants.kCANBus, "Canivore");

  /** Writes event and match identity into the log so a pile of logs can be triaged. */
  private final MatchMetadataLogger matchMetadataLogger = new MatchMetadataLogger();

  // ---- Disabled coast ----
  //
  // Brake while enabled so the robot holds position; coast a few seconds after disable so it
  // can be pushed off the field and around the pit. The delay matters: coasting the instant
  // the robot is disabled means it keeps rolling after an emergency stop, which is exactly
  // when you want it to stop.
  private static final double COAST_DELAY_SECONDS = 3.0;
  private final Timer disabledTimer = new Timer();
  private boolean hasCoasted = false;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // RobotState is a singleton accessed via static getInstance() calls, so AdvantageKit's
    // automatic field scanner cannot find it through the Robot -> RobotContainer object graph.
    // Register it manually so @AutoLogOutput annotations on its methods are picked up and
    // logged every loop cycle. Any other singleton you add needs the same treatment.
    AutoLogOutputManager.addObject(RobotState.getInstance());

    // Log which commands are running. Registers scheduler hooks; must happen before any
    // command is scheduled or the first few will be missed.
    CommandLogger.start();
    Scheduler.getDefault()
        .addEventListener(
            event -> {
              if (runningAuto != null) {
                runningAuto.onSchedulerEvent(event);
              }
            });

    // Configure CTRE hoot logging. Recording itself starts on enable — see PhoenixSignalLogger
    // for why it is not always-on.
    PhoenixSignalLogger.configure();

    // Instantiate our RobotContainer. This will perform all our button bindings.
    robotContainer = new RobotContainer();

    // Offer the OpModes on the Driver Station: teleop, one autonomous per routine and side, and
    // the utility (characterization) routines.
    OpModes.register(this, robotContainer);
    publishOpModes();

    // Short pre-match checklist, visible from the driver station without paper.
    // TODO: fill in the device count and add one line per 2027 mechanism, in the order the
    // drive team exercises them. Keep this in sync with docs/pre-match-checklist.md — that
    // is the full version; this is the pit-side reminder.
    Telemetry.log(
        "Pre-Match Checklist",
        """
        Phoenix Tuner: ___ devices
        USB drive in (no USB = no log)
        Correct code deployed, GitDirty clean
        Not in Test mode; no tuning OpMode selected
        Drive: all 4 modules
        Vision: tags detected
        Correct auto selected
        Robot on auto start pose
        Swap battery
        """);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Start the loop-time measurement first so it covers everything below.
    loopMonitor.begin();
    LoggedTracer.reset();

    // Refresh the cached alliance color once per loop so that AllianceFlipUtil.shouldFlip()
    // doesn't call DriverStation.getAlliance() (which allocates an Optional) dozens of times
    // per cycle. This must run BEFORE the scheduler, since commands read the cached value.
    AllianceFlipUtil.refresh();

    // Read every subsystem's inputs BEFORE the scheduler. Commands V3 has no Subsystem.periodic()
    // and nothing calls one for us, so this is the only place Logger.processInputs() runs for
    // Drive and Vision. Removing it silently breaks replay. See RobotContainer.periodic().
    robotContainer.periodic();

    // Runs the V3 Scheduler: polls triggers, schedules and runs commands, and removes finished
    // or interrupted ones. It runs no subsystem periodic() methods — see above.
    Scheduler.getDefault().run();
    // Whatever the scheduler spent beyond the subsystems' own records (Drive, Vision, and any
    // mechanism that calls LoggedTracer.record) is triggers and command execute()s.
    LoggedTracer.record("Scheduler");

    // Publish the set of commands that ran this cycle (hooks registered in the constructor)
    CommandLogger.periodic();

    // Update battery logger with voltage, then log after scheduler. Systemcore exposes no
    // controller input-current reading, so "Controls/roboRIO" stays 0 A until one exists or the
    // draw is measured; do not read brownout current budgets as complete without it.
    batteryLogger.setBatteryVoltage(RobotController.getBatteryVoltage());
    batteryLogger.periodicAfterScheduler();
    LoggedTracer.record("Logging");

    // Health and diagnostics. All of these self-throttle or short-circuit, so they are cheap
    // to call every loop.
    canBusMonitor.periodic();
    matchMetadataLogger.periodic();
    FaultMonitor.getInstance().periodic();
    LoggedTracer.record("Diagnostics");

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    // Must be last — this is what closes the loop-time measurement.
    loopMonitor.end();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    disabledTimer.restart();
    hasCoasted = false;

    // Close the hoot file. Keeps recordings bounded to matches instead of accumulating while
    // the robot sits powered in the pit.
    PhoenixSignalLogger.stop();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Coast only after the robot has had time to actually stop moving.
    if (!hasCoasted && disabledTimer.hasElapsed(COAST_DELAY_SECONDS)) {
      robotContainer.setDriveBrakeMode(false);
      hasCoasted = true;
    }

    // The auto preview is drawn by AutoOpMode.disabledPeriodic() while an auto OpMode is selected.
  }

  /** Brake on every enable, whichever mode we are entering. */
  private void enableBrakeMode() {
    robotContainer.setDriveBrakeMode(true);
    hasCoasted = false;
    disabledTimer.stop();
  }

  /** Tracks the active autonomous OpMode for scheduler outcome logging. */
  public void trackAuto(AutoOpMode auto) {
    runningAuto = auto;
  }

  /** Everything that should happen on any enable. Each OpMode calls this from its start(). */
  public void onEnable() {
    enableBrakeMode();
    PhoenixSignalLogger.start();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
