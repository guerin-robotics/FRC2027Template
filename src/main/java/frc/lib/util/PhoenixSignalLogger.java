package frc.lib.util;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

/**
 * Controls CTRE's on-device signal logging, which writes {@code .hoot} files.
 *
 * <p>This is <b>separate from and complementary to AdvantageKit</b>, and it is worth understanding
 * the difference:
 *
 * <ul>
 *   <li><b>AdvantageKit</b> logs what the robot code saw — the {@code @AutoLog} inputs, at loop
 *       rate, in the code's own terms. It is what makes replay work.
 *   <li><b>Hoot</b> logs what the CTRE devices did — every signal on every Phoenix device at the
 *       device's own rate, including internal control-loop state the code never reads. Recording
 *       happens on the CANivore, so requesting it costs no additional CAN bandwidth.
 * </ul>
 *
 * <p>When a Kraken misbehaves and the AdvantageKit log shows the command was correct, the hoot file
 * is where the answer is. It is also what CTRE's Tuner X SysId consumes.
 *
 * <p><b>Start/stop policy.</b> Logging runs only while the robot is enabled. Hoot files are large,
 * and a robot sitting powered in the pit for an hour would otherwise fill the drive with recordings
 * of nothing happening. This does mean the seconds immediately before an enable are not captured —
 * if you are chasing something that happens at boot, call {@link #start()} unconditionally instead.
 *
 * <p><b>Disk.</b> Files go to the USB drive alongside the AdvantageKit logs. If the path cannot be
 * set — usually a missing USB stick — logging is left off rather than allowed to fall back to
 * internal roboRIO storage, which is small and whose exhaustion is a much worse failure than
 * missing diagnostics. An {@link Alert} is raised in that case.
 */
public class PhoenixSignalLogger {

  /**
   * Master switch. Set false to disable hoot logging entirely — for example if the USB drive is
   * short on space at an event and AdvantageKit logs matter more.
   */
  private static final boolean ENABLED = true;

  /** Same USB drive AdvantageKit writes to. */
  private static final String LOG_PATH = "/U/logs/";

  private static final Alert pathFailedAlert =
      new Alert(
          "Phoenix hoot logging is off — could not write to " + LOG_PATH + " (USB drive missing?)",
          AlertType.kWarning);

  private static boolean pathConfigured = false;
  private static boolean running = false;

  private PhoenixSignalLogger() {}

  /**
   * Configures the log path. Call once from the {@code Robot} constructor, before any {@link
   * #start()}.
   *
   * <p>No-op outside {@code REAL} mode — there is no USB drive in simulation, and hoot files from a
   * sim run describe nothing real.
   */
  public static void configure() {
    if (!ENABLED || Constants.currentMode != Mode.REAL) {
      Logger.recordOutput("SignalLogger/Enabled", false);
      return;
    }

    var status = SignalLogger.setPath(LOG_PATH);
    pathConfigured = status.isOK();

    pathFailedAlert.set(!pathConfigured);
    Logger.recordOutput("SignalLogger/Enabled", pathConfigured);
    Logger.recordOutput("SignalLogger/Path", pathConfigured ? LOG_PATH : "unavailable");

    if (!pathConfigured) {
      System.out.println(
          "[SignalLogger] Could not set path to "
              + LOG_PATH
              + " ("
              + status
              + "). Hoot logging disabled — check that the USB drive is present.");
    }
  }

  /** Begins recording. Call on every enable. Safe to call when already running. */
  public static void start() {
    if (!pathConfigured || running) {
      return;
    }
    SignalLogger.start();
    running = true;
    Logger.recordOutput("SignalLogger/Running", true);
  }

  /** Stops recording and closes the current file. Call on disable. Safe to call when stopped. */
  public static void stop() {
    if (!running) {
      return;
    }
    SignalLogger.stop();
    running = false;
    Logger.recordOutput("SignalLogger/Running", false);
  }

  /** True while a hoot file is being written. */
  public static boolean isRunning() {
    return running;
  }
}
