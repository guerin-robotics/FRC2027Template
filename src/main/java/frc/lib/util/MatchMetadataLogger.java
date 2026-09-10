package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Records which match a log belongs to, once the driver station tells us.
 *
 * <p><b>Why this exists.</b> The 2026 logs were identifiable only by their timestamp filename.
 * Answering "pull up quals 12 from the second event" meant cross-referencing wall-clock times
 * against the match schedule by hand, for every log, every time. This writes the event name, match
 * type and number directly into the log.
 *
 * <p>This cannot be {@code Logger.recordMetadata()} — metadata has to be set before {@code
 * Logger.start()}, and at that point the driver station has not connected and knows nothing about
 * the match. So it is recorded as output the first time the information becomes available, and
 * again whenever it changes (a replay, or a second event on the same power cycle).
 *
 * <p>Publishes under {@code Match/}.
 */
public class MatchMetadataLogger {

  private String lastSignature = "";

  /** Call once per loop from {@code robotPeriodic()}. Cheap; only writes when something changes. */
  public void periodic() {
    String eventName = DriverStation.getEventName();
    var matchType = DriverStation.getMatchType();
    int matchNumber = DriverStation.getMatchNumber();
    int replayNumber = DriverStation.getReplayNumber();

    // Only re-log when the identity of the match actually changes.
    String signature = eventName + "|" + matchType + "|" + matchNumber + "|" + replayNumber;
    if (signature.equals(lastSignature)) {
      return;
    }
    lastSignature = signature;

    Logger.recordOutput("Match/EventName", eventName);
    Logger.recordOutput("Match/MatchType", matchType.toString());
    Logger.recordOutput("Match/MatchNumber", matchNumber);
    Logger.recordOutput("Match/ReplayNumber", replayNumber);
    Logger.recordOutput("Match/FMSAttached", DriverStation.isFMSAttached());
    Logger.recordOutput(
        "Match/Alliance", DriverStation.getAlliance().map(Enum::toString).orElse("Unknown"));
    Logger.recordOutput("Match/DriverStationLocation", DriverStation.getLocation().orElse(0));
  }
}
