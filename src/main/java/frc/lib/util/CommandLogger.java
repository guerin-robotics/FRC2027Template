package frc.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Logs which commands are running, so a match log can answer "what was the robot trying to do at
 * this timestamp?".
 *
 * <p>WPILib's scheduler does not log this on its own. The 2026 season had no way to see, after the
 * fact, whether a command never started, started and was immediately interrupted, or ran to
 * completion while the mechanism did nothing — which are three completely different bugs that look
 * identical in the mechanism's own signals.
 *
 * <p>Publishes:
 *
 * <ul>
 *   <li>{@code Commands/Active} — array of every command name running right now
 *   <li>{@code Commands/ActiveCount} — how many
 *   <li>{@code Commands/All/<name>} — a boolean per command name, for graphing one command's
 *       activity against a mechanism's response on the same timeline
 * </ul>
 *
 * <p>Names come from {@code Command.getName()}, which is why every factory must call {@code
 * .withName(...)} — see {@code .claude/rules/03-commands.md}. Unnamed commands log as their class
 * name and are nearly useless here.
 *
 * <p>Call {@link #start()} once from the {@code Robot} constructor.
 */
public class CommandLogger {

  /** Command name -> how many instances of it are currently scheduled. */
  private static final Map<String, Integer> activeCounts = new LinkedHashMap<>();

  private static boolean started = false;

  private CommandLogger() {}

  /**
   * Registers the scheduler hooks. Safe to call more than once; only the first call takes effect.
   */
  public static void start() {
    if (started) {
      return;
    }
    started = true;

    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize(command -> update(command, true));
    scheduler.onCommandFinish(command -> update(command, false));
    scheduler.onCommandInterrupt(command -> update(command, false));
  }

  private static void update(Command command, boolean starting) {
    String name = command.getName();
    int count = activeCounts.getOrDefault(name, 0) + (starting ? 1 : -1);

    if (count <= 0) {
      activeCounts.remove(name);
    } else {
      activeCounts.put(name, count);
    }

    // Per-name boolean. Keep publishing false after a command ends so the channel stays in the
    // log rather than disappearing, which makes it graphable across the whole match.
    Logger.recordOutput("Commands/All/" + name, count > 0);
  }

  /**
   * Logs which command currently owns a subsystem.
   *
   * <p>Complements {@code Commands/Active}, which is robot-wide. When one mechanism misbehaves, the
   * question is usually "what was driving *this* subsystem at that moment" — including whether it
   * had fallen back to its default command, which the global list does not make obvious.
   *
   * <p>Call from the subsystem's {@code periodic()}:
   *
   * <pre>
   * CommandLogger.recordCurrentCommand("Drive", this);
   * </pre>
   *
   * @param name Log key prefix, normally the subsystem name
   * @param subsystem The subsystem to inspect
   */
  public static void recordCurrentCommand(String name, Subsystem subsystem) {
    Command current = CommandScheduler.getInstance().requiring(subsystem);
    Logger.recordOutput(name + "/CurrentCommand", current == null ? "None" : current.getName());
  }

  /**
   * Publishes the current active set. Call once per loop from {@code robotPeriodic()}, after the
   * scheduler has run.
   */
  public static void periodic() {
    Logger.recordOutput("Commands/Active", activeCounts.keySet().toArray(new String[0]));
    Logger.recordOutput("Commands/ActiveCount", activeCounts.size());
  }
}
