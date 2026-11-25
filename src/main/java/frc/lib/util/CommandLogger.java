package frc.lib.util;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent;

/**
 * Logs which commands are running, from the V3 scheduler's typed events.
 *
 * <p>When a mechanism misbehaves, the first question is "what was commanding it?" — and the answer
 * is often "nothing", "the default", or "something that was interrupted". That is a command
 * scheduled and interrupted by another, a command canceled by the trigger that started it, or one
 * that ran to completion while the mechanism did nothing: three different bugs that look identical
 * in the mechanism's own signals.
 *
 * <p>Publishes:
 *
 * <ul>
 *   <li>{@code Commands/Active} — array of every command name running right now
 *   <li>{@code Commands/ActiveCount} — how many
 *   <li>{@code Commands/All/<name>} — a boolean per command name, for graphing one command's
 *       activity against a mechanism's response on the same timeline
 *   <li>{@code Commands/LastInterrupt} — {@code "<interrupted> by <interrupter>"}, the answer to
 *       "what stopped that?" that V2 could not give
 * </ul>
 *
 * <p>Names come from {@code Command.name()}, which V3 requires at construction. Whether a name is a
 * <em>useful</em> one is still a review item — see {@code .claude/rules/03-commands.md}.
 *
 * <p>Call {@link #start()} once from the {@code Robot} constructor.
 */
public class CommandLogger {

  /** Command name -> how many instances of it are currently scheduled. */
  private static final Map<String, Integer> activeCounts = new LinkedHashMap<>();

  private static boolean started = false;

  private CommandLogger() {}

  /**
   * Registers the scheduler listener. Safe to call more than once; only the first call takes
   * effect.
   */
  public static void start() {
    if (started) {
      return;
    }
    started = true;

    Scheduler.getDefault()
        .addEventListener(
            event -> {
              switch (event) {
                case SchedulerEvent.Scheduled e -> update(e.command(), true);
                case SchedulerEvent.Completed e -> update(e.command(), false);
                case SchedulerEvent.CompletedWithError e -> update(e.command(), false);
                case SchedulerEvent.Canceled e -> update(e.command(), false);
                case SchedulerEvent.Interrupted e ->
                    Logger.recordOutput(
                        "Commands/LastInterrupt",
                        e.command().name() + " by " + e.interrupter().name());
                default -> {}
              }
            });
  }

  private static void update(Command command, boolean starting) {
    String name = command.name();
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
   * Logs which command currently owns a mechanism.
   *
   * <p>Complements {@code Commands/Active}, which is robot-wide. When one mechanism misbehaves, the
   * question is usually "what was driving *this* mechanism at that moment" — including whether it
   * had fallen back to its default command, which the global list does not make obvious.
   *
   * <p>Already called by {@code Drive} and by each mechanism scaffold in {@code template/}, so a
   * mechanism copied from one gets it. Any other mechanism must call it itself, from its {@code
   * periodic()}:
   *
   * <pre>
   * CommandLogger.recordCurrentCommand(getName(), this);
   * </pre>
   *
   * <p>Logs the top-level command only. A group logs as the group's name, not the step inside it
   * that is running.
   *
   * @param name Log key prefix, normally the mechanism name
   * @param mechanism The mechanism to inspect
   */
  public static void recordCurrentCommand(String name, Mechanism mechanism) {
    List<Command> running = mechanism.getRunningCommands();
    Logger.recordOutput(
        name + "/CurrentCommand", running.isEmpty() ? "None" : running.get(0).name());
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
