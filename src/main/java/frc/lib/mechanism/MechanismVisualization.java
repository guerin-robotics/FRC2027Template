package frc.lib.mechanism;

/**
 * The single switch for every mechanism visualizer in the library.
 *
 * <p>Visualizers are diagnostic. They publish to NetworkTables every loop, which is not free, and
 * the 2026 robot spent its season closer to 30 Hz than the 50 Hz budget with {@code Drive} and
 * {@code Vision} already dominating {@code robotPeriodic}. One flag in one place means switching
 * them all off is a one-line change rather than a hunt through every subsystem.
 *
 * <p>They are most valuable during bring-up and in simulation, where the picture is the only way to
 * see what the model is actually doing. See {@code docs/new-mechanism-bringup.md}.
 *
 * <p><b>Turn this off for competition</b> unless you have checked {@code LoopTiming/} with it on
 * and the budget is intact.
 */
public final class MechanismVisualization {

  private MechanismVisualization() {}

  /**
   * Whether mechanism visualizers publish.
   *
   * <p>Left on by default because a template's first job is bring-up, and a visualizer nobody knew
   * existed helps nobody. The pre-match checklist is where this gets turned off.
   */
  public static final boolean ENABLED = true;
}
