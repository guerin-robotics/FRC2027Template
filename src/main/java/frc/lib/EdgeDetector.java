package frc.lib;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayDeque;
import java.util.function.BooleanSupplier;

/**
 * Edge detection and event counting over a boolean signal.
 *
 * <p>WPILib's {@code Trigger} already gives you {@code onTrue} / {@code onFalse} for scheduling
 * commands. These are for the other case: you want the edge as a plain value inside a periodic
 * method or a condition, without involving the scheduler.
 *
 * <p>Each detector samples its source once per {@code getAsBoolean()} call, so call it exactly once
 * per loop. Calling twice in the same loop consumes the edge in the first call and the second sees
 * nothing.
 */
public class EdgeDetector {

  private EdgeDetector() {}

  /** True on the loop where the source went false to true. */
  public static class Rising implements BooleanSupplier {
    private final BooleanSupplier source;
    private boolean previous = false;

    public Rising(BooleanSupplier source) {
      this.source = source;
    }

    public static Rising of(BooleanSupplier source) {
      return new Rising(source);
    }

    @Override
    public boolean getAsBoolean() {
      boolean current = source.getAsBoolean();
      boolean rising = current && !previous;
      previous = current;
      return rising;
    }
  }

  /** True on the loop where the source went true to false. */
  public static class Falling implements BooleanSupplier {
    private final BooleanSupplier source;
    private boolean previous = false;

    public Falling(BooleanSupplier source) {
      this.source = source;
    }

    public static Falling of(BooleanSupplier source) {
      return new Falling(source);
    }

    @Override
    public boolean getAsBoolean() {
      boolean current = source.getAsBoolean();
      boolean falling = !current && previous;
      previous = current;
      return falling;
    }
  }

  /**
   * Counts rising edges within a sliding time window.
   *
   * <p>This is the useful primitive for jam and fault detection, because a single threshold
   * crossing is usually noise and a repeated one is usually real. "Stator current spiked above the
   * jam threshold three times in the last two seconds" is a far better trigger than "stator current
   * is above the jam threshold right now", which fires on every acceleration.
   *
   * <pre>
   * private final EdgeDetector.Count jamEvents =
   *     new EdgeDetector.Count(() -&gt; inputs.statorAmps &gt; JAM_AMPS, 2.0);
   *
   * // in periodic():
   * if (jamEvents.count() &gt;= 3) {
   *   // jammed — stop and log
   * }
   * </pre>
   */
  public static class Count {
    private final Rising source;
    private final double windowSeconds;
    private final ArrayDeque<Double> events = new ArrayDeque<>();

    /**
     * @param source The signal to watch
     * @param windowSeconds How far back to count
     */
    public Count(BooleanSupplier source, double windowSeconds) {
      this.source = Rising.of(source);
      this.windowSeconds = windowSeconds;
    }

    /**
     * Samples the source and returns how many rising edges occurred within the window.
     *
     * <p>Call once per loop — it advances the underlying edge detector.
     */
    public int count() {
      double now = Timer.getFPGATimestamp();

      if (source.getAsBoolean()) {
        events.addLast(now);
      }

      // Drop anything that has aged out.
      while (!events.isEmpty() && now - events.peekFirst() > windowSeconds) {
        events.removeFirst();
      }

      return events.size();
    }

    /** Forgets all recorded events, e.g. after handling a jam. */
    public void reset() {
      events.clear();
    }
  }
}
