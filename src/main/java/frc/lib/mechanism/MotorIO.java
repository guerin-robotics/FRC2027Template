package frc.lib.mechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * The hardware boundary for every motor-driven mechanism in this codebase.
 *
 * <p>One interface, one log schema, three implementations — real, Phoenix-simulated, and the replay
 * case, which needs no implementation at all because AdvantageKit feeds the inputs class straight
 * from the log. A mechanism written against this interface behaves identically in all three, which
 * is the property that makes a match log debuggable after the fact.
 *
 * <h2>Why one shared schema instead of one per mechanism</h2>
 *
 * <p>Through 2026 every mechanism owned a private {@code XxxIOInputs}. They were the same fields
 * with different names, and the differences between them were accidents rather than decisions — one
 * logged stator current and called it supply, another never read a sticky fault. A single schema
 * means a fix lands everywhere at once, and an AdvantageScope layout built for one mechanism opens
 * on any of them.
 *
 * <p>The cost is real and worth stating: adding a field here is a schema change for every mechanism
 * at once. That is why this class ships everything {@code .claude/rules/02-hardware.md} requires
 * from the start rather than growing a field at a time.
 *
 * <h2>Units</h2>
 *
 * <p>{@link MotorInputs#position} and {@link MotorInputs#velocity} are in <b>mechanism</b> units,
 * not rotor units. Phoenix does that conversion on the device from {@code SensorToMechanismRatio},
 * which {@link MotorConfig} requires you to state. Everything a command or a tolerance deals with
 * is therefore already in the units the mechanism is described in.
 *
 * <p>Gains are in <b>amps</b>, not volts — every closed loop here runs {@code *TorqueCurrentFOC}.
 * See {@code docs/characterization-and-tuning.md} before touching one.
 */
public interface MotorIO {

  /**
   * Everything read back from the leader motor, once per loop.
   *
   * <p>AdvantageKit logs {@code Measure}-typed fields in SI base units, so {@link
   * MotorInputs#temperature} lands in the log as <b>Kelvin</b> no matter what unit it was
   * constructed with. That is a property of the logger, not a bug here; remember it when reading a
   * log.
   */
  @AutoLog
  class MotorInputs {

    /**
     * Is the leader answering on the bus?
     *
     * <p>Derived from the 50 Hz signal group alone and debounced falling over 0.5 s. Folding the
     * slow groups into that check would make the whole mechanism read disconnected for the first
     * quarter second after boot, before the first sticky-fault frame arrives, and every {@code
     * FaultMonitor} alert tied to it would fire for no reason.
     */
    public boolean connected = false;

    // ---- Power ----

    /** Applied output voltage. */
    public Voltage appliedVolts = Volts.of(0);

    /** Current through the windings. Governs heating; this is the stall indicator. */
    public Current statorAmps = Amps.of(0);

    /** Current drawn from the battery. This is the one that belongs in brownout math. */
    public Current supplyAmps = Amps.of(0);

    /**
     * Torque-producing current.
     *
     * <p>Under {@code *TorqueCurrentFOC} this <b>is</b> the control signal, and stator current is
     * not the same thing. Without it a stalled mechanism and a healthy one drawing similar stator
     * current are indistinguishable in replay.
     *
     * <p>Free on the bus: Phoenix 6 packs {@code TorqueCurrent} into the same status frame as
     * {@code StatorCurrent}, which is already registered at 50 Hz.
     */
    public Current torqueAmps = Amps.of(0);

    // ---- Motion, in mechanism units ----

    /** Mechanism position. Not rotor position — the device applies the ratio. */
    public Angle position = Rotations.of(0);

    /** Mechanism velocity. */
    public AngularVelocity velocity = RotationsPerSecond.of(0);

    /** Thermal headroom. Krakens limit their own output before they fault. */
    public Temperature temperature = Celsius.of(0);

    // ---- Closed-loop diagnostics ----
    //
    // Primitives rather than Measure types on purpose: the units depend on the control mode, so
    // there is no single Measure type that is honest. Both are mechanism rotations or
    // rotations/sec depending on whether position or velocity control is running.
    //
    // Read together with the measured value, these turn "it did not get there" into an answer:
    //
    //   reference tracks the goal, measured lags it -> gains too weak, or saturated
    //   reference never reaches the goal            -> profile too slow, or clamped by a soft limit
    //   reference flat at the wrong number          -> the setpoint itself was wrong
    //
    // Those need different fixes, and guessing between them is how a tuning session gets spent on
    // the wrong gain.

    /** What the closed loop was asking for this instant. */
    public double closedLoopReference = 0.0;

    /** Reference minus measured. */
    public double closedLoopError = 0.0;

    // ---- Sticky faults ----
    //
    // These latch until explicitly cleared, so they record what happened BETWEEN polls.
    // bootDuringEnable in particular is the difference between "the motor stopped working for
    // three seconds and we have no idea why" and "that motor rebooted at 47.2 s, go check its
    // power connector". Nothing in 2026 read any of these.

    /** The device rebooted while the robot was enabled. */
    public boolean stickyBootDuringEnable = false;

    /** Supply voltage collapsed at the device. */
    public boolean stickyUndervoltage = false;

    /** The motor hit its thermal limit. */
    public boolean stickyOverTemp = false;

    /** Device-reported hardware failure. */
    public boolean stickyHardwareFault = false;
  }

  /**
   * What is read back from a follower motor.
   *
   * <p>A follower has no closed loop of its own, so the closed-loop channels are absent. Everything
   * else is here because a follower is a real motor drawing real current and generating real heat,
   * and it can fail independently of its leader — a dead follower looks exactly like an
   * underpowered leader.
   */
  @AutoLog
  class FollowerInputs {

    /**
     * Is the follower answering?
     *
     * <p>From {@code TalonFX.isConnected()}, not from signal status. A follower's signals sit at
     * whatever rate the config chose, and a status built from slow signals reports staleness rather
     * than presence.
     */
    public boolean connected = false;

    public Voltage appliedVolts = Volts.of(0);
    public Current statorAmps = Amps.of(0);
    public Current supplyAmps = Amps.of(0);
    public Current torqueAmps = Amps.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public Temperature temperature = Celsius.of(0);

    public boolean stickyBootDuringEnable = false;
    public boolean stickyUndervoltage = false;
    public boolean stickyOverTemp = false;
    public boolean stickyHardwareFault = false;
  }

  /**
   * What is read back from an absolute encoder, when the mechanism has one.
   *
   * <p>Logged as its own group with its own {@link EncoderInputs#connected}, deliberately. A
   * CANcoder can drop off the bus while its motor stays perfectly healthy — that is precisely the
   * failure worth catching, and it disappears the moment the two are merged into one boolean.
   */
  @AutoLog
  class EncoderInputs {

    public boolean connected = false;

    /** Absolute position, with the magnet offset applied, wrapped into the configured range. */
    public Angle absolutePosition = Rotations.of(0);

    /** Accumulated position. Continuous across wraps; this is what the motor fuses against. */
    public Angle position = Rotations.of(0);

    public AngularVelocity velocity = RotationsPerSecond.of(0);

    /**
     * Magnet field strength, as an ordinal because AdvantageKit replays primitives rather than
     * enums. 0 is a bad magnet, 1 adequate, 2 good.
     *
     * <p>A pivot that drifts over a match with every other signal healthy usually shows up here
     * first.
     */
    public int magnetHealth = 0;
  }

  // ============================================================================================
  // Inputs
  // ============================================================================================

  /** Called every loop. Implementations must populate every field. */
  default void updateInputs(MotorInputs inputs) {}

  /** Called every loop, once per follower. */
  default void updateFollowerInputs(int index, FollowerInputs inputs) {}

  /** Called every loop when {@link #hasAbsoluteEncoder()} is true. */
  default void updateEncoderInputs(EncoderInputs inputs) {}

  /** How many followers this motor group has. The leader is not counted. */
  default int followerCount() {
    return 0;
  }

  /** Whether this motor group has an absolute encoder worth logging separately. */
  default boolean hasAbsoluteEncoder() {
    return false;
  }

  // ============================================================================================
  // Control
  // ============================================================================================

  /** Open-loop voltage. Bring-up and characterization; match logic should close a loop. */
  default void setVoltage(Voltage volts) {}

  /** Open-loop torque current. The honest open-loop mode under FOC. */
  default void setTorqueCurrent(Current amps) {}

  /** Open-loop duty cycle, -1 to 1. */
  default void setDutyCycle(double fraction) {}

  /** Closed-loop velocity through the Motion Magic velocity profile. */
  default void setVelocity(AngularVelocity velocity) {}

  /** Closed-loop position through the Motion Magic position profile. */
  default void setPosition(Angle position) {}

  /**
   * Closed-loop position with no profile at all.
   *
   * <p>For short corrective moves where the profile costs more than it buys. Do not reach for this
   * to make a mechanism faster — a profile that is too slow is fixed by raising the acceleration,
   * which stays inside the limits the mechanism was characterized against.
   */
  default void setUnprofiledPosition(Angle position) {}

  /** Stop commanding output. Neutral behaviour follows the configured neutral mode. */
  default void stop() {}

  /**
   * Switch neutral mode at runtime.
   *
   * <p>The one real use is coasting a heavy mechanism so it can be moved by hand in the pit.
   * Competition behaviour comes from the configured mode, not from here.
   */
  default void setBrakeMode(boolean brake) {}

  /**
   * Tell the motor to read the mechanism's current physical position as {@code position}.
   *
   * <p>This is how a zeroing routine lands. It has no effect on a mechanism whose feedback comes
   * from a fused absolute encoder, because there the encoder already knows where it is.
   */
  default void setEncoderPosition(Angle position) {}

  // ============================================================================================
  // Live tuning
  // ============================================================================================

  /**
   * Push a new gain set to the device.
   *
   * <p>Gains live in flash on the motor controller, not in RAM on the roboRIO — that is why the
   * Talon closes its loop at 1 kHz instead of at our 50 Hz. The cost is that this is a blocking CAN
   * transaction, so callers must gate it on an actual change rather than calling it every loop.
   */
  default void setGains(Gains gains) {}

  /**
   * Push a new Motion Magic profile to the device. Same blocking-CAN caveat as {@link #setGains}.
   */
  default void setMotionProfile(MotionProfile profile) {}

  /**
   * Raise or lower the supply current limit at runtime.
   *
   * <p>A safety limit, not a tuning knob. See {@code .claude/rules/00-safety.md} before using it.
   */
  default void setSupplyCurrentLimit(Current limit) {}
}
