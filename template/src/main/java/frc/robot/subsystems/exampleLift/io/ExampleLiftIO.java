package frc.robot.subsystems.exampleLift.io;

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
 * IO interface for a linear position-controlled mechanism.
 *
 * <p><b>RULE:</b> hardware calls (TalonFX, CANcoder) belong in the IO implementation, never in the
 * subsystem class. That is what makes AdvantageKit log replay work — see {@code
 * .claude/rules/01-architecture.md}.
 *
 * <p><b>Everything here is in mechanism ROTATIONS, not inches.</b> The IO layer speaks the units
 * the hardware speaks; the conversion to inches happens once, in {@code ExampleLiftConstants}, and
 * is applied by the subsystem. Putting inches in the log schema would mean the recorded value
 * depends on a drum diameter that might be wrong, and a replay could not be re-interpreted after
 * that number was corrected.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename {@code ExampleLift} throughout, add a field for every signal
 * worth seeing in a match log, and add a default no-op for every control output.
 */
public interface ExampleLiftIO {

  @AutoLog
  class ExampleLiftIOInputs {

    /**
     * Is the motor answering?
     *
     * <p>Derived from the 50 Hz signal group alone and debounced. Register with {@code
     * FaultMonitor} so a motor that drops off the bus mid-match announces itself in the pit.
     */
    public boolean connected = false;

    /**
     * True only if every required startup configuration write was confirmed.
     *
     * <p>This records the result at construction, independent of live bus health. Logging it as an
     * input lets replay apply the same closed-loop gate.
     */
    public boolean configured = false;

    // ---- Power and motion ----

    public Voltage motorVoltage = Volts.of(0);

    /**
     * Current through the windings. Governs heating; this is the stall indicator.
     *
     * <p>On this mechanism it is also how a hard stop is recognised during zeroing — though the
     * routine keys off {@link #motorSupplyAmps} together with velocity, because a lift creeping
     * downward at 3 V is not in the chopping regime where the two diverge much.
     */
    public Current motorStatorAmps = Amps.of(0);

    /** Current drawn from the battery. This is the one that belongs in brownout math. */
    public Current motorSupplyAmps = Amps.of(0);

    /**
     * Torque-producing current.
     *
     * <p>Under {@code MotionMagicTorqueCurrentFOC} this <b>is</b> the control signal, and stator
     * current is not the same thing. On a lift it reads directly as load: a correctly-tuned kG
     * shows as a roughly constant torque current while holding at any height.
     *
     * <p>Costs no extra CAN bandwidth — Phoenix 6 packs it into the same status frame as {@code
     * StatorCurrent}.
     */
    public Current motorTorqueCurrentAmps = Amps.of(0);

    /**
     * Drum position in mechanism rotations.
     *
     * <p><b>Relative, and meaningless until the mechanism has been zeroed.</b> It reads zero at
     * boot wherever the carriage happens to be sitting. That is the whole reason this scaffold has
     * a zeroing routine — see {@code ExampleLiftCommands.zero()}.
     */
    public Angle motorPosition = Rotations.of(0);

    public AngularVelocity motorVelocity = RotationsPerSecond.of(0);

    /**
     * Device temperature.
     *
     * <p>Worth watching on a lift more than on most mechanisms: holding at height is a sustained
     * stall against gravity, which is this robot's worst thermal case.
     *
     * <p>GOTCHA: AdvantageKit logs {@code Measure}-typed fields in SI base units, so this logs in
     * <b>Kelvin</b>. To read Celsius in AdvantageScope, log a primitive with the unit in the name.
     */
    public Temperature motorTemperature = Celsius.of(0);

    // ---- Closed-loop diagnostics ----
    //
    // Log these on every closed-loop mechanism. Without them a log shows the carriage at the wrong
    // height and gives you no way to tell which of three different problems you have:
    //
    //   reference tracks the goal, measured lags it  -> gains too weak, or saturated
    //   reference never reaches the goal             -> profile too slow, or clamped by a soft
    // limit
    //   reference flat at the wrong number           -> the setpoint itself was wrong
    //
    // Primitives rather than Measure types because the units follow the control mode; here they are
    // drum rotations.

    public double closedLoopReference = 0.0;
    public double closedLoopError = 0.0;

    // ---- Sticky faults ----
    //
    // These latch until cleared, so they record what happened BETWEEN polls. bootDuringEnable is
    // especially load-bearing here: a motor that reboots mid-match loses its zero, and from that
    // point every height it reports is wrong by however far the carriage had travelled. Nothing
    // else in the log would tell you that.

    public boolean stickyBootDuringEnable = false;
    public boolean stickyUndervoltage = false;
    public boolean stickyOverTemp = false;
    public boolean stickyHardwareFault = false;

    // ---- Reverse soft limit ----

    /**
     * Whether the REVERSE soft limit is on, as last confirmed by the device.
     *
     * <p>An input, not a subsystem field, because it is the outcome of a CAN write the code cannot
     * assume landed. Logging it is what lets replay take the same branch the robot did, and it is
     * the channel to check if a zeroed lift ever drives below its bottom bound.
     */
    public boolean reverseSoftLimitEnabled = false;
  }

  /** Called every loop. Implementations must update every field in {@code inputs}. */
  default void updateInputs(ExampleLiftIOInputs inputs) {}

  /**
   * Open-loop control.
   *
   * <p>Used by the zeroing routine to creep downward, and by bring-up. It bypasses gravity
   * compensation, so zero volts lets the carriage fall — {@link #stop()} is not a safe resting
   * state on this scaffold.
   */
  default void setVoltage(Voltage volts) {}

  /** Closed-loop, profiled position control, in drum rotations. */
  default void setPosition(Angle position) {}

  /** Stops commanding output. See the warning on {@link #setVoltage}. */
  default void stop() {}

  /**
   * Sends "the CURRENT position is zero" to the device, <b>without waiting</b> for it to answer.
   *
   * <p>Only call this once the carriage is genuinely at the hard stop. A zero taken at an unknown
   * position is worse than no zero: the mechanism believes it for the rest of the match, soft
   * limits included, and every height afterward is wrong by a silent fixed offset.
   *
   * <p>Non-blocking on purpose: waiting for the device holds the whole robot loop, and with a slow
   * or missing Talon that is over a second per attempt. Whether the zero landed is decided by the
   * subsystem from the logged position over the next few loops, which also makes the outcome replay
   * exactly. See {@code ExampleLift.requestZero()}.
   */
  default void zeroPosition() {}

  /**
   * Switches the REVERSE soft limit on or off, and reports the result in {@link
   * ExampleLiftIOInputs#reverseSoftLimitEnabled}. Leader only.
   *
   * <p>The lift <b>boots with it off</b>: before zeroing, the bound is measured from wherever the
   * carriage happened to be at power-on, so it means nothing — and with it on, the Talon would
   * refuse to creep down toward the stop at all. {@code ExampleLiftCommands.zero()} switches it on
   * once a zero is confirmed, and off again only when re-zeroing a lift that already had one.
   *
   * <p>This is the one write the routine still <b>waits</b> for, with a short bounded timeout. A
   * lost "enable" would leave the lift with no lower bound and nothing saying so, and the device
   * has no cheap status signal that reports the bound, so it is confirmed rather than assumed.
   */
  default void setReverseSoftLimitEnabled(boolean enabled) {}
}
