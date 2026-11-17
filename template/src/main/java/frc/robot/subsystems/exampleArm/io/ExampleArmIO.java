package frc.robot.subsystems.exampleArm.io;

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
 * IO interface for a rotary position-controlled mechanism.
 *
 * <p><b>RULE:</b> hardware calls (TalonFX, CANcoder) belong in the IO implementation, never in the
 * subsystem class. That is what makes AdvantageKit log replay work — see {@code
 * .claude/rules/01-architecture.md}.
 *
 * <p>The inputs class is the log schema. Real and sim share it through the generated {@code
 * ExampleArmIOInputsAutoLogged}, so a field added here appears in both.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename {@code ExampleArm} throughout, add a field for every signal
 * worth seeing in a match log, and add a default no-op for every control output.
 */
public interface ExampleArmIO {

  @AutoLog
  class ExampleArmIOInputs {

    /**
     * Is the motor answering?
     *
     * <p>Derived from the 50 Hz signal group alone and debounced. Register with {@code
     * FaultMonitor} so a motor that drops off the bus mid-match announces itself in the pit.
     */
    public boolean motorConnected = false;

    /**
     * Is the CANcoder answering?
     *
     * <p><b>A separate device gets a separate status.</b> A CANcoder can drop out while its motor
     * stays perfectly healthy — that is precisely the failure worth catching, and it disappears if
     * the two are merged into one boolean. On a fused-CANcoder mechanism the symptom is a position
     * loop that starts fighting an encoder that is no longer reporting.
     */
    public boolean encoderConnected = false;

    // ---- Power and motion ----

    public Voltage motorVoltage = Volts.of(0);

    /** Current through the windings. Governs heating; this is the stall indicator. */
    public Current motorStatorAmps = Amps.of(0);

    /** Current drawn from the battery. This is the one that belongs in brownout math. */
    public Current motorSupplyAmps = Amps.of(0);

    /**
     * Torque-producing current.
     *
     * <p>Under {@code MotionMagicTorqueCurrentFOC} this <b>is</b> the control signal, and stator
     * current is not the same thing. On an arm it is also the most readable measure of gravity
     * load: a correctly-tuned kG shows as torque current tracking the cosine of the angle.
     *
     * <p>Costs no extra CAN bandwidth — Phoenix 6 packs it into the same status frame as {@code
     * StatorCurrent}.
     */
    public Current motorTorqueCurrentAmps = Amps.of(0);

    /** Fused mechanism position — CANcoder absolute, extended by the rotor. Degrees when logged. */
    public Angle motorPosition = Rotations.of(0);

    public AngularVelocity motorVelocity = RotationsPerSecond.of(0);

    /**
     * The CANcoder's own absolute reading, before fusion.
     *
     * <p>Worth logging separately from {@link #motorPosition}. When the two disagree, the magnet
     * offset or a ratio is wrong, and that is invisible if only the fused value is recorded.
     */
    public Angle encoderAbsolutePosition = Rotations.of(0);

    /**
     * Device temperature.
     *
     * <p>GOTCHA: AdvantageKit logs {@code Measure}-typed fields in SI base units, so this logs in
     * <b>Kelvin</b>. To read Celsius in AdvantageScope, log a primitive with the unit in the name.
     */
    public Temperature motorTemperature = Celsius.of(0);

    // ---- Closed-loop diagnostics ----
    //
    // Log these on every closed-loop mechanism. Without them a log shows the arm in the wrong place
    // and gives you no way to tell which of three different problems you have:
    //
    //   reference tracks the goal, measured lags it  -> gains too weak, or saturated
    //   reference never reaches the goal             -> profile too slow, or clamped by a soft
    // limit
    //   reference flat at the wrong number           -> the setpoint itself was wrong
    //
    // Those need very different fixes, and guessing between them is how a tuning session gets spent
    // on the wrong gain. Primitives rather than Measure types because the units follow the control
    // mode; here they are mechanism rotations.

    public double closedLoopReference = 0.0;
    public double closedLoopError = 0.0;

    // ---- Sticky faults ----
    //
    // These latch until cleared, so they record what happened BETWEEN polls. bootDuringEnable turns
    // "the arm went limp for three seconds" into "that motor rebooted at 47.2 s, check its power
    // connector". Registered at ~4 Hz — they latch, so a faster rate buys nothing.

    public boolean stickyBootDuringEnable = false;
    public boolean stickyUndervoltage = false;
    public boolean stickyOverTemp = false;
    public boolean stickyHardwareFault = false;
  }

  /** Called every loop. Implementations must update every field in {@code inputs}. */
  default void updateInputs(ExampleArmIOInputs inputs) {}

  /**
   * Open-loop control.
   *
   * <p>Bring-up and characterization only. Note that this bypasses gravity compensation entirely,
   * so commanding zero volts on an arm lets it fall — {@link #stop()} is not a safe resting state
   * on this scaffold, which is why the default command holds a position instead.
   */
  default void setVoltage(Voltage volts) {}

  /** Closed-loop, profiled position control. The primary control path for this scaffold. */
  default void setPosition(Angle position) {}

  /** Stops commanding output. See the warning on {@link #setVoltage}. */
  default void stop() {}
}
