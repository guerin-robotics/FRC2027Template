package frc.robot.subsystems.exampleRoller.io;

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
 * IO interface for a velocity-controlled mechanism.
 *
 * <p><b>RULE:</b> hardware calls (TalonFX, CANcoder) belong in the IO implementation, never in the
 * subsystem class. That is what makes AdvantageKit log replay work — see {@code
 * .claude/rules/01-architecture.md}.
 *
 * <p>The inputs class is the log schema. Real and sim share it through the generated {@code
 * ExampleRollerIOInputsAutoLogged}, so a field added here appears in both and a replay of an old
 * log against new code still lines up.
 *
 * <p>TEMPLATE INSTRUCTIONS:
 *
 * <ol>
 *   <li>Rename {@code ExampleRoller} to your mechanism name throughout this file.
 *   <li>Add a field for every signal worth seeing in a match log. Adding one is cheap; wishing you
 *       had it while looking at a log from a match you already lost is not.
 *   <li>Add a default no-op method for every control output. Defaults mean a sim implementation can
 *       ignore what it does not model without failing to compile.
 * </ol>
 */
public interface ExampleRollerIO {

  @AutoLog
  class ExampleRollerIOInputs {

    /**
     * Is the device answering?
     *
     * <p>Derived from the 50 Hz signal group alone and debounced — see {@code
     * ExampleRollerIOReal.updateInputs}. Register this with {@code FaultMonitor} so a motor that
     * drops off the bus mid-match announces itself in the pit instead of just going quiet.
     */
    public boolean connected = false;

    // ---- Power and motion ----

    public Voltage motorVoltage = Volts.of(0);

    /** Current through the windings. Governs heating; this is the stall indicator. */
    public Current motorStatorAmps = Amps.of(0);

    /** Current drawn from the battery. This is the one that belongs in brownout math. */
    public Current motorSupplyAmps = Amps.of(0);

    /**
     * Torque-producing current.
     *
     * <p>Under {@code MotionMagicVelocityTorqueCurrentFOC} this <b>is</b> the control signal, and
     * stator current is not the same thing. Without this channel a stalled mechanism and a healthy
     * one drawing similar stator current are indistinguishable in replay.
     *
     * <p>Costs no extra CAN bandwidth: Phoenix 6 packs {@code TorqueCurrent} into the same status
     * frame as {@code StatorCurrent}, which is already registered at 50 Hz.
     */
    public Current motorTorqueCurrentAmps = Amps.of(0);

    /** Mechanism velocity. The whole point of this scaffold. */
    public AngularVelocity motorVelocity = RotationsPerSecond.of(0);

    /**
     * Accumulated mechanism position.
     *
     * <p>Kept even though nothing controls on it: Phoenix packs position and velocity into the same
     * status frame, so it is genuinely free, and counting revolutions through a feeder is a real
     * use. It is <b>not</b> absolute — it reads zero at boot wherever the mechanism happens to be,
     * which is fine here and is exactly why a position mechanism needs a different scaffold.
     */
    public Angle motorPosition = Rotations.of(0);

    /**
     * Device temperature.
     *
     * <p>GOTCHA: AdvantageKit logs {@code Measure}-typed fields in SI base units, so this logs in
     * <b>Kelvin</b> regardless of the unit it was constructed with. To read Celsius in
     * AdvantageScope, log a primitive with the unit in the name instead: {@code public double
     * motorTempCelsius = 0.0;}
     */
    public Temperature motorTemperature = Celsius.of(0);

    // ---- Closed-loop diagnostics ----
    //
    // Log these on every closed-loop mechanism. The reference is what the profile was ASKING for
    // this instant; the error is how far off it was. Together with the measured value they turn
    // "it didn't get up to speed" into an answer:
    //
    //   reference tracking the goal, measured lagging it  -> gains too weak, or saturated
    //   reference NOT reaching the goal                   -> profile too slow (acceleration)
    //   reference flat at the wrong number                -> the setpoint was wrong
    //
    // Primitives, not Measure types, on purpose: the units depend on the control mode, so there
    // is no single Measure type that is honest here. For this scaffold they are rotations/sec.

    public double closedLoopReference = 0.0;
    public double closedLoopError = 0.0;

    // ---- Sticky faults ----
    //
    // These latch until cleared, so they record what happened BETWEEN polls. bootDuringEnable is
    // the one that turns "the motor died for three seconds" into "the motor rebooted at 47.2 s,
    // go check its power connector". Nothing in 2026 read any of these.
    //
    // Registered at ~4 Hz — they latch, so a faster rate buys nothing.

    public boolean stickyBootDuringEnable = false;
    public boolean stickyUndervoltage = false;
    public boolean stickyOverTemp = false;
    public boolean stickyHardwareFault = false;
  }

  /** Called every loop. Implementations must update every field in {@code inputs}. */
  default void updateInputs(ExampleRollerIOInputs inputs) {}

  /** Open-loop control. Useful during bring-up and for characterization; not for match logic. */
  default void setVoltage(Voltage volts) {}

  /** Closed-loop velocity control. The primary control path for this scaffold. */
  default void setVelocity(AngularVelocity velocity) {}

  /** Always provide a stop. A command that ends without stopping leaves the mechanism running. */
  default void stop() {}
}
