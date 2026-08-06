package frc.robot.subsystems.example.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for ExampleSubsystem.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Rename "Example" → your subsystem name throughout this file 2. Add
 * fields to ExampleSubsystemInputs for every signal you want logged (voltage, velocity, position,
 * current, temperature, etc.) 3. Add a default no-op method for every control output 4. Real and
 * sim implementations override these methods
 *
 * <p>RULE: Hardware calls (TalonFX, CANcoder, etc.) belong in the IO implementation, never in the
 * subsystem class itself.
 */
public interface ExampleSubsystemIO {

  @AutoLog
  class ExampleSubsystemIOInputs {
    // Is the device answering? Derived from the 50 Hz signal group only and debounced — see
    // ExampleSubsystemIOReal. Register this with FaultMonitor so a motor that drops off the bus
    // mid-match announces itself instead of just going quiet.
    public boolean connected = false;

    // --- Logged signals (add every sensor/status you want to see in AdvantageKit) ---
    public Voltage motorVoltage = Volts.of(0);
    public Current motorStatorAmps = Amps.of(0);
    public Current motorSupplyAmps = Amps.of(0);

    // Torque-producing current. If this mechanism uses any *TorqueCurrentFOC control
    // request, log this — it IS the control signal, and stator current is not. A stalled
    // mechanism and a healthy one can draw similar stator current while their torque
    // current differs completely, which makes them indistinguishable in replay without
    // this channel. Costs no extra CAN bandwidth: Phoenix 6 packs TorqueCurrent into the
    // same status frame as StatorCurrent, which you are already registering below.
    public Current motorTorqueCurrentAmps = Amps.of(0);

    public AngularVelocity motorVelocity = RotationsPerSecond.of(0);

    /** Mechanism position. Only meaningful for position-controlled mechanisms. */
    public Angle motorPosition = Rotations.of(0);

    // GOTCHA: AdvantageKit logs Measure-typed fields in SI base units, so a
    // Temperature logs in KELVIN regardless of the unit you construct it with.
    // If you want to read Celsius in AdvantageScope, log a primitive instead
    // and put the unit in the field name: public double motorTempCelsius = 0.0;
    public Temperature motorTemperature = Celsius.of(0);

    // ---- Closed-loop diagnostics ----
    //
    // Log these on every closed-loop mechanism. The reference is what the profile was ASKING
    // for this instant; the error is how far off it was. Together with the measured value they
    // turn "it didn't get there" into an answer:
    //
    //   reference tracking the goal, measured lagging it   → gains too weak, or saturated
    //   reference NOT reaching the goal                    → profile too slow, or clamped by
    //                                                        a soft limit
    //   reference flat at the wrong number                 → the setpoint was wrong
    //
    // Primitives, not Measure types, on purpose: the units depend on the control mode
    // (rotations for position, rot/s for velocity), so there is no single Measure type that
    // is honest here. AdvantageScope shows them next to the measured value either way.
    public double closedLoopReference = 0.0;
    public double closedLoopError = 0.0;

    // Sticky faults latch until cleared, so they record what happened BETWEEN polls. Log at
    // least these four — bootDuringEnable is the one that turns "the motor died for three
    // seconds" into "the motor rebooted at 47.2 s, check its power connector".
    // Register at ~4 Hz; they latch, so a fast rate buys nothing.
    public boolean stickyBootDuringEnable = false;
    public boolean stickyUndervoltage = false;
    public boolean stickyOverTemp = false;
    public boolean stickyHardwareFault = false;

    // Add closed-loop reference/error if using PID:
    // public AngularVelocity closedLoopReference = RotationsPerSecond.of(0);
    // public AngularVelocity closedLoopError = RotationsPerSecond.of(0);
  }

  /** Called every loop. Implementations must update all fields in `inputs`. */
  default void updateInputs(ExampleSubsystemIOInputs inputs) {}

  // --- Control outputs (add one per actuator command) ---

  default void setVoltage(Voltage volts) {}

  /** Velocity control — rollers, flywheels, feeders. Delete if this mechanism is positional. */
  default void setVelocity(AngularVelocity velocity) {}

  /** Position control — hoods, pivots, arms. Delete if this mechanism is velocity-controlled. */
  default void setPosition(Angle position) {}

  /** Always provide a stop. A command that ends without stopping leaves the mechanism running. */
  default void stop() {}

  // Relative encoders only (no CANcoder, or a CANcoder that can't cover full travel in one
  // turn) — see ExampleSubsystemIOReal's commented ZEROING block for the real implementation,
  // and ExampleCommands' commented ZEROING block for the routine that decides when to call it.
  //
  //   /** Declares the CURRENT position as zero. Only call once actually at the hard stop. */
  //   default void zeroPosition() {}
}
