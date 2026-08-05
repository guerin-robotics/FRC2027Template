package frc.robot.subsystems.example.io;

import static edu.wpi.first.units.Units.*;

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

    // GOTCHA: AdvantageKit logs Measure-typed fields in SI base units, so a
    // Temperature logs in KELVIN regardless of the unit you construct it with.
    // If you want to read Celsius in AdvantageScope, log a primitive instead
    // and put the unit in the field name: public double motorTempCelsius = 0.0;
    public Temperature motorTemperature = Celsius.of(0);

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

  default void setVelocity(AngularVelocity velocity) {}

  // Add more as needed:
  // default void setPosition(Angle angle) {}
  // default void stop() {}
}
