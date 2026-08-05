package frc.robot.subsystems.example.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PhoenixUtil;
import frc.robot.HardwareConstants; // TODO: create — see .claude/rules/02-hardware.md

/**
 * Real hardware implementation of ExampleSubsystemIO.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Replace "TODO_MOTOR_CAN_ID" with your CAN ID constant from
 * HardwareConstants 2. Replace "TODO_CAN_BUS" with "rio" or "Canivore" 3. Configure
 * TalonFXConfiguration for your motor (current limits, neutral mode, etc.) 4. Add StatusSignal
 * fields for every signal in ExampleSubsystemIOInputs 5. Add control request objects (VoltageOut,
 * VelocityTorqueCurrentFOC, etc.) 6. Call BaseStatusSignal.refreshAll() at the top of
 * updateInputs()
 */
public class ExampleSubsystemIOReal implements ExampleSubsystemIO {

  private final TalonFX motor;

  // Status signals — one per logged field
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorStatorAmps;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorSupplyAmps;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorTorqueCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> motorTemperature;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public ExampleSubsystemIOReal() {
    motor = new TalonFX(HardwareConstants.CanIds.EXAMPLE_MOTOR, "rio"); // TODO: update CAN bus

    TalonFXConfiguration config = new TalonFXConfiguration();
    // TODO: configure current limits, neutral mode, PID gains
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    // Acquire signal handles
    motorVoltage = motor.getMotorVoltage();
    motorStatorAmps = motor.getStatorCurrent();
    motorSupplyAmps = motor.getSupplyCurrent();
    motorTorqueCurrent = motor.getTorqueCurrent();
    motorVelocity = motor.getVelocity();
    motorTemperature = motor.getDeviceTemp();

    // Set update frequency (50 Hz is standard; use 250 Hz for odometry-critical signals)
    // Torque current goes in the 50 Hz group next to stator/supply current, NOT in a
    // slower diagnostic group — it is a control signal, not a diagnostic.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorVoltage,
        motorStatorAmps,
        motorSupplyAmps,
        motorTorqueCurrent,
        motorVelocity,
        motorTemperature);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ExampleSubsystemIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorVoltage,
        motorStatorAmps,
        motorSupplyAmps,
        motorTorqueCurrent,
        motorVelocity,
        motorTemperature);

    inputs.motorVoltage = motorVoltage.getValue();
    inputs.motorStatorAmps = motorStatorAmps.getValue();
    inputs.motorSupplyAmps = motorSupplyAmps.getValue();
    inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValue();
    inputs.motorVelocity = motorVelocity.getValue();
    inputs.motorTemperature = motorTemperature.getValue();
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    // TODO: replace VoltageOut with VelocityTorqueCurrentFOC if using closed-loop velocity
    // motor.setControl(velocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
  }

  // ==========================================================================================
  // FOLLOWER MOTORS — delete this block if this mechanism has one motor
  // ==========================================================================================
  //
  // A follower mirrors its leader in hardware. Never command a follower directly while the
  // leader is running; the two requests fight and the mechanism stalls or oscillates.
  //
  //   private final TalonFX follower;
  //
  //   // ...in the constructor, AFTER configuring the leader:
  //   follower = new TalonFX(HardwareConstants.CanIds.EXAMPLE_FOLLOWER, "rio");
  //   PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));
  //
  //   // opposeLeaderDirection is TRUE when the follower is physically mounted facing the
  //   // opposite way from the leader. Getting this wrong makes the two motors fight each
  //   // other — high current, no motion, and it will cook a gearbox. Verify at low output
  //   // before running closed-loop.
  //   follower.setControl(new Follower(motor.getDeviceID(), /* opposeLeaderDirection= */ false));
  //
  // LOG THE FOLLOWER TOO. It is a real motor drawing real current and generating real heat,
  // and it can fail independently of the leader — a follower that has quietly stopped looks
  // exactly like a leader that is underpowered.
  //
  //   private final StatusSignal<Current> followerStatorAmps;
  //   ...
  //   followerStatorAmps = follower.getStatorCurrent();
  //
  // AND REGISTER ITS SIGNALS. This is the trap: optimizeBusUtilization() disables every
  // signal that was not explicitly registered. In 2026 the intake roller's follower signals
  // were never added to setUpdateFrequencyForAll(), so after optimization they published at
  // the 4 Hz default. The data looked present and was simply stale, which is far harder to
  // notice than data that is missing.
  //
  //   BaseStatusSignal.setUpdateFrequencyForAll(50, followerStatorAmps, /* ...and the rest */);
  //   follower.optimizeBusUtilization();
  //
  // Import: com.ctre.phoenix6.controls.Follower
}
