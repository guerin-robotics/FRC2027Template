package frc.robot.subsystems.exampleArm.io;

import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import frc.lib.device.CanIdRegistry;
import frc.lib.util.PhoenixUtil;
import frc.robot.Constants;
import frc.robot.subsystems.exampleArm.ExampleArmConstants;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

/**
 * Real hardware implementation for a rotary position-controlled mechanism.
 *
 * <p>This file holds <b>no numbers</b>. Every value comes from {@link ExampleArmConstants}.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename throughout, add both CAN IDs to {@code Constants.CanIds}, and
 * pick the bus deliberately.
 */
public class ExampleArmIOReal implements ExampleArmIO {

  private final TalonFX motor;
  private final CANcoder encoder;
  private final boolean configured;

  // ---- Status signals, cached once ----
  //
  // Calling motor.getStatorCurrent() inside updateInputs() would allocate a new signal object
  // every loop, which is garbage the collector has to clean up inside the 20 ms budget.

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorStatorAmps;
  private final StatusSignal<Current> motorSupplyAmps;
  private final StatusSignal<Current> motorTorqueCurrent;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Temperature> motorTemperature;

  private final StatusSignal<Double> closedLoopReference;
  private final StatusSignal<Double> closedLoopError;

  private final StatusSignal<Boolean> stickyBootDuringEnable;
  private final StatusSignal<Boolean> stickyUndervoltage;
  private final StatusSignal<Boolean> stickyOverTemp;
  private final StatusSignal<Boolean> stickyHardware;

  private final StatusSignal<Angle> encoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> encoderVelocity;
  private final StatusSignal<MagnetHealthValue> encoderMagnetHealth;

  /**
   * Debounces the motor connection check.
   *
   * <p>{@code kFalling} starts optimistic and only reports a disconnect once the condition has held
   * for half a second, so a single dropped frame does not raise an alert mid-match.
   */
  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  /** Separate debouncer for the encoder — it is a separate device that fails independently. */
  private final Debouncer encoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  // ---- Control requests, built ONCE ----

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  public ExampleArmIOReal() {
    // TODO: add the real IDs to Constants.CanIds, and pick the bus deliberately — MECHANISM_BUS_A
    // here, or
    // TunerConstants.kCANBus for the CANivore.
    //
    // Claim every ID before constructing its device. A duplicate then stops boot with both owners
    // named, instead of two devices half-working on one ID.
    CanIdRegistry.claim(
        Constants.CanIds.EXAMPLE_ARM_MOTOR,
        Constants.CanIds.MECHANISM_BUS_A,
        ExampleArmConstants.NAME + " motor");
    CanIdRegistry.claim(
        Constants.CanIds.EXAMPLE_ARM_ENCODER,
        Constants.CanIds.MECHANISM_BUS_A,
        ExampleArmConstants.NAME + " encoder");
    motor = new TalonFX(Constants.CanIds.EXAMPLE_ARM_MOTOR, Constants.CanIds.MECHANISM_BUS_A);
    encoder = new CANcoder(Constants.CanIds.EXAMPLE_ARM_ENCODER, Constants.CanIds.MECHANISM_BUS_A);

    // ORDER MATTERS: the encoder is configured FIRST, because the motor reads it while applying
    // its own config to set up rotor/CANcoder fusion. Configure the motor first and the fusion is
    // established against an unconfigured encoder — right magnet, wrong offset.
    boolean encoderConfigured = configureEncoder();

    // The config is built in the constants file, not here. tryUntilOk because CTRE devices
    // silently ignore configuration if the CAN bus is busy at startup — without the retry, the
    // motor boots with no current limits, no soft limits and no gains.
    //
    // ONE apply, of the WHOLE config. Do not follow it with a second apply() of a freshly-built
    // sub-config — that is a real trap and it shipped on the 2026 intake pivot.
    //
    // apply() on a sub-config writes the ENTIRE sub-group to the device, so fields the caller never
    // touched are written too, at their class defaults. The 2026 code set RotorToSensorRatio inside
    // its TalonFXConfiguration, applied it, and then applied a separate `new FeedbackConfigs()`
    // carrying only the remote sensor ID and source — which reset both ratios to 1.0 and silently
    // undid the line two applies earlier.
    //
    // It survived 2026 only because that mechanism used RemoteCANcoder, where position comes
    // straight off the encoder and RotorToSensorRatio is not in the position path. Under
    // FusedCANcoder it would not: the ratio is how the rotor extends the absolute reading, so
    // losing it makes the fusion wrong by the whole reduction while every apply reports success.
    boolean motorConfigured =
        PhoenixUtil.tryUntilOk(
            5, () -> motor.getConfigurator().apply(ExampleArmConstants.getFXConfig()));
    configured = encoderConfigured && motorConfigured;

    // ---- Acquire signal handles ----
    motorVoltage = motor.getMotorVoltage();
    motorStatorAmps = motor.getStatorCurrent();
    motorSupplyAmps = motor.getSupplyCurrent();
    motorTorqueCurrent = motor.getTorqueCurrent();
    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorTemperature = motor.getDeviceTemp();
    closedLoopReference = motor.getClosedLoopReference();
    closedLoopError = motor.getClosedLoopError();
    stickyBootDuringEnable = motor.getStickyFault_BootDuringEnable();
    stickyUndervoltage = motor.getStickyFault_Undervoltage();
    stickyOverTemp = motor.getStickyFault_DeviceTemp();
    stickyHardware = motor.getStickyFault_Hardware();
    encoderAbsolutePosition = encoder.getAbsolutePosition();
    encoderVelocity = encoder.getVelocity();
    encoderMagnetHealth = encoder.getMagnetHealth();

    // ---- Signal rates. This split is the team standard; use it on every mechanism. ----
    //
    // 50 Hz — everything the control loop or a match review needs every cycle. Torque current
    // belongs here, not in the slow group: under TorqueCurrentFOC it IS the control signal.
    // Closed-loop REFERENCE belongs here because comparing it against measured position is how
    // profile saturation becomes visible, and that comparison is meaningless if the two are
    // sampled at different rates.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorPosition,
        motorVelocity,
        motorStatorAmps,
        motorSupplyAmps,
        motorTorqueCurrent,
        motorVoltage,
        closedLoopReference);

    // 10 Hz — diagnostics that change slowly. Temperature moves over minutes. Closed-loop ERROR is
    // reference minus measured, both already at 50 Hz; the channel is a convenience for scrubbing
    // a log, not an input to anything.
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, motorTemperature, closedLoopError);

    // 4 Hz — sticky faults, which latch until cleared.
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, stickyBootDuringEnable, stickyUndervoltage, stickyOverTemp, stickyHardware);

    // LAST, on every device. See configureEncoder() for the encoder's own call.
    motor.optimizeBusUtilization();
  }

  /**
   * Configures the CANcoder. Called before the motor config, from the constructor.
   *
   * <p>Contains the one trap that is specific to a fused-CANcoder mechanism — see the comment on
   * {@code optimizeBusUtilization} below.
   */
  private boolean configureEncoder() {
    CANcoderConfiguration cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.MagnetOffset = ExampleArmConstants.MAGNET_OFFSET_ROTATIONS;
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        ExampleArmConstants.SENSOR_DISCONTINUITY_POINT;
    cfg.MagnetSensor.SensorDirection = ExampleArmConstants.ENCODER_DIRECTION;
    boolean confirmed = PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(cfg));

    // THE TRAP. Register the encoder's Position and Velocity BEFORE optimizing, or optimization
    // drops them to the 4 Hz floor. The MOTOR reads those signals off the bus to do the
    // rotor/CANcoder fusion, so the fused position then updates twelve times slower than the loop
    // that depends on it.
    //
    // It presents as a position loop that lags and hunts — nothing that looks like a configuration
    // problem, and nothing that reproduces in sim.
    //
    // ABSOLUTE position at 50 Hz too. It is the channel you scrub against the motor's position to
    // check the magnet offset and the fusion seed; sampled slower, the two disagree by however far
    // the arm moved in between, which reads as an offset error that is not there. It also feeds
    // encoderConnected, which a 4 Hz signal would report as stale rather than present.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, encoder.getPosition(), encoder.getVelocity(), encoder.getAbsolutePosition());

    // Magnet health at 10 Hz: it is a mounting property, not something that changes as the arm
    // moves. Registered here, before optimizing, or it falls to the 4 Hz floor by omission.
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, encoder.getMagnetHealth());
    encoder.optimizeBusUtilization();
    return confirmed;
  }

  @Override
  public void updateInputs(ExampleArmIOInputs inputs) {
    // Refresh the slow groups. Cheap — a signal that has not updated repeats its last value.
    // Magnet health is refreshed here, NOT in the encoder's connected group below: it is a 10 Hz
    // signal, and folding it into that status would let the slower frame govern connected.
    BaseStatusSignal.refreshAll(
        encoderMagnetHealth,
        motorTemperature,
        closedLoopError,
        stickyBootDuringEnable,
        stickyUndervoltage,
        stickyOverTemp,
        stickyHardware);

    // Motor connection comes from its 50 Hz group ALONE.
    //
    // refreshAll() returns the WORST status of everything passed to it, so folding the 10 Hz and
    // 4 Hz signals into this call would let the slowest one govern: for the first quarter second
    // after boot, before the first sticky-fault frame arrives, the mechanism reads disconnected and
    // every FaultMonitor alert tied to it fires for no reason.
    var motorStatus =
        BaseStatusSignal.refreshAll(
            motorPosition,
            motorVelocity,
            motorStatorAmps,
            motorSupplyAmps,
            motorTorqueCurrent,
            motorVoltage,
            closedLoopReference);
    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());

    // The encoder gets its OWN status from its OWN signals. Merging it into the motor's boolean
    // would hide the exact failure this separation exists to catch.
    var encoderStatus = BaseStatusSignal.refreshAll(encoderAbsolutePosition, encoderVelocity);
    inputs.encoderConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.configured = configured;

    inputs.motorVoltage = motorVoltage.getValue();
    inputs.motorStatorAmps = motorStatorAmps.getValue();
    inputs.motorSupplyAmps = motorSupplyAmps.getValue();
    inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValue();
    inputs.motorPosition = motorPosition.getValue();
    inputs.motorVelocity = motorVelocity.getValue();
    inputs.encoderAbsolutePosition = encoderAbsolutePosition.getValue();
    inputs.encoderMagnetHealth = encoderMagnetHealth.getValue().value;
    inputs.motorTemperature = motorTemperature.getValue();
    inputs.closedLoopReference = closedLoopReference.getValueAsDouble();
    inputs.closedLoopError = closedLoopError.getValueAsDouble();
    inputs.stickyBootDuringEnable = stickyBootDuringEnable.getValue();
    inputs.stickyUndervoltage = stickyUndervoltage.getValue();
    inputs.stickyOverTemp = stickyOverTemp.getValue();
    inputs.stickyHardwareFault = stickyHardware.getValue();
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Angle position) {
    motor.setControl(positionRequest.withPosition(position.in(Rotations)));
  }

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }

  // ==========================================================================================
  // FOLLOWER MOTORS — delete this block if this mechanism has one motor
  // ==========================================================================================
  //
  // A follower mirrors its leader in hardware. Never command a follower directly while the leader
  // is running; the two requests fight, and the mechanism stalls or oscillates while drawing heavy
  // current. On a geared arm that is enough to strip a gearbox.
  //
  //   private final TalonFX follower;
  //
  //   // ...in the constructor, AFTER configuring the leader:
  //   CanIdRegistry.claim(
  //       Constants.CanIds.EXAMPLE_ARM_FOLLOWER,
  //       Constants.CanIds.MECHANISM_BUS_A,
  //       ExampleArmConstants.NAME + " follower");
  //   follower = new TalonFX(Constants.CanIds.EXAMPLE_ARM_FOLLOWER,
  // Constants.CanIds.MECHANISM_BUS_A);
  //
  //   // The leader's whole config, built once and applied once — MINUS THE TRAVEL BOUNDS. A
  //   // follower runs a Follower request, not a position loop, so its soft limits gate output
  //   // against a position nothing reads, and on an Opposed follower in a frame that runs the
  //   // other way. The arm drops to one motor's torque partway through legal travel while both
  //   // devices report obeying their limits. The leader owns travel.
  //   TalonFXConfiguration followerConfig = ExampleArmConstants.getFXConfig();
  //   followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
  //   followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  //   // Include the follower apply result in the AND that determines configured.
  //   PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(followerConfig));
  //
  //   // MotorAlignmentValue.Opposed when the follower is physically mounted facing the opposite
  //   // way from the leader, Aligned when it faces the same way. Verify at low output before
  //   // running closed-loop.
  //   //
  //   // NOTE: Phoenix 6 2026 replaced Follower(int, boolean) with
  //   // Follower(int, MotorAlignmentValue). A boolean here will not compile.
  //   follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
  //
  // LOG THE FOLLOWER, and CHECK IT WITH follower.isConnected() rather than signal status — its
  // signals sit at the 4 Hz optimize floor by design, so a status built from them reports staleness
  // rather than presence.
  //
  // DECIDE ITS SIGNAL RATE deliberately. In 2026 the intake roller's follower signals were acquired
  // but never registered, so optimizeBusUtilization() left them at 4 Hz. The data looked present
  // and was simply stale — far harder to notice than data that is missing. 4 Hz is a fine rate for
  // a follower; the point is to arrive there on purpose.
  //
  // Imports: com.ctre.phoenix6.controls.Follower, com.ctre.phoenix6.signals.MotorAlignmentValue
}
