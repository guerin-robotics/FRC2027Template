package frc.robot.subsystems.exampleArm.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.LoggedTunableNumber;
import frc.lib.PhoenixUtil;
import frc.robot.Constants;
import frc.robot.subsystems.exampleArm.ExampleArmConstants;
import java.util.Arrays;
import java.util.stream.Stream;

/**
 * Real hardware implementation for a rotary position-controlled mechanism.
 *
 * <p>This file holds <b>no numbers</b>. Every value comes from {@link ExampleArmConstants}.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename throughout, add both CAN IDs to {@code Constants.CanIds}, and
 * pick the bus deliberately.
 */
public class ExampleArmIOReal implements ExampleArmIO {

  /**
   * Every dashboard value pushed to the Talon: the Slot0 gains plus the Motion Magic profile.
   *
   * <p>Built once from the constants file's own arrays so a value cannot be tunable but unwatched.
   */
  private static final LoggedTunableNumber[] LIVE_TUNABLES =
      Stream.concat(
              Arrays.stream(ExampleArmConstants.TUNABLE_GAINS),
              Arrays.stream(ExampleArmConstants.TUNABLE_PROFILE))
          .toArray(LoggedTunableNumber[]::new);

  private final TalonFX motor;
  private final CANcoder encoder;

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

  /**
   * Debounces the motor connection check.
   *
   * <p>{@code kFalling} starts optimistic and only reports a disconnect once the condition has held
   * for half a second, so a single dropped frame does not raise an alert mid-match.
   */
  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Separate debouncer for the encoder — it is a separate device that fails independently. */
  private final Debouncer encoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // ---- Control requests, built ONCE ----

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  public ExampleArmIOReal() {
    // TODO: add the real IDs to Constants.CanIds, and pick the bus deliberately — RIO_BUS here, or
    // TunerConstants.kCANBus for the CANivore.
    motor = new TalonFX(Constants.CanIds.EXAMPLE_ARM_MOTOR, Constants.CanIds.RIO_BUS);
    encoder = new CANcoder(Constants.CanIds.EXAMPLE_ARM_ENCODER, Constants.CanIds.RIO_BUS);

    // ORDER MATTERS: the encoder is configured FIRST, because the motor reads it while applying
    // its own config to set up rotor/CANcoder fusion. Configure the motor first and the fusion is
    // established against an unconfigured encoder — right magnet, wrong offset.
    configureEncoder();

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
    //
    // updateTunedConfig() below does apply sub-configs, and is safe for one reason: the objects it
    // applies come from getFXConfig(), fully populated. Never hand-build one.
    PhoenixUtil.tryUntilOk(
        5, () -> motor.getConfigurator().apply(ExampleArmConstants.getFXConfig()));

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
  private void configureEncoder() {
    CANcoderConfiguration cfg = new CANcoderConfiguration();
    cfg.MagnetSensor.MagnetOffset = ExampleArmConstants.MAGNET_OFFSET_ROTATIONS;
    cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        ExampleArmConstants.SENSOR_DISCONTINUITY_POINT;
    cfg.MagnetSensor.SensorDirection = ExampleArmConstants.ENCODER_DIRECTION;
    PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(cfg));

    // THE TRAP. Register the encoder's Position and Velocity BEFORE optimizing, or optimization
    // drops them to the 4 Hz floor. The MOTOR reads those signals off the bus to do the
    // rotor/CANcoder fusion, so the fused position then updates twelve times slower than the loop
    // that depends on it.
    //
    // It presents as a position loop that lags and hunts — nothing that looks like a configuration
    // problem, and nothing that reproduces in sim.
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, encoder.getPosition(), encoder.getVelocity());
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ExampleArmIOInputs inputs) {
    // Refresh the slow groups. Cheap — a signal that has not updated repeats its last value.
    BaseStatusSignal.refreshAll(
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

    inputs.motorVoltage = motorVoltage.getValue();
    inputs.motorStatorAmps = motorStatorAmps.getValue();
    inputs.motorSupplyAmps = motorSupplyAmps.getValue();
    inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValue();
    inputs.motorPosition = motorPosition.getValue();
    inputs.motorVelocity = motorVelocity.getValue();
    inputs.encoderAbsolutePosition = encoderAbsolutePosition.getValue();
    inputs.motorTemperature = motorTemperature.getValue();
    inputs.closedLoopReference = closedLoopReference.getValueAsDouble();
    inputs.closedLoopError = closedLoopError.getValueAsDouble();
    inputs.stickyBootDuringEnable = stickyBootDuringEnable.getValue();
    inputs.stickyUndervoltage = stickyUndervoltage.getValue();
    inputs.stickyOverTemp = stickyOverTemp.getValue();
    inputs.stickyHardwareFault = stickyHardware.getValue();

    updateTunedConfig();
  }

  /**
   * Pushes dashboard-edited gains and profile values to the Talon, but only when one moved.
   *
   * <p>Both live in flash on the motor controller, not in RAM on the roboRIO — that is why the
   * Talon can close its loop at 1 kHz instead of at our 50 Hz. The cost is that changing one is a
   * blocking CAN transaction rather than a field write, so this cannot be done unconditionally
   * every loop the way a WPILib {@code PIDController} gain can.
   *
   * <p>Five things here are deliberate:
   *
   * <ul>
   *   <li><b>Gated on {@code tuningMode}.</b> In competition this returns immediately and does zero
   *       CAN work.
   *   <li><b>{@code apply(config.Slot0)}, not {@code apply(config)}.</b> A full config apply would
   *       rewrite current limits, soft limits, inversion and feedback ratios every time you nudged
   *       a gain.
   *   <li><b>The Slot0 block comes from {@code getFXConfig()}, not hand-built here.</b> This one is
   *       load-bearing on an arm: a {@code new Slot0Configs()} carrying only kS..kD leaves {@code
   *       GravityType} and {@code StaticFeedforwardSign} at their class defaults, so tuning kP
   *       would silently switch gravity compensation from {@code Arm_Cosine} to {@code
   *       Elevator_Static}. Taking the whole block from the canonical config means gains and their
   *       modifiers can never disagree.
   *   <li><b>Slot0 <i>and</i> MotionMagic.</b> The gains are in Slot0, but cruise velocity and
   *       acceleration are in MotionMagic. Applying only Slot0 leaves the profile knobs moving on
   *       the dashboard while the mechanism ignores them.
   *   <li><b>{@code ifChanged}, not every loop.</b> Expect the one loop it fires on to overrun the
   *       20 ms budget. Acceptable in a tuning session, which is the only time it happens.
   * </ul>
   *
   * <p>Note that {@code hasChanged} reports true on its first call, so this fires once on the first
   * loop after enabling and re-applies values the constructor already wrote.
   *
   * <p>See docs/tunables.md.
   */
  private void updateTunedConfig() {
    if (!Constants.tuningMode) {
      return;
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          TalonFXConfiguration config = ExampleArmConstants.getFXConfig();

          PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config.Slot0));
          PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config.MotionMagic));
        },
        LIVE_TUNABLES);
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
  //   follower = new TalonFX(Constants.CanIds.EXAMPLE_ARM_FOLLOWER, Constants.CanIds.RIO_BUS);
  //   PhoenixUtil.tryUntilOk(
  //       5, () -> follower.getConfigurator().apply(ExampleArmConstants.getFXConfig()));
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
