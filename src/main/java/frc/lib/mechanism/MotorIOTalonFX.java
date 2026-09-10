package frc.lib.mechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.PhoenixUtil;

/**
 * The real hardware implementation: one TalonFX leader, any number of followers, and an optional
 * CANcoder.
 *
 * <p>This class holds <b>no numbers</b>. Every value comes from the {@link MotorConfig} handed to
 * it, which is what lets a reviewer compare two mechanisms and see only the differences that
 * matter. What it does hold is the parts that are easy to get subtly wrong and that cost a season
 * to discover: the signal-rate split, the order the CANcoder is configured in, and where {@code
 * connected} comes from.
 *
 * <h2>The signal-rate split</h2>
 *
 * <p>50 Hz for anything the control loop or a match review needs every cycle, 10 Hz for diagnostics
 * that move slowly, 4 Hz for sticky faults, which latch and so gain nothing from a faster rate.
 * Torque current is in the 50 Hz group because under {@code TorqueCurrentFOC} it is the control
 * signal, not a diagnostic; closed-loop reference is there because comparing it against the
 * measurement is how saturation becomes visible, and that comparison is meaningless if the two are
 * sampled at different rates.
 *
 * <p>{@code optimizeBusUtilization()} runs last, on every device. It slows everything unregistered
 * to 4 Hz rather than disabling it, which is friendlier and is also the trap: a forgotten signal is
 * <i>stale</i> rather than <i>missing</i>, so the channel exists, the numbers look plausible, and
 * they are a quarter second old.
 */
public class MotorIOTalonFX implements MotorIO {

  protected final MotorConfig motorConfig;
  protected final TalonFX motor;
  protected final TalonFX[] followers;
  protected final CANcoder encoder;

  /**
   * The live config.
   *
   * <p>Kept so live tuning can apply {@code config.Slot0} and {@code config.MotionMagic} taken from
   * a fully-built config rather than from a hand-built sub-config. Applying a hand-built block
   * writes the whole sub-group, resetting every field it does not mention — which is how the 2026
   * intake pivot silently reset the feedback ratios it had configured two calls earlier.
   */
  private final TalonFXConfiguration config;

  // ---- Leader signals, one per logged field ----
  //
  // Cached once here rather than fetched inside updateInputs(). Calling motor.getStatorCurrent()
  // every loop allocates a signal object every loop, which is garbage the collector has to clear
  // inside the 20 ms budget — and pause behaviour on this robot comes from not allocating rather
  // than from any GC flag.

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorAmps;
  private final StatusSignal<Current> supplyAmps;
  private final StatusSignal<Current> torqueAmps;
  private final StatusSignal<Double> closedLoopReference;

  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Double> closedLoopError;

  private final StatusSignal<Boolean> stickyBootDuringEnable;
  private final StatusSignal<Boolean> stickyUndervoltage;
  private final StatusSignal<Boolean> stickyOverTemp;
  private final StatusSignal<Boolean> stickyHardware;

  // ---- Follower signals, indexed by follower ----

  private final StatusSignal<Voltage>[] followerVolts;
  private final StatusSignal<Current>[] followerStator;
  private final StatusSignal<Current>[] followerSupply;
  private final StatusSignal<Current>[] followerTorque;
  private final StatusSignal<AngularVelocity>[] followerVelocity;
  private final StatusSignal<Temperature>[] followerTemp;
  private final StatusSignal<Boolean>[] followerStickyBoot;
  private final StatusSignal<Boolean>[] followerStickyUndervoltage;
  private final StatusSignal<Boolean>[] followerStickyOverTemp;
  private final StatusSignal<Boolean>[] followerStickyHardware;

  // ---- Encoder signals ----

  private final StatusSignal<Angle> encoderAbsolutePosition;
  private final StatusSignal<Angle> encoderPosition;
  private final StatusSignal<AngularVelocity> encoderVelocity;
  private final StatusSignal<com.ctre.phoenix6.signals.MagnetHealthValue> encoderMagnetHealth;

  /**
   * Debounces the connection check.
   *
   * <p>{@code kFalling} starts optimistic and only reports a disconnect once the condition has held
   * for half a second, so a single dropped frame does not raise an alert mid-match. Same shape
   * {@code ModuleIOTalonFX} uses for the swerve modules.
   */
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Debouncer encoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // ---- Control requests, built once ----

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC unprofiledPositionRequest =
      new PositionTorqueCurrentFOC(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  @SuppressWarnings("unchecked")
  public MotorIOTalonFX(MotorConfig motorConfig) {
    this.motorConfig = motorConfig;
    this.config = motorConfig.toTalonFXConfiguration();

    // ---- Encoder first, if there is one ----
    //
    // Order matters. Under FusedCANcoder the motor reads the encoder's position off the bus to do
    // the fusion, so the encoder has to be configured and publishing before the motor is told to
    // depend on it.
    if (motorConfig.hasEncoder()) {
      encoder = new CANcoder(motorConfig.encoderCanId(), motorConfig.bus());
      PhoenixUtil.tryUntilOk(
          5, () -> encoder.getConfigurator().apply(motorConfig.toCANcoderConfiguration()));

      encoderAbsolutePosition = encoder.getAbsolutePosition();
      encoderPosition = encoder.getPosition();
      encoderVelocity = encoder.getVelocity();
      encoderMagnetHealth = encoder.getMagnetHealth();

      // THE TRAP. Position and Velocity must be registered BEFORE optimizing, or optimization
      // drops them to the 4 Hz floor. The MOTOR reads those signals off the bus to do the fusion,
      // so the fused position would then update twelve times slower than the loop depending on it.
      //
      // It presents as a position loop that lags and hunts — nothing that looks like a
      // configuration problem, and nothing that reproduces in simulation.
      //
      // AbsolutePosition is in this group too, per the team standard in
      // .claude/rules/02-hardware.md, and ModuleIOTalonFX puts the swerve CANcoders' at 50 Hz for
      // the same reason. It is the channel you scrub to answer whether the magnet offset is right
      // and whether the fusion was seeded correctly, and that comparison is against the motor's
      // position — which is at 50 Hz. Two channels sampled at different rates disagree by however
      // far the mechanism moved between them, which reads as an offset error that is not there.
      BaseStatusSignal.setUpdateFrequencyForAll(
          50.0, encoderPosition, encoderVelocity, encoderAbsolutePosition);

      // Magnet health is a slowly-changing diagnostic: it reports whether the magnet is in range,
      // which is a mounting property, not something that varies as the mechanism moves.
      BaseStatusSignal.setUpdateFrequencyForAll(10.0, encoderMagnetHealth);
      encoder.optimizeBusUtilization();
    } else {
      encoder = null;
      encoderAbsolutePosition = null;
      encoderPosition = null;
      encoderVelocity = null;
      encoderMagnetHealth = null;
    }

    // ---- Leader ----
    motor = new TalonFX(motorConfig.canId(), motorConfig.bus());

    // tryUntilOk because CTRE devices silently ignore configuration if the CAN bus is busy at
    // startup. Without the retry a motor boots with no current limits and no gains, and the first
    // match move can brown out the robot.
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    statorAmps = motor.getStatorCurrent();
    supplyAmps = motor.getSupplyCurrent();
    torqueAmps = motor.getTorqueCurrent();
    closedLoopReference = motor.getClosedLoopReference();

    temperature = motor.getDeviceTemp();
    closedLoopError = motor.getClosedLoopError();

    stickyBootDuringEnable = motor.getStickyFault_BootDuringEnable();
    stickyUndervoltage = motor.getStickyFault_Undervoltage();
    stickyOverTemp = motor.getStickyFault_DeviceTemp();
    stickyHardware = motor.getStickyFault_Hardware();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        statorAmps,
        supplyAmps,
        torqueAmps,
        closedLoopReference);
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, temperature, closedLoopError);
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, stickyBootDuringEnable, stickyUndervoltage, stickyOverTemp, stickyHardware);

    // ---- Followers ----
    int count = motorConfig.followers().size();
    followers = new TalonFX[count];
    followerVolts = new StatusSignal[count];
    followerStator = new StatusSignal[count];
    followerSupply = new StatusSignal[count];
    followerTorque = new StatusSignal[count];
    followerVelocity = new StatusSignal[count];
    followerTemp = new StatusSignal[count];
    followerStickyBoot = new StatusSignal[count];
    followerStickyUndervoltage = new StatusSignal[count];
    followerStickyOverTemp = new StatusSignal[count];
    followerStickyHardware = new StatusSignal[count];

    for (int i = 0; i < count; i++) {
      MotorConfig.Follower spec = motorConfig.followers().get(i);
      TalonFX follower = new TalonFX(spec.canId(), motorConfig.bus());
      followers[i] = follower;

      // The follower gets the leader's config: same limits, same neutral mode, same inversion
      // convention. Alignment is expressed through the Follower request below, not by inverting
      // one of the two configs — doing it in both places cancels out and is a miserable bug.
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));

      // Phoenix 6 2026 replaced Follower(int, boolean) with Follower(int, MotorAlignmentValue).
      // Opposed when the follower faces the other way from the leader, Aligned when it faces the
      // same way.
      follower.setControl(
          new Follower(
              motorConfig.canId(),
              spec.opposed() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

      followerVolts[i] = follower.getMotorVoltage();
      followerStator[i] = follower.getStatorCurrent();
      followerSupply[i] = follower.getSupplyCurrent();
      followerTorque[i] = follower.getTorqueCurrent();
      followerVelocity[i] = follower.getVelocity();
      followerTemp[i] = follower.getDeviceTemp();
      followerStickyBoot[i] = follower.getStickyFault_BootDuringEnable();
      followerStickyUndervoltage[i] = follower.getStickyFault_Undervoltage();
      followerStickyOverTemp[i] = follower.getStickyFault_DeviceTemp();
      followerStickyHardware[i] = follower.getStickyFault_Hardware();

      // Registered explicitly, at a rate somebody chose. The default is 4 Hz — the same place
      // optimizeBusUtilization would have left them — but arrived at on purpose.
      BaseStatusSignal.setUpdateFrequencyForAll(
          motorConfig.followerSignalHz(),
          followerVolts[i],
          followerStator[i],
          followerSupply[i],
          followerTorque[i],
          followerVelocity[i],
          followerTemp[i]);
      BaseStatusSignal.setUpdateFrequencyForAll(
          4.0,
          followerStickyBoot[i],
          followerStickyUndervoltage[i],
          followerStickyOverTemp[i],
          followerStickyHardware[i]);
      follower.optimizeBusUtilization();
    }

    // MUST BE LAST on the leader, after every frequency above is set.
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    // Refresh the slow groups first. Cheap — a signal that has not updated repeats its last value,
    // so polling a 10 Hz or 4 Hz signal at 50 Hz costs nothing.
    BaseStatusSignal.refreshAll(
        temperature,
        closedLoopError,
        stickyBootDuringEnable,
        stickyUndervoltage,
        stickyOverTemp,
        stickyHardware);

    // Connection comes from the 50 Hz group ALONE. refreshAll() returns the worst status of
    // everything passed to it, so folding the slow groups into this call would let the slowest one
    // govern: for the first quarter second after boot, before the first sticky-fault frame
    // arrives, the mechanism would read disconnected and every FaultMonitor alert tied to it would
    // fire for no reason.
    var status =
        BaseStatusSignal.refreshAll(
            position,
            velocity,
            appliedVolts,
            statorAmps,
            supplyAmps,
            torqueAmps,
            closedLoopReference);
    inputs.connected = connectedDebounce.calculate(status.isOK());

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.statorAmps = statorAmps.getValue();
    inputs.supplyAmps = supplyAmps.getValue();
    inputs.torqueAmps = torqueAmps.getValue();
    inputs.temperature = temperature.getValue();
    inputs.closedLoopReference = closedLoopReference.getValueAsDouble();
    inputs.closedLoopError = closedLoopError.getValueAsDouble();
    inputs.stickyBootDuringEnable = stickyBootDuringEnable.getValue();
    inputs.stickyUndervoltage = stickyUndervoltage.getValue();
    inputs.stickyOverTemp = stickyOverTemp.getValue();
    inputs.stickyHardwareFault = stickyHardware.getValue();
  }

  @Override
  public void updateFollowerInputs(int index, FollowerInputs inputs) {
    BaseStatusSignal.refreshAll(
        followerVolts[index],
        followerStator[index],
        followerSupply[index],
        followerTorque[index],
        followerVelocity[index],
        followerTemp[index],
        followerStickyBoot[index],
        followerStickyUndervoltage[index],
        followerStickyOverTemp[index],
        followerStickyHardware[index]);

    // isConnected(), not signal status: a follower's signals sit at a deliberately slow rate, and a
    // status built from slow signals reports staleness rather than presence.
    inputs.connected = followers[index].isConnected();

    inputs.appliedVolts = followerVolts[index].getValue();
    inputs.statorAmps = followerStator[index].getValue();
    inputs.supplyAmps = followerSupply[index].getValue();
    inputs.torqueAmps = followerTorque[index].getValue();
    inputs.velocity = followerVelocity[index].getValue();
    inputs.temperature = followerTemp[index].getValue();
    inputs.stickyBootDuringEnable = followerStickyBoot[index].getValue();
    inputs.stickyUndervoltage = followerStickyUndervoltage[index].getValue();
    inputs.stickyOverTemp = followerStickyOverTemp[index].getValue();
    inputs.stickyHardwareFault = followerStickyHardware[index].getValue();
  }

  @Override
  public void updateEncoderInputs(EncoderInputs inputs) {
    // A separate device gets a separate status. A CANcoder can drop off the bus while its motor
    // stays perfectly healthy, and that is exactly the failure worth catching.
    var status =
        BaseStatusSignal.refreshAll(
            encoderPosition, encoderVelocity, encoderAbsolutePosition, encoderMagnetHealth);
    inputs.connected = encoderConnectedDebounce.calculate(status.isOK());

    inputs.absolutePosition = encoderAbsolutePosition.getValue();
    inputs.position = encoderPosition.getValue();
    inputs.velocity = encoderVelocity.getValue();
    inputs.magnetHealth = encoderMagnetHealth.getValue().value;
  }

  @Override
  public int followerCount() {
    return followers.length;
  }

  @Override
  public boolean hasAbsoluteEncoder() {
    return encoder != null;
  }

  // ============================================================================================
  // Control
  // ============================================================================================

  @Override
  public void setVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setTorqueCurrent(Current amps) {
    motor.setControl(torqueRequest.withOutput(amps.in(Amps)));
  }

  @Override
  public void setDutyCycle(double fraction) {
    motor.setControl(dutyCycleRequest.withOutput(fraction));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
  }

  @Override
  public void setPosition(Angle position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void setUnprofiledPosition(Angle position) {
    motor.setControl(unprofiledPositionRequest.withPosition(position));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.setNeutralMode(mode);
    for (TalonFX follower : followers) {
      follower.setNeutralMode(mode);
    }
  }

  @Override
  public void setEncoderPosition(Angle position) {
    PhoenixUtil.tryUntilOk(5, () -> motor.setPosition(position));
  }

  // ============================================================================================
  // Live tuning
  // ============================================================================================

  @Override
  public void setGains(Gains gains) {
    MotorConfig.applyGains(config, gains);
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config.Slot0));
  }

  @Override
  public void setMotionProfile(MotionProfile profile) {
    MotorConfig.applyProfile(config, profile);
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config.MotionMagic));
  }

  @Override
  public void setSupplyCurrentLimit(Current limit) {
    config.CurrentLimits.SupplyCurrentLimit = limit.in(Amps);
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config.CurrentLimits));
  }
}
