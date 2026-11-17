package frc.robot.subsystems.exampleLift.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.device.CanIdRegistry;
import frc.lib.util.PhoenixUtil;
import frc.robot.Constants;
import frc.robot.subsystems.exampleLift.ExampleLiftConstants;

/**
 * Real hardware implementation for a linear position-controlled mechanism.
 *
 * <p>This file holds <b>no numbers</b>. Every value comes from {@link ExampleLiftConstants}.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename throughout, add the CAN ID to {@code Constants.CanIds}, and pick
 * the bus deliberately.
 */
public class ExampleLiftIOReal implements ExampleLiftIO {

  private final TalonFX motor;

  // ---- Status signals, cached once ----
  //
  // Calling motor.getStatorCurrent() inside updateInputs() would allocate a new signal object every
  // loop, which is garbage the collector has to clean up inside the 20 ms budget.

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

  /**
   * Debounces the connection check.
   *
   * <p>{@code kFalling} starts optimistic and only reports a disconnect once the condition has held
   * for half a second, so a single dropped frame does not raise an alert mid-match.
   */
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // ---- Control requests, built ONCE ----

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  public ExampleLiftIOReal() {
    // TODO: add the real ID to Constants.CanIds, and pick the bus deliberately — RIO_BUS here, or
    // TunerConstants.kCANBus for the CANivore.
    // Claim the ID before constructing the device. A duplicate then stops boot with both owners
    // named, instead of two devices half-working on one ID.
    CanIdRegistry.claim(
        Constants.CanIds.EXAMPLE_LIFT_MOTOR,
        Constants.CanIds.RIO_BUS,
        ExampleLiftConstants.NAME + " motor");
    motor = new TalonFX(Constants.CanIds.EXAMPLE_LIFT_MOTOR, Constants.CanIds.RIO_BUS);

    // The config is built in the constants file, not here. tryUntilOk because CTRE devices silently
    // ignore configuration if the CAN bus is busy at startup — without the retry, the motor boots
    // with no current limits, no soft limits and no gains.
    PhoenixUtil.tryUntilOk(
        5, () -> motor.getConfigurator().apply(ExampleLiftConstants.getFXConfig()));

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

    // ---- Signal rates. This split is the team standard; use it on every mechanism. ----
    //
    // 50 Hz — everything the control loop or a match review needs every cycle. Torque current
    // belongs here, not in the slow group: under TorqueCurrentFOC it IS the control signal.
    // Closed-loop REFERENCE belongs here because comparing it against measured position is how
    // profile saturation becomes visible, and that comparison is meaningless if the two are sampled
    // at different rates.
    //
    // Velocity is 50 Hz here for a second reason: the zeroing routine's stall detection reads it,
    // and a stall check running on a stale velocity would confirm the hard stop early.
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
    // reference minus measured, both already at 50 Hz.
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, motorTemperature, closedLoopError);

    // 4 Hz — sticky faults, which latch until cleared.
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, stickyBootDuringEnable, stickyUndervoltage, stickyOverTemp, stickyHardware);

    // MUST BE LAST. optimizeBusUtilization() slows every signal not registered above to 4 Hz — a
    // Talon publishes dozens of signals by default and the unused ones are pure CAN bandwidth.
    //
    // It SLOWS rather than disables, deliberately, so the data still reaches the log. That makes a
    // forgotten signal STALE rather than MISSING, which is the harder failure to notice.
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ExampleLiftIOInputs inputs) {
    // Refresh the slow groups. Cheap — a signal that has not updated repeats its last value.
    BaseStatusSignal.refreshAll(
        motorTemperature,
        closedLoopError,
        stickyBootDuringEnable,
        stickyUndervoltage,
        stickyOverTemp,
        stickyHardware);

    // Connection comes from the 50 Hz group ALONE.
    //
    // refreshAll() returns the WORST status of everything passed to it, so folding the 10 Hz and
    // 4 Hz signals into this call would let the slowest one govern: for the first quarter second
    // after boot, before the first sticky-fault frame arrives, the mechanism reads disconnected and
    // every FaultMonitor alert tied to it fires for no reason.
    var status =
        BaseStatusSignal.refreshAll(
            motorPosition,
            motorVelocity,
            motorStatorAmps,
            motorSupplyAmps,
            motorTorqueCurrent,
            motorVoltage,
            closedLoopReference);
    inputs.connected = connectedDebounce.calculate(status.isOK());

    inputs.motorVoltage = motorVoltage.getValue();
    inputs.motorStatorAmps = motorStatorAmps.getValue();
    inputs.motorSupplyAmps = motorSupplyAmps.getValue();
    inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValue();
    inputs.motorPosition = motorPosition.getValue();
    inputs.motorVelocity = motorVelocity.getValue();
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

  @Override
  public void zeroPosition() {
    // tryUntilOk, with a timeout, because this is the one write where a silent failure is
    // catastrophic: the mechanism carries on believing a zero it never actually set, and every
    // height and soft limit afterward is off by a fixed unknown offset.
    PhoenixUtil.tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));
  }

  @Override
  public void setReverseSoftLimitEnabled(boolean enabled) {
    // The block comes from getFXConfig(), never a hand-built SoftwareLimitSwitchConfigs: apply()
    // writes the whole sub-group, and a fresh one would also reset the FORWARD bound to its class
    // default — off. See .claude/rules/02-hardware.md.
    //
    // Leader only. A follower mirrors the leader's output and owns no travel; its config has both
    // bounds switched off, and pushing this block to it would hand it the leader's forward bound.
    var softLimits = ExampleLiftConstants.getFXConfig().SoftwareLimitSwitch;
    softLimits.ReverseSoftLimitEnable = enabled;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(softLimits, 0.25));
  }

  // ==========================================================================================
  // FOLLOWER MOTORS — delete this block if this mechanism has one motor
  // ==========================================================================================
  //
  // Lifts are the mechanism most likely to need two motors, and the one where getting the follower
  // wrong is most expensive: the two fight, draw heavy current, and the carriage does not move.
  //
  //   private final TalonFX follower;
  //
  //   // ...in the constructor, AFTER configuring the leader:
  //   CanIdRegistry.claim(
  //       Constants.CanIds.EXAMPLE_LIFT_FOLLOWER,
  //       Constants.CanIds.RIO_BUS,
  //       ExampleLiftConstants.NAME + " follower");
  //   follower = new TalonFX(Constants.CanIds.EXAMPLE_LIFT_FOLLOWER, Constants.CanIds.RIO_BUS);
  //
  //   // The leader's whole config, built once and applied once — MINUS THE TRAVEL BOUNDS. A
  //   // follower runs a Follower request, not a position loop, and nothing reads its position, so
  //   // its soft limits would gate output against a register no code looks at. On an Opposed
  //   // follower that register runs BACKWARDS relative to the leader's, so the leader's window
  //   // lands on the wrong half of travel: the lift drops to one motor's torque partway through
  //   // legal motion while both devices report obeying their limits. The leader owns travel.
  //   TalonFXConfiguration followerConfig = ExampleLiftConstants.getFXConfig();
  //   followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
  //   followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  //   PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(followerConfig));
  //
  //   // MotorAlignmentValue.Opposed when the follower is physically mounted facing the opposite
  //   // way from the leader, Aligned when it faces the same way. On a two-motor gearbox driving
  //   // one drum this is usually Opposed. VERIFY AT LOW OUTPUT before running closed-loop.
  //   //
  //   // NOTE: Phoenix 6 2026 replaced Follower(int, boolean) with
  //   // Follower(int, MotorAlignmentValue). A boolean here will not compile.
  //   follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
  //
  // NEVER command a follower directly while the leader is running — including from the zeroing
  // routine, which drives the leader open-loop. The Follower request keeps mirroring throughout, so
  // zeroing works unchanged; the mistake would be adding a second setVoltage() call for the
  // follower.
  //
  // Two more places the follower has to be remembered:
  //
  //   - setReverseSoftLimitEnabled(): LEADER ONLY. The follower's bounds are off for good; pushing
  //     the leader's block to it would hand it the leader's forward bound in the wrong frame.
  //   - zeroPosition(): zero the follower too, with follower.setPosition(0.0, 0.25). Nothing reads
  //     it, but a follower a whole boot offset away from its leader is confusing to scrub past in
  //     a hoot log. Zero is zero in either alignment, so no sign correction is needed.
  //
  // LOG THE FOLLOWER, and CHECK IT WITH follower.isConnected() rather than signal status — its
  // signals sit at the 4 Hz optimize floor by design. Decide that rate deliberately: in 2026 the
  // intake roller's follower signals landed there by omission and the data looked present while
  // being a quarter second old.
  //
  // Imports: com.ctre.phoenix6.controls.Follower, com.ctre.phoenix6.signals.MotorAlignmentValue
}
