package frc.robot.subsystems.exampleLift.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.subsystems.exampleLift.ExampleLiftConstants;
import java.util.Arrays;
import java.util.stream.Stream;

/**
 * Real hardware implementation for a linear position-controlled mechanism.
 *
 * <p>This file holds <b>no numbers</b>. Every value comes from {@link ExampleLiftConstants}.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename throughout, add the CAN ID to {@code Constants.CanIds}, and pick
 * the bus deliberately.
 */
public class ExampleLiftIOReal implements ExampleLiftIO {

  /**
   * Every dashboard value pushed to the Talon: the Slot0 gains plus the Motion Magic profile.
   *
   * <p>Built once from the constants file's own arrays so a value cannot be tunable but unwatched.
   */
  private static final LoggedTunableNumber[] LIVE_TUNABLES =
      Stream.concat(
              Arrays.stream(ExampleLiftConstants.TUNABLE_GAINS),
              Arrays.stream(ExampleLiftConstants.TUNABLE_PROFILE))
          .toArray(LoggedTunableNumber[]::new);

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

    updateTunedConfig();
  }

  /**
   * Pushes dashboard-edited gains and profile values to the Talon, but only when one moved.
   *
   * <p>Both live in flash on the motor controller, not in RAM on the roboRIO — that is why the
   * Talon can close its loop at 1 kHz instead of at our 50 Hz. The cost is that changing one is a
   * blocking CAN transaction rather than a field write, so this cannot be done unconditionally
   * every loop.
   *
   * <p>Five things here are deliberate:
   *
   * <ul>
   *   <li><b>Gated on {@code tuningMode}.</b> In competition this returns immediately and does zero
   *       CAN work.
   *   <li><b>{@code apply(config.Slot0)}, not {@code apply(config)}.</b> A full config apply would
   *       rewrite current limits, soft limits, inversion and feedback ratios every time you nudged
   *       a gain — and on this mechanism rewriting the soft limits mid-tuning-session is exactly
   *       the kind of surprise you do not want with a carriage in the air.
   *   <li><b>The Slot0 block comes from {@code getFXConfig()}, not hand-built here.</b> A {@code
   *       new Slot0Configs()} carrying only kS..kD would leave {@code GravityType} at its class
   *       default, silently changing how gravity is compensated while you tuned kP.
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
          TalonFXConfiguration config = ExampleLiftConstants.getFXConfig();

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

  @Override
  public void zeroPosition() {
    // tryUntilOk, with a timeout, because this is the one write where a silent failure is
    // catastrophic: the mechanism carries on believing a zero it never actually set, and every
    // height and soft limit afterward is off by a fixed unknown offset.
    PhoenixUtil.tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));
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
  //   follower = new TalonFX(Constants.CanIds.EXAMPLE_LIFT_FOLLOWER, Constants.CanIds.RIO_BUS);
  //   PhoenixUtil.tryUntilOk(
  //       5, () -> follower.getConfigurator().apply(ExampleLiftConstants.getFXConfig()));
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
  // LOG THE FOLLOWER, and CHECK IT WITH follower.isConnected() rather than signal status — its
  // signals sit at the 4 Hz optimize floor by design. Decide that rate deliberately: in 2026 the
  // intake roller's follower signals landed there by omission and the data looked present while
  // being a quarter second old.
  //
  // Imports: com.ctre.phoenix6.controls.Follower, com.ctre.phoenix6.signals.MotorAlignmentValue
}
