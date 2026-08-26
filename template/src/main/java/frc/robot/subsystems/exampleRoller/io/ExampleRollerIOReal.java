package frc.robot.subsystems.exampleRoller.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
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
import frc.robot.subsystems.exampleRoller.ExampleRollerConstants;
import java.util.Arrays;
import java.util.stream.Stream;

/**
 * Real hardware implementation for a velocity-controlled mechanism.
 *
 * <p>This file holds <b>no numbers</b>. Every value comes from {@link ExampleRollerConstants},
 * which is what lets a reviewer compare two mechanisms' IO files and see only the differences that
 * matter.
 *
 * <p>TEMPLATE INSTRUCTIONS:
 *
 * <ol>
 *   <li>Rename {@code ExampleRoller} to your mechanism name throughout.
 *   <li>Add the CAN ID to {@code Constants.CanIds}, and pick the bus deliberately.
 *   <li>Add a {@link StatusSignal} field for every input you added to the inputs class, register it
 *       at the right rate, and read it in {@link #updateInputs}.
 * </ol>
 */
public class ExampleRollerIOReal implements ExampleRollerIO {

  /**
   * Every dashboard value pushed to the Talon: the Slot0 gains plus the Motion Magic profile.
   *
   * <p>Built once from the constants file's own arrays so a value cannot be tunable but unwatched.
   * A tunable missing from this list still moves on the dashboard and never reaches the motor,
   * which reads as "the tunable is broken".
   */
  private static final LoggedTunableNumber[] LIVE_TUNABLES =
      Stream.concat(
              Arrays.stream(ExampleRollerConstants.TUNABLE_GAINS),
              Arrays.stream(ExampleRollerConstants.TUNABLE_PROFILE))
          .toArray(LoggedTunableNumber[]::new);

  private final TalonFX motor;

  // ---- Status signals, one per logged field ----
  //
  // Cached once here rather than fetched inside updateInputs(). Calling motor.getStatorCurrent()
  // every loop allocates a new signal object every loop, which is garbage the collector has to
  // clean up inside the 20 ms budget.

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorStatorAmps;
  private final StatusSignal<Current> motorSupplyAmps;
  private final StatusSignal<Current> motorTorqueCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Angle> motorPosition;
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
   * for half a second, so a single dropped frame does not raise an alert mid-match. Same shape
   * {@code ModuleIOTalonFX} uses for the swerve modules — copy it rather than inventing something.
   */
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // ---- Control requests ----
  //
  // Built ONCE. Allocating a request object per loop is a per-cycle allocation inside the 20 ms
  // budget, and pause behaviour on this robot comes from not allocating rather than from a GC flag
  // (see .claude/rules/04-build.md).

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  public ExampleRollerIOReal() {
    // TODO: add the real ID to Constants.CanIds, and pick the bus deliberately — RIO_BUS here, or
    // TunerConstants.kCANBus for the CANivore. The (int, String) constructor is deprecated for
    // removal in Phoenix 6; pass a CANBus object.
    motor = new TalonFX(Constants.CanIds.EXAMPLE_ROLLER_MOTOR, Constants.CanIds.RIO_BUS);

    // The config is built in the constants file, not here. tryUntilOk because CTRE devices
    // silently ignore configuration if the CAN bus is busy at startup — without the retry, the
    // motor boots with no current limits and no gains, and the first match move can brown out
    // the robot.
    PhoenixUtil.tryUntilOk(
        5, () -> motor.getConfigurator().apply(ExampleRollerConstants.getFXConfig()));

    // ---- Acquire signal handles ----
    motorVoltage = motor.getMotorVoltage();
    motorStatorAmps = motor.getStatorCurrent();
    motorSupplyAmps = motor.getSupplyCurrent();
    motorTorqueCurrent = motor.getTorqueCurrent();
    motorVelocity = motor.getVelocity();
    motorPosition = motor.getPosition();
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
    // Closed-loop REFERENCE belongs here too, because comparing it against measured velocity is
    // how profile saturation becomes visible, and that comparison is meaningless if the two are
    // sampled at different rates.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorVelocity,
        motorPosition,
        motorStatorAmps,
        motorSupplyAmps,
        motorTorqueCurrent,
        motorVoltage,
        closedLoopReference);

    // 10 Hz — diagnostics that change slowly. Temperature moves over minutes. Closed-loop ERROR
    // sits here rather than at 50 Hz because it is reference minus measured, both of which are
    // already at 50 Hz; the channel is a convenience for scrubbing a log, not an input to
    // anything.
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, motorTemperature, closedLoopError);

    // 4 Hz — sticky faults, which latch until cleared, so a fast rate buys nothing.
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, stickyBootDuringEnable, stickyUndervoltage, stickyOverTemp, stickyHardware);

    // MUST BE LAST, and must run on every device. optimizeBusUtilization() slows every signal not
    // registered above to 4 Hz — a Talon publishes dozens of signals by default and the unused
    // ones are pure CAN bandwidth.
    //
    // It SLOWS rather than disables, deliberately, so the data still reaches the log. That makes a
    // forgotten signal STALE rather than MISSING, which is the harder failure to notice: the
    // channel is there, the numbers look plausible, and they are a quarter second old. Pass a
    // frequency to change the floor — optimizeBusUtilization(0.0) to genuinely disable.
    //
    // Signals share status frames. If any signal in a frame has an explicit frequency, that
    // frequency is honoured for the whole frame, so the cost of registering one signal is not
    // strictly one signal's worth of bandwidth.
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ExampleRollerIOInputs inputs) {
    // Refresh the slow groups. Cheap — a signal that has not updated simply repeats its last
    // value, so polling a 10 Hz or 4 Hz signal at 50 Hz costs nothing.
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
    // after boot, before the first sticky-fault frame arrives, the mechanism reads disconnected
    // and every FaultMonitor alert tied to it fires for no reason.
    var status =
        BaseStatusSignal.refreshAll(
            motorVelocity,
            motorPosition,
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
    inputs.motorVelocity = motorVelocity.getValue();
    inputs.motorPosition = motorPosition.getValue();
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
   *       CAN work. The robot runs the compiled-in defaults, applied once in the constructor.
   *   <li><b>{@code apply(config.Slot0)}, not {@code apply(config)}.</b> The Slot0 overload writes
   *       only the gain block. A full config apply would rewrite current limits, soft limits,
   *       inversion and feedback ratios every time you nudged a gain.
   *   <li><b>The Slot0 block comes from {@code getFXConfig()}, not hand-built here.</b> A {@code
   *       new Slot0Configs()} carrying only kS..kD would leave every modifier at its class default.
   *       On this scaffold that is survivable; on {@code exampleArm} it would silently switch
   *       gravity compensation from {@code Arm_Cosine} to {@code Elevator_Static} while you tuned
   *       kP. Taking the whole block from the canonical config means the gains and their modifiers
   *       can never disagree, and keeping all three scaffolds identical here is what stops that
   *       trap from being reintroduced by a copy.
   *   <li><b>Slot0 <i>and</i> MotionMagic.</b> The gains are in Slot0, but acceleration is in
   *       MotionMagic. Applying only Slot0 leaves the profile knob moving on the dashboard while
   *       the mechanism ignores it.
   *   <li><b>{@code ifChanged}, not every loop.</b> Expect the one loop it fires on to overrun the
   *       20 ms budget. That is acceptable in a tuning session, which is the only time it happens.
   * </ul>
   *
   * <p>Note that {@code hasChanged} reports true on its first call, so this fires once on the first
   * loop after enabling and re-applies values the constructor already wrote. Harmless, but it means
   * the first change you see in a log is not a change.
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
          // Rebuilding the config re-reads every accessor, which reads the tunables.
          TalonFXConfiguration config = ExampleRollerConstants.getFXConfig();

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
  public void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
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
  // current.
  //
  //   private final TalonFX follower;
  //
  //   // ...in the constructor, AFTER configuring the leader:
  //   follower = new TalonFX(Constants.CanIds.EXAMPLE_ROLLER_FOLLOWER, Constants.CanIds.RIO_BUS);
  //   PhoenixUtil.tryUntilOk(
  //       5, () -> follower.getConfigurator().apply(ExampleRollerConstants.getFXConfig()));
  //
  //   // MotorAlignmentValue.Opposed when the follower is physically mounted facing the opposite
  //   // way from the leader, Aligned when it faces the same way. Getting this wrong makes the two
  //   // motors fight each other — high current, no motion, and it will cook a gearbox. Verify at
  //   // low output before running closed-loop.
  //   //
  //   // NOTE: Phoenix 6 2026 replaced the old Follower(int, boolean) with
  //   // Follower(int, MotorAlignmentValue). A boolean here will not compile.
  //   follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
  //
  // LOG THE FOLLOWER TOO. It is a real motor drawing real current and generating real heat, and it
  // can fail independently of the leader — a follower that has quietly stopped looks exactly like a
  // leader that is underpowered.
  //
  //   private final StatusSignal<Current> followerStatorAmps;
  //   ...
  //   followerStatorAmps = follower.getStatorCurrent();
  //
  // CHECK IT WITH follower.isConnected(), NOT signal status. Its signals sit at the 4 Hz optimize
  // floor by design, so a status built from them reports staleness rather than presence.
  //
  // AND DECIDE ITS SIGNAL RATE. In 2026 the intake roller's follower signals were acquired but
  // never added to setUpdateFrequencyForAll(), so optimizeBusUtilization() left them at the 4 Hz
  // default. The data looked present and was simply stale, which is far harder to notice than data
  // that is missing.
  //
  // 4 Hz is a fine rate for a follower — enough to see it is alive, drawing current and not
  // overheating. The point is to arrive there on purpose. Register explicitly if you want more:
  //
  //   BaseStatusSignal.setUpdateFrequencyForAll(50, followerStatorAmps, /* ...and the rest */);
  //   follower.optimizeBusUtilization();
  //
  // Imports: com.ctre.phoenix6.controls.Follower, com.ctre.phoenix6.signals.MotorAlignmentValue
}
