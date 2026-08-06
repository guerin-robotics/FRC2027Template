package frc.robot.subsystems.example.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.LoggedTunableNumber;
import frc.lib.PhoenixUtil;
import frc.robot.Constants;
import frc.robot.subsystems.example.ExampleSubsystemConstants;
import java.util.Arrays;
import java.util.stream.Stream;

/**
 * Real hardware implementation of ExampleSubsystemIO.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Replace "TODO_MOTOR_CAN_ID" with your CAN ID constant from
 * Constants.CanIds 2. Replace "TODO_CAN_BUS" with "rio" or "Canivore" 3. Configure
 * TalonFXConfiguration for your motor (current limits, neutral mode, etc.) 4. Add StatusSignal
 * fields for every signal in ExampleSubsystemIOInputs 5. Add control request objects (VoltageOut,
 * VelocityTorqueCurrentFOC, etc.) 6. Call BaseStatusSignal.refreshAll() at the top of
 * updateInputs()
 */
public class ExampleSubsystemIOReal implements ExampleSubsystemIO {

  /**
   * Every dashboard value that gets pushed to the Talon: the Slot0 gains plus the Motion Magic
   * profile. Built once so a value cannot be tunable but unwatched — a tunable missing from this
   * list still moves on the dashboard and never reaches the motor.
   */
  private static final LoggedTunableNumber[] LIVE_TUNABLES =
      Stream.concat(
              Arrays.stream(ExampleSubsystemConstants.TUNABLE_GAINS),
              Arrays.stream(ExampleSubsystemConstants.TUNABLE_PROFILE))
          .toArray(LoggedTunableNumber[]::new);

  private final TalonFX motor;

  // Status signals — one per logged field
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorStatorAmps;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorSupplyAmps;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorTorqueCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> motorTemperature;

  // Closed-loop diagnostics. These are what turn "it didn't get there" into an answer: the
  // reference says what the profile was ASKING for, the error says how far off it was. Without
  // them a log shows the mechanism in the wrong place with no way to tell whether the setpoint
  // was wrong, the profile was saturating, or the gains could not keep up.
  private final StatusSignal<Double> closedLoopReference;
  private final StatusSignal<Double> closedLoopError;

  // Sticky faults latch until cleared, so they record what happened BETWEEN polls. 4 Hz is
  // plenty — see .claude/rules/02-hardware.md.
  private final StatusSignal<Boolean> stickyBootDuringEnable;
  private final StatusSignal<Boolean> stickyUndervoltage;
  private final StatusSignal<Boolean> stickyOverTemp;
  private final StatusSignal<Boolean> stickyHardware;

  /**
   * Debounces the connection check.
   *
   * <p>{@code kFalling} starts optimistic and only reports a disconnect once the condition has held
   * for half a second, so one dropped frame does not raise an alert mid-match. Same shape {@code
   * ModuleIOTalonFX} uses for the swerve modules.
   */
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Control requests. Build these ONCE — allocating a request object every loop is a
  // per-cycle allocation the GC has to clean up inside the 20 ms budget.
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  /** Velocity control, profiled. Delete if this mechanism is position-controlled. */
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  /** Position control, profiled. Delete if this mechanism is velocity-controlled. */
  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  public ExampleSubsystemIOReal() {
    // TODO: add the real ID to Constants.CanIds, and pick the bus deliberately — RIO_BUS here,
    // or TunerConstants.kCANBus for the CANivore. The (int, String) constructor is deprecated
    // for removal in Phoenix 6; pass a CANBus.
    motor = new TalonFX(Constants.CanIds.EXAMPLE_MOTOR, Constants.CanIds.RIO_BUS);

    // The config is built in the constants file, not here — see ExampleSubsystemConstants.
    // Keep this IO free of numbers so every mechanism's IO reads the same and the only place to
    // look for a value is the one file named for it.
    PhoenixUtil.tryUntilOk(
        5, () -> motor.getConfigurator().apply(ExampleSubsystemConstants.getVelocityFXConfig()));

    // Acquire signal handles
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
    // 50 Hz — everything the control loop or a match review needs per cycle. Torque current
    // belongs here, not in the slow group: under TorqueCurrentFOC it IS the control signal.
    // Closed-loop REFERENCE belongs here too, because comparing it against measured velocity
    // or position is how you see profile saturation, and that comparison is worthless if the
    // two are sampled at different rates.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorVelocity,
        motorPosition,
        motorStatorAmps,
        motorSupplyAmps,
        motorTorqueCurrent,
        motorVoltage,
        closedLoopReference);

    // 10 Hz — diagnostics that change slowly. Temperature moves over minutes. Closed-loop
    // ERROR is here rather than at 50 Hz because it is derivable from reference minus measured,
    // both of which are already at 50 Hz; this channel is a convenience for scrubbing a log,
    // not an input to anything.
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, motorTemperature, closedLoopError);

    // 4 Hz — sticky faults, which latch until cleared, so a fast rate buys nothing.
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, stickyBootDuringEnable, stickyUndervoltage, stickyOverTemp, stickyHardware);

    // MUST BE LAST, and must run on every device. optimizeBusUtilization() slows every signal
    // not registered above to 4 Hz — a Talon publishes dozens of signals by default and the
    // unused ones are pure CAN bandwidth.
    //
    // It SLOWS rather than disables, deliberately, so the data still reaches the log. That
    // makes a forgotten signal stale rather than missing, which is the harder failure to
    // notice: the channel is there, the numbers look plausible, and they are a quarter second
    // old. Pass a frequency to change the floor — optimizeBusUtilization(0.0) to truly
    // disable.
    //
    // Signals share status frames. If any signal in a frame has an explicit frequency, that
    // frequency is honoured for the whole frame, so the cost of registering one signal is not
    // strictly one signal's worth of bandwidth.
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ExampleSubsystemIOInputs inputs) {
    // Refresh the slow groups. Cheap — a signal that has not updated simply repeats its last
    // value, so calling this at 50 Hz on a 10 Hz or 4 Hz signal costs nothing.
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
    // 4 Hz signals in here would let the slowest one govern: for the first quarter second after
    // boot, before the first slow frame arrives, the mechanism reads disconnected and every
    // FaultMonitor alert tied to it fires for no reason.
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

    // A FOLLOWER is checked with follower.isConnected() instead — its signals sit at the 4 Hz
    // optimize floor by design, so a status built from them reports staleness, not presence.
    //
    // A SEPARATE DEVICE gets its own status. A CANcoder can drop out while its motor stays
    // healthy, which is exactly the failure worth catching, and merging the two hides it.

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
   *       only the gain block. A full config apply would also rewrite current limits, soft limits,
   *       inversion and feedback ratios every time you nudged a gain.
   *   <li><b>The Slot0 block comes from {@code getFXConfig()}, not hand-built here.</b> A {@code
   *       new Slot0Configs()} carrying only kS..kD would leave {@code GravityType} and {@code
   *       StaticFeedforwardSign} at their defaults — so tuning kP on an arm would silently switch
   *       its gravity compensation from {@code Arm_Cosine} to {@code Elevator_Static}. Taking the
   *       whole block from the canonical config means the gains and their modifiers can never
   *       disagree.
   *   <li><b>Slot0 <i>and</i> MotionMagic.</b> The gains are in Slot0, but cruise velocity and
   *       acceleration are in MotionMagic. Applying only Slot0 leaves the profile knobs moving on
   *       the dashboard while the mechanism ignores them, which reads as a broken tunable.
   *   <li><b>{@code ifChanged}, not every loop.</b> Expect the one loop it fires on to overrun the
   *       20 ms budget. That is acceptable in a tuning session, which is the only time it can
   *       happen.
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
          TalonFXConfiguration config = ExampleSubsystemConstants.getVelocityFXConfig();

          // BOTH blocks. The gains are in Slot0, but cruise velocity and acceleration are in
          // MotionMagic — apply only Slot0 and the profile knobs move on the dashboard while
          // the mechanism ignores them, which reads as "the tunable is broken".
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
  public void setPosition(Angle position) {
    motor.setControl(positionRequest.withPosition(position.in(Rotations)));
  }

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }

  // ==========================================================================================
  // CANCODER SETUP — position-controlled mechanisms only, delete otherwise
  // ==========================================================================================
  //
  // A position mechanism needs an absolute encoder so it knows where it is at boot, without
  // being driven into a limit switch first. Configure it BEFORE the motor — the motor reads the
  // encoder while applying its own config.
  //
  //   private final CANcoder encoder =
  //       new CANcoder(Constants.CanIds.EXAMPLE_ENCODER, Constants.CanIds.RIO_BUS);
  //
  //   // in the constructor, in this order:
  //   configureEncoder();
  //   PhoenixUtil.tryUntilOk(
  //       5, () -> motor.getConfigurator().apply(ExampleSubsystemConstants.getFXConfig()));
  //
  //   private void configureEncoder() {
  //     CANcoderConfiguration cfg = new CANcoderConfiguration();
  //     cfg.MagnetSensor.MagnetOffset = ExampleSubsystemConstants.MAGNET_OFFSET_ROTATIONS;
  //     cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
  //         ExampleSubsystemConstants.SENSOR_DISCONTINUITY_POINT;
  //     cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
  //     PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(cfg));
  //
  //     // Connect the motor to it as a remote, fused sensor — fluent .withX() style, not field
  //     // assignment, so this reads as one clearly-scoped operation. Do this as part of the
  //     // main config in ExampleSubsystemConstants.getFXConfig(), not a separate apply() call:
  //     //
  //     //   config.Feedback
  //     //       .withFeedbackRemoteSensorID(Constants.CanIds.EXAMPLE_ENCODER)
  //     //       .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
  //     //       .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO)
  //     //       .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO);
  //     //
  //     // A second, separate apply() of a fresh FeedbackConfigs object after the main config is
  //     // a real trap: apply() on a partial config writes the WHOLE sub-group to the device, so
  //     // untouched fields on that second object — RotorToSensorRatio included — go back to
  //     // their class defaults (1.0) and silently undo what the first apply() just set.
  //
  //     // THE TRAP. Register the encoder's Position and Velocity BEFORE optimizing, or
  //     // optimization drops them to the 4 Hz default. The motor reads those signals off the
  //     // bus to do the rotor/CANcoder fusion, so the fused position then updates twelve times
  //     // slower than the loop depending on it. That presents as a position loop that lags and
  //     // hunts — nothing that looks like a configuration problem.
  //     BaseStatusSignal.setUpdateFrequencyForAll(
  //         50.0, encoder.getPosition(), encoder.getVelocity());
  //     encoder.optimizeBusUtilization();
  //   }

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
  //   follower = new TalonFX(Constants.CanIds.EXAMPLE_FOLLOWER, Constants.CanIds.RIO_BUS);
  //   PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));
  //
  //   // MotorAlignmentValue.Opposed when the follower is physically mounted facing the
  //   // opposite way from the leader, Aligned when it faces the same way. Getting this wrong
  //   // makes the two motors fight each other — high current, no motion, and it will cook a
  //   // gearbox. Verify at low output before running closed-loop.
  //   //
  //   // NOTE: Phoenix 6 2026 replaced the old Follower(int, boolean) with
  //   // Follower(int, MotorAlignmentValue). A boolean here will not compile.
  //   follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
  //
  // LOG THE FOLLOWER TOO. It is a real motor drawing real current and generating real heat,
  // and it can fail independently of the leader — a follower that has quietly stopped looks
  // exactly like a leader that is underpowered.
  //
  //   private final StatusSignal<Current> followerStatorAmps;
  //   ...
  //   followerStatorAmps = follower.getStatorCurrent();
  //
  // AND DECIDE ITS SIGNAL RATE. In 2026 the intake roller's follower signals were acquired but
  // never added to setUpdateFrequencyForAll(), so optimizeBusUtilization() left them at the
  // 4 Hz default. The data looked present and was simply stale, which is far harder to notice
  // than data that is missing.
  //
  // 4 Hz is a fine rate for a follower — it is enough to see the motor is alive, drawing
  // current and not overheating. The point is to arrive there on purpose. If you want the
  // follower at the same rate as the leader, register its signals explicitly.
  //
  //   BaseStatusSignal.setUpdateFrequencyForAll(50, followerStatorAmps, /* ...and the rest */);
  //   follower.optimizeBusUtilization();
  //
  // Imports: com.ctre.phoenix6.controls.Follower, com.ctre.phoenix6.signals.MotorAlignmentValue

  // ==========================================================================================
  // ZEROING — relative encoders only (no CANcoder, or a CANcoder that can't cover full travel
  // in one turn). Delete this block if the mechanism has an absolute encoder that fits — see
  // "Before fitting an absolute encoder" in template/GUIDE.md.
  // ==========================================================================================
  //
  // A motor's internal rotor position is relative: it means nothing until something establishes
  // where zero actually is, and power-up silently assumes the mechanism is already there. Build
  // a routine that drives into a hard stop and declares that position zero. This is the IO-level
  // primitive only — the routine that decides WHEN to call it lives in the command factory, not
  // here. See ExampleCommands' commented ZEROING block for the full shape, and
  // docs/new-mechanism-bringup.md Phase 1 step 6 for why each piece matters.
  //
  //   @Override
  //   public void zeroPosition() {
  //     PhoenixUtil.tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));
  //   }
  //
  // Add `default void zeroPosition() {}` to ExampleSubsystemIO alongside the other control
  // outputs if this mechanism needs it.
}
