package frc.robot.subsystems.example.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.LoggedTunableNumber;
import frc.lib.PhoenixUtil;
import frc.robot.Constants;
import frc.robot.subsystems.example.ExampleSubsystemConstants;

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

  private final TalonFX motor;

  // Status signals — one per logged field
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorStatorAmps;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorSupplyAmps;
  private final StatusSignal<edu.wpi.first.units.measure.Current> motorTorqueCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> motorTemperature;

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
        motorPosition,
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
        motorPosition,
        motorTemperature);

    inputs.motorVoltage = motorVoltage.getValue();
    inputs.motorStatorAmps = motorStatorAmps.getValue();
    inputs.motorSupplyAmps = motorSupplyAmps.getValue();
    inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValue();
    inputs.motorVelocity = motorVelocity.getValue();
    inputs.motorPosition = motorPosition.getValue();
    inputs.motorTemperature = motorTemperature.getValue();

    updateTunedGains();
  }

  /**
   * Pushes dashboard-edited gains to the Talon, but only when one actually moved.
   *
   * <p>Slot0 gains live in flash on the motor controller, not in RAM on the roboRIO — that is why
   * the Talon can close its loop at 1 kHz instead of at our 50 Hz. The cost is that changing one is
   * a blocking CAN transaction rather than a field write, so this cannot be done unconditionally
   * every loop the way a WPILib {@code PIDController} gain can.
   *
   * <p>Three things here are deliberate:
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
  private void updateTunedGains() {
    if (!Constants.tuningMode) {
      return;
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          // Rebuilding the config re-reads every gain accessor, which reads the tunables.
          Slot0Configs gains = ExampleSubsystemConstants.getVelocityFXConfig().Slot0;
          PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(gains));
        },
        ExampleSubsystemConstants.TUNABLE_GAINS);
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
  // AND REGISTER ITS SIGNALS. This is the trap: optimizeBusUtilization() disables every
  // signal that was not explicitly registered. In 2026 the intake roller's follower signals
  // were never added to setUpdateFrequencyForAll(), so after optimization they published at
  // the 4 Hz default. The data looked present and was simply stale, which is far harder to
  // notice than data that is missing.
  //
  //   BaseStatusSignal.setUpdateFrequencyForAll(50, followerStatorAmps, /* ...and the rest */);
  //   follower.optimizeBusUtilization();
  //
  // Imports: com.ctre.phoenix6.controls.Follower, com.ctre.phoenix6.signals.MotorAlignmentValue
}
