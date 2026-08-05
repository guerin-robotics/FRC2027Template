package frc.robot.subsystems.example.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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

    // PICK ONE and delete the other. See the two methods below.
    //   Velocity — rollers, flywheels, feeders, transports
    //   Position — hoods, pivots, arms, elevators, turrets
    TalonFXConfiguration config = configureForVelocityControl();

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

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
  // CONTROL MODE CONFIGS — keep the one this mechanism uses, delete the other
  // ==========================================================================================

  /**
   * Configuration for <b>velocity control</b> — the mechanism spins and you care about its speed.
   *
   * <p>In 2026 this covered the flywheel, prestage, both feeders, the transport and the intake
   * roller. It reads position off the motor's internal rotor; no external encoder is involved,
   * because nothing needs to know where a roller is, only how fast it is turning.
   */
  private TalonFXConfiguration configureForVelocityControl() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // rollers usually coast
    config.MotorOutput.Inverted =
        ExampleSubsystemConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Report in mechanism units rather than motor rotations, so every setpoint, gain and logged
    // velocity is in terms of the thing that actually spins.
    config.Feedback.SensorToMechanismRatio = ExampleSubsystemConstants.GEAR_RATIO;

    config.CurrentLimits.SupplyCurrentLimit = ExampleSubsystemConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerLimit =
        ExampleSubsystemConstants.SUPPLY_CURRENT_TRIGGER_AMPS;
    config.CurrentLimits.SupplyCurrentLowerTime =
        ExampleSubsystemConstants.SUPPLY_CURRENT_TRIGGER_TIME.in(Seconds);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ExampleSubsystemConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Clamp what the closed loop may ASK for, not just what the stator will pass. Under
    // TorqueCurrentFOC the request is a current, so the stator limit alone does not bound it.
    config.TorqueCurrent.PeakForwardTorqueCurrent =
        ExampleSubsystemConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        -ExampleSubsystemConstants.STATOR_CURRENT_LIMIT_AMPS;

    // Ramps velocity changes instead of stepping them. Acceleration only — cruise velocity is
    // meaningless for a velocity profile, since the setpoint IS the cruise.
    config.MotionMagic.MotionMagicAcceleration =
        ExampleSubsystemConstants.MOTION_MAGIC_ACCELERATION;

    // kV carries steady state, kP corrects the rest, kS breaks static friction.
    config.Slot0.kS = ExampleSubsystemConstants.getKS();
    config.Slot0.kV = ExampleSubsystemConstants.getKV();
    config.Slot0.kA = ExampleSubsystemConstants.getKA();
    config.Slot0.kP = ExampleSubsystemConstants.getKP();
    config.Slot0.kI = ExampleSubsystemConstants.getKI();
    config.Slot0.kD = ExampleSubsystemConstants.getKD();

    return config;
  }

  /**
   * Configuration for <b>position control</b> — the mechanism moves to a place and holds it.
   *
   * <p>In 2026 this covered the hood and the intake pivot. It differs from velocity control in four
   * ways that all matter:
   *
   * <ol>
   *   <li><b>An absolute encoder.</b> A position mechanism has to know where it is at boot, without
   *       first being driven into a limit switch. That means a CANcoder, fused with the rotor so
   *       multi-turn travel still tracks.
   *   <li><b>Two ratios, not one.</b> The encoder sits partway down the gear train, so Phoenix
   *       needs the motor-to-encoder ratio and the encoder-to-mechanism ratio separately.
   *   <li><b>Software limits.</b> A roller cannot travel too far. A pivot can, and will, into a
   *       hard stop at full current.
   *   <li><b>Brake, not coast.</b> A coasting arm falls.
   * </ol>
   *
   * <p>Requires a {@code CANcoder} field and {@link #configureEncoder()} run <b>before</b> this —
   * the motor reads the encoder during its own configuration.
   */
  @SuppressWarnings("unused") // delete this and the method if the mechanism is velocity-controlled
  private TalonFXConfiguration configureForPositionControl() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // a coasting arm falls
    config.MotorOutput.Inverted =
        ExampleSubsystemConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // FusedCANcoder combines the CANcoder's absolute reading with the rotor's incremental counts:
    // absolute position at boot, plus fine resolution and multi-turn tracking while running.
    // Plain RemoteCANcoder only sees the absolute value and wraps to zero every encoder turn,
    // which breaks any mechanism whose encoder shaft turns more than once across its range.
    //
    // config.Feedback.FeedbackRemoteSensorID = Constants.CanIds.EXAMPLE_ENCODER;
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = ExampleSubsystemConstants.ROTOR_TO_SENSOR_RATIO;
    config.Feedback.SensorToMechanismRatio = ExampleSubsystemConstants.SENSOR_TO_MECHANISM_RATIO;

    config.CurrentLimits.SupplyCurrentLimit = ExampleSubsystemConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ExampleSubsystemConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.TorqueCurrent.PeakForwardTorqueCurrent =
        ExampleSubsystemConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        -ExampleSubsystemConstants.STATOR_CURRENT_LIMIT_AMPS;

    // Profile the motion. A raw position request commands maximum effort instantly, which on a
    // mechanism with real inertia means slamming into the setpoint and into the hard stops.
    config.MotionMagic.MotionMagicAcceleration =
        ExampleSubsystemConstants.MOTION_MAGIC_ACCELERATION;
    config.MotionMagic.MotionMagicCruiseVelocity =
        ExampleSubsystemConstants.MOTION_MAGIC_CRUISE_VELOCITY;

    // Enforced by the motor controller, so they hold even when a command sends a bad setpoint.
    // These protect the mechanism — treat changing them the way you would a current limit.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ExampleSubsystemConstants.FORWARD_SOFT_LIMIT_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ExampleSubsystemConstants.REVERSE_SOFT_LIMIT_ROTATIONS;

    // Position loops use kS, kP and kD. kG is added for anything gravity acts on — set the
    // matching GravityType, and remember Arm_Cosine requires zero to be horizontal.
    config.Slot0.kS = ExampleSubsystemConstants.getKS();
    config.Slot0.kG = ExampleSubsystemConstants.getKG();
    config.Slot0.kP = ExampleSubsystemConstants.getKP();
    config.Slot0.kD = ExampleSubsystemConstants.getKD();

    // Take the sign of kS from the closed-loop error, not from measured velocity. At rest on a
    // setpoint the velocity is ~0 and noisy, so UseVelocitySign flips back and forth and the
    // mechanism chatters.
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    return config;
  }

  /**
   * CANcoder setup for position control. Call <b>before</b> configuring the motor.
   *
   * <pre>
   * private final CANcoder encoder =
   *     new CANcoder(Constants.CanIds.EXAMPLE_ENCODER, Constants.CanIds.RIO_BUS);
   *
   * // in the constructor, in this order:
   * configureEncoder();
   * PhoenixUtil.tryUntilOk(5, () -&gt; motor.getConfigurator().apply(configureForPositionControl()));
   *
   * // and inside this method:
   * CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
   * encoderConfig.MagnetSensor.MagnetOffset = ExampleSubsystemConstants.MAGNET_OFFSET_ROTATIONS;
   * encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
   *     ExampleSubsystemConstants.SENSOR_DISCONTINUITY_POINT;
   * encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
   * PhoenixUtil.tryUntilOk(5, () -&gt; encoder.getConfigurator().apply(encoderConfig));
   * </pre>
   *
   * <p><b>THE TRAP.</b> After configuring, the CANcoder's Position and Velocity signals must be
   * registered explicitly before {@code optimizeBusUtilization()}:
   *
   * <pre>
   * BaseStatusSignal.setUpdateFrequencyForAll(50.0, encoder.getPosition(), encoder.getVelocity());
   * encoder.optimizeBusUtilization();
   * </pre>
   *
   * The motor reads those signals off the bus to do the fusion. Without the registration,
   * optimization drops them to the 4 Hz default and the fused position updates twelve times slower
   * than the loop depending on it — which presents as a position loop that lags and hunts, not as
   * anything resembling a configuration problem.
   */
  @SuppressWarnings("unused") // delete along with configureForPositionControl() if unused
  private void configureEncoder() {
    // Intentionally empty — see the javadoc. This needs a CANcoder field that only exists once
    // you have decided this mechanism is position-controlled.
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
  //   follower = new TalonFX(Constants.CanIds.EXAMPLE_FOLLOWER, Constants.CanIds.RIO_BUS);
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
