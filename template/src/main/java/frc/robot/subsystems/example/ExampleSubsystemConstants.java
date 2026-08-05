package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

/**
 * Constants and motor configuration for ExampleSubsystem.
 *
 * <p><b>THIS FILE DOES NOT COMPILE ON PURPOSE.</b> The values that cannot be guessed — CAN IDs,
 * gear ratios, soft-limit thresholds, magnet offset — have their <i>declarations</i> commented out
 * while the config below still <i>assigns</i> them. That combination is deliberate: the compiler
 * names each missing value, one error per thing you owe it.
 *
 * <p>Commenting out the assignment instead would compile, and would be worse. Phoenix defaults
 * {@code SensorToMechanismRatio} to 1.0, so a config that silently skips it reports motor rotations
 * while every setpoint and gain in this file assumes mechanism rotations. That is a confidently
 * wrong robot, which is much harder to notice than one that refuses to build.
 *
 * <p>Everything that already has a value has a defensible default. Those are safe starting points,
 * not measurements.
 *
 * <h2>What does NOT go in this file</h2>
 *
 * <p><b>Setpoints.</b> This file describes how the mechanism is <i>built</i> — gains, gear ratios,
 * current limits, soft limits, tolerances, the sim model. What it is <i>commanded to</i> — target
 * velocities, heights, angles, voltages — belongs in {@code Constants.Setpoints}, together with
 * every other mechanism's.
 *
 * <p>The test is whether a driver might ask you to change it between matches. "Run the intake a
 * little faster" should send you to one file, not on a hunt across every subsystem package. Note
 * that a <i>tolerance</i> stays here — how close counts as "there" is a property of the mechanism,
 * not a knob the drive team turns.
 *
 * <p>Command factories take setpoints as parameters, and {@code RobotContainer} supplies them from
 * {@code Constants.Setpoints}. See {@code .claude/rules/03-commands.md}.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Rename "ExampleSubsystem" → your subsystem name 2. Keep either
 * {@link #getVelocityFXConfig()} or {@link #getPositionFXConfig()} and rename it {@code
 * getFXConfig()} — delete the other 3. Supply the commented-out values 4. Delete the constants this
 * mechanism has no use for
 *
 * <h2>The config lives here, not in the IO</h2>
 *
 * Every mechanism exposes a {@code getFXConfig()} returning a fully-built {@link
 * TalonFXConfiguration}, and every one sets the same blocks in the same order:
 *
 * <pre>
 * CurrentLimits → TorqueCurrent → Voltage → MotorOutput → SoftwareLimitSwitch
 *   → Feedback → Slot0 → MotionMagic
 * </pre>
 *
 * That uniformity is the point. When every mechanism's config reads the same, a reviewer comparing
 * two of them sees only the differences that matter, and "this one is missing its soft limits"
 * becomes visible at a glance instead of buried in a differently-shaped constructor. The IO
 * implementation shrinks to one line and holds no numbers at all.
 *
 * <p><b>WHAT DOES NOT GO HERE:</b>
 *
 * <ul>
 *   <li><b>CAN IDs.</b> {@code Constants.CanIds}, so every ID is visible in one table.
 *   <li><b>Setpoints, timeouts and tolerances used by commands.</b> {@code Constants} — anything
 *       you would change in the pit lives in one file.
 * </ul>
 *
 * <p><b>UNITS:</b> under {@code *TorqueCurrentFOC} the gains are in <b>AMPS</b>, not volts. See
 * {@code docs/characterization-and-tuning.md}.
 */
public class ExampleSubsystemConstants {

  private ExampleSubsystemConstants() {}

  /** Log key and alert prefix. One name used everywhere, so a grep finds all of it. */
  public static final String NAME = "ExampleSubsystem";

  // ==========================================================================================
  // CURRENT AND OUTPUT LIMITS — the defaults here are deliberate, not arbitrary
  // ==========================================================================================
  //
  // Safety limits, not tuning knobs (.claude/rules/00-safety.md). These are a conservative
  // starting point for a Kraken on a mechanism whose loading is not yet known.
  //
  // SUPPLY vs STATOR are different measurements and are not interchangeable:
  //   Supply — current drawn from the battery. Bounds brownout risk.
  //   Stator — current through the windings. Bounds heating and torque.
  // At low speed under load, supply is a fraction of stator.

  /** Steady-state supply current ceiling, in amps. */
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  /** Supply current allowed briefly before the limit engages. */
  public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 40.0;

  /** How long the lower limit may be exceeded — lets a mechanism draw inrush without clamping. */
  public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.1);

  /** Winding current ceiling, in amps. Governs heating and stall torque. */
  public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;

  /**
   * Peak torque current the closed loop may REQUEST, in amps.
   *
   * <p>Distinct from the stator limit, and both are needed. Under {@code TorqueCurrentFOC} the
   * control output is itself a current request, so without this clamp the loop can ask for
   * arbitrary current and only the stator limit stops it — after the fact, by saturating. Clamping
   * the request keeps the controller operating in a range it can actually deliver.
   */
  public static final double PEAK_FORWARD_TORQUE_CURRENT_AMPS = 80.0;

  /** Peak reverse torque current. Negative. */
  public static final double PEAK_REVERSE_TORQUE_CURRENT_AMPS = -80.0;

  /** Peak forward voltage. 12 V is nominal battery, so this is "no artificial cap". */
  public static final double PEAK_FORWARD_VOLTAGE = 12.0;

  /** Peak reverse voltage. Negative. */
  public static final double PEAK_REVERSE_VOLTAGE = -12.0;

  // ==========================================================================================
  // SOFT LIMITS — travel bounds enforced by the motor controller
  // ==========================================================================================
  //
  // Enforced in the controller, so they hold even when a command sends a bad setpoint or a gain
  // sends the mechanism running. That makes them mechanism protection rather than convenience —
  // treat changing one the way you would treat changing a current limit.
  //
  // Velocity mechanisms leave both disabled: a roller has no travel to bound, and a soft limit on
  // one silently stops it once accumulated position drifts past the threshold.

  /** Enable the forward travel limit. False for anything that spins continuously. */
  public static final boolean FORWARD_SOFT_LIMIT_ENABLED = true;

  /** Enable the reverse travel limit. False for anything that spins continuously. */
  public static final boolean REVERSE_SOFT_LIMIT_ENABLED = true;

  // TODO: supply the travel limits in MECHANISM rotations, then uncomment.
  // Measure by moving the mechanism to each hard stop, reading the reported position, then backing
  // off. A soft limit set exactly at the hard stop still lets the mechanism reach it.
  //
  // public static final double FORWARD_SOFT_LIMIT_ROTATIONS = ;
  // public static final double REVERSE_SOFT_LIMIT_ROTATIONS = ;

  // ==========================================================================================
  // MECHANICAL — none of this can be guessed
  // ==========================================================================================

  /** True if the motor is mounted so positive output produces negative mechanism motion. */
  public static final boolean INVERTED = false;

  // TODO: supply the gearing, then uncomment.
  //
  // VELOCITY mechanisms need only SENSOR_TO_MECHANISM_RATIO — motor rotations per mechanism
  // rotation, so setpoints and gains read in terms of the thing that actually spins.
  //
  // POSITION mechanisms with an absolute encoder need BOTH, split at the encoder:
  //   ROTOR_TO_SENSOR_RATIO      motor rotations per encoder rotation  (rotor/CANcoder fusion)
  //   SENSOR_TO_MECHANISM_RATIO  encoder rotations per mechanism rotation
  //
  // Worked example — the 2026 hood chain was
  //   motor → 30T belt → 20T shaft → CANcoder → 12T lantern → 122T hood
  //   ROTOR_TO_SENSOR_RATIO     = 30.0 / 20.0  = 1.5
  //   SENSOR_TO_MECHANISM_RATIO = 122.0 / 12.0 ≈ 10.17
  //
  // public static final double ROTOR_TO_SENSOR_RATIO = ;
  // public static final double SENSOR_TO_MECHANISM_RATIO = ;

  // TODO: position mechanisms only — supply the CANcoder calibration, then uncomment.
  //
  // MAGNET_OFFSET is calibration data, like the swerve encoder offsets: found by physically moving
  // the mechanism to a known position and reading the raw sensor. Not guessable, and it changes
  // whenever the encoder or magnet is disturbed.
  //
  // SENSOR_DISCONTINUITY_POINT is where the absolute reading wraps — 1.0 gives [0, 1), 0.5 gives
  // [-0.5, 0.5). Put the discontinuity somewhere the mechanism never travels, or position jumps a
  // full rotation mid-motion.
  //
  // public static final double MAGNET_OFFSET_ROTATIONS = ;
  // public static final double SENSOR_DISCONTINUITY_POINT = ;

  // ==========================================================================================
  // MOTION PROFILE
  // ==========================================================================================

  /** Profile acceleration, in mechanism rotations per second squared. */
  public static final double MOTION_MAGIC_ACCELERATION = 100.0;

  /**
   * Profile cruise velocity, in mechanism rotations per second.
   *
   * <p>Position control only — meaningless for a velocity profile, where the setpoint is the
   * cruise.
   */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 10.0;

  // ==========================================================================================
  // TOLERANCES
  // ==========================================================================================

  /**
   * How close counts as "at velocity", in mechanism rotations/sec.
   *
   * <p>Too tight and the mechanism never reports ready, so every sequence runs to its timeout
   * instead of proceeding. Too loose and it acts before it has arrived.
   */
  public static final double VELOCITY_TOLERANCE_ROTATIONS_PER_SEC = 2.0;

  /** How close counts as "at position", in mechanism rotations. 1° is 1/360. */
  public static final double POSITION_TOLERANCE_ROTATIONS = 1.0 / 360.0;

  // ==========================================================================================
  // GAINS — REAL ROBOT
  // ==========================================================================================
  //
  // Placeholders. Measure them: kS -> kV -> kA -> kP for velocity, kG -> kS -> kP -> kD for
  // position. Never start with kP. See docs/characterization-and-tuning.md Part 3.

  /** Static friction feedforward — output needed to break the mechanism loose. */
  public static final double KS = 0.0;

  /** Velocity feedforward — output per unit of steady-state velocity. */
  public static final double KV = 0.0;

  /** Acceleration feedforward — output per unit of acceleration. */
  public static final double KA = 0.0;

  /** Gravity feedforward. Arms and elevators only; set the matching {@link GravityTypeValue}. */
  public static final double KG = 0.0;

  /** Proportional gain. Corrects what feedforward misses — it should not be doing the work. */
  public static final double KP = 0.0;

  /** Integral gain. Leave at zero unless you can explain its behavior during a stall. */
  public static final double KI = 0.0;

  /** Derivative gain. Damps overshoot on position loops; amplifies noise on velocity loops. */
  public static final double KD = 0.0;

  // ==========================================================================================
  // SIMULATION
  // ==========================================================================================

  /**
   * Simulation constants, deliberately nested and deliberately separate from the real gains.
   *
   * <p><b>Why sim needs its own gains.</b> The sim model is not the robot. No backlash, no belt
   * stretch, no friction beyond what the model declares, and an inertia that is usually a guess.
   * Gains that behave well against that model are frequently wrong on hardware, and gains measured
   * on hardware often will not converge in sim. One shared set means one of the two is lying.
   *
   * <p>Nested rather than flattened so {@code Sim.KP} beside {@code KP} in a diff makes it obvious
   * which one is being edited.
   */
  public static class Sim {

    private Sim() {}

    /** The motor(s) driving this mechanism, for the sim model. */
    public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

    /** How many motors drive it. Must agree with {@link #MOTOR}. */
    public static final int NUM_MOTORS = 1;

    /**
     * Moment of inertia in kg·m².
     *
     * <p>Sets how fast the simulated mechanism accelerates, so a wrong value makes sim-tuned gains
     * meaningless. Worth a CAD number rather than a guess if the mechanism matters.
     */
    public static final double MOI = 0.001;

    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KG = 0.0;
    public static final double KP = 1.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
  }

  // ==========================================================================================
  // MODE-AWARE GAIN ACCESSORS
  // ==========================================================================================
  //
  // Use these rather than reading KP / Sim.KP directly, so no caller has to know which mode it is
  // in. REPLAY resolves to the REAL gains, which is correct — replay re-runs recorded hardware
  // inputs and should behave like the real robot.

  private static boolean isSim() {
    return Constants.currentMode == Constants.Mode.SIM;
  }

  private static boolean isReal() {
    return Constants.currentMode == Constants.Mode.REAL;
  }

  public static double getKS() {
    return isSim() ? Sim.KS : KS;
  }

  public static double getKV() {
    return isSim() ? Sim.KV : KV;
  }

  public static double getKA() {
    return isSim() ? Sim.KA : KA;
  }

  public static double getKG() {
    return isSim() ? Sim.KG : KG;
  }

  public static double getKP() {
    return isSim() ? Sim.KP : KP;
  }

  public static double getKI() {
    return isSim() ? Sim.KI : KI;
  }

  public static double getKD() {
    return isSim() ? Sim.KD : KD;
  }

  // ==========================================================================================
  // MOTOR CONFIG — keep ONE of these, rename it getFXConfig(), delete the other
  // ==========================================================================================

  /**
   * Configuration for <b>velocity control</b> — the mechanism spins and you care about its speed.
   *
   * <p>In 2026 this covered the flywheel, prestage, both feeders, the transport and the intake
   * roller. Feedback comes off the motor's internal rotor; no external encoder is involved, because
   * nothing needs to know where a roller <i>is</i>, only how fast it is turning.
   *
   * @return a fully-built config, ready for {@code PhoenixUtil.tryUntilOk(5, ...)}
   */
  public static TalonFXConfiguration getVelocityFXConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // ---- Current limits ----
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME.in(Seconds);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;

    // ---- Torque current clamp ----
    // Sim does not model torque current, so applying the clamp there only confuses the model.
    if (isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_FORWARD_TORQUE_CURRENT_AMPS;
      config.TorqueCurrent.PeakReverseTorqueCurrent = PEAK_REVERSE_TORQUE_CURRENT_AMPS;
    }

    // ---- Voltage clamp ----
    config.Voltage.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
    config.Voltage.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;

    // ---- Motor output ----
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // rollers coast
    config.MotorOutput.Inverted =
        INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // ---- Soft limits ----
    // Off for anything that spins continuously — see the note on the enable constants above.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // ---- Feedback ----
    // Internal rotor only. SensorToMechanismRatio makes velocity read in mechanism units.
    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;

    // ---- Gains ----
    config.Slot0.kS = getKS();
    config.Slot0.kV = getKV();
    config.Slot0.kA = getKA();
    config.Slot0.kP = getKP();
    config.Slot0.kI = getKI();
    config.Slot0.kD = getKD();

    // ---- Motion profile ----
    // Acceleration only. For a velocity profile the setpoint IS the cruise.
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;

    return config;
  }

  /**
   * Configuration for <b>position control</b> — the mechanism moves to a place and holds it.
   *
   * <p>In 2026 this covered the hood and the intake pivot. Four things differ from velocity control
   * and all of them matter:
   *
   * <ol>
   *   <li><b>An absolute encoder.</b> A position mechanism must know where it is at boot without
   *       first being driven into a limit switch. That means a CANcoder, fused with the rotor so
   *       multi-turn travel still tracks — plain {@code RemoteCANcoder} wraps to zero every encoder
   *       turn.
   *   <li><b>Two ratios, not one</b>, split at the encoder.
   *   <li><b>Soft limits enabled.</b> A roller cannot travel too far. A pivot can, and will, into a
   *       hard stop at full current.
   *   <li><b>Brake, not coast.</b> A coasting arm falls.
   * </ol>
   *
   * <p>The IO must also configure the CANcoder <b>before</b> applying this, and must register the
   * encoder's Position and Velocity signals at 50 Hz before {@code optimizeBusUtilization()} — see
   * {@code ExampleSubsystemIOReal.configureEncoder()} for that trap.
   *
   * @return a fully-built config, ready for {@code PhoenixUtil.tryUntilOk(5, ...)}
   */
  public static TalonFXConfiguration getPositionFXConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // ---- Current limits ----
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME.in(Seconds);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;

    // ---- Torque current clamp ----
    if (isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_FORWARD_TORQUE_CURRENT_AMPS;
      config.TorqueCurrent.PeakReverseTorqueCurrent = PEAK_REVERSE_TORQUE_CURRENT_AMPS;
    }

    // ---- Voltage clamp ----
    config.Voltage.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
    config.Voltage.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;

    // ---- Motor output ----
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // a coasting arm falls
    config.MotorOutput.Inverted =
        INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // ---- Soft limits ----
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = FORWARD_SOFT_LIMIT_ENABLED;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = REVERSE_SOFT_LIMIT_ENABLED;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT_ROTATIONS;

    // ---- Feedback ----
    config.Feedback.FeedbackRemoteSensorID = Constants.CanIds.EXAMPLE_ENCODER;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;
    config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;

    // ---- Gains ----
    config.Slot0.kS = getKS();
    config.Slot0.kG = getKG();
    config.Slot0.kP = getKP();
    config.Slot0.kD = getKD();
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Elevator_Static for a lift
    // Take the sign of kS from the closed-loop error rather than measured velocity. At rest on a
    // setpoint the velocity is ~0 and noisy, so UseVelocitySign flips and the mechanism chatters.
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    // ---- Motion profile ----
    // A raw position request commands maximum effort instantly, which on a mechanism with real
    // inertia means slamming into the setpoint and into the hard stops.
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

    return config;
  }
}
