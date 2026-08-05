package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

/**
 * Constants for ExampleSubsystem.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Rename "ExampleSubsystem" → your subsystem name 2. Delete the
 * constants this mechanism does not have — an open-loop roller needs no PID block 3. Fill in real
 * measured values; every number below is a placeholder 4. One constants file per subsystem, living
 * beside the subsystem it belongs to
 *
 * <p><b>WHAT DOES NOT GO HERE:</b>
 *
 * <ul>
 *   <li><b>CAN IDs.</b> Those live in {@code Constants.CanIds} so every ID on the robot is visible
 *       in one table. See {@code .claude/rules/02-hardware.md}.
 *   <li><b>Setpoints, timeouts and tolerances.</b> Those go in {@code Constants} — the rule is that
 *       anything you would change in the pit between matches lives in one file, so nobody has to
 *       remember which of several places it was.
 * </ul>
 *
 * <p>What belongs here instead is everything that describes how this mechanism is <i>built</i> or
 * <i>characterized</i>: gains, current limits, gear ratio, sim model parameters, and any
 * interpolation table. A distance-to-RPM map is structure rather than a knob, and putting it in
 * {@code Constants} would bury the values people actually reach for.
 *
 * <p><b>UNITS:</b> if this mechanism uses any {@code *TorqueCurrentFOC} control request — and the
 * drivetrain in this template does — then {@link #KS}, {@link #KV}, {@link #KA} and {@link #KP} are
 * in <b>AMPS</b>, not volts. Values that look absurd as volts are often correct as amps. See {@code
 * docs/characterization-and-tuning.md} before touching any of them.
 *
 * <p><b>TUNING:</b> to adjust a gain live instead of redeploying, wrap it in a {@code
 * LoggedTunableNumber}. It reads from the dashboard when {@code Constants.tuningMode} is on and
 * returns the value here when it is off:
 *
 * <pre>
 * private static final LoggedTunableNumber kP = new LoggedTunableNumber("Example/kP", KP);
 * </pre>
 *
 * Whatever a tuning session lands on, write it back into this file and commit it — dashboard values
 * live in NetworkTables and are gone at the next reboot.
 */
public class ExampleSubsystemConstants {

  // ==========================================================================================
  // CURRENT LIMITS
  // ==========================================================================================
  //
  // These are safety limits, not tuning knobs (.claude/rules/00-safety.md). Too high risks a
  // brownout or cooked windings; too low and the mechanism cannot do its job. Raise one only
  // after you can say why the old value was insufficient.
  //
  // SUPPLY vs STATOR are different measurements and are not interchangeable:
  //   Supply — current drawn from the battery. Bounds brownout risk.
  //   Stator — current through the windings. Bounds heating and torque.
  // At low speed under load, supply is a fraction of stator. Mixing them up is exactly the bug
  // that made every 2026 drivetrain power figure wrong.

  /** Steady-state supply current ceiling, in amps. */
  public static final int SUPPLY_CURRENT_LIMIT_AMPS = 40;

  /**
   * Supply current that must be exceeded for {@link #SUPPLY_CURRENT_TRIGGER_TIME} before the limit
   * engages. Lets a mechanism draw a brief inrush spike without being clamped.
   */
  public static final int SUPPLY_CURRENT_TRIGGER_AMPS = 35;

  /** How long {@link #SUPPLY_CURRENT_TRIGGER_AMPS} must be exceeded before limiting. */
  public static final Time SUPPLY_CURRENT_TRIGGER_TIME = Seconds.of(1.0);

  /** Winding current ceiling, in amps. This is the one that governs heating and stall. */
  public static final int STATOR_CURRENT_LIMIT_AMPS = 100;

  // ==========================================================================================
  // MECHANICAL
  // ==========================================================================================

  /**
   * Motor rotations per mechanism rotation. <b>Velocity control only.</b>
   *
   * <p>Set this as {@code FeedbackConfigs.SensorToMechanismRatio} in the IO so the encoder reports
   * in mechanism units. Then every setpoint, gain and logged velocity in this file is in terms of
   * the thing that actually moves, rather than the motor shaft — which is the difference between
   * numbers a human can sanity-check and numbers nobody can.
   *
   * <p>Position mechanisms with an absolute encoder split this into two ratios instead — see {@link
   * #ROTOR_TO_SENSOR_RATIO} and {@link #SENSOR_TO_MECHANISM_RATIO}.
   */
  public static final double GEAR_RATIO = 24.0 / 11.0;

  // ---- Position control with a fused CANcoder -------------------------------------------------
  //
  // When an absolute encoder sits partway down the gear train, the ratio has to be split at the
  // encoder. Phoenix needs both halves: one to fuse rotor counts with the CANcoder, one to report
  // position in mechanism units.
  //
  // Worked example from the 2026 hood, whose chain was
  //   motor → 30T belt → 20T shaft → CANcoder → 12T lantern → 122T hood
  //   ROTOR_TO_SENSOR_RATIO    = 30/20  = 1.5     (motor turns per CANcoder turn)
  //   SENSOR_TO_MECHANISM_RATIO = 122/12 ≈ 10.17  (CANcoder turns per hood turn)

  /** Motor rotations per absolute-encoder rotation. Used for rotor/CANcoder fusion. */
  public static final double ROTOR_TO_SENSOR_RATIO = 1.5;

  /** Absolute-encoder rotations per mechanism rotation. Makes position read in mechanism units. */
  public static final double SENSOR_TO_MECHANISM_RATIO = 122.0 / 12.0;

  /**
   * CANcoder magnet offset, in rotations.
   *
   * <p>Calibration data, like the swerve encoder offsets — found by physically moving the mechanism
   * to a known position and reading the raw sensor. Not guessable, and it changes if the encoder or
   * magnet is ever removed.
   */
  public static final double MAGNET_OFFSET_ROTATIONS = 0.0;

  /**
   * Where the absolute sensor wraps, in rotations.
   *
   * <p>1.0 gives a [0, 1) range; 0.5 gives [-0.5, 0.5). Choose so the discontinuity sits somewhere
   * the mechanism never travels — a wrap in the middle of the range makes position jump a full
   * rotation mid-motion.
   */
  public static final double SENSOR_DISCONTINUITY_POINT = 1.0;

  /** Upper travel limit, in mechanism rotations. Enforced by the motor controller. */
  public static final double FORWARD_SOFT_LIMIT_ROTATIONS = 0.25;

  /** Lower travel limit, in mechanism rotations. */
  public static final double REVERSE_SOFT_LIMIT_ROTATIONS = 0.0;

  /** True if the motor is mounted so positive output produces negative mechanism motion. */
  public static final boolean INVERTED = false;

  /**
   * How close to the commanded velocity counts as "at velocity", in mechanism rotations/sec.
   *
   * <p>Too tight and the mechanism never reports ready, so every sequence runs to its timeout
   * instead of proceeding. Too loose and it fires early and misses. This is the kind of value that
   * gets adjusted between matches — consider wrapping it in a {@code LoggedTunableNumber}.
   */
  public static final double VELOCITY_TOLERANCE_ROTATIONS_PER_SEC = 2.0;

  /**
   * How close to the commanded position counts as "at position", in mechanism rotations.
   *
   * <p>Same trade as the velocity tolerance: too tight and the mechanism never reports ready so
   * every sequence runs to its timeout, too loose and it acts before it has arrived. For an angular
   * mechanism it is usually easier to reason about in degrees — 1° is 1/360 rotation.
   */
  public static final double POSITION_TOLERANCE_ROTATIONS = 1.0 / 360.0;

  // ==========================================================================================
  // MOTION PROFILE (MotionMagic)
  // ==========================================================================================
  //
  // Only needed for position control with real inertia or travel. A raw position request
  // commands maximum effort instantly; a profile ramps it. Cruise velocity and acceleration are
  // Hard Stop items — too aggressive damages mechanisms.

  /** Profile acceleration, in mechanism rotations per second squared. */
  public static final double MOTION_MAGIC_ACCELERATION = 100.0;

  /** Profile cruise velocity, in mechanism rotations per second. 0 means "as fast as possible". */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;

  // ==========================================================================================
  // GAINS — REAL ROBOT
  // ==========================================================================================
  //
  // Measure these, do not guess them. Order is kS -> kV -> kA -> kP for velocity control, and
  // kG -> kS -> kP -> kD for position control. Never start with kP.
  // See docs/characterization-and-tuning.md Part 3.

  /** Static friction feedforward — output needed to break the mechanism loose. */
  public static final double KS = 1.5;

  /** Velocity feedforward — output per unit of steady-state velocity. */
  public static final double KV = 0.0;

  /** Acceleration feedforward — output per unit of acceleration. */
  public static final double KA = 0.0;

  /** Gravity feedforward. Only for arms and elevators; set the matching {@code GravityType}. */
  public static final double KG = 0.0;

  /** Proportional gain. Corrects what feedforward misses — it should not be doing the work. */
  public static final double KP = 0.0;

  /**
   * Integral gain. Leave at zero unless you can explain what it does during a stall — integral
   * windup on a jammed mechanism dumps the accumulated output the instant the jam clears.
   */
  public static final double KI = 0.0;

  /** Derivative gain. Damps overshoot on position loops; amplifies noise on velocity loops. */
  public static final double KD = 0.0;

  // ==========================================================================================
  // SIMULATION
  // ==========================================================================================

  /**
   * Simulation constants, deliberately kept nested and deliberately separate from the real gains
   * above.
   *
   * <p><b>Why sim needs its own gains.</b> The sim model is not the robot. It has no backlash, no
   * belt stretch, no friction beyond what the model states, and an inertia figure that is usually a
   * guess. Gains that behave well against that model are frequently wrong on hardware, and gains
   * measured on hardware often will not even converge in sim. Forcing them to share one set means
   * one of the two is always lying to you.
   *
   * <p>Keeping them in a nested block rather than flattened is the point: {@code Sim.KP} next to
   * {@code KP} in a diff makes it obvious which one you are editing.
   */
  public static class Sim {

    /** The motor(s) driving this mechanism, for the sim model. */
    public static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

    /** How many motors drive it. Must agree with {@link #MOTOR}. */
    public static final int NUM_MOTORS = 1;

    /**
     * Moment of inertia in kg·m².
     *
     * <p>Usually an estimate. It sets how fast the simulated mechanism accelerates, so a wrong
     * value makes sim-tuned gains meaningless — worth a CAD number rather than a guess if the
     * mechanism matters.
     */
    public static final double MOI = 0.001;

    // Sim-specific gains. Same meanings as the real ones above.
    public static final double KS = 2.0;
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
  // Use these in the subsystem and IO rather than reading KP / Sim.KP directly. The caller then
  // does not care which mode it is in, and there is exactly one place that decides.
  //
  // Note REPLAY resolves to the REAL gains, which is correct: replay re-runs recorded hardware
  // inputs through the code, so it should behave like the real robot.

  private static boolean isSim() {
    return Constants.currentMode == Constants.Mode.SIM;
  }

  /** Static friction feedforward for the current mode. */
  public static double getKS() {
    return isSim() ? Sim.KS : KS;
  }

  /** Velocity feedforward for the current mode. */
  public static double getKV() {
    return isSim() ? Sim.KV : KV;
  }

  /** Acceleration feedforward for the current mode. */
  public static double getKA() {
    return isSim() ? Sim.KA : KA;
  }

  /** Gravity feedforward for the current mode. */
  public static double getKG() {
    return isSim() ? Sim.KG : KG;
  }

  /** Proportional gain for the current mode. */
  public static double getKP() {
    return isSim() ? Sim.KP : KP;
  }

  /** Integral gain for the current mode. */
  public static double getKI() {
    return isSim() ? Sim.KI : KI;
  }

  /** Derivative gain for the current mode. */
  public static double getKD() {
    return isSim() ? Sim.KD : KD;
  }
}
