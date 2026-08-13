package frc.robot.subsystems.example;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.lib.LoggedTunableNumber;
import frc.lib.MotorSpecs;
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
  // GEAR_RATIO is the TOTAL reduction: motor rotations per mechanism rotation, end to end.
  // It is the number that sets top speed, and the one to state when someone asks "what's the
  // ratio". Everything else is derived from it.
  //
  // public static final double GEAR_RATIO = ;
  //
  // Phoenix then needs that total SPLIT AT THE ENCODER, and the split is decided by ONE
  // question: WHERE IS THE ENCODER? Answer that before writing either number.
  //
  //   ROTOR_TO_SENSOR_RATIO      motor rotations per encoder rotation  (rotor/CANcoder fusion)
  //   SENSOR_TO_MECHANISM_RATIO  encoder rotations per mechanism rotation
  //
  // The two must multiply back to GEAR_RATIO. If they do not, either the top speed or the
  // reported position is wrong, and nothing will tell you which.
  //
  // ---- Case 1: motor encoder only, no CANcoder ----
  // The rotor is the sensor, so the whole reduction sits on the far side of it.
  //   ROTOR_TO_SENSOR_RATIO     = 1.0
  //   SENSOR_TO_MECHANISM_RATIO = GEAR_RATIO
  //
  // ---- Case 2: CANcoder on the mechanism itself ----
  // Typically a hex-bore CANcoder on the shaft the mechanism actually turns on. The encoder
  // already reads mechanism rotations, so there is nothing left below it.
  //   ROTOR_TO_SENSOR_RATIO     = GEAR_RATIO
  //   SENSOR_TO_MECHANISM_RATIO = 1.0
  //
  // Cases 1 and 2 cover nearly everything this team builds. Prefer them.
  //
  // ---- Case 3: CANcoder on an intermediate shaft ----
  // There is still gearing BETWEEN the encoder and the mechanism. Split the total where the
  // encoder physically sits.
  //
  // The 2026 hood is the example, and it is worth reading carefully because the trap is in the
  // vocabulary. That code calls the encoder's shaft the "output shaft" — meaning the GEARBOX
  // output, not the mechanism. A 12T lantern gear on that shaft still drives a 122T hood gear
  // after it, so the encoder turns about ten times per hood rotation:
  //
  //   motor → belt → shaft (CANcoder here) → 12T lantern → 122T hood
  //   ROTOR_TO_SENSOR_RATIO     = 5.33            (measured, motor to that shaft)
  //   SENSOR_TO_MECHANISM_RATIO = 122.0 / 12.0    ≈ 10.17
  //   GEAR_RATIO                = 5.33 * 10.17    ≈ 54.2
  //
  // "Output shaft" is ambiguous and has cost time before. The only question that matters is
  // whether the encoder turns 1:1 with the thing you are trying to measure. If anything geared
  // sits after it, you are in case 3.
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
  // 1.0 IS RARELY THE RIGHT ANSWER, despite being the obvious-looking default. It puts the wrap
  // at 0, and 0 is almost always the stow position — the one place the mechanism sits most of
  // the match. The reading then flips between ~0.0 and ~1.0 every time it settles there, and a
  // position loop chases a full rotation of phantom error.
  //
  // 0.5 puts the wrap at half a turn away, which for any mechanism travelling less than 180°
  // from zero is nowhere near its range. Start there for arms and pivots.
  //
  // public static final double MAGNET_OFFSET_ROTATIONS = ;
  // public static final double SENSOR_DISCONTINUITY_POINT = ;

  // ==========================================================================================
  // LINEAR GEOMETRY — delete this block entirely for a rotating mechanism
  // ==========================================================================================
  //
  // Phoenix only ever knows about rotations. Turning those into inches of carriage travel needs
  // two numbers that cannot be derived from the gear ratio, so both have to be measured or read
  // off the CAD.
  //
  // TODO: supply the drum geometry, then uncomment.
  //
  // DRUM_PITCH_DIAMETER is the diameter at which the rope, belt or chain actually rides — the
  // PITCH diameter, not the outer diameter of the spool flange. On a sprocket it is the chain
  // pitch circle; on a pulley it is the belt pitch line. Using the outer diameter makes every
  // height read high by a few percent, which looks like a tuning problem for a long time.
  //
  // STAGE_COUNT is the rigging multiplier, and it is the one that gets missed. A single-stage
  // lift moves one circumference per drum rotation. A 2-stage cascade moves TWICE that, because
  // the stages travel together. Get this wrong and every height is off by an exact integer
  // factor — which is the tell, if you ever see it.
  //
  // public static final Distance DRUM_PITCH_DIAMETER = Inches.of();
  // public static final int STAGE_COUNT = ;
  //
  // Travel per mechanism rotation, once both are known:
  //
  //   public static final Distance TRAVEL_PER_ROTATION =
  //       DRUM_PITCH_DIAMETER.times(Math.PI * STAGE_COUNT);
  //
  //   public static double inchesToRotations(double inches) {
  //     return inches / TRAVEL_PER_ROTATION.in(Inches);
  //   }
  //
  //   public static double rotationsToInches(double rotations) {
  //     return rotations * TRAVEL_PER_ROTATION.in(Inches);
  //   }

  // ==========================================================================================
  // UNIT CONVERSION — the boundary between team units and Phoenix units
  // ==========================================================================================
  //
  // We declare in RPM, RPM/sec, degrees and inches because those are the units a person can
  // reason about with the robot in front of them. Phoenix works in rotations and rotations per
  // second, always.
  //
  // Every conversion lives here so there is exactly one place to check, and each is written with
  // WPILib units rather than a bare /60.0 or /360.0. An inverted magic number is a factor-of-60
  // error that compiles, deploys, and moves the mechanism — just not the way anyone expected.

  /** Mechanism RPM to the rotations/sec Phoenix wants. */
  public static double rpmToRotationsPerSec(double rpm) {
    return RPM.of(rpm).in(RotationsPerSecond);
  }

  /**
   * Mechanism RPM/sec to the rotations/sec² Phoenix wants.
   *
   * <p>Typed as an angular <i>acceleration</i>, not an angular velocity. The numeric factor is the
   * same 60 either way, so writing this with {@code RPM.of(...)} would give the right answer today
   * — and would quietly stop being right the moment someone reuses the helper for something that is
   * not a per-second rate.
   */
  public static double rpmPerSecToRotationsPerSecSquared(double rpmPerSec) {
    return RPM.per(Second).of(rpmPerSec).in(RotationsPerSecondPerSecond);
  }

  /** Mechanism degrees to the rotations Phoenix wants. */
  public static double degreesToRotations(double degrees) {
    return Degrees.of(degrees).in(Rotations);
  }

  /** Phoenix rotations back to degrees, for logging and tolerance checks. */
  public static double rotationsToDegrees(double rotations) {
    return Rotations.of(rotations).in(Degrees);
  }

  // ==========================================================================================
  // THEORETICAL TOP SPEED
  // ==========================================================================================

  /** Which motor drives this mechanism. Picks the free speed and the sim model together. */
  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** How many motors, leader and followers together. Torque scales with this; speed does not. */
  public static final int MOTOR_COUNT = 1;

  /**
   * Theoretical top speed at the mechanism, in RPM. Computed, never typed in.
   *
   * <p>This is a ceiling, not a target. Free speed is the motor with nothing attached; load,
   * friction and the current limit all take a share. A cruise velocity above this does not fail
   * loudly — the motor simply saturates and the profile stops being followed, which looks like
   * sloppy tuning rather than an impossible request.
   *
   * <p>It is also the sanity check on the gear ratio. If this number is nowhere near what the
   * mechanism has to do, the ratio is wrong, and catching that here beats catching it on the
   * practice field.
   *
   * <p>Uses {@link #GEAR_RATIO}, the total reduction — <b>not</b> {@code
   * SENSOR_TO_MECHANISM_RATIO}. On a mechanism with a fused CANcoder those are different numbers,
   * and using the split half would overstate top speed by exactly the other half.
   */
  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(GEAR_RATIO);

  // ==========================================================================================
  // MOTION PROFILE
  // ==========================================================================================
  //
  // UNITS: RPM and RPM per second, throughout. Phoenix works in rotations and rotations per
  // second, so getFXConfig() converts at the boundary — that division by 60 lives in exactly
  // one place, and it is written with WPILib units rather than a bare /60.0 so it cannot be
  // inverted by accident.
  //
  // Both are tunable. A profile is the thing you most want to adjust with the mechanism in
  // front of you, and it is much safer to change than a gain: too slow just wastes time,
  // whereas too much kP oscillates. See docs/tunables.md.
  //
  // TODO: decide which kind of mechanism this is, then uncomment ONE block below.
  //
  // These are the only defaults in this file where the WRONG answer would still compile, which
  // is why they are commented out like the values that cannot be guessed. Left with a fixed
  // default, an arm copied from this scaffold silently inherits the lift's profile: 30x the
  // acceleration it should have, aimed at a hard stop.
  //
  // ---- LINEAR position: elevator, lift, extension ----
  // Half of theoretical leaves headroom for load, which a lift needs because it fights gravity
  // the whole way up. 9000 RPM/s is close to unlimited on purpose — motion ends up bounded by
  // cruise velocity and the current limit rather than by acceleration.
  //
  //   public static final double CRUISE_VELOCITY_RPM_DEFAULT = MAX_SPEED_RPM / 2.0;
  //   public static final double ACCELERATION_RPM_PER_SEC_DEFAULT = 9000.0;
  //
  // ---- ROTATION position: arm, pivot, hood, turret ----
  // A flat 60 RPM regardless of what the gearing could do, and 300 RPM/s. Both are deliberately
  // slow starting points, meant to be raised on purpose rather than lowered after something
  // breaks: an arm is where an over-fast profile does mechanical damage, and gravity torque
  // changes with angle in a way an aggressive profile will not respect.
  //
  //   public static final double CRUISE_VELOCITY_RPM_DEFAULT = 60.0;
  //   public static final double ACCELERATION_RPM_PER_SEC_DEFAULT = 300.0;
  //
  // ---- VELOCITY: roller, flywheel, feeder ----
  // No cruise velocity — the commanded velocity IS the cruise. Delete CRUISE_VELOCITY_RPM
  // below, and drop it from TUNABLE_PROFILE.
  //
  //   public static final double ACCELERATION_RPM_PER_SEC_DEFAULT = 9000.0;

  /**
   * Profile acceleration, in mechanism RPM per second. Tunable — the value above is only the
   * compiled-in default.
   */
  public static final LoggedTunableNumber ACCELERATION_RPM_PER_SEC =
      new LoggedTunableNumber("Example/AccelRpmPerSec", ACCELERATION_RPM_PER_SEC_DEFAULT);

  /**
   * Profile cruise velocity, in mechanism RPM. Tunable — the value above is only the compiled-in
   * default.
   *
   * <p>Position control only. Delete this for a velocity mechanism, where the setpoint is the
   * cruise.
   */
  public static final LoggedTunableNumber CRUISE_VELOCITY_RPM =
      new LoggedTunableNumber("Example/CruiseVelocityRpm", CRUISE_VELOCITY_RPM_DEFAULT);

  /** Everything re-applied to the Talon when a dashboard value moves. */
  public static final LoggedTunableNumber[] TUNABLE_PROFILE = {
    ACCELERATION_RPM_PER_SEC, CRUISE_VELOCITY_RPM
  };

  // ==========================================================================================
  // TOLERANCES
  // ==========================================================================================

  /**
   * How close counts as "at velocity", in mechanism RPM.
   *
   * <p>Too tight and the mechanism never reports ready, so every sequence runs to its timeout
   * instead of proceeding. Too loose and it acts before it has arrived.
   */
  public static final double VELOCITY_TOLERANCE_RPM = 120.0;

  /** How close counts as "at position", in degrees. */
  public static final double POSITION_TOLERANCE_DEGREES = 1.0;

  /**
   * Where horizontal sits, in mechanism degrees. Rotating mechanisms with gravity load only.
   *
   * <p>Used as the {@code Arm_Cosine} reference — see {@link #getPositionFXConfig()}. Zero is
   * correct only when the mechanism's zero position is genuinely level. Delete this for an elevator
   * or for anything gravity does not act on.
   */
  public static final double GRAVITY_HORIZONTAL_OFFSET_DEGREES = 0.0;

  /** How close counts as "at position" for a linear mechanism, in inches. */
  public static final double POSITION_TOLERANCE_INCHES = 0.25;

  // ==========================================================================================
  // VISUALIZATION — display only. Delete with ExampleSubsystemVisualizer if unused.
  // ==========================================================================================

  /**
   * Geometry for {@link ExampleSubsystemVisualizer}.
   *
   * <p><b>None of this affects the robot.</b> Every value here changes a picture and nothing else,
   * which makes it the one block in this file you can safely guess at and adjust by eye while
   * watching AdvantageScope. Do that rather than deferring it — a visualizer nobody bothered to
   * scale is one nobody looks at.
   *
   * <p>The canvas is in meters because {@code LoggedMechanism2d} is, but it is a drawing, not the
   * robot. Only the {@code PIVOT_*} offsets describe real space, and those come off the CAD.
   */
  public static class Visualization {

    private Visualization() {}

    /**
     * Master switch. Publishing a Mechanism2d to NetworkTables every loop is not free, and the 2026
     * robot ran closer to 30 Hz than 50 Hz all season.
     *
     * <p>Leave it on through bring-up, where it is the whole point. Before competition, check
     * {@code LoopTiming/} with it enabled and turn it off if the budget is tight — see {@code
     * .claude/rules/00-safety.md}.
     */
    public static final boolean ENABLED = true;

    /** Canvas size. Cosmetic — pick something the mechanism fits inside with room to spare. */
    public static final double CANVAS_WIDTH_METERS = 1.0;

    public static final double CANVAS_HEIGHT_METERS = 1.0;

    /** Where the fixed point sits on the canvas: the pivot for an arm, the base for a lift. */
    public static final double ROOT_X_METERS = 0.5;

    public static final double ROOT_Y_METERS = 0.2;

    /** Drawn length of a rotating mechanism's arm. Cosmetic; scale it to fill the canvas. */
    public static final double LENGTH_METERS = 0.4;

    /**
     * Linear mechanisms only — drawn length at zero travel.
     *
     * <p>A ligament of zero length is not drawn at all, so without this a fully retracted carriage
     * silently disappears and reads as a broken visualizer rather than a retracted mechanism.
     */
    public static final double MIN_LENGTH_METERS = 0.05;

    /**
     * Linear mechanisms only — canvas meters drawn per inch of real travel.
     *
     * <p>Pure display scale, unrelated to {@code TRAVEL_PER_ROTATION}. Choose it so full travel
     * roughly fills the canvas: for 24 in of travel on a 1 m canvas, about 0.035.
     */
    public static final double INCHES_TO_CANVAS_METERS = 0.035;

    // ----------------------------------------------------------------------------------------
    // 3D component pose — the only real-space values here
    // ----------------------------------------------------------------------------------------
    //
    // Robot frame: X forward, Y left, Z up, origin at the robot's center on the floor.
    //
    // AdvantageScope rotates a component mesh ABOUT the pose it is given, so this has to be the
    // mechanism's actual axis of rotation — the pivot shaft, not the arm tip. Measure it in CAD.
    // Both default to the robot origin, which draws the component in the middle of the robot and
    // is obviously wrong on sight, which is the intent.

    /** Pivot axis (rotating) or retracted carriage position (linear), in the robot frame. */
    public static final Translation3d PIVOT_OFFSET = Translation3d.kZero;

    /** Fixed mounting rotation of the component, before the mechanism's own motion is added. */
    public static final Rotation3d PIVOT_ROTATION = Rotation3d.kZero;

    /**
     * Which way the mesh turns as the measured angle increases. Rotating mechanisms only.
     *
     * <p>Depends on which way the mechanism faces in the robot frame. Guessing is fine — flip the
     * sign while watching AdvantageScope. It is cosmetic and cannot affect the robot.
     */
    public static final double PITCH_SIGN = 1.0;
  }

  // ==========================================================================================
  // JAM DETECTION — mechanisms that can stall against a game piece. Delete otherwise.
  // See the JAM DETECTION block in ExampleSubsystem for the code these feed.
  // ==========================================================================================
  //
  // NONE OF THESE ARE GUESSABLE. They come from logging stator current during a real jam AND
  // during a normal pickup, because telling those two apart is the entire job.
  //
  //   /** Stator current, in amps, above which the mechanism counts as "working hard".
  //    * Sits BELOW STATOR_CURRENT_LIMIT_AMPS — a jam that already saturated the limit has
  //    * been a jam for a while. */
  //   public static final double JAM_STATOR_CURRENT_AMPS = 55.0;
  //
  //   /** Measured velocity below this FRACTION of commanded counts as "not turning". A
  //    * fraction, not an absolute RPM, so one threshold works at every setpoint. */
  //   public static final double JAM_VELOCITY_FRACTION = 0.25;
  //
  //   /** How long all conditions must hold. MUST EXCEED SPIN-UP TIME — see the note in
  //    * ExampleSubsystem. Raise it if ACCELERATION_RPM_PER_SEC ever drops. */
  //   public static final double JAM_DEBOUNCE_SECONDS = 0.5;
  //
  //   /** Commanded RPM below which the check is skipped, so a stopped mechanism does not
  //    * trivially satisfy "measured is below 25% of commanded". */
  //   public static final double JAM_MIN_COMMANDED_RPM = 100.0;

  // ==========================================================================================
  // ZEROING — relative encoders only. Delete this whole block if the mechanism has an absolute
  // encoder that fits within one turn. See ExampleCommands' commented ZEROING block for how
  // these get used, and docs/new-mechanism-bringup.md Phase 1 step 6 for why each one matters.
  // ==========================================================================================
  //
  // None of these are guessable — pick them with the mechanism in front of you. Two public
  // reference points, both real robots, both keying off velocity rather than current alone:
  // Team 5419's 2024 elevator creeps at -30% duty cycle (~3.6 V at a nominal 12 V bus) and
  // confirms the stop after velocity stays under ~0.02 rot/s for 0.08 s. Team 6328's
  // GenericSlamElevator uses the same velocity-near-zero-for-a-dwell idea.
  //
  //   /** Downward creep voltage while seeking the hard stop. Negative = down. */
  //   public static final Voltage ZEROING_VOLTAGE = Volts.of(-3.0);
  //
  //   /** Supply current, in amps, that counts as "hit the hard stop" while creeping. */
  //   public static final double ZEROING_STALL_CURRENT_AMPS = 25.0;
  //
  //   /** Mechanism velocity, in RPM, below which it counts as "not moving" while creeping. */
  //   public static final double ZEROING_VELOCITY_THRESHOLD_RPM = 5.0;
  //
  //   /** How long BOTH conditions must hold before counting as a real stall, not a momentary
  //    * current spike from static friction breakaway. */
  //   public static final double ZEROING_STALL_DEBOUNCE_SECONDS = 0.1;
  //
  //   /** Give up and refuse to zero if the stop is never found within this budget. */
  //   public static final double ZEROING_TIMEOUT_SECONDS = 3.0;

  // ==========================================================================================
  // GAINS — REAL ROBOT
  // ==========================================================================================
  //
  // Placeholders. Measure them: kS -> kV -> kA -> kP for velocity, kG -> kS -> kP -> kD for
  // position. Never start with kP. See docs/characterization-and-tuning.md Part 3.
  //
  // These are LoggedTunableNumbers so a tuning session does not need a redeploy per iteration.
  // With Constants.tuningMode off they return the defaults below and cost nothing — no
  // dashboard entry is even created.
  //
  // The default in each constructor is what the robot runs in competition. The dashboard value
  // lives only in NetworkTables and is gone at the next reboot, so a session that ends without
  // writing the numbers back into this file and committing them accomplished nothing.
  //
  // These are the gains the TALON runs, on the device. Changing one means a CAN write, which is
  // why ExampleSubsystemIOReal re-applies Slot0 only when a value actually moves rather than
  // every loop. See docs/tunables.md.

  /** Static friction feedforward — output needed to break the mechanism loose. */
  public static final LoggedTunableNumber KS = new LoggedTunableNumber("Example/kS", 0.0);

  /** Velocity feedforward — output per unit of steady-state velocity. */
  public static final LoggedTunableNumber KV = new LoggedTunableNumber("Example/kV", 0.0);

  /** Acceleration feedforward — output per unit of acceleration. */
  public static final LoggedTunableNumber KA = new LoggedTunableNumber("Example/kA", 0.0);

  /** Gravity feedforward. Arms and elevators only; set the matching {@link GravityTypeValue}. */
  public static final LoggedTunableNumber KG = new LoggedTunableNumber("Example/kG", 0.0);

  /** Proportional gain. Corrects what feedforward misses — it should not be doing the work. */
  public static final LoggedTunableNumber KP = new LoggedTunableNumber("Example/kP", 0.0);

  /** Integral gain. Leave at zero unless you can explain its behavior during a stall. */
  public static final LoggedTunableNumber KI = new LoggedTunableNumber("Example/kI", 0.0);

  /** Derivative gain. Damps overshoot on position loops; amplifies noise on velocity loops. */
  public static final LoggedTunableNumber KD = new LoggedTunableNumber("Example/kD", 0.0);

  /** Every real-robot gain, for the {@code ifChanged} watch list in the IO layer. */
  public static final LoggedTunableNumber[] TUNABLE_GAINS = {KS, KV, KA, KG, KP, KI, KD};

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
   *
   * <p><b>Sim gains are plain doubles, not tunables, on purpose.</b> Tunables exist to avoid the
   * two-minute edit-build-deploy-enable cycle. Sim has no deploy step — restarting it is seconds —
   * so a dashboard knob buys nothing and only adds a second set of numbers that can disagree with
   * the file. Edit these here and re-run.
   */
  public static class Sim {

    private Sim() {}

    /** The motor(s) driving this mechanism, for the sim model. */
    public static final DCMotor MOTOR = ExampleSubsystemConstants.MOTOR.gearbox(MOTOR_COUNT);

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
    return isSim() ? Sim.KS : KS.get();
  }

  public static double getKV() {
    return isSim() ? Sim.KV : KV.get();
  }

  public static double getKA() {
    return isSim() ? Sim.KA : KA.get();
  }

  public static double getKG() {
    return isSim() ? Sim.KG : KG.get();
  }

  public static double getKP() {
    return isSim() ? Sim.KP : KP.get();
  }

  public static double getKI() {
    return isSim() ? Sim.KI : KI.get();
  }

  public static double getKD() {
    return isSim() ? Sim.KD : KD.get();
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
    // RPM/sec in, rotations/sec^2 out. A velocity profile has no cruise velocity — the
    // commanded velocity IS the cruise.
    config.MotionMagic.MotionMagicAcceleration =
        rpmPerSecToRotationsPerSecSquared(ACCELERATION_RPM_PER_SEC.get());

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
    // Connects the motor to the CANcoder as a remote, fused sensor — the motor fuses the
    // CANcoder's absolute position with its own rotor so multi-turn tracking still works.
    config.Feedback.withFeedbackRemoteSensorID(Constants.CanIds.EXAMPLE_ENCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO)
        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO);

    // ---- Gains ----
    config.Slot0.kS = getKS();
    config.Slot0.kG = getKG();
    config.Slot0.kP = getKP();
    config.Slot0.kD = getKD();
    // Arm_Cosine for anything that pivots, Elevator_Static for a lift. The difference is real:
    // gravity torque on an arm varies with the cosine of its angle, peaking horizontal and
    // vanishing vertical, while an elevator fights the same weight everywhere.
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Elevator_Static for a lift

    // WHERE IS HORIZONTAL? Arm_Cosine scales kG by cos(position + offset), and it assumes the
    // peak lands at a cosine argument of zero. If the mechanism's zero is its STOW position
    // rather than horizontal — which is the usual way to define zero — the compensation peaks
    // in the wrong place: too little hold current where gravity is strongest, too much where
    // there is none.
    //
    // Set this to the negative of the mechanism angle at which the arm is level. If the pivot
    // reads 30 degrees when horizontal, the offset is -30 degrees. Leave it at zero ONLY if
    // zero is genuinely horizontal.
    config.Slot0.GravityArmPositionOffset = degreesToRotations(GRAVITY_HORIZONTAL_OFFSET_DEGREES);
    // Take the sign of kS from the closed-loop error rather than measured velocity. At rest on a
    // setpoint the velocity is ~0 and noisy, so UseVelocitySign flips and the mechanism chatters.
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    // ---- Motion profile ----
    // A raw position request commands maximum effort instantly, which on a mechanism with real
    // inertia means slamming into the setpoint and into the hard stops.
    // RPM and RPM/sec in, rotations/sec and rotations/sec^2 out.
    config.MotionMagic.MotionMagicAcceleration =
        rpmPerSecToRotationsPerSecSquared(ACCELERATION_RPM_PER_SEC.get());
    config.MotionMagic.MotionMagicCruiseVelocity = rpmToRotationsPerSec(CRUISE_VELOCITY_RPM.get());

    return config;
  }
}
