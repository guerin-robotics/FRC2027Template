package frc.robot.subsystems.exampleLift;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.LoggedTunableNumber;
import frc.lib.MotorSpecs;
import frc.robot.Constants;

/**
 * Constants and motor configuration for a <b>LINEAR POSITION-CONTROLLED</b> mechanism.
 *
 * <p>Copy this scaffold for an elevator, lift, extension or telescoping stage — anything that
 * travels in a straight line to a height and holds it.
 *
 * <p><b>Wrong scaffold?</b> {@code exampleRoller} for anything that spins and only has a speed;
 * {@code exampleArm} for anything that pivots to an angle.
 *
 * <h2>Why this scaffold has no CANcoder</h2>
 *
 * <p>{@code exampleArm} uses a fused CANcoder so it knows where it is at boot. This one uses the
 * <b>motor encoder plus a zeroing routine against a hard stop</b>, and that difference is the whole
 * reason the two are separate files.
 *
 * <p>An absolute reading repeats every sensor turn, so a mechanism whose sensor moves further than
 * one rotation cannot say which turn it is on. An arm travelling 90 degrees is fine. A 24 in
 * elevator on a 2 in drum travels 3.82 rotations, so a reading of 0.5 could be 3.1, 9.4, 15.7 or
 * 22.0 inches — four plausible heights, and nothing to choose between them.
 *
 * <p><b>The ambiguity never shows up in sim, because sim always starts at zero.</b> It shows up as
 * a mechanism that boots to a confidently wrong height, once, on the practice field.
 *
 * <p>If your lift genuinely does travel under one sensor rotation — a short extension, or a sensor
 * geared down for the purpose — copy {@code exampleArm}'s encoder block instead and delete the
 * zeroing routine. Check the arithmetic before deciding; do not assume.
 *
 * <h2>THIS FILE DOES NOT COMPILE ON PURPOSE</h2>
 *
 * <p>Five values cannot be guessed, so their <i>declarations</i> are commented out while the code
 * below still <i>assigns</i> them. The compiler then names each one as an error:
 *
 * <pre>
 * Constants.CanIds.EXAMPLE_LIFT_MOTOR   CAN ID
 * GEAR_RATIO                            total reduction, motor to drum
 * DRUM_PITCH_DIAMETER                   where the rope/belt/chain actually rides
 * STAGE_COUNT                           rigging multiplier
 * MAX_TRAVEL_INCHES                     travel bound
 * </pre>
 *
 * <p>Expect exactly those five and nothing else. Anything more is rot in the scaffold.
 *
 * <p><b>Never guess the drum diameter or the stage count.</b> Both silently scale every height the
 * mechanism reports, so wrong values produce a subsystem that compiles, runs, logs plausible
 * numbers, and is wrong everywhere. That is why they are here rather than defaulted.
 *
 * <h2>What does NOT go in this file</h2>
 *
 * <p><b>Setpoints.</b> This file describes how the mechanism is <i>built</i> — gains, geometry,
 * current limits, soft limits, tolerance, the sim model. The heights it is <i>commanded to</i> —
 * stow, L2, L3 — belong in {@code Constants.Setpoints}. The test is whether a driver might ask you
 * to change it between matches.
 *
 * <p><b>CAN IDs.</b> {@code Constants.CanIds}, so every ID on the robot is visible in one table.
 *
 * <p><b>UNITS:</b> positions in <b>inches</b>, velocities in <b>RPM</b>, and under {@code
 * *TorqueCurrentFOC} the gains are in <b>AMPS</b>, not volts. See {@code
 * docs/characterization-and-tuning.md}.
 */
public class ExampleLiftConstants {

  private ExampleLiftConstants() {}

  /** Log key and alert prefix. One name used everywhere, so a grep finds all of it. */
  public static final String NAME = "ExampleLift";

  // ==========================================================================================
  // CURRENT AND OUTPUT LIMITS — the defaults here are deliberate, not arbitrary
  // ==========================================================================================
  //
  // Safety limits, not tuning knobs (.claude/rules/00-safety.md). Conservative starting points for
  // a Kraken on a mechanism whose loading is not yet known.
  //
  //   Supply — current drawn from the battery. Bounds brownout risk.
  //   Stator — current through the windings. Bounds heating and torque.
  //
  // A lift holding at height is a sustained stall against gravity, which is the worst thermal case
  // this robot has. Watch the temperature channel during bring-up before raising either limit.

  /** Steady-state supply current ceiling, in amps. */
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  /** Supply current allowed briefly before the limit engages. */
  public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 40.0;

  /** How long the lower limit may be exceeded — lets the mechanism draw inrush without clamping. */
  public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.1);

  /**
   * Winding current ceiling, in amps. Governs heating and stall torque.
   *
   * <p><b>40 A is a deliberately weak starting point, not a measurement.</b> A position mechanism
   * commanded somewhere it cannot reach drives into its own hard stop with everything the limit
   * allows, and keeps doing it for as long as the setpoint stands. Starting low means the first
   * wrong setpoint pushes instead of slams — the difference between a bent bracket and a rebuilt
   * mechanism. {@code exampleRoller} starts at 80 because a velocity mechanism has no hard stop to
   * find.
   *
   * <p>TODO: raise this once the mechanism has moved under load and a log shows what it actually
   * draws. A lift lifting its carriage frequently needs more than 40 A, and a two-motor lift splits
   * the draw between them. Too little current does not fail loudly: the profile saturates and the
   * mechanism never arrives, which reads as a tuning problem and is not one. Diagnose it from the
   * closed-loop reference channel — reference tracking the goal with measured lagging behind is the
   * signature — not by feel.
   *
   * <p>Raise this and {@link #PEAK_FORWARD_TORQUE_CURRENT_AMPS} together; see the note there.
   */
  public static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;

  /**
   * Peak torque current the closed loop may REQUEST, in amps.
   *
   * <p>Distinct from the stator limit, and both are needed. Under {@code TorqueCurrentFOC} the
   * control output is itself a current request, so without this clamp the loop can ask for
   * arbitrary current and only the stator limit stops it — after the fact, by saturating.
   *
   * <p><b>Keep this at or below {@link #STATOR_CURRENT_LIMIT_AMPS}.</b> A request clamp set above
   * the stator limit is not a clamp at all: the loop asks for current the device will refuse, which
   * is precisely the after-the-fact saturation this field exists to prevent. The two move together
   * — raising the stator limit without raising this one leaves the mechanism just as weak, and
   * raising this one alone does nothing but re-introduce the saturation.
   */
  public static final double PEAK_FORWARD_TORQUE_CURRENT_AMPS = 40.0;

  /** Peak reverse torque current. Negative, and the mirror of the forward clamp. */
  public static final double PEAK_REVERSE_TORQUE_CURRENT_AMPS = -40.0;

  /** Peak forward voltage. 12 V is nominal battery, so this is "no artificial cap". */
  public static final double PEAK_FORWARD_VOLTAGE = 12.0;

  /** Peak reverse voltage. Negative. */
  public static final double PEAK_REVERSE_VOLTAGE = -12.0;

  // ==========================================================================================
  // MECHANICAL — none of this can be guessed
  // ==========================================================================================

  /** True if the motor is mounted so positive output drives the carriage down rather than up. */
  public static final boolean INVERTED = false;

  // TODO: supply the gearing, then uncomment.
  //
  // GEAR_RATIO is the TOTAL reduction: motor rotations per DRUM rotation, end to end.
  //
  // public static final double GEAR_RATIO = ;

  /**
   * Encoder rotations per mechanism rotation.
   *
   * <p>The whole reduction, because the sensor is the motor's own rotor — see the "no CANcoder"
   * note at the top of this file. There is deliberately no {@code ROTOR_TO_SENSOR_RATIO} to get
   * wrong.
   */
  public static final double SENSOR_TO_MECHANISM_RATIO = GEAR_RATIO;

  // ==========================================================================================
  // LINEAR GEOMETRY — the two numbers that turn rotations into inches
  // ==========================================================================================
  //
  // Phoenix only ever knows about rotations. Turning those into inches of carriage travel needs two
  // numbers that cannot be derived from the gear ratio, so both have to be measured or read off the
  // CAD.
  //
  // TODO: supply the drum geometry, then uncomment.
  //
  // DRUM_PITCH_DIAMETER is the diameter at which the rope, belt or chain ACTUALLY RIDES — the PITCH
  // diameter, not the outer diameter of the spool flange. On a sprocket it is the chain pitch
  // circle; on a pulley it is the belt pitch line. Using the outer diameter makes every height read
  // high by a few percent, which looks like a tuning problem for a long time.
  //
  // STAGE_COUNT is the rigging multiplier, and it is the one that gets missed. A single-stage lift
  // moves one circumference per drum rotation. A 2-stage cascade moves TWICE that, because the
  // stages travel together. Get this wrong and every height is off by an EXACT INTEGER FACTOR —
  // which is the tell, if you ever see it.
  //
  // public static final Distance DRUM_PITCH_DIAMETER = Inches.of();
  // public static final int STAGE_COUNT = ;

  /** Carriage travel per drum rotation. Derived, so the two inputs above stay the source. */
  public static final Distance TRAVEL_PER_ROTATION =
      DRUM_PITCH_DIAMETER.times(Math.PI * STAGE_COUNT);

  // ==========================================================================================
  // SOFT LIMITS — travel bounds enforced by the motor controller
  // ==========================================================================================
  //
  // Enforced ON THE DEVICE, so they hold even when a command sends a bad setpoint or a gain sends
  // the mechanism running. That makes them mechanism protection rather than convenience — treat
  // changing one the way you would treat changing a current limit (.claude/rules/00-safety.md).
  //
  // Declared in INCHES because that is the unit everything else about this mechanism is in.
  // getFXConfig() converts to the rotations Phoenix wants, at the one boundary.
  //
  // THESE ARE ONLY MEANINGFUL ONCE THE MECHANISM HAS BEEN ZEROED. A soft limit is a bound on the
  // reported position, and the reported position is a lie until the zeroing routine has run. That
  // is not a reason to skip them — it is the reason ExampleLiftCommands.zero() refuses to zero at
  // an unknown position rather than guessing.

  /**
   * Bottom of travel, in inches.
   *
   * <p>Zero, because the zeroing routine defines the hard stop as zero. Set it slightly positive if
   * the carriage should never return fully to the stop under closed-loop control — a mechanism
   * holding position against a hard stop draws current until something gives.
   */
  public static final double MIN_TRAVEL_INCHES = 0.0;

  // TODO: supply the top of travel, then uncomment.
  //
  // Measure it by driving the carriage to the upper hard stop, reading the reported height, then
  // backing off. A soft limit set exactly at the hard stop still lets the mechanism reach it.
  //
  // public static final double MAX_TRAVEL_INCHES = ;

  // ==========================================================================================
  // UNIT CONVERSION — the boundary between team units and Phoenix units
  // ==========================================================================================
  //
  // We declare in inches and RPM because those are the units a person can reason about with the
  // robot in front of them. Phoenix works in rotations and rotations per second, always.
  //
  // Every conversion lives here so there is exactly one place to check. A bare arithmetic
  // conversion in an IO class is how a factor error gets in — it compiles, deploys, and moves the
  // mechanism, just not the way anyone expected.

  /** Carriage inches to mechanism rotations. */
  public static double inchesToRotations(double inches) {
    return inches / TRAVEL_PER_ROTATION.in(Inches);
  }

  /** Mechanism rotations back to carriage inches, for logging and tolerance checks. */
  public static double rotationsToInches(double rotations) {
    return rotations * TRAVEL_PER_ROTATION.in(Inches);
  }

  /** Mechanism RPM to the rotations/sec Phoenix wants. */
  public static double rpmToRotationsPerSec(double rpm) {
    return RPM.of(rpm).in(RotationsPerSecond);
  }

  /**
   * Mechanism RPM/sec to the rotations/sec squared Phoenix wants.
   *
   * <p>Typed as an angular <i>acceleration</i>, not an angular velocity. The numeric factor is the
   * same 60 either way, so writing this with {@code RPM.of(...)} would give the right answer today
   * — and would quietly stop being right the moment someone reused the helper for something that is
   * not a per-second rate.
   */
  public static double rpmPerSecToRotationsPerSecSquared(double rpmPerSec) {
    return RPM.per(Second).of(rpmPerSec).in(RotationsPerSecondPerSecond);
  }

  /**
   * Mechanism RPM/sec squared to the rotations/sec cubed Phoenix wants, for jerk.
   *
   * <p>The numeric factor is 60 again, so this could have reused the acceleration helper above and
   * been right. It does not, for the reason stated there: a helper named for the quantity it
   * converts stays correct when someone changes it, and one borrowed for a quantity it is not named
   * after does not.
   */
  public static double rpmPerSecSquaredToRotationsPerSecCubed(double rpmPerSecSquared) {
    return RPM.per(Second)
        .per(Second)
        .of(rpmPerSecSquared)
        .in(RotationsPerSecondPerSecond.per(Second));
  }

  // ==========================================================================================
  // THEORETICAL TOP SPEED
  // ==========================================================================================

  /** Which motor drives this mechanism. Picks the free speed and the sim model together. */
  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  /** How many motors, leader and followers together. Torque scales with this; speed does not. */
  public static final int MOTOR_COUNT = 1;

  /**
   * Theoretical top speed at the drum, in RPM. Computed, never typed in.
   *
   * <p>A ceiling, not a target: free speed is the motor with nothing attached, and a lift fights
   * its own weight the whole way up. It is also the sanity check on the gear ratio — multiply it by
   * {@link #TRAVEL_PER_ROTATION} and see whether the resulting inches per second is anywhere near
   * what this mechanism is supposed to do.
   */
  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(GEAR_RATIO);

  // ==========================================================================================
  // MOTION PROFILE
  // ==========================================================================================
  //
  // CRUISE — half of theoretical, which leaves headroom for load. A lift fights gravity the whole
  // way up, and a saturated motor simply stops following the profile.
  //
  // Measured against real elevators this is conservative, deliberately: 6328's 2025 elevator
  // cruised at 2.0 m/s (79 in/s) and 254's at 3.6 m/s (140 in/s). Half of theoretical will usually
  // land below both. Raise it once the mechanism is characterized; it is a tunable.
  //
  // ACCELERATION — 900 RPM/s, about 0.4 g on a lift in the 10 in/rotation range.
  //
  // THIS USED TO BE 9000, on the reasoning that acceleration should be near-unlimited and let
  // cruise velocity and the current limit bound the motion. That reasoning holds only if the
  // current limit actually binds first, and with the deliberately low STATOR_CURRENT_LIMIT_AMPS
  // above it frequently does not. On a light carriage 9000 RPM/s is a real 4+ g: the profile
  // reaches cruise in about one loop and roughly a third of an inch, so it is a step rather than a
  // ramp. That is the opposite of what a low current limit is there to buy you.
  //
  // 900 is calibrated against 6328's 2025 elevator, which ran 4.0 m/s^2 (0.41 g) — an
  // AdvantageKit codebase using the same tunable-profile pattern as this one. 254 ran 4.0 g, but
  // with cruise pinned at the motor's voltage ceiling and jerk doing the smoothing. That is a
  // tuned endpoint reached by a team with match data, not a starting default.
  //
  // RE-DERIVE IT FOR YOUR MECHANISM. RPM/s is not an acceleration you can compare across robots —
  // it means different things on different drums. Multiply by TRAVEL_PER_ROTATION and divide by 60
  // for in/s^2. Aim near 0.4 g to start, then raise it while watching closedLoopReference against
  // the measured position: once the reference stops being followed, the current limit is binding
  // and a larger number buys nothing.
  //
  // These differ from exampleArm's on purpose. An arm cruises at a flat 60 RPM and accelerates at
  // 300 RPM/s, because an over-fast profile on a pivoting mechanism does mechanical damage — and
  // a pivot's units are degrees, not inches, so neither number carries across. That is precisely
  // why the three scaffolds are separate files rather than one file with a comment telling you
  // which block to delete.
  //
  // Both are tunable. A profile is the thing you most want to adjust with the mechanism in front of
  // you, and it is far safer to change than a gain. See docs/tunables.md.

  /** Compiled-in default for {@link #CRUISE_VELOCITY_RPM}, in drum RPM. */
  public static final double CRUISE_VELOCITY_RPM_DEFAULT = MAX_SPEED_RPM / 2.0;

  /**
   * Compiled-in default for {@link #ACCELERATION_RPM_PER_SEC}, in drum RPM per second.
   *
   * <p>About 0.4 g on a typical lift — see the note above for where that came from and how to
   * convert it into your own mechanism's inches per second squared.
   */
  public static final double ACCELERATION_RPM_PER_SEC_DEFAULT = 900.0;

  /** Profile cruise velocity, in drum RPM. Tunable at runtime. */
  public static final LoggedTunableNumber CRUISE_VELOCITY_RPM =
      new LoggedTunableNumber(NAME + "/CruiseVelocityRpm", CRUISE_VELOCITY_RPM_DEFAULT);

  /** Profile acceleration, in drum RPM per second. Tunable at runtime. */
  public static final LoggedTunableNumber ACCELERATION_RPM_PER_SEC =
      new LoggedTunableNumber(NAME + "/AccelRpmPerSec", ACCELERATION_RPM_PER_SEC_DEFAULT);

  /**
   * Compiled-in default for {@link #JERK_RPM_PER_SEC_SQUARED}, in mechanism RPM per second squared.
   *
   * <p>Ten times the acceleration default, which is the usual starting ratio. Jerk bounds how fast
   * the acceleration itself is allowed to change, so it rounds the corners at the start and the end
   * of a profile rather than stepping into them. Those two corners are where a mechanism lurches
   * and where it arrives hardest, which makes this the profile-side companion to the low stator
   * limit above.
   *
   * <p>Too low and it becomes the binding constraint: the profile never reaches the acceleration
   * you asked for, and every motion is slower than the cruise and acceleration numbers suggest.
   * That is why the default is expressed as a multiple — retune acceleration and this stays
   * proportionate instead of quietly taking over. Zero disables jerk limiting entirely, which is
   * Phoenix's default and what this scaffold used to do.
   */
  public static final double JERK_RPM_PER_SEC_SQUARED_DEFAULT =
      ACCELERATION_RPM_PER_SEC_DEFAULT * 10.0;

  /** Profile jerk, in mechanism RPM per second squared. Tunable at runtime. */
  public static final LoggedTunableNumber JERK_RPM_PER_SEC_SQUARED =
      new LoggedTunableNumber(NAME + "/JerkRpmPerSecSq", JERK_RPM_PER_SEC_SQUARED_DEFAULT);

  /** Everything re-applied to the Talon when a dashboard profile value moves. */
  public static final LoggedTunableNumber[] TUNABLE_PROFILE = {
    ACCELERATION_RPM_PER_SEC, CRUISE_VELOCITY_RPM, JERK_RPM_PER_SEC_SQUARED
  };

  // ==========================================================================================
  // TOLERANCE
  // ==========================================================================================

  /**
   * How close counts as "at height", in carriage inches.
   *
   * <p>A tolerance in degrees would mean nothing on a lift — this is why the linear and rotary
   * scaffolds are separate. Too tight and the mechanism never reports ready, so every sequence runs
   * to its timeout instead of proceeding. Too loose and it acts before it has arrived.
   *
   * <p>Tunable, because the right value is the one you find by watching the mechanism report ready
   * against what it is actually doing. It is read live on every {@code isAtPosition()} call, so a
   * dashboard edit takes effect on the next loop with no re-apply and no CAN traffic.
   */
  public static final double POSITION_TOLERANCE_INCHES_DEFAULT = 0.25;

  /** How close counts as "at height", in carriage inches. Tunable at runtime. */
  public static final LoggedTunableNumber POSITION_TOLERANCE_INCHES =
      new LoggedTunableNumber(NAME + "/PositionToleranceIn", POSITION_TOLERANCE_INCHES_DEFAULT);

  // ==========================================================================================
  // ZEROING — how this mechanism learns where it is
  // ==========================================================================================
  //
  // The motor's internal rotor position is RELATIVE: it means nothing until something establishes
  // where zero actually is, and power-up silently assumes the mechanism is already there. The
  // zeroing routine drives the carriage down into its hard stop and declares that position zero.
  //
  // These are live rather than commented, unlike the geometry above, and the distinction is
  // deliberate: a wrong drum diameter is INVISIBLE — the mechanism reports plausible heights that
  // are all wrong — whereas a wrong creep voltage or stall threshold is obvious within one bring-up
  // session, because the carriage either fails to reach the stop or stops short of it while you
  // watch. Pick them with the mechanism in front of you; do not ship the defaults untested.
  //
  // Two public reference points, both real robots, both keying off VELOCITY rather than current
  // alone: Team 5419's 2024 elevator creeps at -30% duty cycle (about 3.6 V at a nominal 12 V bus)
  // and confirms the stop after velocity stays under about 0.02 rot/s for 0.08 s; Team 6328's
  // GenericSlamElevator uses the same velocity-near-zero-for-a-dwell idea. 5419's current-based
  // check exists in their source but is commented out and unused.

  /** Downward creep voltage while seeking the hard stop. Negative is down. */
  public static final Voltage ZEROING_VOLTAGE = Volts.of(-3.0);

  /**
   * Stator current, in amps, that counts as "pushing against the hard stop" while creeping.
   *
   * <p>Tunable: this is the single value most likely to be wrong on the first try, and finding it
   * means creeping into the stop and reading the log. Read live in {@code isAtZeroingStall()}.
   */
  public static final double ZEROING_STALL_CURRENT_AMPS_DEFAULT = 25.0;

  /** Stall-detection stator current, in amps. Tunable at runtime. */
  public static final LoggedTunableNumber ZEROING_STALL_CURRENT_AMPS =
      new LoggedTunableNumber(NAME + "/ZeroingStallAmps", ZEROING_STALL_CURRENT_AMPS_DEFAULT);

  /** Compiled-in default for {@link #ZEROING_VELOCITY_THRESHOLD_RPM}, in drum RPM. */
  public static final double ZEROING_VELOCITY_THRESHOLD_RPM_DEFAULT = 5.0;

  /**
   * Drum velocity, in RPM, below which the carriage counts as "not moving" while creeping. Tunable
   * at runtime, and read live in {@code isAtZeroingStall()}.
   */
  public static final LoggedTunableNumber ZEROING_VELOCITY_THRESHOLD_RPM =
      new LoggedTunableNumber(
          NAME + "/ZeroingVelocityThresholdRpm", ZEROING_VELOCITY_THRESHOLD_RPM_DEFAULT);

  /**
   * How long BOTH conditions must hold before this counts as a real stall.
   *
   * <p>Current alone spikes at the instant the motor breaks static friction, long before the
   * carriage is anywhere near the stop. Requiring high current AND near-zero velocity for a dwell
   * is what tells the two apart.
   *
   * <p><b>Not a tunable, deliberately.</b> A {@code Debouncer} reads its dwell once, when it is
   * constructed in the {@code ExampleLift} constructor, so a dashboard edit would move the logged
   * number and change nothing about the robot. Same for the timeout below, which {@code
   * .withTimeout()} captures when the command is built. A knob that appears to work and does not is
   * worse than no knob — edit these here and redeploy.
   */
  public static final double ZEROING_STALL_DEBOUNCE_SECONDS = 0.1;

  /** Give up and refuse to zero if the stop is never found within this budget. Not tunable. */
  public static final double ZEROING_TIMEOUT_SECONDS = 3.0;

  // ==========================================================================================
  // VISUALIZATION — display only. Delete with ExampleLiftVisualizer if unused.
  // ==========================================================================================

  /**
   * Geometry for {@link ExampleLiftVisualizer}.
   *
   * <p><b>None of this affects the robot.</b> Every value here changes a picture and nothing else,
   * which makes it the one block in this file you can safely guess at and adjust by eye while
   * watching AdvantageScope. Do that rather than deferring it — a visualizer nobody bothered to
   * scale is one nobody looks at.
   *
   * <p>The canvas is in meters because {@code LoggedMechanism2d} is, but it is a drawing, not the
   * robot. Only the {@code CARRIAGE_*} offsets describe real space, and those come off the CAD.
   */
  public static class Visualization {

    private Visualization() {}

    /**
     * Master switch. Publishing a Mechanism2d to NetworkTables every loop is not free, and the 2026
     * robot ran closer to 30 Hz than 50 Hz all season.
     *
     * <p>Leave it on through bring-up, where it is the whole point. Before competition, check
     * {@code LoopTiming/} with it enabled and turn it off if the budget is tight.
     */
    public static final boolean ENABLED = true;

    /** Canvas size. Cosmetic — pick something the mechanism fits inside with room to spare. */
    public static final double CANVAS_WIDTH_METERS = 1.0;

    public static final double CANVAS_HEIGHT_METERS = 1.0;

    /** Where the base of the travel sits on the canvas. Cosmetic. */
    public static final double ROOT_X_METERS = 0.5;

    public static final double ROOT_Y_METERS = 0.1;

    /**
     * Drawn length at zero travel, in canvas meters.
     *
     * <p>A ligament of zero length is not drawn at all, so without this a fully retracted carriage
     * silently disappears and reads as a broken visualizer rather than a retracted mechanism.
     */
    public static final double MIN_LENGTH_METERS = 0.05;

    /**
     * Canvas meters drawn per inch of real travel.
     *
     * <p>Pure display scale, unrelated to {@link #TRAVEL_PER_ROTATION}. Choose it so full travel
     * roughly fills the canvas: for 24 in of travel on a 1 m canvas, about 0.035.
     */
    public static final double INCHES_TO_CANVAS_METERS = 0.035;

    // ----------------------------------------------------------------------------------------
    // 3D component pose — the only real-space values here
    // ----------------------------------------------------------------------------------------
    //
    // Robot frame: X forward, Y left, Z up, origin at the robot's center on the floor.
    //
    // A lift TRANSLATES rather than rotates, so the carriage's travel goes into the pose's Z and
    // its rotation stays fixed. This is the retracted position — where the carriage sits at zero.
    // It defaults to the robot origin, which draws the component in the middle of the robot and is
    // obviously wrong on sight. That is the intent.

    /** Retracted carriage position in the robot frame. */
    public static final Translation3d CARRIAGE_OFFSET = Translation3d.kZero;

    /** Fixed mounting rotation of the component. A lift does not turn, so this never changes. */
    public static final Rotation3d CARRIAGE_ROTATION = Rotation3d.kZero;
  }

  // ==========================================================================================
  // GAINS — REAL ROBOT
  // ==========================================================================================
  //
  // Placeholders. Measure them in this order for a POSITION loop: kG -> kS -> kP -> kD. Never start
  // with kP. See docs/characterization-and-tuning.md Part 3.
  //
  // kG FIRST: find the current that just holds the carriage still at mid-travel, before any
  // feedback is involved. Everything after that is correcting a mechanism that is already nearly
  // balanced. Skip it and kP ends up carrying the weight, which means the carriage sags at rest and
  // the "fix" is more kP until it oscillates.
  //
  // UNLIKE AN ARM, kG IS CONSTANT HERE. A lift fights the same weight at every height, which is
  // what GravityTypeValue.Elevator_Static means. If kG appears to need to change with height, the
  // rigging is doing something the model does not describe — chase that rather than fitting a
  // curve.
  //
  // kV and kA exist for characterization runs but are usually left at zero on a profiled position
  // loop — Motion Magic generates its own velocity and acceleration feedforward from the profile.

  /** Gravity feedforward, in amps. The current that holds the carriage. Find this first. */
  public static final LoggedTunableNumber KG = new LoggedTunableNumber(NAME + "/kG", 0.0);

  /** Static friction feedforward — output needed to break the mechanism loose. */
  public static final LoggedTunableNumber KS = new LoggedTunableNumber(NAME + "/kS", 0.0);

  /** Velocity feedforward. Usually zero on a profiled position loop. */
  public static final LoggedTunableNumber KV = new LoggedTunableNumber(NAME + "/kV", 0.0);

  /** Acceleration feedforward. Usually zero on a profiled position loop. */
  public static final LoggedTunableNumber KA = new LoggedTunableNumber(NAME + "/kA", 0.0);

  /** Proportional gain. Corrects what feedforward misses — it should not be carrying the load. */
  public static final LoggedTunableNumber KP = new LoggedTunableNumber(NAME + "/kP", 0.0);

  /** Integral gain. Leave at zero unless you can explain its behavior during a stall. */
  public static final LoggedTunableNumber KI = new LoggedTunableNumber(NAME + "/kI", 0.0);

  /** Derivative gain. Damps overshoot on a position loop — this is where it earns its place. */
  public static final LoggedTunableNumber KD = new LoggedTunableNumber(NAME + "/kD", 0.0);

  /** Every real-robot gain, for the {@code ifChanged} watch list in the IO layer. */
  public static final LoggedTunableNumber[] TUNABLE_GAINS = {KS, KV, KA, KG, KP, KI, KD};

  // ==========================================================================================
  // SIMULATION
  // ==========================================================================================

  /**
   * Simulation constants, deliberately nested and deliberately separate from the real gains.
   *
   * <p><b>Why sim needs its own gains.</b> The sim model is not the robot. No backlash, no belt
   * stretch, no friction beyond what the model declares, and a carriage mass that is usually a
   * guess. Gains that behave well against that model are frequently wrong on hardware, and gains
   * measured on hardware often will not converge in sim.
   *
   * <p><b>Sim gains are plain doubles, not tunables, on purpose.</b> Tunables exist to avoid the
   * two-minute edit-build-deploy-enable cycle. Sim has no deploy step, so a dashboard knob buys
   * nothing and only adds a second set of numbers that can disagree with the file.
   */
  public static class Sim {

    private Sim() {}

    /** The motor(s) driving this mechanism, for the sim model. */
    public static final DCMotor MOTOR = ExampleLiftConstants.MOTOR.gearbox(MOTOR_COUNT);

    /**
     * Moving mass in kilograms — carriage plus everything it lifts.
     *
     * <p>{@code ElevatorSim} needs a mass and a drum radius rather than an inertia, because gravity
     * force on a lift is mass times g regardless of geometry. This is the number kG has to cancel,
     * so a guess here makes every sim conclusion about kG meaningless.
     */
    public static final double CARRIAGE_MASS_KG = 5.0;

    /** Whether the sim applies gravity. True for a vertical lift; false for a horizontal slide. */
    public static final boolean SIMULATE_GRAVITY = true;

    public static final double KG = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
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
  // MOTOR CONFIG
  // ==========================================================================================

  /**
   * The one config for this mechanism, applied at construction and re-applied piecewise when a
   * tunable moves.
   *
   * <p>Differences from {@code exampleArm}, all of them consequences of travelling in a line:
   *
   * <ol>
   *   <li><b>No remote sensor.</b> Feedback is the motor's own rotor; the zeroing routine supplies
   *       the reference. See the note at the top of this file for why.
   *   <li><b>{@code Elevator_Static} gravity</b>, not {@code Arm_Cosine}. A lift fights the same
   *       weight at every height, so there is no angle for the compensation to follow — and no
   *       {@code GravityArmPositionOffset} to get wrong.
   *   <li><b>Soft limits in inches</b>, converted at this boundary.
   * </ol>
   *
   * <p>Brake mode is shared with the arm and for the same reason: a coasting lift falls.
   *
   * @return a fully-built config, ready for {@code PhoenixUtil.tryUntilOk(5, ...)}
   */
  public static TalonFXConfiguration getFXConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // ---- Current limits ----
    // Enforced on the real robot only, the same way the torque clamp below is. WPILib's sim models
    // total current draw rather than the winding and battery currents these limits govern, so a
    // limit applied in sim clamps a number that does not mean what the limit means — sim then
    // behaves unlike both the real robot and the model the gains were tuned against. The limits
    // stay set either way, so a Tuner X self-test still reads the intended values.
    config.CurrentLimits.SupplyCurrentLimitEnable = isReal();
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME.in(Seconds);
    config.CurrentLimits.StatorCurrentLimitEnable = isReal();
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
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // a coasting lift falls
    config.MotorOutput.Inverted =
        INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // ---- Soft limits ----
    // Inches in, rotations out — the one conversion boundary.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = inchesToRotations(MAX_TRAVEL_INCHES);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = inchesToRotations(MIN_TRAVEL_INCHES);

    // ---- Feedback ----
    // Internal rotor only. SensorToMechanismRatio makes position read in drum rotations, which is
    // what inchesToRotations() and every soft limit above assume.
    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;

    // ---- Gains ----
    config.Slot0.kS = getKS();
    config.Slot0.kG = getKG();
    config.Slot0.kP = getKP();
    config.Slot0.kD = getKD();

    // Elevator_Static: the same weight at every height. An arm would use Arm_Cosine, whose gravity
    // torque varies with the cosine of the angle. Getting this backwards on a lift means the
    // compensation fades toward the top of travel, exactly where the mechanism is most extended.
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Take the sign of kS from the closed-loop error rather than measured velocity. At rest on a
    // setpoint the velocity is near zero and noisy, so UseVelocitySign flips and the mechanism
    // chatters — on a lift holding height that is an audible buzz and real wear.
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    // ---- Motion profile ----
    // RPM and RPM/sec in, rotations/sec and rotations/sec^2 out.
    config.MotionMagic.MotionMagicAcceleration =
        rpmPerSecToRotationsPerSecSquared(ACCELERATION_RPM_PER_SEC.get());
    config.MotionMagic.MotionMagicCruiseVelocity = rpmToRotationsPerSec(CRUISE_VELOCITY_RPM.get());
    config.MotionMagic.MotionMagicJerk =
        rpmPerSecSquaredToRotationsPerSecCubed(JERK_RPM_PER_SEC_SQUARED.get());

    return config;
  }
}
