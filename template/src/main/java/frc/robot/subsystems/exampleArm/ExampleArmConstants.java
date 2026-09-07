package frc.robot.subsystems.exampleArm;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.lib.LoggedTunableNumber;
import frc.lib.MotorSpecs;
import frc.robot.Constants;

/**
 * Constants and motor configuration for a <b>ROTARY POSITION-CONTROLLED</b> mechanism.
 *
 * <p>Copy this scaffold for an arm, pivot, hood, wrist or turret — anything that rotates to an
 * angle and holds it. In 2026 that covered the hood and the intake pivot.
 *
 * <p><b>Wrong scaffold?</b> {@code exampleRoller} for anything that spins and only has a speed;
 * {@code exampleLift} for anything that travels in a straight line.
 *
 * <h2>THIS FILE DOES NOT COMPILE ON PURPOSE</h2>
 *
 * <p>Eight values cannot be guessed, so their <i>declarations</i> are commented out while the code
 * below still <i>assigns</i> them. The compiler then names each one as an error:
 *
 * <pre>
 * Constants.CanIds.EXAMPLE_ARM_MOTOR      CAN ID
 * Constants.CanIds.EXAMPLE_ARM_ENCODER    CAN ID
 * GEAR_RATIO                              total reduction, motor to mechanism
 * ROTOR_TO_SENSOR_RATIO                   motor rotations per encoder rotation
 * SENSOR_TO_MECHANISM_RATIO               encoder rotations per mechanism rotation
 * MAGNET_OFFSET_ROTATIONS                 CANcoder calibration
 * FORWARD_SOFT_LIMIT_DEGREES              travel bound
 * REVERSE_SOFT_LIMIT_DEGREES              travel bound
 * </pre>
 *
 * <p>Expect exactly those eight and nothing else. Anything more is rot in the scaffold.
 *
 * <p>Commenting out the <i>assignments</i> instead would compile, and would be worse. Phoenix
 * defaults {@code SensorToMechanismRatio} to 1.0, so a config that silently skips it reports motor
 * rotations while every setpoint, tolerance and soft limit here assumes mechanism rotations — a
 * confidently wrong robot, which is much harder to notice than one that refuses to build. The same
 * argument applies with more force to the soft limits: a missing travel bound does not fail, it
 * drives an arm into a hard stop at full current.
 *
 * <h2>What does NOT go in this file</h2>
 *
 * <p><b>Setpoints.</b> This file describes how the mechanism is <i>built</i> — gains, gear ratio,
 * current limits, soft limits, tolerance, the sim model. The angles it is <i>commanded to</i> —
 * stow, deploy, score — belong in {@code Constants.Setpoints}, together with every other
 * mechanism's. The test is whether a driver might ask you to change it between matches. A tolerance
 * stays here; how close counts as "there" is a property of the mechanism.
 *
 * <p><b>CAN IDs.</b> {@code Constants.CanIds}, so every ID on the robot is visible in one table.
 *
 * <p><b>UNITS:</b> positions in <b>degrees</b>, velocities in <b>RPM</b>, and under {@code
 * *TorqueCurrentFOC} the gains are in <b>AMPS</b>, not volts. See {@code
 * docs/characterization-and-tuning.md} — a steer kP in the thousands is normal, because it is amps
 * per rotation.
 */
public class ExampleArmConstants {

  private ExampleArmConstants() {}

  /** Log key and alert prefix. One name used everywhere, so a grep finds all of it. */
  public static final String NAME = "ExampleArm";

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
  // A stalled arm holding against gravity is the case where these diverge most: stator stays high
  // while supply is a fraction of it, because the controller is chopping.

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
   * draws. An arm accelerating its own weight away from stow frequently needs more than 40 A. Too
   * little current does not fail loudly: the profile saturates and the mechanism never arrives,
   * which reads as a tuning problem and is not one. Diagnose it from the closed-loop reference
   * channel — reference tracking the goal with measured lagging behind is the signature — not by
   * feel.
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
  // SOFT LIMITS — travel bounds enforced by the motor controller
  // ==========================================================================================
  //
  // Enforced ON THE DEVICE, so they hold even when a command sends a bad setpoint or a gain sends
  // the mechanism running. That makes them mechanism protection rather than convenience — treat
  // changing one the way you would treat changing a current limit (.claude/rules/00-safety.md).
  //
  // Declared in DEGREES because that is the unit everything else about this mechanism is in.
  // getFXConfig() converts to the rotations Phoenix wants, at the one boundary.
  //
  // TODO: supply the travel bounds, then uncomment.
  //
  // Measure them by moving the mechanism to each hard stop, reading the reported angle, then
  // backing off a degree or two. A soft limit set exactly at the hard stop still lets the
  // mechanism reach it, and an arm resting on a hard stop under closed-loop hold draws current
  // until something gives.
  //
  // public static final double FORWARD_SOFT_LIMIT_DEGREES = ;
  // public static final double REVERSE_SOFT_LIMIT_DEGREES = ;

  // ==========================================================================================
  // MECHANICAL — none of this can be guessed
  // ==========================================================================================

  /** True if the motor is mounted so positive output produces negative mechanism motion. */
  public static final boolean INVERTED = false;

  // TODO: supply the gearing, then uncomment.
  //
  // GEAR_RATIO is the TOTAL reduction: motor rotations per mechanism rotation, end to end. It is
  // the number that sets top speed and the one to quote when someone asks "what is the ratio".
  //
  // public static final double GEAR_RATIO = ;
  //
  // Phoenix then needs that total SPLIT AT THE ENCODER, and the split is decided by ONE question:
  // WHERE IS THE ENCODER? Answer that before writing either number. "Does it have a CANcoder" is
  // not enough — you need the shaft.
  //
  //   ROTOR_TO_SENSOR_RATIO      motor rotations per encoder rotation  (rotor/CANcoder fusion)
  //   SENSOR_TO_MECHANISM_RATIO  encoder rotations per mechanism rotation
  //
  // The two must multiply back to GEAR_RATIO. If they do not, either the top speed or the reported
  // position is wrong, and nothing will tell you which.
  //
  // ---- Case 1: CANcoder on the mechanism itself ----
  // A hex-bore CANcoder on the shaft the mechanism actually turns on. The encoder already reads
  // mechanism rotations, so there is nothing left below it. THIS IS THE COMMON CASE.
  //   ROTOR_TO_SENSOR_RATIO     = GEAR_RATIO
  //   SENSOR_TO_MECHANISM_RATIO = 1.0
  //
  // ---- Case 2: CANcoder on an intermediate shaft ----
  // There is still gearing BETWEEN the encoder and the mechanism. Split the total where the
  // encoder physically sits.
  //
  // The 2026 hood is the example, and it is worth reading carefully because the trap is in the
  // vocabulary. That code calls the encoder's shaft the "output shaft" — meaning the GEARBOX
  // output, not the mechanism. A 12T lantern gear on that shaft still drives a 122T hood gear
  // after it, so the encoder turns about ten times per hood rotation:
  //
  //   motor -> belt -> shaft (CANcoder here) -> 12T lantern -> 122T hood
  //   ROTOR_TO_SENSOR_RATIO     = 5.33            (measured, motor to that shaft)
  //   SENSOR_TO_MECHANISM_RATIO = 122.0 / 12.0    (about 10.17)
  //   GEAR_RATIO                = 5.33 * 10.17    (about 54.2)
  //
  // "Output shaft" is ambiguous and has cost this team time before. The only question that
  // matters: DOES THE ENCODER TURN 1:1 WITH THE THING YOU ARE MEASURING? If anything geared sits
  // after it, you are in case 2.
  //
  // public static final double ROTOR_TO_SENSOR_RATIO = ;
  // public static final double SENSOR_TO_MECHANISM_RATIO = ;

  /**
   * How far the two ratios may drift from {@link #GEAR_RATIO} before the check below rejects them,
   * as a fraction.
   *
   * <p>One percent, not something tighter, because the numbers people quote are hand-rounded: the
   * 2026 hood's 5.33 × 10.17 comes to 54.206 against a quoted total of 54.2. A real mistake here is
   * never off by a percent — it is off by a whole gear ratio, because the usual error is putting
   * the entire reduction on the wrong side of the encoder.
   */
  private static final double RATIO_CONSISTENCY_TOLERANCE = 0.01;

  /*
   * Guards the one relationship between the three ratios that nothing else can catch.
   *
   * ROTOR_TO_SENSOR_RATIO * SENSOR_TO_MECHANISM_RATIO must equal GEAR_RATIO. The comment above says
   * so, and until now nothing enforced it — which made it exactly the kind of silent wrongness the
   * rest of this file is built to prevent. Get the split backwards and the config still applies, the
   * mechanism still moves, and either the reported position or the computed top speed is wrong by
   * the whole reduction with nothing in the log to say which.
   *
   * Throwing at class load is deliberate. These are compile-time constants, so if they disagree they
   * disagree on the bench, on the first sim run, and in RobotContainerSmokeTest once this subsystem
   * is wired — never for the first time at an event. A loud failure that can only happen before you
   * leave the shop is strictly better than a quiet one that ships.
   */
  static {
    double product = ROTOR_TO_SENSOR_RATIO * SENSOR_TO_MECHANISM_RATIO;
    if (Math.abs(product - GEAR_RATIO) > GEAR_RATIO * RATIO_CONSISTENCY_TOLERANCE) {
      throw new IllegalStateException(
          String.format(
              "%s: ROTOR_TO_SENSOR_RATIO (%.4f) * SENSOR_TO_MECHANISM_RATIO (%.4f) = %.4f, "
                  + "which is not GEAR_RATIO (%.4f). The split must multiply back to the total "
                  + "reduction. Ask where the encoder physically sits: on the mechanism shaft it is "
                  + "(GEAR_RATIO, 1.0); on an intermediate shaft, split the total where the encoder "
                  + "is. See the case table above this check.",
              NAME, ROTOR_TO_SENSOR_RATIO, SENSOR_TO_MECHANISM_RATIO, product, GEAR_RATIO));
    }
  }

  // TODO: supply the CANcoder calibration, then uncomment.
  //
  // MAGNET_OFFSET is calibration data, exactly like the swerve encoder offsets: found by
  // physically moving the mechanism to a known position and reading the raw sensor. Not guessable,
  // and it changes whenever the encoder or magnet is disturbed.
  //
  // public static final double MAGNET_OFFSET_ROTATIONS = ;

  /**
   * Where the absolute reading wraps, in sensor rotations.
   *
   * <p>1.0 gives a range of [0, 1); 0.5 gives [-0.5, 0.5). Put the discontinuity somewhere the
   * mechanism never travels, or position jumps a full rotation mid-motion.
   *
   * <p><b>1.0 is rarely the right answer</b> despite being the obvious-looking default. It puts the
   * wrap at 0, and 0 is almost always the stow position — the one place the mechanism sits most of
   * the match. The reading then flips between about 0.0 and about 1.0 every time it settles there,
   * and a position loop chases a full rotation of phantom error.
   *
   * <p>0.5 puts the wrap half a turn away, which for any mechanism travelling less than 180 degrees
   * from zero is nowhere near its range. That is the default here, and it is right for most arms
   * and pivots. Change it only if you can say where this mechanism's travel actually sits.
   */
  public static final double SENSOR_DISCONTINUITY_POINT = 0.5;

  /**
   * Which way the CANcoder counts up.
   *
   * <p>A property of how the magnet and sensor are physically oriented, so it belongs here rather
   * than in the IO — and it genuinely varies. The 2026 intake pivot needed {@code
   * Clockwise_Positive}, so do not read this default as the usual answer; there isn't one.
   *
   * <p><b>This is sensor inversion, not motor inversion.</b> The two are independent axes in
   * Phoenix 6 and fixing one while the other is wrong causes oscillation: the loop reads the
   * mechanism moving away from the setpoint and drives harder. If the arm runs away from its goal,
   * check {@link #INVERTED} and this together, and confirm at low open-loop output before enabling
   * closed-loop.
   */
  public static final SensorDirectionValue ENCODER_DIRECTION =
      SensorDirectionValue.CounterClockwise_Positive;

  // ==========================================================================================
  // BEFORE YOU FIT THAT ENCODER — check the travel fits in one sensor rotation
  // ==========================================================================================
  //
  // An absolute reading repeats every turn, so a mechanism whose SENSOR moves further than one
  // rotation cannot say which turn it is on. With the encoder on the mechanism shaft (case 1) the
  // sensor turns exactly as far as the mechanism does, so any arm travelling under 360 degrees is
  // safe — which is nearly all of them.
  //
  // THE AMBIGUITY NEVER SHOWS UP IN SIM, because sim always starts at zero. It shows up as a
  // mechanism that boots to a plausible but wrong angle, once, on the practice field.
  //
  // If the travel does exceed one sensor rotation, either gear the sensor down, or drop the
  // CANcoder and use the motor encoder with a zeroing routine against a hard stop — which is what
  // exampleLift does, and why that scaffold is built the way it is.

  // ==========================================================================================
  // UNIT CONVERSION — the boundary between team units and Phoenix units
  // ==========================================================================================
  //
  // We declare in degrees and RPM because those are the units a person can reason about with the
  // robot in front of them. Phoenix works in rotations and rotations per second, always.
  //
  // Every conversion lives here so there is exactly one place to check, and each is written with
  // WPILib units rather than a bare /360.0 or /60.0. An inverted magic number compiles, deploys,
  // and moves the mechanism — just not the way anyone expected.

  /** Mechanism degrees to the rotations Phoenix wants. */
  public static double degreesToRotations(double degrees) {
    return Degrees.of(degrees).in(Rotations);
  }

  /** Phoenix rotations back to degrees, for logging and tolerance checks. */
  public static double rotationsToDegrees(double rotations) {
    return Rotations.of(rotations).in(Degrees);
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
   * Theoretical top speed at the mechanism, in RPM. Computed, never typed in.
   *
   * <p>Uses {@link #GEAR_RATIO}, the total reduction — <b>not</b> {@code
   * SENSOR_TO_MECHANISM_RATIO}. On a mechanism with a fused CANcoder those are different numbers,
   * and using the split half would overstate top speed by exactly the other half. With the encoder
   * on the mechanism shaft the other half is 1.0, so quoting it would give you the motor's raw free
   * speed.
   *
   * <p>This is a ceiling, not a target, and mostly it is the sanity check on the gear ratio. An arm
   * geared 100:1 has a top speed in the tens of RPM; if this prints something in the thousands, the
   * ratio is wrong.
   */
  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(GEAR_RATIO);

  // ==========================================================================================
  // MOTION PROFILE
  // ==========================================================================================
  //
  // A raw position request commands maximum effort instantly, which on a mechanism with real
  // inertia means slamming into the setpoint and into the hard stops. Motion Magic profiles it.
  //
  // 60 RPM and 300 RPM/s are deliberately slow starting points, flat numbers regardless of what
  // the gearing could do. They are meant to be RAISED on purpose rather than lowered after
  // something breaks: an arm is where an over-fast profile does mechanical damage, and gravity
  // torque changes with angle in a way an aggressive profile will not respect.
  //
  // These differ from exampleLift's on purpose. A lift cruises at MAX_SPEED_RPM / 2 and
  // accelerates at 9000 RPM/s. An arm inheriting a lift's row gets roughly 30x the acceleration
  // it should have, aimed at a hard stop — which is precisely why the three scaffolds are separate
  // files rather than one file with a comment telling you which block to delete.
  //
  // Both are tunable. A profile is the thing you most want to adjust with the mechanism in front
  // of you, and it is far safer to change than a gain. See docs/tunables.md.

  /** Compiled-in default for {@link #CRUISE_VELOCITY_RPM}, in mechanism RPM. */
  public static final double CRUISE_VELOCITY_RPM_DEFAULT = 60.0;

  /** Compiled-in default for {@link #ACCELERATION_RPM_PER_SEC}, in mechanism RPM per second. */
  public static final double ACCELERATION_RPM_PER_SEC_DEFAULT = 300.0;

  /** Profile cruise velocity, in mechanism RPM. Tunable at runtime. */
  public static final LoggedTunableNumber CRUISE_VELOCITY_RPM =
      new LoggedTunableNumber(NAME + "/CruiseVelocityRpm", CRUISE_VELOCITY_RPM_DEFAULT);

  /** Profile acceleration, in mechanism RPM per second. Tunable at runtime. */
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
  // TOLERANCE AND GRAVITY REFERENCE
  // ==========================================================================================

  /**
   * How close counts as "at position", in mechanism degrees.
   *
   * <p>Too tight and the mechanism never reports ready, so every sequence runs to its timeout
   * instead of proceeding. Too loose and it acts before it has arrived.
   *
   * <p>Tunable, because the right value is the one you find by watching the mechanism report ready
   * against what it is actually doing. It is read live on every {@code isAtPosition()} call, so a
   * dashboard edit takes effect on the next loop with no re-apply and no CAN traffic.
   */
  public static final double POSITION_TOLERANCE_DEGREES_DEFAULT = 1.0;

  /** How close counts as "at position", in mechanism degrees. Tunable at runtime. */
  public static final LoggedTunableNumber POSITION_TOLERANCE_DEGREES =
      new LoggedTunableNumber(NAME + "/PositionToleranceDeg", POSITION_TOLERANCE_DEGREES_DEFAULT);

  /**
   * Where horizontal sits, in mechanism degrees.
   *
   * <p>{@code Arm_Cosine} scales kG by the cosine of the position plus this offset, and it assumes
   * the peak lands at a cosine argument of zero. If the mechanism's zero is its <b>stow</b>
   * position rather than horizontal — which is the usual way to define zero — the compensation
   * peaks in the wrong place: too little hold current where gravity is strongest, too much where
   * there is none.
   *
   * <p>Set this to the <b>negative</b> of the mechanism angle at which the arm is level. If the
   * pivot reads 30 degrees when horizontal, this is -30. Leave it at zero <i>only</i> if zero is
   * genuinely horizontal.
   *
   * <p>Live rather than commented because zero is a real, common answer and a wrong value here is
   * visible immediately during bring-up: the arm sags on one side of its travel and overshoots on
   * the other.
   *
   * <p>Tunable, and listed in {@link #TUNABLE_GAINS} rather than {@link #TUNABLE_PROFILE}, because
   * it is written to {@code Slot0.GravityArmPositionOffset} — it rides the same {@code Slot0}
   * re-apply as kG, which is the only way a change to it reaches the device. It belongs with the
   * gains for a second reason too: it and kG are two halves of one measurement, and tuning either
   * without the other is how an arm ends up with a kG that only holds at one angle.
   */
  public static final double GRAVITY_HORIZONTAL_OFFSET_DEGREES_DEFAULT = 0.0;

  /** Where horizontal sits, in mechanism degrees. Tunable at runtime. */
  public static final LoggedTunableNumber GRAVITY_HORIZONTAL_OFFSET_DEGREES =
      new LoggedTunableNumber(
          NAME + "/GravityHorizontalOffsetDeg", GRAVITY_HORIZONTAL_OFFSET_DEGREES_DEFAULT);

  // ==========================================================================================
  // VISUALIZATION — display only. Delete with ExampleArmVisualizer if unused.
  // ==========================================================================================

  /**
   * Geometry for {@link ExampleArmVisualizer}.
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
     * {@code LoopTiming/} with it enabled and turn it off if the budget is tight.
     */
    public static final boolean ENABLED = true;

    /** Canvas size. Cosmetic — pick something the mechanism fits inside with room to spare. */
    public static final double CANVAS_WIDTH_METERS = 1.0;

    public static final double CANVAS_HEIGHT_METERS = 1.0;

    /** Where the pivot sits on the canvas. Cosmetic. */
    public static final double ROOT_X_METERS = 0.5;

    public static final double ROOT_Y_METERS = 0.5;

    /** Drawn length of the arm. Cosmetic; scale it to fill the canvas. */
    public static final double LENGTH_METERS = 0.4;

    // ----------------------------------------------------------------------------------------
    // 3D component pose — the only real-space values here
    // ----------------------------------------------------------------------------------------
    //
    // Robot frame: X forward, Y left, Z up, origin at the robot's center on the floor.
    //
    // AdvantageScope rotates a component mesh ABOUT the pose it is given, so this has to be the
    // mechanism's actual axis of rotation — the pivot shaft, not the arm tip. Measure it in CAD.
    // Both default to the robot origin, which draws the component in the middle of the robot and
    // is obviously wrong on sight. That is the intent.

    /** Pivot axis in the robot frame. */
    public static final Translation3d PIVOT_OFFSET = Translation3d.kZero;

    /** Fixed mounting rotation of the component, before the mechanism's own motion is added. */
    public static final Rotation3d PIVOT_ROTATION = Rotation3d.kZero;

    /**
     * Which way the mesh turns as the measured angle increases.
     *
     * <p>Depends on which way the mechanism faces in the robot frame. Guessing is fine — flip the
     * sign while watching AdvantageScope. It is cosmetic and cannot affect the robot.
     */
    public static final double PITCH_SIGN = 1.0;
  }

  // ==========================================================================================
  // GAINS — REAL ROBOT
  // ==========================================================================================
  //
  // Placeholders. Measure them in this order for a POSITION loop: kG -> kS -> kP -> kD. Never
  // start with kP. See docs/characterization-and-tuning.md Part 3.
  //
  // kG FIRST, and it matters more here than anywhere else: find the current that just holds the
  // arm level against gravity, before any feedback is involved. Everything after that is
  // correcting a mechanism that is already nearly balanced. Skip it and kP ends up carrying the
  // weight, which means the arm sags at rest and the "fix" is more kP until it oscillates.
  //
  // kV and kA exist in the array below for characterization runs but are usually left at zero on
  // a profiled position loop — Motion Magic generates its own velocity and acceleration
  // feedforward from the profile.
  //
  // These are LoggedTunableNumbers so a tuning session does not need a redeploy per iteration.
  // With Constants.tuningMode off they return the defaults below and cost nothing.
  //
  // The default in each constructor is what the robot runs in competition. The dashboard value
  // lives only in NetworkTables and is gone at the next reboot, so a session that ends without
  // writing the numbers back into this file and committing them accomplished nothing.

  /** Gravity feedforward, in amps. The current that holds the arm level. Find this first. */
  public static final LoggedTunableNumber KG = new LoggedTunableNumber(NAME + "/kG", 0.0);

  /** Static friction feedforward — output needed to break the mechanism loose. */
  public static final LoggedTunableNumber KS = new LoggedTunableNumber(NAME + "/kS", 0.0);

  /** Velocity feedforward. Usually zero on a profiled position loop. */
  public static final LoggedTunableNumber KV = new LoggedTunableNumber(NAME + "/kV", 0.0);

  /** Acceleration feedforward. Usually zero on a profiled position loop. */
  public static final LoggedTunableNumber KA = new LoggedTunableNumber(NAME + "/kA", 0.0);

  /** Proportional gain. Corrects what feedforward misses — it should not be carrying the arm. */
  public static final LoggedTunableNumber KP = new LoggedTunableNumber(NAME + "/kP", 0.0);

  /** Integral gain. Leave at zero unless you can explain its behavior during a stall. */
  public static final LoggedTunableNumber KI = new LoggedTunableNumber(NAME + "/kI", 0.0);

  /** Derivative gain. Damps overshoot on a position loop — this is where it earns its place. */
  public static final LoggedTunableNumber KD = new LoggedTunableNumber(NAME + "/kD", 0.0);

  /** Every real-robot gain, for the {@code ifChanged} watch list in the IO layer. */
  public static final LoggedTunableNumber[] TUNABLE_GAINS = {
    KS, KV, KA, KG, KP, KI, KD, GRAVITY_HORIZONTAL_OFFSET_DEGREES
  };

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
   * <p><b>Sim gains are plain doubles, not tunables, on purpose.</b> Tunables exist to avoid the
   * two-minute edit-build-deploy-enable cycle. Sim has no deploy step, so a dashboard knob buys
   * nothing and only adds a second set of numbers that can disagree with the file.
   */
  public static class Sim {

    private Sim() {}

    /** The motor(s) driving this mechanism, for the sim model. */
    public static final DCMotor MOTOR = ExampleArmConstants.MOTOR.gearbox(MOTOR_COUNT);

    /**
     * Distance from the pivot to the arm's center of mass, in meters.
     *
     * <p>{@code SingleJointedArmSim} needs a length and a mass rather than a bare inertia, because
     * gravity torque depends on where the mass actually sits. Measure it in CAD; a guess here makes
     * every sim conclusion about kG meaningless.
     */
    public static final double ARM_LENGTH_METERS = 0.5;

    /** Arm mass in kilograms. */
    public static final double ARM_MASS_KG = 3.0;

    /** Whether the sim applies gravity torque. True for an arm; a turret would set this false. */
    public static final boolean SIMULATE_GRAVITY = true;

    /** Angle the arm starts at in sim, in degrees. Usually the stow position. */
    public static final double STARTING_ANGLE_DEGREES = 0.0;

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
   * <p>Four things differ from {@code exampleRoller}, and all four matter:
   *
   * <ol>
   *   <li><b>An absolute encoder.</b> A position mechanism must know where it is at boot without
   *       first being driven into a limit switch. That means a CANcoder, <i>fused</i> with the
   *       rotor so multi-turn travel still tracks — plain {@code RemoteCANcoder} wraps to zero
   *       every encoder turn.
   *   <li><b>Two ratios, not one</b>, split at the encoder.
   *   <li><b>Soft limits enabled.</b> A roller cannot travel too far. A pivot can, and will, into a
   *       hard stop at full current.
   *   <li><b>Brake, not coast.</b> A coasting arm falls.
   * </ol>
   *
   * <p>The IO must configure the CANcoder <b>before</b> applying this, and must register the
   * encoder's Position and Velocity at 50 Hz before {@code optimizeBusUtilization()} — see {@code
   * ExampleArmIOReal.configureEncoder()} for that trap.
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
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // a coasting arm falls
    config.MotorOutput.Inverted =
        INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // ---- Soft limits ----
    // Degrees in, rotations out — the one conversion boundary.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        degreesToRotations(FORWARD_SOFT_LIMIT_DEGREES);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        degreesToRotations(REVERSE_SOFT_LIMIT_DEGREES);

    // ---- Feedback ----
    // Connects the motor to the CANcoder as a remote, FUSED sensor: the motor fuses the CANcoder's
    // absolute position with its own rotor, so the mechanism knows where it is at boot AND
    // multi-turn tracking still works while running.
    //
    // Fluent .withX() style rather than field assignment, so this reads as one clearly-scoped
    // operation and cannot be half-applied.
    config.Feedback.withFeedbackRemoteSensorID(Constants.CanIds.EXAMPLE_ARM_ENCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO)
        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO);

    // ---- Gains ----
    config.Slot0.kS = getKS();
    config.Slot0.kG = getKG();
    config.Slot0.kP = getKP();
    config.Slot0.kD = getKD();

    // Arm_Cosine, because gravity torque on a pivoting mechanism varies with the cosine of its
    // angle — peaking horizontal, vanishing vertical. An elevator fights the same weight
    // everywhere and uses Elevator_Static; see exampleLift.
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Where horizontal is. See GRAVITY_HORIZONTAL_OFFSET_DEGREES — leaving this at zero when zero
    // is the stow position puts the gravity compensation peak in the wrong place.
    config.Slot0.GravityArmPositionOffset =
        degreesToRotations(GRAVITY_HORIZONTAL_OFFSET_DEGREES.get());

    // Take the sign of kS from the closed-loop error rather than measured velocity. At rest on a
    // setpoint the velocity is near zero and noisy, so UseVelocitySign flips and the mechanism
    // chatters — which on an arm holding position is a constant audible buzz.
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
