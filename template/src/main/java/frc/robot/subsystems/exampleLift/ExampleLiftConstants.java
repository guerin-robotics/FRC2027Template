package frc.robot.subsystems.exampleLift;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Distance;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.linear.LinearGeometry;
import frc.lib.mechanism.linear.LinearSettings;
import frc.lib.mechanism.linear.LinearSimModel;
import frc.lib.util.MotorSpecs;
import frc.robot.Constants;

/**
 * Everything that describes how this <b>linear position</b> mechanism is built.
 *
 * <h2>THIS FILE DOES NOT COMPILE ON PURPOSE</h2>
 *
 * <p>Five values cannot be guessed, so their <i>declarations</i> are commented out while the code
 * below still <i>uses</i> them. The compiler names each one as an error:
 *
 * <pre>
 * Constants.CanIds.EXAMPLE_LIFT_MOTOR   CAN ID
 * GEAR_RATIO                            total reduction, motor to drum
 * DRUM_PITCH_DIAMETER                   where the rope, belt or chain actually rides
 * STAGE_COUNT                           rigging multiplier
 * MAX_TRAVEL                            travel bound
 * </pre>
 *
 * <p>Expect exactly those five and nothing else.
 *
 * <p><b>Never guess the drum diameter or the stage count.</b> Both scale every height the mechanism
 * reports, so wrong values produce a subsystem that compiles, runs, logs plausible numbers and is
 * wrong everywhere at once — much harder to notice than a mechanism that refuses to move. The pitch
 * diameter is where the rope <i>rides</i>, which is the drum plus one rope thickness, not the
 * number stamped on the part. The stage count is what you get by counting the rigging on the
 * physical mechanism, not by trusting a CAD file called "two-stage".
 *
 * <p>See {@code ExampleRollerConstants} for what does not belong in this file, and for the note
 * about gains being in amps.
 */
public final class ExampleLiftConstants {

  private ExampleLiftConstants() {}

  /** Log key, alert prefix and tunable prefix. */
  public static final String NAME = "ExampleLift";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  // TODO: supply the total reduction from motor to drum, then uncomment.
  //
  // public static final double GEAR_RATIO = 12.0;

  // TODO: measure the drum and count the stages, then uncomment both.
  //
  // public static final Distance DRUM_PITCH_DIAMETER = Inches.of(1.5);
  // public static final int STAGE_COUNT = 2;

  // TODO: measure the usable travel from the bottom stop, then uncomment.
  //
  // public static final Distance MAX_TRAVEL = Inches.of(40);

  /**
   * The single place a drum rotation becomes carriage travel.
   *
   * <p>Shared by the travel bounds below, the mechanism reporting heights, the visualizer drawing
   * them and the simulation sizing its effective radius. One formula, four consumers — which is why
   * they cannot disagree. In 2026 this arithmetic was written out separately wherever it was
   * needed.
   */
  public static final LinearGeometry GEOMETRY =
      new LinearGeometry(DRUM_PITCH_DIAMETER, STAGE_COUNT);

  /** Where the carriage sits against the bottom stop. Measure it; it is not always exactly zero. */
  public static final Distance MIN_HEIGHT = Inches.of(0);

  /** How the motor group is configured. */
  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.LINEAR)
          // TODO: add the ID to Constants.CanIds and pick the bus deliberately.
          .canId(Constants.CanIds.EXAMPLE_LIFT_MOTOR, Constants.CanIds.RIO_BUS)
          .sensorToMechanismRatio(GEAR_RATIO)

          // Travel bounds, converted through the geometry above so the numbers a human writes are
          // inches and the numbers the device gets are rotations, with one conversion between them.
          .softLimits(GEOMETRY.rotationsFor(MIN_HEIGHT), GEOMETRY.rotationsFor(MAX_TRAVEL))

          // Deliberately weak, for the same reason as the arm: a position mechanism given an
          // unreachable setpoint drives into its hard stop and holds there. An elevator has more
          // mass behind it than an arm does. Raise these once the mechanism is known to be sane.
          .supplyCurrentLimit(30.0)
          .statorCurrentLimit(40.0)

          // Zero gains, including kG. Find kG the same way as on the arm: bring the carriage up on
          // open-loop current and note what just holds it. MechanismKind.LINEAR has already
          // configured Elevator_Static, so that current is constant at every height.
          .gains(Gains.zero())
          .motionProfile(
              MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)))
          .build();

  /** How close counts as there, and how a rotation becomes travel. */
  public static final LinearSettings SETTINGS = new LinearSettings(Inches.of(0.5), GEOMETRY);

  /**
   * The physics simulation obeys.
   *
   * <p>The carriage mass is everything the motors lift: carriage, the stages above it, and whatever
   * it is holding. Worth a CAD number, because {@code kG} is proportional to it.
   *
   * <p>The gearbox is sized from {@link #CONFIG}, so adding a follower to the config makes the
   * simulation a two-motor elevator without anything else being changed. That matters: simulated as
   * one motor it reaches half the acceleration and every gain found against it is wrong.
   */
  public static final LinearSimModel SIM =
      new LinearSimModel(MOTOR.gearbox(CONFIG.motorCount()), Pounds.of(15), MIN_HEIGHT, true);

  // ============================================================================================
  // FOLLOWER — the common case for a lift, so it is spelled out
  // ============================================================================================
  //
  // Two motors on one gearbox is the usual elevator. Add one line to the builder above:
  //
  //   .follower(Constants.CanIds.EXAMPLE_LIFT_FOLLOWER, true)
  //
  // The boolean is `opposed`: true when the follower is physically mounted facing the opposite way
  // from the leader. Getting it wrong makes the two motors fight — high current, no motion, and it
  // will cook a gearbox. Verify at low output before running closed-loop.
  //
  // Three things then happen on their own, which is the point of the follower living in the config
  // rather than in an IO file:
  //
  //   - the follower is configured identically and logged as its own group, with its own connected
  //     flag and its own sticky faults, so a dead follower is distinguishable from an underpowered
  //     leader rather than looking exactly like one
  //   - its supply current joins the battery report, so a two-motor mechanism is accounted as two
  //   - MOTOR.gearbox(CONFIG.motorCount()) above becomes a two-motor gearbox, so the simulation
  //     gets the acceleration right without a second edit
  //
  // Follower signals default to 4 Hz, which is a reasonable rate — enough to see it is alive,
  // drawing current and not overheating. Pass .followerSignalHz(50.0) if you want more. The point
  // is that 4 Hz is now a choice: in 2026 the intake roller's follower signals landed there by
  // omission, and the data looked present while being a quarter second old.
  //
  // ZEROING INTERACTS WITH THIS. LinearCommands.zeroAtHardStop drives the leader with an
  // open-loop voltage; the follower mirrors it in hardware, so the same voltage puts roughly twice
  // the force into the hard stop. Reduce the zeroing voltage when you add the second motor.
}
