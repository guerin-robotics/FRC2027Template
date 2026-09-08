package frc.robot.subsystems.exampleArm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.Feedback;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.rotary.RotarySettings;
import frc.lib.mechanism.rotary.RotarySimModel;
import frc.lib.util.MotorSpecs;
import frc.robot.Constants;

/**
 * Everything that describes how this <b>rotary position</b> mechanism is built.
 *
 * <h2>THIS FILE DOES NOT COMPILE ON PURPOSE</h2>
 *
 * <p>Seven values cannot be guessed, so their <i>declarations</i> are commented out while the code
 * below still <i>uses</i> them. The compiler names each one as an error:
 *
 * <pre>
 * Constants.CanIds.EXAMPLE_ARM_MOTOR      CAN ID
 * Constants.CanIds.EXAMPLE_ARM_ENCODER    CAN ID
 * ROTOR_TO_SENSOR_RATIO                   motor rotations per encoder rotation
 * SENSOR_TO_MECHANISM_RATIO               encoder rotations per mechanism rotation
 * MAGNET_OFFSET_ROTATIONS                 CANcoder calibration
 * FORWARD_SOFT_LIMIT                      travel bound
 * REVERSE_SOFT_LIMIT                      travel bound
 * </pre>
 *
 * <p>Expect exactly those seven values. {@code REVERSE_SOFT_LIMIT} is named twice, because the
 * simulation also starts the arm there; every other one is named once. It used to be eight — the
 * total gear ratio is now derived from the two ratios by {@code
 * MotorConfig.rotorToMechanismRatio()}, so there is no longer a third number that has to agree with
 * the other two.
 *
 * <p>The soft limits are the ones worth being careful about. A missing travel bound does not fail:
 * it drives the arm into a hard stop at whatever the current limit allows and holds it there. The
 * builder refuses to produce a {@code ROTARY} config without them for exactly that reason, so
 * leaving them out is a startup exception naming this mechanism rather than a broken gearbox.
 *
 * <h2>Where is the encoder?</h2>
 *
 * <p>Answer that before writing either ratio. "Does it have a CANcoder" is not the question; "what
 * does the CANcoder turn relative to" is.
 *
 * <ul>
 *   <li><b>CANcoder on the mechanism shaft.</b> A hex-bore encoder on the shaft the arm actually
 *       pivots about. The encoder already reads mechanism rotations, so {@code
 *       SENSOR_TO_MECHANISM_RATIO} is 1.0 and {@code ROTOR_TO_SENSOR_RATIO} is the whole reduction.
 *   <li><b>CANcoder on an intermediate shaft.</b> Split the reduction at the encoder: motor to
 *       encoder is {@code ROTOR_TO_SENSOR_RATIO}, encoder to mechanism is {@code
 *       SENSOR_TO_MECHANISM_RATIO}.
 *   <li><b>No CANcoder.</b> Drop the {@code .encoder(...)} call entirely, put the whole reduction
 *       in {@code SENSOR_TO_MECHANISM_RATIO}, and add a zeroing routine — the motor's internal
 *       encoder reads zero at boot wherever the arm happens to be resting.
 * </ul>
 *
 * <p>Under {@code FUSED_CANCODER} the rotor ratio is <b>load-bearing</b>: it is how the rotor
 * extends the absolute reading, so a wrong value puts the fusion off by the whole reduction while
 * every configuration call still reports success.
 *
 * <p>See {@code ExampleRollerConstants} for what does not belong in this file, and for the note
 * about gains being in amps.
 */
public final class ExampleArmConstants {

  private ExampleArmConstants() {}

  /** Log key, alert prefix and tunable prefix. */
  public static final String NAME = "ExampleArm";

  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  // TODO: work out where the encoder sits (see the class javadoc), then uncomment both.
  //
  // public static final double ROTOR_TO_SENSOR_RATIO = 60.0;
  // public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

  // TODO: supply the CANcoder calibration from the CTRE Tuner X wizard, then uncomment.
  // Not guessable, and not transferable between robots or even between encoder swaps.
  //
  // public static final double MAGNET_OFFSET_ROTATIONS = 0.0;

  // TODO: measure the travel bounds on the real mechanism, then uncomment.
  // These are where the arm physically stops, minus a margin. Zero is wherever the encoder
  // calibration put it — usually horizontal, but that is a decision you make, not a given.
  //
  // public static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-5);
  // public static final Angle FORWARD_SOFT_LIMIT = Degrees.of(95);

  /** How the motor group is configured. */
  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROTARY)
          // TODO: add both IDs to Constants.CanIds. The encoder must be on the same bus as the
          // motor — under fusion the motor reads the encoder's position off that bus.
          .canId(Constants.CanIds.EXAMPLE_ARM_MOTOR, Constants.CanIds.RIO_BUS)
          .encoder(
              Feedback.FUSED_CANCODER,
              Constants.CanIds.EXAMPLE_ARM_ENCODER,
              MAGNET_OFFSET_ROTATIONS,
              ROTOR_TO_SENSOR_RATIO)
          .sensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO)
          .softLimits(REVERSE_SOFT_LIMIT, FORWARD_SOFT_LIMIT)

          // Deliberately weak. A position mechanism given a setpoint it cannot reach drives into
          // its own hard stop with everything the limit allows, so a new arm starts with barely
          // enough authority to move itself and is raised once it is known to be sane. See
          // docs/new-mechanism-bringup.md.
          .supplyCurrentLimit(30.0)
          .statorCurrentLimit(40.0)

          // Zero gains, including kG. Bring the arm up on open-loop voltage first, find the
          // current that just holds it horizontal, and that is your starting kG — in AMPS.
          // MechanismKind.ROTARY has already configured Arm_Cosine, so kG is scaled by the cosine
          // of the angle without anything further being said here.
          .gains(Gains.zero())

          // WHERE IS THE ARM LEVEL? Answer it before tuning kG.
          //
          // Arm_Cosine scales kG by the cosine of the position it is handed, so it assumes gravity
          // peaks at zero. The soft limits above put this scaffold's zero at horizontal, which is
          // why there is no offset here. If your zero is the stow position instead, uncomment:
          //
          //   .gravityOffset(Degrees.of(20))   // the angle at which the arm is level
          //
          // Left wrong, nothing errors: the pivot gets too little hold current where gravity is
          // strongest and too much where there is none, and it reads in a log as a badly tuned kG
          // rather than as a wrong reference.

          // Slow. An arm's profile is the difference between a smooth move and a mechanism that
          // slams into its own travel limits; start conservative and raise it deliberately.
          .motionProfile(
              MotionProfile.of(RotationsPerSecond.of(0.5), RotationsPerSecondPerSecond.of(1.0)))
          .build();

  /**
   * How close counts as there, and how long the arm is.
   *
   * <p>The length is used for the picture and for the simulation's gravity torque. A rough number
   * draws a usable picture; the simulation wants a real one.
   */
  public static final RotarySettings SETTINGS = new RotarySettings(Degrees.of(2.0), Inches.of(20));

  /**
   * The physics simulation obeys.
   *
   * <p>Mass and length rather than a bare moment of inertia, because gravity torque depends on
   * where the mass actually sits. A guessed centre of mass makes every simulated conclusion about
   * {@code kG} meaningless, and {@code kG} is most of what an arm's loop is fighting.
   *
   * <p>Set the last argument false for a mechanism that turns in a horizontal plane. A turret has
   * an angle but no gravity load, and simulating one teaches a {@code kG} that does not exist.
   */
  public static final RotarySimModel SIM =
      new RotarySimModel(
          MOTOR.gearbox(CONFIG.motorCount()), Pounds.of(8), REVERSE_SOFT_LIMIT, true);
}
