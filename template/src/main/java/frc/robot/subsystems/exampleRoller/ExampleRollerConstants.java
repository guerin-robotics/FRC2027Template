package frc.robot.subsystems.exampleRoller;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import frc.lib.MotorSpecs;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.roller.RollerSettings;
import frc.lib.mechanism.roller.RollerSimModel;
import frc.robot.Constants;

/**
 * Everything that describes how this <b>velocity-controlled</b> mechanism is built.
 *
 * <h2>THIS FILE DOES NOT COMPILE ON PURPOSE</h2>
 *
 * <p>Two values cannot be guessed, so their <i>declarations</i> are commented out while the code
 * below still <i>uses</i> them. The compiler then names each one, once, as an error:
 *
 * <pre>
 * Constants.CanIds.EXAMPLE_ROLLER_MOTOR   the CAN ID, added to Constants.CanIds
 * GEAR_RATIO                              total reduction, motor to mechanism
 * </pre>
 *
 * <p>Expect exactly those two and nothing else. Anything more is rot in the scaffold.
 *
 * <p>Commenting out the <i>uses</i> instead would compile, and would be worse. Phoenix defaults
 * {@code SensorToMechanismRatio} to 1.0, so a config that silently skipped it would report motor
 * rotations while every setpoint, tolerance and gain here assumed mechanism rotations. That is a
 * confidently wrong robot, which is much harder to notice than one that refuses to build.
 *
 * <p>Everything else has a defensible default. Those are safe starting points, not measurements.
 *
 * <h2>What does NOT go in this file</h2>
 *
 * <p><b>Setpoints.</b> This file says how the mechanism is <i>built</i> — ratio, limits, gains, the
 * sim model, how close counts as at speed. What it is <i>commanded to</i> — target velocities —
 * belongs in {@code Constants.Setpoints}, with every other mechanism's.
 *
 * <p>The test is whether a driver might ask you to change it between matches. "Run the intake a
 * little faster" should send you to one file, not on a hunt across every subsystem package. A
 * <i>tolerance</i> stays here: how close counts as "at speed" is a property of the mechanism, not a
 * knob the drive team turns.
 *
 * <p><b>CAN IDs.</b> {@code Constants.CanIds}, so every ID on the robot is visible in one table.
 *
 * <h2>Units</h2>
 *
 * <p>Gains are in <b>amps</b>, not volts — every closed loop here runs {@code *TorqueCurrentFOC}.
 * Read {@code docs/characterization-and-tuning.md} before touching one.
 */
public final class ExampleRollerConstants {

  private ExampleRollerConstants() {}

  /** Log key, alert prefix and tunable prefix. One name everywhere, so one grep finds all of it. */
  public static final String NAME = "ExampleRoller";

  /** Drives the speed arithmetic below and the simulation model, so the two cannot disagree. */
  public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;

  // TODO: supply the total reduction from motor to mechanism, then uncomment.
  // A 3:1 followed by a 2:1 is 6.0. Greater than 1 means the mechanism turns slower than the motor.
  //
  // public static final double GEAR_RATIO = 6.0;

  /**
   * How the motor group is configured.
   *
   * <p>{@link MotorConfig} refuses to build without the values that have no safe default, so the
   * absence of a current limit here would be a startup exception naming this mechanism rather than
   * a motor that quietly boots with no protection.
   */
  public static final MotorConfig CONFIG =
      MotorConfig.builder(NAME, MechanismKind.ROLLER)
          // TODO: add the ID to Constants.CanIds and pick the bus deliberately. RIO_BUS here, or
          // TunerConstants.kCANBus for the CANivore — swerve owns that one, and anything else
          // placed there needs a documented reason.
          .canId(Constants.CanIds.EXAMPLE_ROLLER_MOTOR, Constants.CanIds.RIO_BUS)
          .sensorToMechanismRatio(GEAR_RATIO)

          // ---- Current limits ----
          //
          // Safety values, not tuning knobs (.claude/rules/00-safety.md). Supply bounds brownout
          // risk; stator bounds heating and stall torque. At low speed under load supply is only a
          // fraction of stator, because the controller is chopping.
          //
          // 80 A stator here where exampleArm and exampleLift start at 40, and the difference is
          // not caution — it is that a roller has nowhere to slam. A position mechanism given a
          // setpoint it cannot reach drives into its own hard stop and holds there; a roller given
          // a velocity it cannot reach simply spins slower.
          .supplyCurrentLimit(40.0)
          .statorCurrentLimit(80.0)

          // ---- Control ----
          //
          // Zero gains are the honest starting point for a mechanism nobody has characterized.
          // Under TorqueCurrentFOC, kP is amps per rotation-per-second of error.
          .gains(Gains.zero())

          // Cruise velocity is meaningless under MotionMagicVelocity — the commanded velocity is
          // the target and this only governs the ramp to it. rampOnly() says that deliberately.
          // Keep the acceleration honest: the jam detector's dwell has to exceed spin-up time.
          .motionProfile(MotionProfile.rampOnly(RotationsPerSecondPerSecond.of(50.0)))
          .build();

  /**
   * Theoretical top speed. Not a target — a ceiling.
   *
   * <p>Free speed is the motor with nothing attached; a real mechanism carries load, friction and a
   * current limit. 80% is an optimistic working figure. It is also the sanity check on the gear
   * ratio: if this is nowhere near what the mechanism needs to do, the ratio is wrong, and finding
   * that here beats finding it on the practice field.
   *
   * <p>Read off {@link #CONFIG} rather than off {@code GEAR_RATIO} directly, so this is computed
   * from the ratio the device was actually given and the two cannot drift apart.
   */
  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(CONFIG.rotorToMechanismRatio());

  /**
   * How this mechanism behaves: what counts as at speed, and what a jam looks like.
   *
   * <p>Delete the jam detection for a flywheel spinning in free air — it has no jam signature to
   * look for, and "this cannot jam" should mean the detector is absent rather than merely quiet.
   * Keep it for anything that can stall against a game piece: a roller, feeder, intake or
   * transport. 2026 ran its rollers open-loop with no feedback and jams were <b>silent</b>.
   */
  public static final RollerSettings SETTINGS =
      RollerSettings.of(RPM.of(100))
          .withJamDetection(
              new RollerSettings.JamDetection(
                  // Below this the mechanism is idle and cannot be jammed.
                  RPM.of(500),
                  // Fraction of the commanded speed below which it counts as "not turning".
                  0.2,
                  // Stator, not supply — at the low-speed high-load condition that defines a jam,
                  // supply sits much closer to the noise floor.
                  edu.wpi.first.units.Units.Amps.of(45),
                  // MUST exceed spin-up time. During a normal spin-up the command is high, the
                  // measurement is low and the current is high — exactly the jam signature — so a
                  // shorter dwell fires on every single start. Check it against the acceleration
                  // above, and raise it if that ever drops.
                  Seconds.of(0.5)));

  /**
   * The physics simulation obeys.
   *
   * <p>The gearbox is sized from {@link #CONFIG} so a two-motor mechanism is simulated as two
   * motors; simulated as one it reaches half the acceleration and every gain found against it is
   * wrong.
   *
   * <p>The moment of inertia wants a CAD mass-properties number. A guess makes the mechanism spin
   * up at the wrong rate and nothing else — fine for checking command logic, useless for tuning, so
   * know which one you are doing.
   */
  public static final RollerSimModel SIM =
      new RollerSimModel(MOTOR.gearbox(CONFIG.motorCount()), KilogramSquareMeters.of(0.004));
}
