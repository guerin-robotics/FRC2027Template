package frc.robot.subsystems.exampleRoller;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import frc.lib.LoggedTunableNumber;
import frc.lib.MotorSpecs;
import frc.robot.Constants;

/**
 * Constants and motor configuration for a <b>VELOCITY-CONTROLLED</b> mechanism.
 *
 * <p>Copy this scaffold for a roller, flywheel, feeder, transport or intake — anything that spins
 * and where the question is "how fast is it turning", never "where is it". In 2026 that covered the
 * flywheel, the prestage, both feeders, the transport and the intake roller.
 *
 * <p><b>If the mechanism has to reach a position and hold it, this is the wrong scaffold.</b> Copy
 * {@code exampleArm} for anything that pivots, {@code exampleLift} for anything that travels in a
 * line.
 *
 * <h2>THIS FILE DOES NOT COMPILE ON PURPOSE</h2>
 *
 * <p>Two values cannot be guessed, so their <i>declarations</i> are commented out while the code
 * below still <i>assigns</i> them. The compiler then names each one, once, as an error:
 *
 * <pre>
 * Constants.CanIds.EXAMPLE_ROLLER_MOTOR   the CAN ID, added to Constants.CanIds
 * GEAR_RATIO                              total reduction, motor to mechanism
 * </pre>
 *
 * <p>Expect exactly those two and nothing else. Anything more is rot in the scaffold.
 *
 * <p>Commenting out the <i>assignments</i> instead would compile, and would be worse. Phoenix
 * defaults {@code SensorToMechanismRatio} to 1.0, so a config that silently skips it reports motor
 * rotations while every setpoint, tolerance and gain in this file assumes mechanism rotations. That
 * is a confidently wrong robot, which is much harder to notice than one that refuses to build.
 *
 * <p>Everything else here has a defensible default. Those are safe starting points, not
 * measurements.
 *
 * <h2>What does NOT go in this file</h2>
 *
 * <p><b>Setpoints.</b> This file describes how the mechanism is <i>built</i> — gains, gear ratio,
 * current limits, tolerance, the sim model. What it is <i>commanded to</i> — target velocities,
 * voltages — belongs in {@code Constants.Setpoints}, together with every other mechanism's.
 *
 * <p>The test is whether a driver might ask you to change it between matches. "Run the intake a
 * little faster" should send you to one file, not on a hunt across every subsystem package. Note
 * that a <i>tolerance</i> stays here — how close counts as "at speed" is a property of the
 * mechanism, not a knob the drive team turns.
 *
 * <p><b>CAN IDs.</b> {@code Constants.CanIds}, so every ID on the robot is visible in one table.
 *
 * <p>Command factories take setpoints as parameters, and {@code RobotContainer} supplies them from
 * {@code Constants.Setpoints}. See {@code .claude/rules/03-commands.md}.
 *
 * <h2>The config lives here, not in the IO</h2>
 *
 * <p>{@link #getFXConfig()} returns a fully-built {@link TalonFXConfiguration}, setting the same
 * blocks in the same order every mechanism does:
 *
 * <pre>
 * CurrentLimits -&gt; TorqueCurrent -&gt; Voltage -&gt; MotorOutput -&gt; SoftwareLimitSwitch
 *   -&gt; Feedback -&gt; Slot0 -&gt; MotionMagic
 * </pre>
 *
 * <p>That uniformity is the point. When every mechanism's config reads the same, a reviewer
 * comparing two of them sees only the differences that matter. The IO implementation shrinks to one
 * line and holds no numbers at all.
 *
 * <p><b>UNITS:</b> under {@code *TorqueCurrentFOC} the gains are in <b>AMPS</b>, not volts. See
 * {@code docs/characterization-and-tuning.md}.
 */
public class ExampleRollerConstants {

  private ExampleRollerConstants() {}

  /** Log key and alert prefix. One name used everywhere, so a grep finds all of it. */
  public static final String NAME = "ExampleRoller";

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
  // MECHANICAL
  // ==========================================================================================

  /** True if the motor is mounted so positive output produces negative mechanism motion. */
  public static final boolean INVERTED = false;

  // TODO: supply the gearing, then uncomment.
  //
  // GEAR_RATIO is the TOTAL reduction: motor rotations per mechanism rotation, end to end. It is
  // the number that sets top speed, and the one to state when someone asks "what is the ratio".
  //
  // public static final double GEAR_RATIO = ;

  /**
   * Encoder rotations per mechanism rotation.
   *
   * <p>For a velocity mechanism this is always the whole reduction, because the sensor is always
   * the motor's own rotor. A roller has no position worth measuring absolutely, so there is no
   * CANcoder, and nothing sits between the rotor and the gearbox input.
   *
   * <p>That is why this scaffold has no {@code ROTOR_TO_SENSOR_RATIO} to get wrong. A position
   * mechanism has to split the total reduction at wherever the encoder physically sits; see {@code
   * ExampleArmConstants} for that decision and the three cases it breaks into.
   */
  public static final double SENSOR_TO_MECHANISM_RATIO = GEAR_RATIO;

  // ==========================================================================================
  // UNIT CONVERSION — the boundary between team units and Phoenix units
  // ==========================================================================================
  //
  // We declare in RPM and RPM/sec because those are the units a person can reason about with the
  // robot in front of them. Phoenix works in rotations and rotations per second, always.
  //
  // Every conversion lives here so there is exactly one place to check, and each is written with
  // WPILib units rather than a bare /60.0. An inverted magic number is a factor-of-60 error that
  // compiles, deploys, and moves the mechanism — just not the way anyone expected.

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
   * friction and the current limit all take a share. A commanded velocity above this does not fail
   * loudly — the motor simply saturates and never reports at-speed, so every sequence waiting on it
   * runs to its timeout instead. That reads as a tuning problem and is not one.
   *
   * <p>It is also the sanity check on the gear ratio. If this number is nowhere near what the
   * mechanism has to do, the ratio is wrong, and catching that here beats catching it on the
   * practice field.
   */
  public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(GEAR_RATIO);

  // ==========================================================================================
  // MOTION PROFILE
  // ==========================================================================================
  //
  // A velocity mechanism has NO cruise velocity: the commanded velocity IS the cruise. Only
  // acceleration is profiled, and it exists to keep spin-up from being a current step.
  //
  // 9000 RPM/s is close to unlimited on purpose. Spin-up ends up bounded by the current limit and
  // the mechanism's own inertia rather than by this number, which is what you want on a roller —
  // there is no hard stop to slam into and no gravity torque to respect. Lower it only if the
  // current draw at spin-up is causing a brownout, and say so in the commit.
  //
  // Tunable, because a profile is the thing you most want to adjust with the mechanism in front
  // of you, and it is far safer to change than a gain: too slow just wastes time, whereas too
  // much kP oscillates. See docs/tunables.md.

  /** Compiled-in default for {@link #ACCELERATION_RPM_PER_SEC}, in mechanism RPM per second. */
  public static final double ACCELERATION_RPM_PER_SEC_DEFAULT = 9000.0;

  /** Profile acceleration, in mechanism RPM per second. Tunable at runtime. */
  public static final LoggedTunableNumber ACCELERATION_RPM_PER_SEC =
      new LoggedTunableNumber(NAME + "/AccelRpmPerSec", ACCELERATION_RPM_PER_SEC_DEFAULT);

  /**
   * Everything re-applied to the Talon when a dashboard value moves.
   *
   * <p>One entry, because there is no cruise velocity on a velocity mechanism. It stays an array so
   * the IO layer's {@code ifChanged} watch list reads identically across all three scaffolds.
   */
  public static final LoggedTunableNumber[] TUNABLE_PROFILE = {ACCELERATION_RPM_PER_SEC};

  // ==========================================================================================
  // TOLERANCE
  // ==========================================================================================

  /**
   * How close counts as "at velocity", in mechanism RPM.
   *
   * <p>Too tight and the mechanism never reports ready, so every sequence runs to its timeout
   * instead of proceeding. Too loose and it acts before it has spun up. 120 RPM is a starting point
   * for a flywheel in the low thousands; scale it to the mechanism, then check it against a real
   * log rather than leaving it at whatever was copied.
   */
  public static final double VELOCITY_TOLERANCE_RPM = 120.0;

  // ==========================================================================================
  // JAM DETECTION — keep this if the mechanism can stall against a game piece
  // ==========================================================================================
  //
  // Rollers, feeders, intakes and transports can all jam. A flywheel spinning in free air cannot
  // — delete this block and the matching one in ExampleRoller if that is what you are building.
  //
  // 2026 ran its rollers open-loop with no feedback, so jams were SILENT: the mechanism stopped
  // working and nothing in the log said why. This is the cheapest instrumentation on the list and
  // it is the one that was missing. See GUIDE.md section D.3.
  //
  // NONE OF THESE ARE GUESSABLE. They come from logging stator current during a real jam AND
  // during a normal pickup, because telling those two apart is the entire job. Left commented for
  // the same reason GEAR_RATIO is: a plausible-looking wrong threshold compiles and produces a
  // detector that either cries wolf every match or never fires at all.
  //
  //   /** Stator current, in amps, above which the mechanism counts as "working hard". Sits
  //    * BELOW STATOR_CURRENT_LIMIT_AMPS — a jam that already saturated the limit has been a
  //    * jam for a while. */
  //   public static final double JAM_STATOR_CURRENT_AMPS = 55.0;
  //
  //   /** Measured velocity below this FRACTION of commanded counts as "not turning". A
  //    * fraction, not an absolute RPM, so one threshold works at every setpoint. */
  //   public static final double JAM_VELOCITY_FRACTION = 0.25;
  //
  //   /** How long all conditions must hold. MUST EXCEED SPIN-UP TIME — see the note in
  //    * ExampleRoller. Raise it if ACCELERATION_RPM_PER_SEC ever drops. */
  //   public static final double JAM_DEBOUNCE_SECONDS = 0.5;
  //
  //   /** Commanded RPM below which the check is skipped, so a stopped mechanism does not
  //    * trivially satisfy "measured is below 25% of commanded". */
  //   public static final double JAM_MIN_COMMANDED_RPM = 100.0;

  // ==========================================================================================
  // GAINS — REAL ROBOT
  // ==========================================================================================
  //
  // Placeholders. Measure them in this order for a velocity loop: kS -> kV -> kA -> kP. Never
  // start with kP. See docs/characterization-and-tuning.md Part 3.
  //
  // THERE IS NO kG HERE. A roller's mass is balanced about its axis, so gravity does no net work
  // on it and a gravity feedforward has nothing to compensate. exampleArm and exampleLift both
  // have one, and both need it.
  //
  // These are LoggedTunableNumbers so a tuning session does not need a redeploy per iteration.
  // With Constants.tuningMode off they return the defaults below and cost nothing — no dashboard
  // entry is even created.
  //
  // The default in each constructor is what the robot runs in competition. The dashboard value
  // lives only in NetworkTables and is gone at the next reboot, so a session that ends without
  // writing the numbers back into this file and committing them accomplished nothing.
  //
  // These are the gains the TALON runs, on the device. Changing one means a CAN write, which is
  // why ExampleRollerIOReal re-applies Slot0 only when a value actually moves rather than every
  // loop. See docs/tunables.md.

  /** Static friction feedforward — output needed to break the mechanism loose. */
  public static final LoggedTunableNumber KS = new LoggedTunableNumber(NAME + "/kS", 0.0);

  /** Velocity feedforward — output per unit of steady-state velocity. Does most of the work. */
  public static final LoggedTunableNumber KV = new LoggedTunableNumber(NAME + "/kV", 0.0);

  /** Acceleration feedforward — output per unit of acceleration. Governs spin-up. */
  public static final LoggedTunableNumber KA = new LoggedTunableNumber(NAME + "/kA", 0.0);

  /** Proportional gain. Corrects what feedforward misses — it should not be doing the work. */
  public static final LoggedTunableNumber KP = new LoggedTunableNumber(NAME + "/kP", 0.0);

  /** Integral gain. Leave at zero unless you can explain its behavior during a stall. */
  public static final LoggedTunableNumber KI = new LoggedTunableNumber(NAME + "/kI", 0.0);

  /**
   * Derivative gain.
   *
   * <p>Usually zero on a velocity loop. Velocity is already a derivative of the sensor, so kD
   * differentiates it a second time and turns encoder noise into output chatter. It damps overshoot
   * on a position loop, which is a different scaffold.
   */
  public static final LoggedTunableNumber KD = new LoggedTunableNumber(NAME + "/kD", 0.0);

  /** Every real-robot gain, for the {@code ifChanged} watch list in the IO layer. */
  public static final LoggedTunableNumber[] TUNABLE_GAINS = {KS, KV, KA, KP, KI, KD};

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
    public static final DCMotor MOTOR = ExampleRollerConstants.MOTOR.gearbox(MOTOR_COUNT);

    /**
     * Moment of inertia in kg-m squared, at the mechanism.
     *
     * <p>Sets how fast the simulated mechanism accelerates, so a wrong value makes sim-tuned gains
     * meaningless. For a flywheel it is essentially the whole model — worth a CAD number rather
     * than a guess.
     */
    public static final double MOI = 0.001;

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
   * <p>Four things differ from the position scaffolds, and all four follow from "a roller has no
   * position":
   *
   * <ol>
   *   <li><b>No external encoder.</b> Feedback comes off the motor's internal rotor. Nothing needs
   *       to know where a roller <i>is</i>, only how fast it is turning, so there is no CANcoder to
   *       configure and no absolute position to recover at boot.
   *   <li><b>Soft limits off.</b> A roller has no travel to bound, and a soft limit on one silently
   *       stops it once accumulated position drifts past the threshold — a mechanism that works for
   *       two minutes and then quietly refuses to spin.
   *   <li><b>Coast, not brake.</b> A roller that brakes on every release grinds game pieces and
   *       throws away spin-up energy. Nothing falls when it coasts.
   *   <li><b>Acceleration only.</b> The commanded velocity is the cruise, so there is no cruise
   *       velocity to set.
   * </ol>
   *
   * @return a fully-built config, ready for {@code PhoenixUtil.tryUntilOk(5, ...)}
   */
  public static TalonFXConfiguration getFXConfig() {
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
    // Explicitly disabled rather than left at the default, so a reader comparing this config
    // against a position mechanism's sees a decision rather than an omission.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // ---- Feedback ----
    // Internal rotor only. SensorToMechanismRatio makes velocity read in mechanism units, which
    // is what every tolerance and setpoint in this file assumes.
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
    // Acceleration only. RPM/sec in, rotations/sec^2 out.
    config.MotionMagic.MotionMagicAcceleration =
        rpmPerSecToRotationsPerSecSquared(ACCELERATION_RPM_PER_SEC.get());

    return config;
  }
}
