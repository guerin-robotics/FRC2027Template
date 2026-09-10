package frc.lib.mechanism.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.mechanism.Mechanism;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.MotorIO;
import frc.lib.mechanism.MotorIOTalonFX;
import frc.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * A velocity-controlled mechanism: roller, flywheel, feeder, transport, intake.
 *
 * <p>Anything that spins, where the question is "how fast is it turning" and never "where is it".
 * In 2026 that covered the flywheel, the prestage, both feeders, the transport and the intake
 * roller — six subsystems that were six copies of this class with different names.
 *
 * <p><b>If the mechanism has to reach a position and hold it, this is the wrong class.</b> Use
 * {@code RotaryMechanism} for anything that pivots, {@code LinearMechanism} for anything that
 * travels in a line.
 *
 * <h2>No visualizer, deliberately</h2>
 *
 * <p>A spinning drum has no position worth drawing, and a ligament turning at 6000 RPM sampled at
 * 50 Hz aliases into a bar that appears to drift slowly backward. The velocity plot beside {@code
 * closedLoopReference} already answers every question a picture would, and answers it better. The
 * two position kinds do ship visualizers, because for those it earns its place.
 */
public class RollerMechanism extends Mechanism {

  private final RollerSettings settings;

  private final LoggedTunableNumber toleranceRpm;
  private final LoggedTunableNumber jamMinCommandedRpm;
  private final LoggedTunableNumber jamVelocityFraction;
  private final LoggedTunableNumber jamStatorAmps;

  /**
   * Debounces the jam condition.
   *
   * <p>Null when the mechanism has no jam detection configured, which is the honest representation
   * of "this cannot jam" — a flywheel in free air has no jam signature to look for.
   */
  private final Debouncer jamDebounce;

  /** Last commanded velocity, so {@link #isAtVelocity()} has something to compare against. */
  private AngularVelocity goalVelocity = RotationsPerSecond.of(0);

  private boolean jammed = false;
  private boolean wasJammed = false;
  private int jamCount = 0;

  protected RollerMechanism(MotorConfig config, RollerSettings settings, MotorIO io) {
    super(config, io);

    if (config.kind() != MechanismKind.ROLLER) {
      throw new IllegalArgumentException(
          "RollerMechanism '"
              + config.name()
              + "' was given a "
              + config.kind()
              + " config. The kind decides soft-limit requirements, the gravity model and the"
              + " static-feedforward sign, so a mismatch is not cosmetic.");
    }

    this.settings = settings;
    this.toleranceRpm =
        new LoggedTunableNumber(
            config.name() + "/ToleranceRpm", settings.velocityTolerance().in(RPM));

    if (settings.jam().isPresent()) {
      RollerSettings.JamDetection jam = settings.jam().get();
      jamMinCommandedRpm =
          new LoggedTunableNumber(
              config.name() + "/Jam/MinCommandedRpm", jam.minCommandedVelocity().in(RPM));
      jamVelocityFraction =
          new LoggedTunableNumber(config.name() + "/Jam/VelocityFraction", jam.velocityFraction());
      jamStatorAmps =
          new LoggedTunableNumber(
              config.name() + "/Jam/StatorAmps", jam.statorThreshold().in(Amps));
      jamDebounce = new Debouncer(jam.dwell().in(Seconds), Debouncer.DebounceType.kRising);
    } else {
      jamMinCommandedRpm = null;
      jamVelocityFraction = null;
      jamStatorAmps = null;
      jamDebounce = null;
    }
  }

  /** Real hardware. */
  public static RollerMechanism real(MotorConfig config, RollerSettings settings) {
    return new RollerMechanism(config, settings, new MotorIOTalonFX(config));
  }

  /**
   * Log replay. The IO does nothing; AdvantageKit feeds the inputs class straight from the log.
   *
   * <p>The mechanism's own logic still runs, which is the point — a tolerance or a jam threshold
   * changed after a match can be replayed against the log from that match.
   */
  public static RollerMechanism replay(MotorConfig config, RollerSettings settings) {
    return new RollerMechanism(config, settings, new MotorIO() {});
  }

  /** Physics simulation, with the real gains running on a simulated Talon. */
  public static RollerMechanismSim sim(
      MotorConfig config, RollerSettings settings, RollerSimModel model) {
    return new RollerMechanismSim(config, settings, model);
  }

  @Override
  public void periodic() {
    super.periodic();

    updateJamDetection();

    // The goal is logged beside the measurement. Without it a log shows the mechanism at 2000 RPM
    // and cannot say whether that was right.
    Logger.recordOutput(name + "/GoalRpm", goalVelocity.in(RPM));
    Logger.recordOutput(name + "/VelocityRpm", inputs.velocity.in(RPM));
    Logger.recordOutput(name + "/AtVelocity", isAtVelocity());
  }

  // ============================================================================================
  // Control
  // ============================================================================================

  /** Closed-loop velocity. The primary control path for this kind of mechanism. */
  public void setVelocity(AngularVelocity velocity) {
    goalVelocity = velocity;
    io.setVelocity(velocity);
  }

  /** What was last asked for. Zero after any open-loop command or a stop. */
  public AngularVelocity getGoalVelocity() {
    return goalVelocity;
  }

  // The open-loop paths clear the goal, which the jam detector depends on. Without this it keeps
  // comparing against a setpoint nobody is commanding any more, and reports a jam the moment an
  // open-loop command runs the mechanism slower than the last closed-loop request.

  @Override
  public void setVoltage(Voltage volts) {
    goalVelocity = RotationsPerSecond.of(0);
    super.setVoltage(volts);
  }

  @Override
  public void setTorqueCurrent(Current amps) {
    goalVelocity = RotationsPerSecond.of(0);
    super.setTorqueCurrent(amps);
  }

  @Override
  public void setDutyCycle(double fraction) {
    goalVelocity = RotationsPerSecond.of(0);
    super.setDutyCycle(fraction);
  }

  @Override
  public void stop() {
    goalVelocity = RotationsPerSecond.of(0);
    super.stop();
  }

  // ============================================================================================
  // State
  // ============================================================================================

  /**
   * True when the mechanism is within tolerance of the requested velocity.
   *
   * <p>Compared in RPM, because that is the unit the tolerance is written in and the unit anyone
   * reading the log is thinking in.
   *
   * <p>A pure predicate: it is logged from {@link #periodic()}, never from inside here. Recording
   * from a getter means the channel updates only as often as something happens to call it, so a gap
   * in the log reads as "the robot stopped checking" rather than "nothing asked".
   */
  public boolean isAtVelocity() {
    return Math.abs(inputs.velocity.in(RPM) - goalVelocity.in(RPM)) < toleranceRpm.get();
  }

  /**
   * True while the mechanism is jammed. Clears when the jam does.
   *
   * <p>Always false on a mechanism with no jam detection configured. Safe to poll from anywhere —
   * the debouncer is advanced exactly once per loop in {@link #periodic()}, never from in here. A
   * debouncer advances its timer every time it is polled, so calling it from a getter would make
   * the dwell depend on how many callers happened to ask: a command polling it and a {@code
   * FaultMonitor} condition reading it in the same loop would trip it in half the time.
   */
  public boolean isJammed() {
    return jammed;
  }

  /** How many distinct jams since boot. Worth an alert when it climbs across a match. */
  public int getJamCount() {
    return jamCount;
  }

  /**
   * Detects jams, once per loop.
   *
   * <p>Detection lives here; the response does not. Reversing to clear a jam is a policy decision
   * with a game-strategy answer — it can eject a piece the driver wanted — so the mechanism owns
   * the signal and a command factory owns what to do about it.
   */
  private void updateJamDetection() {
    if (jamDebounce == null) {
      return;
    }

    boolean raw = isJamConditionPresent();
    jammed = jamDebounce.calculate(raw);
    if (jammed && !wasJammed) {
      jamCount++;
    }
    wasJammed = jammed;

    Logger.recordOutput(name + "/Jammed", jammed);
    Logger.recordOutput(name + "/JamCount", jamCount);
    // The raw conjunction too. Comparing it against Jammed in a log is how you tell "threshold too
    // low, it keeps flickering" from "dwell too long, it never latches".
    Logger.recordOutput(name + "/JamConditionRaw", raw);
  }

  private boolean isJamConditionPresent() {
    double commandedRpm = Math.abs(goalVelocity.in(RPM));
    if (commandedRpm < jamMinCommandedRpm.get()) {
      return false; // idle is not jammed
    }
    boolean notTurning =
        Math.abs(inputs.velocity.in(RPM)) < commandedRpm * jamVelocityFraction.get();
    boolean workingHard = inputs.statorAmps.in(Amps) > jamStatorAmps.get();
    return notTurning && workingHard;
  }

  /** The settings this mechanism was built with. */
  public RollerSettings settings() {
    return settings;
  }
}
