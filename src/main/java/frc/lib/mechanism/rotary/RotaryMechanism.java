package frc.lib.mechanism.rotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.LoggedTunableNumber;
import frc.lib.mechanism.Mechanism;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.MotorConfig.SoftLimits;
import frc.lib.mechanism.MotorIO;
import frc.lib.mechanism.MotorIOTalonFX;
import org.littletonrobotics.junction.Logger;

/**
 * A rotary position mechanism: arm, pivot, hood, turret, wrist.
 *
 * <p>Anything where the question is "where is it" and the answer is an angle. Gravity varies with
 * that angle, which is why {@link MechanismKind#ROTARY} configures {@code Arm_Cosine} gravity
 * compensation by default and why an arm's {@code kG} is not an elevator's.
 *
 * <h2>Travel bounds are not optional here</h2>
 *
 * <p>{@code MotorConfig} refuses to build a rotary config without soft limits, and this class uses
 * them for a second purpose: every commanded angle is clamped to them before it reaches the device,
 * and a request that had to be clamped is logged. The Talon would refuse to travel past the limit
 * anyway, but silently — the mechanism simply stops short and nothing says whether the gains are
 * weak or the setpoint was outside the world.
 */
public class RotaryMechanism extends Mechanism {

  private final RotarySettings settings;
  private final SoftLimits limits;
  private final LoggedTunableNumber toleranceDegrees;
  private final RotaryVisualizer visualizer;

  /** Last commanded angle, after clamping. What {@link #isAtPosition()} compares against. */
  private Angle goalPosition;

  /** True while a closed-loop position is commanded. Cleared by any open-loop path. */
  private boolean closedLoop = false;

  protected RotaryMechanism(MotorConfig config, RotarySettings settings, MotorIO io) {
    super(config, io);

    if (config.kind() != MechanismKind.ROTARY) {
      throw new IllegalArgumentException(
          "RotaryMechanism '"
              + config.name()
              + "' was given a "
              + config.kind()
              + " config. The kind decides soft-limit requirements, the gravity model and the"
              + " static-feedforward sign, so a mismatch is not cosmetic.");
    }

    this.settings = settings;
    // Present by construction: the builder will not produce a ROTARY config without them.
    this.limits = config.softLimits().orElseThrow();
    this.toleranceDegrees =
        new LoggedTunableNumber(
            config.name() + "/ToleranceDegrees", settings.positionTolerance().in(Degrees));
    this.visualizer = new RotaryVisualizer(config.name(), settings, limits);

    // Start the goal at the reverse bound rather than at zero. Zero may be outside the mechanism's
    // travel entirely, and a goal outside the bounds would make isAtPosition() report false from
    // boot until something commanded a real angle.
    this.goalPosition = Rotations.of(limits.reverseRotations());
  }

  /** Real hardware. */
  public static RotaryMechanism real(MotorConfig config, RotarySettings settings) {
    return new RotaryMechanism(config, settings, new MotorIOTalonFX(config));
  }

  /** Log replay. The IO does nothing; AdvantageKit feeds the inputs class from the log. */
  public static RotaryMechanism replay(MotorConfig config, RotarySettings settings) {
    return new RotaryMechanism(config, settings, new MotorIO() {});
  }

  /** Physics simulation, with the real gains running on a simulated Talon. */
  public static RotaryMechanismSim sim(
      MotorConfig config, RotarySettings settings, RotarySimModel model) {
    return new RotaryMechanismSim(config, settings, model);
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(name + "/GoalDegrees", goalPosition.in(Degrees));
    Logger.recordOutput(name + "/PositionDegrees", inputs.position.in(Degrees));
    Logger.recordOutput(name + "/VelocityDegreesPerSec", inputs.velocity.in(DegreesPerSecond));
    Logger.recordOutput(name + "/AtPosition", isAtPosition());

    visualizer.update(inputs.position, goalPosition, isAtPosition());
  }

  // ============================================================================================
  // Control
  // ============================================================================================

  /**
   * Drives to an angle along the Motion Magic profile.
   *
   * <p>The angle is clamped to the configured travel bounds first. A clamped request is logged
   * under {@code GoalWasClamped} — the mechanism stopping short at a bound and the mechanism being
   * unable to reach a reachable setpoint look identical on a position plot, and they need opposite
   * fixes.
   */
  public void setPosition(Angle position) {
    goalPosition = clampToLimits(position);
    closedLoop = true;
    io.setPosition(goalPosition);
  }

  /**
   * Drives to an angle with no motion profile.
   *
   * <p>For short corrective moves where the profile costs more than it buys. Not a way to make the
   * mechanism faster — a profile that is too slow is fixed by raising its acceleration, which stays
   * inside the limits the mechanism was characterized against.
   */
  public void setUnprofiledPosition(Angle position) {
    goalPosition = clampToLimits(position);
    closedLoop = true;
    io.setUnprofiledPosition(goalPosition);
  }

  private Angle clampToLimits(Angle requested) {
    double rotations = requested.in(Rotations);
    double clamped =
        MathUtil.clamp(rotations, limits.reverseRotations(), limits.forwardRotations());
    Logger.recordOutput(name + "/GoalWasClamped", clamped != rotations);
    if (clamped != rotations) {
      Logger.recordOutput(name + "/RequestedGoalDegrees", requested.in(Degrees));
    }
    return Rotations.of(clamped);
  }

  @Override
  public void setVoltage(Voltage volts) {
    closedLoop = false;
    super.setVoltage(volts);
  }

  @Override
  public void stop() {
    closedLoop = false;
    super.stop();
  }

  // ============================================================================================
  // State
  // ============================================================================================

  /** The angle last commanded, after clamping to the travel bounds. */
  public Angle getGoalPosition() {
    return goalPosition;
  }

  /** True while a closed-loop position is being commanded. */
  public boolean isClosedLoop() {
    return closedLoop;
  }

  /**
   * True when the mechanism is within tolerance of its goal.
   *
   * <p>Compared in degrees, because that is the unit the tolerance is written in and the unit
   * anyone reading a log is thinking in.
   */
  public boolean isAtPosition() {
    return nearGoal(goalPosition, Degrees.of(toleranceDegrees.get()));
  }

  /**
   * True when the mechanism is within {@code tolerance} of {@code target}.
   *
   * <p>For a caller that needs a different tolerance than the mechanism's own — a coarse "roughly
   * clear of the frame" check alongside a tight "ready to score" one.
   */
  public boolean nearGoal(Angle target, Angle tolerance) {
    return MathUtil.isNear(inputs.position.in(Degrees), target.in(Degrees), tolerance.in(Degrees));
  }

  /** The travel bounds this mechanism was configured with. */
  public SoftLimits limits() {
    return limits;
  }

  /** The settings this mechanism was built with. */
  public RotarySettings settings() {
    return settings;
  }
}
