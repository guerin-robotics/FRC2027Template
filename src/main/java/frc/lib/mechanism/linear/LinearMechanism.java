package frc.lib.mechanism.linear;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.mechanism.Mechanism;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.MotorConfig.SoftLimits;
import frc.lib.mechanism.MotorIO;
import frc.lib.mechanism.MotorIOTalonFX;
import frc.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * A linear position mechanism: elevator, lift, extension.
 *
 * <p>Anything where the question is "where is it" and the answer is a distance. Gravity is constant
 * rather than angle-dependent, which is why {@link MechanismKind#LINEAR} configures {@code
 * Elevator_Static} compensation — an elevator's {@code kG} holds the same current at every height.
 *
 * <h2>Everything outside this class is in distance</h2>
 *
 * <p>The device works in mechanism rotations; commands, tolerances and setpoints work in inches.
 * {@link LinearGeometry} is the only place that conversion happens, and it is shared with the
 * constants file that wrote the travel bounds, the visualizer that draws them and the simulation
 * that enforces them. Four consumers of one formula, rather than the same arithmetic written out
 * four times as it was in 2026.
 *
 * <h2>Zeroing</h2>
 *
 * <p>An elevator on the motor's internal encoder has no idea where it is at boot — position reads
 * zero wherever the carriage happens to be sitting. {@link #zeroAt} is how a zeroing routine lands
 * its answer; {@code LinearCommands.zeroAtHardStop} is the routine itself. A mechanism with a fused
 * CANcoder needs neither.
 */
public class LinearMechanism extends Mechanism {

  private final LinearSettings settings;
  private final LinearGeometry geometry;
  private final SoftLimits limits;
  private final LoggedTunableNumber toleranceInches;
  private final LinearVisualizer visualizer;

  /** Last commanded height, after clamping. What {@link #isAtPosition()} compares against. */
  private Distance goalPosition;

  private boolean closedLoop = false;

  protected LinearMechanism(MotorConfig config, LinearSettings settings, MotorIO io) {
    super(config, io);

    if (config.kind() != MechanismKind.LINEAR) {
      throw new IllegalArgumentException(
          "LinearMechanism '"
              + config.name()
              + "' was given a "
              + config.kind()
              + " config. The kind decides soft-limit requirements, the gravity model and the"
              + " static-feedforward sign, so a mismatch is not cosmetic.");
    }

    this.settings = settings;
    this.geometry = settings.geometry();
    // Present by construction: the builder will not produce a LINEAR config without them.
    this.limits = config.softLimits().orElseThrow();
    this.toleranceInches =
        new LoggedTunableNumber(
            config.name() + "/ToleranceInches", settings.positionTolerance().in(Inches));
    this.visualizer = new LinearVisualizer(config.name(), geometry, limits);

    // Start the goal at the bottom of travel. An elevator's reverse bound is where it rests when
    // nothing has commanded it, so this is both a safe default and an honest one.
    this.goalPosition = geometry.distanceFor(Rotations.of(limits.reverseRotations()));
  }

  /** Real hardware. */
  public static LinearMechanism real(MotorConfig config, LinearSettings settings) {
    return new LinearMechanism(config, settings, new MotorIOTalonFX(config));
  }

  /** Log replay. The IO does nothing; AdvantageKit feeds the inputs class from the log. */
  public static LinearMechanism replay(MotorConfig config, LinearSettings settings) {
    return new LinearMechanism(config, settings, new MotorIO() {});
  }

  /** Physics simulation, with the real gains running on a simulated Talon. */
  public static LinearMechanismSim sim(
      MotorConfig config, LinearSettings settings, LinearSimModel model) {
    return new LinearMechanismSim(config, settings, model);
  }

  @Override
  public void periodic() {
    super.periodic();

    Distance height = getHeight();
    Logger.recordOutput(name + "/GoalInches", goalPosition.in(Inches));
    Logger.recordOutput(name + "/HeightInches", height.in(Inches));
    Logger.recordOutput(name + "/VelocityInchesPerSec", getLinearVelocity().in(InchesPerSecond));
    Logger.recordOutput(name + "/AtPosition", isAtPosition());

    visualizer.update(height, goalPosition, isAtPosition());
  }

  // ============================================================================================
  // Control
  // ============================================================================================

  /**
   * Drives to a height along the Motion Magic profile.
   *
   * <p>Clamped to the configured travel bounds first, and a clamped request is logged. On an
   * elevator this matters more than on an arm: the top of travel is usually a hard stop the
   * carriage can be driven into with the full current limit behind it.
   */
  public void setPosition(Distance height) {
    goalPosition = clampToLimits(height);
    closedLoop = true;
    io.setPosition(geometry.rotationsFor(goalPosition));
  }

  /** Drives to a height with no motion profile. For short corrective moves only. */
  public void setUnprofiledPosition(Distance height) {
    goalPosition = clampToLimits(height);
    closedLoop = true;
    io.setUnprofiledPosition(geometry.rotationsFor(goalPosition));
  }

  private Distance clampToLimits(Distance requested) {
    double rotations = geometry.rotationsFor(requested).in(Rotations);
    double clamped =
        MathUtil.clamp(rotations, limits.reverseRotations(), limits.forwardRotations());
    Logger.recordOutput(name + "/GoalWasClamped", clamped != rotations);
    if (clamped != rotations) {
      Logger.recordOutput(name + "/RequestedGoalInches", requested.in(Inches));
    }
    return geometry.distanceFor(Rotations.of(clamped));
  }

  /**
   * Tells the mechanism that the carriage is currently at {@code height}.
   *
   * <p>Where a zeroing routine lands its answer. Has no effect on a mechanism whose feedback comes
   * from a fused absolute encoder, because there the encoder already knows.
   *
   * <p>The goal is reset to the new height at the same time. Without that, a zeroing routine that
   * ran while some other height was commanded would leave the mechanism believing it had a metre to
   * travel the moment the loop closed again.
   */
  public void zeroAt(Distance height) {
    io.setEncoderPosition(geometry.rotationsFor(height));
    goalPosition = height;
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

  /** Carriage height. */
  public Distance getHeight() {
    return geometry.distanceFor(inputs.position);
  }

  /** Carriage speed. */
  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(
        geometry.travelPerRotation().in(Meters) * inputs.velocity.in(RotationsPerSecond));
  }

  /** The height last commanded, after clamping to the travel bounds. */
  public Distance getGoalPosition() {
    return goalPosition;
  }

  /** True while a closed-loop position is being commanded. */
  public boolean isClosedLoop() {
    return closedLoop;
  }

  /** True when the carriage is within tolerance of its goal. */
  public boolean isAtPosition() {
    return nearGoal(goalPosition, Inches.of(toleranceInches.get()));
  }

  /** Within {@code tolerance} of {@code target}, for a caller needing its own tolerance. */
  public boolean nearGoal(Distance target, Distance tolerance) {
    return MathUtil.isNear(getHeight().in(Inches), target.in(Inches), tolerance.in(Inches));
  }

  /** The travel bounds, as heights. */
  public Distance getMinHeight() {
    return geometry.distanceFor(Rotations.of(limits.reverseRotations()));
  }

  /** The travel bounds, as heights. */
  public Distance getMaxHeight() {
    return geometry.distanceFor(Rotations.of(limits.forwardRotations()));
  }

  /** The drum and rigging geometry this mechanism was built with. */
  public LinearGeometry geometry() {
    return geometry;
  }

  /** The travel bounds this mechanism was configured with, in mechanism rotations. */
  public SoftLimits limits() {
    return limits;
  }

  /** The settings this mechanism was built with. */
  public LinearSettings settings() {
    return settings;
  }
}
