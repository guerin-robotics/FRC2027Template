package frc.lib.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A {@link ProfiledPIDController} whose gains and motion constraints are tunable from the
 * dashboard.
 *
 * <p>Declaring five {@link LoggedTunableNumber}s and an {@code ifChanged} block by hand is the same
 * five lines every time, and the failure when it is done wrong is quiet: a constraint that is
 * tunable on the dashboard but never actually reaches the controller looks like a gain that does
 * nothing.
 *
 * <p>This is for <b>roboRIO-side</b> control loops only — a heading controller, an alignment
 * controller, anything where the {@code ProfiledPIDController} object lives in RAM and the robot
 * code computes the output. Gains that live on a TalonFX are a different problem with a different
 * pattern; see {@code docs/tunables.md}.
 *
 * <p>Call {@link #updateGains()} once per loop before {@code calculate()}. With {@code
 * Constants.tuningMode} off, the tunables return their compiled-in defaults and the update is a
 * handful of comparisons.
 *
 * <pre>{@code
 * private final LoggedTunableProfiledPID headingController =
 *     new LoggedTunableProfiledPID("Drive/Heading", 8.5, 0.0, 0.3, 12.0, 20.0);
 *
 * // In the command body, every loop:
 * headingController.updateGains();
 * double omega = headingController.calculate(currentHeading, targetHeading);
 * }</pre>
 *
 * <p>Adapted from FRC 3467's version, with the change noted on {@link #updateGains()}.
 */
public class LoggedTunableProfiledPID extends ProfiledPIDController {

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber maxVelocity;
  private final LoggedTunableNumber maxAcceleration;

  /** Constructs a tunable profiled controller with the default 20 ms period. */
  public LoggedTunableProfiledPID(
      String name, double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
    this(name, kP, kI, kD, maxVelocity, maxAcceleration, 0.02);
  }

  /**
   * @param name Dashboard key prefix. Gains appear as {@code Tuning/<name>/kP} and so on
   * @param period Loop period in seconds. Pass the robot's actual period if it is not 20 ms
   */
  public LoggedTunableProfiledPID(
      String name,
      double kP,
      double kI,
      double kD,
      double maxVelocity,
      double maxAcceleration,
      double period) {
    super(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), period);

    this.kP = new LoggedTunableNumber(name + "/kP", kP);
    this.kI = new LoggedTunableNumber(name + "/kI", kI);
    this.kD = new LoggedTunableNumber(name + "/kD", kD);
    this.maxVelocity = new LoggedTunableNumber(name + "/maxVelocity", maxVelocity);
    this.maxAcceleration = new LoggedTunableNumber(name + "/maxAcceleration", maxAcceleration);
  }

  /**
   * Applies any dashboard changes to the controller. Call once per loop, before {@code
   * calculate()}.
   *
   * <p>Gains and constraints are checked separately because {@link #setConstraints} rebuilds the
   * motion profile, and doing that on every kP nudge would discard profile state mid-motion.
   *
   * <p>Note this uses {@link LoggedTunableNumber#ifChanged}, which deliberately does not
   * short-circuit. Writing {@code kP.hasChanged(id) || kI.hasChanged(id)} instead would skip the
   * later checks once one returned true, leaving those tunables' stored values un-updated so they
   * report changed again on the following loop. The result is a controller that rebuilds itself for
   * one extra cycle after every edit — harmless here, genuinely not harmless when the same shape is
   * copied to something that writes over CAN.
   */
  public void updateGains() {
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            setConstraints(
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get())),
        maxVelocity,
        maxAcceleration);
  }
}
