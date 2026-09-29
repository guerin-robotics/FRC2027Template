package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.mechanism.roller.RollerMechanism;
import frc.lib.mechanism.roller.RollerSubsystem;
import frc.lib.util.FieldConstants;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * The 2026 five-motor flywheel, ported from Rebuilt2026 {@code Flywheel}.
 *
 * <p>The generic roller behavior — logging, battery reporting, "at speed" — comes from {@link
 * RollerMechanism}. What is here is the 2026 shot selection: speed from distance to the hub or the
 * pass target, the dashboard tuning speed, and the shoot-on-the-move experiment.
 */
public class Flywheel extends RollerSubsystem {

  private final FlywheelVisualizer visualizer = new FlywheelVisualizer();
  private final LoggedNetworkNumber tuningRpm =
      new LoggedNetworkNumber("Tune/flywheel/tuningRPM", 20);

  private Supplier<Angle> hoodAngleSupplier = () -> Degrees.of(0);

  public Flywheel(RollerMechanism mechanism) {
    super(mechanism);
  }

  /**
   * Feeds the trajectory visualizer the hood angle. A supplier, so Flywheel never holds a Hood —
   * RobotContainer passes {@code hood::getPosition}.
   */
  public void setHoodAngleSupplier(Supplier<Angle> supplier) {
    this.hoodAngleSupplier = supplier;
  }

  @Override
  public void periodic() {
    super.periodic();
    visualizer.updateTrajectory(getVelocity(), hoodAngleSupplier.get());
  }

  /** Within {@link FlywheelConstants#SETTINGS}' tolerance of the goal. 2026's name for it. */
  public boolean isSpunUp() {
    return isAtVelocity();
  }

  public Angle getFlywheelAngle() {
    return roller.getPosition();
  }

  // ---- Shot selection ----

  public void setSpeedForHub() {
    setVelocity(ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub());
  }

  public void setSpeedForTarget(Translation3d target) {
    setVelocity(ShotCalculator.getInstance().getFlywheelSpeedForTarget(target));
  }

  public void setSpeedForDistance(Distance distance) {
    setVelocity(ShotCalculator.getInstance().getFlywheelSpeedForDistance(distance));
  }

  public void setSpeedForPassing() {
    setVelocity(ShotCalculator.getInstance().getFlywheelSpeedForPassTarget());
  }

  public AngularVelocity getTuningRPM() {
    return RPM.of(tuningRpm.get());
  }

  public void setTuningRPM() {
    setVelocity(getTuningRPM());
  }

  /**
   * Shoot-on-the-move: the hub-distance speed corrected for the robot's hub-relative velocity.
   *
   * <p>Ported verbatim, including three 2026 bugs that make its output wrong. It is bound only in
   * simulation, which is the only reason it did no harm:
   *
   * <ul>
   *   <li>the target is the <b>unflipped</b> hub, so on red it aims at the blue hub;
   *   <li>an RPM is used as the magnitude of a linear velocity (m/s) and then subtracted from one;
   *   <li>{@code hoodDegrees} is passed to {@code Radians.of}, and the sim binding hands it a
   *       degree value read once at binding time.
   * </ul>
   */
  public void shootDynamic(double hoodDegrees) {
    // Fuel to hub
    Translation2d fuelToGoalDistance =
        new Translation2d(
            (FieldConstants.Hub.topCenterPoint.getX()
                - RobotState.getInstance().getEstimatedPose().getX()),
            (FieldConstants.Hub.topCenterPoint.getY()
                - RobotState.getInstance().getEstimatedPose().getY()));
    double distance = fuelToGoalDistance.getNorm();
    AngularVelocity idealSpeed =
        ShotCalculator.getInstance().getFlywheelSpeedForDistance(Meters.of(distance));
    Translation2d fuelToGoalVector =
        new Translation2d(idealSpeed.magnitude(), RobotState.getInstance().getAngleToAllianceHub());
    // Robot to hub
    Translation2d robotToGoalVector =
        new Translation2d(
            (RobotState.getInstance().getHubRelativeVelocity().vxMetersPerSecond),
            (RobotState.getInstance().getHubRelativeVelocity().vyMetersPerSecond));
    // Fuel to robot (what we're finding)
    Translation2d fuelToRobotVector =
        new Translation2d(
            (fuelToGoalVector.getX() - robotToGoalVector.getX()),
            (fuelToGoalVector.getY() - robotToGoalVector.getY()));
    // Linear velocity (m/s) to rps using flywheel rotations per meter
    AngularVelocity velocityInRPS =
        RotationsPerSecond.of(
            fuelToRobotVector.getNorm()
                * FlywheelConstants.Mechanical.FLYWHEEL_ROTATIONS_PER_METER);
    AngularVelocity velocityInRPM = RPM.of(velocityInRPS.magnitude());
    // Finally, divide by cos of hood angle
    AngularVelocity finalVelocity =
        RPM.of(velocityInRPM.magnitude() / Math.cos(Radians.of(hoodDegrees).magnitude()));
    setVelocity(finalVelocity);
  }
}
