package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation check for {@link DriveCommands#driveToPose}: schedules it against a physics-sim {@link
 * Drive} (the same {@code ModuleIOSim} wiring {@code RobotContainer} uses for {@code SIM}) and
 * asserts the pose estimate converges on the goal.
 *
 * <p>Neither {@code ModuleIOSim} nor the no-op {@code GyroIO} used here register anything with
 * {@code PhoenixOdometryThread}, so that thread never actually starts (see its {@code start()}
 * override) and every {@code CommandScheduler.run()} deterministically advances the sim by one
 * fixed 20 ms physics step — no wall-clock sleeping needed to get real odometry samples, unlike a
 * {@code ModuleIOTalonFX}-backed test would.
 *
 * <p>Confirms the command converges at all — with {@link #driveController}'s gains still an untuned
 * placeholder (see the comment on that field in {@code DriveCommands}), this is not a substitute
 * for a real {@code /pid-tune} pass before the command is bound to a button.
 */
class DriveToPoseSimTest {

  private static final double DT_SECONDS = 0.02;
  private static final double SIM_DURATION_SECONDS = 5.0;

  private static final double POSITION_TOLERANCE_METERS = 0.05;
  private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);

  @BeforeAll
  static void initializeHal() {
    // See GainSweepTest's javadoc for why this must not be inside a Java assert.
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);

    // Drive.periodic() stops every module while the DriverStation reports disabled.
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // Logger.start() normally exits the JVM unless the main class extends LoggedRobot — there
    // is no robot class at all here, just this test. AdvancedHooks.disableRobotBaseCheck() is
    // AdvantageKit's documented escape hatch for exactly this (custom/no robot base). No data
    // receivers are added: Constants.tuningMode is false, so LoggedTunableNumber (and therefore
    // LoggedTunableProfiledPID) never touches NetworkTables either, so nothing here needs one.
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  @Test
  void driveToPoseConvergesOnTarget() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

    Pose2d target = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(90.0));
    CommandScheduler.getInstance().schedule(DriveCommands.driveToPose(drive, () -> target));

    int steps = (int) (SIM_DURATION_SECONDS / DT_SECONDS);
    for (int i = 0; i < steps; i++) {
      CommandScheduler.getInstance().run();
    }

    Pose2d finalPose = drive.getPose();
    double distanceErrorMeters = finalPose.getTranslation().getDistance(target.getTranslation());
    double headingErrorRadians =
        Math.abs(finalPose.getRotation().minus(target.getRotation()).getRadians());

    System.out.println("********** driveToPose Sim Check **********");
    System.out.printf("\tTarget pose:   %s%n", target);
    System.out.printf("\tFinal pose:    %s%n", finalPose);
    System.out.printf("\tDistance error: %.4f m%n", distanceErrorMeters);
    System.out.printf("\tHeading error:  %.2f deg%n", Math.toDegrees(headingErrorRadians));

    assertTrue(
        distanceErrorMeters < POSITION_TOLERANCE_METERS,
        "driveToPose did not converge on target translation within "
            + SIM_DURATION_SECONDS
            + "s: error="
            + distanceErrorMeters
            + " m");
    assertTrue(
        headingErrorRadians < HEADING_TOLERANCE_RADIANS,
        "driveToPose did not converge on target heading within "
            + SIM_DURATION_SECONDS
            + "s: error="
            + Math.toDegrees(headingErrorRadians)
            + " deg");
  }
}
