package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * Sanity checks that {@link Drive#runVelocity} and odometry agree on which way is which.
 *
 * <p>This is not a control-loop test — it commands raw {@link ChassisSpeeds} directly, with no PID
 * and no {@code DriveCommands} factory in between. It exists to catch the failure catalog in {@code
 * .claude/rules/00-safety.md}: a flipped module inversion, a swapped module position in {@code
 * TunerConstants}, or a kinematics sign error all show up here as "commanded +X does not produce +X
 * odometry" — cheaply, in a few seconds of sim, instead of on the first drive of the season. It
 * intentionally says nothing about whether the PID gains anywhere in {@code DriveCommands} are
 * good; {@link frc.robot.commands.DriveToPoseSimTest} and {@link
 * frc.robot.commands.JoystickDriveAtAngleSimTest} cover that layer.
 *
 * <p>See {@code DriveToPoseSimTest}'s javadoc for why {@code ModuleIOSim} needs no wall-clock
 * sleeping to produce real odometry samples.
 */
class DriveOdometrySimTest {

  private static final double DT_SECONDS = 0.02;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  private static Drive newSimDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIOSim(TunerConstants.FrontLeft),
        new ModuleIOSim(TunerConstants.FrontRight),
        new ModuleIOSim(TunerConstants.BackLeft),
        new ModuleIOSim(TunerConstants.BackRight));
  }

  /** Runs the scheduler (so {@code Drive.periodic()} fires) while a velocity is held constant. */
  private static void driveFor(Drive drive, ChassisSpeeds speeds, double seconds) {
    int steps = (int) (seconds / DT_SECONDS);
    for (int i = 0; i < steps; i++) {
      drive.runVelocity(speeds);
      CommandScheduler.getInstance().run();
    }
  }

  @Test
  void commandingPositiveXMovesOdometryInPositiveX() {
    Drive drive = newSimDrive();
    driveFor(drive, new ChassisSpeeds(1.0, 0.0, 0.0), 1.0);

    System.out.println("********** Drive Odometry Sim Check: +X **********");
    System.out.println("\tFinal pose: " + drive.getPose());

    assertTrue(
        drive.getPose().getX() > 0.3,
        "Commanding +1 m/s in X for 1s should move the pose meaningfully in +X; got "
            + drive.getPose());
    assertTrue(
        Math.abs(drive.getPose().getY()) < 0.1,
        "Commanding pure +X should not produce meaningful Y motion; got " + drive.getPose());
  }

  @Test
  void commandingPositiveYMovesOdometryInPositiveY() {
    Drive drive = newSimDrive();
    driveFor(drive, new ChassisSpeeds(0.0, 1.0, 0.0), 1.0);

    System.out.println("********** Drive Odometry Sim Check: +Y **********");
    System.out.println("\tFinal pose: " + drive.getPose());

    assertTrue(
        drive.getPose().getY() > 0.3,
        "Commanding +1 m/s in Y for 1s should move the pose meaningfully in +Y; got "
            + drive.getPose());
    assertTrue(
        Math.abs(drive.getPose().getX()) < 0.1,
        "Commanding pure +Y should not produce meaningful X motion; got " + drive.getPose());
  }

  @Test
  void commandingPositiveOmegaIncreasesHeadingCounterclockwise() {
    Drive drive = newSimDrive();
    driveFor(drive, new ChassisSpeeds(0.0, 0.0, 1.0), 1.0);

    System.out.println("********** Drive Odometry Sim Check: +Omega **********");
    System.out.println("\tFinal pose: " + drive.getPose());

    // WPILib's convention is CCW-positive, so a positive commanded omega must increase heading.
    assertTrue(
        drive.getPose().getRotation().getRadians() > 0.1,
        "Commanding +1 rad/s omega for 1s should increase heading (CCW-positive); got "
            + drive.getPose());
  }
}
