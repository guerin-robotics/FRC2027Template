package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation check for {@link DriveCommands#joystickDriveAtAngle}: with the translation joystick
 * held at zero, the robot should spin in place to face the commanded heading and hold it.
 *
 * <p>This is the shared building block every 2027 alignment command is expected to be built from
 * (see the class javadoc on {@link DriveCommands}) — a regression in {@code angleController}'s
 * behavior here would quietly break every command built on top of it, not just this one.
 *
 * <p>See {@link DriveToPoseSimTest}'s javadoc for why no wall-clock sleeping is needed to get real
 * odometry samples out of {@code ModuleIOSim}, and for why {@link #cancelScheduledCommands()} below
 * must not be removed — {@code angleController} is exactly the shared static state that javadoc
 * warns about, and this class both reads and is read by it.
 */
class JoystickDriveAtAngleSimTest {

  private static final double DT_SECONDS = 0.02;
  private static final double SIM_DURATION_SECONDS = 3.0;
  private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);

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

  @AfterEach
  void cancelScheduledCommands() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Test
  void joystickDriveAtAngleConvergesOnHeadingWithNoJoystickInput() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

    Rotation2d target = Rotation2d.fromDegrees(135.0);
    CommandScheduler.getInstance()
        .schedule(DriveCommands.joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, () -> target));

    int steps = (int) (SIM_DURATION_SECONDS / DT_SECONDS);
    for (int i = 0; i < steps; i++) {
      CommandScheduler.getInstance().run();
    }

    double headingErrorRadians = Math.abs(drive.getRotation().minus(target).getRadians());

    System.out.println("********** joystickDriveAtAngle Sim Check **********");
    System.out.printf("\tTarget heading: %.2f deg%n", target.getDegrees());
    System.out.printf("\tFinal heading:  %.2f deg%n", drive.getRotation().getDegrees());
    System.out.printf("\tHeading error:  %.2f deg%n", Math.toDegrees(headingErrorRadians));

    assertTrue(
        headingErrorRadians < HEADING_TOLERANCE_RADIANS,
        "joystickDriveAtAngle did not converge on target heading within "
            + SIM_DURATION_SECONDS
            + "s: error="
            + Math.toDegrees(headingErrorRadians)
            + " deg");
  }
}
