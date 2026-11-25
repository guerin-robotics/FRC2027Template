package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.FieldConstants;
import frc.robot.generated.TunerConstants;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * The maple-sim drivetrain end to end: {@code Drive} commands the wheels, the physics world moves
 * the chassis, and odometry follows. Catches the failures that look fine in a log — module order
 * swapped between maple-sim and {@code Drive}, a gyro that does not follow the chassis, a pose
 * reset that moves the estimate but not the simulated robot, a field boundary that is not there.
 *
 * <p>Waits for a condition with a loop cap instead of running a fixed number of loops; see {@code
 * docs/testing.md}.
 */
class DriveSimulationTest {
  private static final double LOOP_SECONDS = 0.02;
  private static final int MAX_LOOPS = 600;

  private static Drive drive;
  private static SwerveDriveSimulation simulation;

  @BeforeAll
  static void build() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();

    // Step the FPGA clock by hand so Drive and the physics world see the same 20 ms every loop.
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    simulation = DriveSimulation.get();
    var modules = simulation.getModules();
    drive =
        new Drive(
            new GyroIOMapleSim(simulation.getGyroSimulation()),
            new ModuleIOMapleSim(modules[0], TunerConstants.FrontLeft.SlipCurrent),
            new ModuleIOMapleSim(modules[1], TunerConstants.FrontRight.SlipCurrent),
            new ModuleIOMapleSim(modules[2], TunerConstants.BackLeft.SlipCurrent),
            new ModuleIOMapleSim(modules[3], TunerConstants.BackRight.SlipCurrent));
    // Drive is stepped by hand below; it has no business in the scheduler.
    CommandScheduler.getInstance().unregisterSubsystem(drive);
  }

  @AfterAll
  static void restore() {
    drive.setPose(centerPose());
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  private static Pose2d centerPose() {
    return new Pose2d(
        FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0, Rotation2d.ZERO);
  }

  /** One robot loop: command, run Drive, step the physics, advance the clock. */
  private static void loop(ChassisVelocities speeds) {
    drive.runVelocity(speeds);
    drive.periodic();
    DriveSimulation.periodic();
    SimHooks.stepTiming(LOOP_SECONDS);
  }

  @Test
  void hasOnePhysicsModulePerDriveModule() {
    assertEquals(4, simulation.getModules().length);
  }

  @Test
  void drivingForwardMovesTheChassisAndOdometryFollows() {
    drive.setPose(centerPose());
    double startX = DriveSimulation.getTruePose().getX();

    int loops = 0;
    while (DriveSimulation.getTruePose().getX() < startX + 1.0 && loops++ < MAX_LOOPS) {
      loop(new ChassisVelocities(1.0, 0.0, 0.0));
    }

    Pose2d truth = DriveSimulation.getTruePose();
    assertTrue(truth.getX() >= startX + 1.0, "Chassis never moved 1 m forward; got " + truth);
    assertEquals(0.0, truth.getY() - FieldConstants.fieldWidth / 2.0, 0.15, "Drifted sideways");
    // Odometry integrates the module positions; if module order or the gyro were wrong it would
    // disagree with the chassis the physics world actually moved.
    assertEquals(truth.getX(), drive.getPose().getX(), 0.15, "Odometry disagrees with the chassis");
    assertEquals(truth.getY(), drive.getPose().getY(), 0.15, "Odometry disagrees with the chassis");
  }

  @Test
  void rotatingTheChassisTurnsTheGyro() {
    drive.setPose(centerPose());
    int loops = 0;
    while (DriveSimulation.getTruePose().getRotation().getDegrees() < 45.0 && loops++ < MAX_LOOPS) {
      loop(new ChassisVelocities(0.0, 0.0, 1.5));
    }
    double truthDegrees = DriveSimulation.getTruePose().getRotation().getDegrees();
    assertTrue(truthDegrees >= 45.0, "Chassis never rotated 45 degrees; got " + truthDegrees);
    assertEquals(
        truthDegrees, drive.getRotation().getDegrees(), 5.0, "Estimated heading left the chassis");
  }

  @Test
  void resettingThePoseMovesTheSimulatedRobotToo() {
    Pose2d target = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(90.0));
    drive.setPose(target);
    Pose2d truth = DriveSimulation.getTruePose();
    assertEquals(target.getX(), truth.getX(), 1e-6);
    assertEquals(target.getY(), truth.getY(), 1e-6);
    assertEquals(90.0, truth.getRotation().getDegrees(), 1e-3);
  }

  @Test
  void theFieldBoundaryStopsTheRobot() {
    drive.setPose(centerPose());
    // Drive straight at the far wall for far longer than it takes to reach it.
    for (int i = 0; i < MAX_LOOPS; i++) {
      loop(new ChassisVelocities(2.0, 0.0, 0.0));
    }
    double x = DriveSimulation.getTruePose().getX();
    assertTrue(x < FieldConstants.fieldLength, "Robot drove through the wall to x=" + x);
    assertTrue(x > FieldConstants.fieldLength / 2.0, "Robot never reached the wall; x=" + x);
  }
}
