// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOPhotonVision;
import frc.robot.subsystems.vision.io.VisionIOPhotonVisionSim;
import frc.robot.util.FaultMonitor;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 *
 * <p><b>This is the wiring layer.</b> It may hold references to every subsystem in order to pass
 * them to commands and compose bindings — that is its job, and the only place in the codebase where
 * that is allowed. It must not contain game logic. If you find yourself writing scoring or state
 * logic here, it belongs in a command factory or in {@link RobotState}.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2),
                new VisionIOPhotonVision(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // Replay requires one VisionIO per camera so all cameras' logged inputs are replayed.
        // If you change the camera count, change it in all three branches above.
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    // IMPORTANT: register PathPlanner named commands and event triggers BEFORE building the
    // auto chooser. AutoBuilder.buildAutoChooser() parses the .auto files and resolves named
    // commands at build time — anything registered after this line is silently ignored.
    //
    //   NamedCommands.registerCommand("Score", ScoringSequences.score(...));
    //
    // Event triggers must NOT declare subsystem requirements, or they will interrupt the
    // path-following command. See .claude/rules/03-commands.md.

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Register fault conditions. These roll up into a single "Robot OK" dashboard boolean
    // plus a named fault list, so the pit does not have to check a dozen indicators
    // individually — see FaultMonitor for the 2026 failure that motivated this.
    // Register any new mechanism's health conditions here too.
    FaultMonitor.getInstance().register("Gyro disconnected", () -> !drive.isGyroConnected());
    FaultMonitor.getInstance().register("Pose off field", drive::isPoseOffField);
    for (int i = 0; i < vision.getCameraCount(); i++) {
      final int cameraIndex = i;
      FaultMonitor.getInstance()
          .register(
              "Camera " + cameraIndex + " disconnected",
              () -> !vision.isCameraConnected(cameraIndex));
      // A connected but uncalibrated camera raises no error and produces no pose — it just
      // silently stops contributing. Only report it when the camera is actually connected,
      // otherwise a disconnected camera trips both faults.
      FaultMonitor.getInstance()
          .register(
              "Camera " + cameraIndex + " uncalibrated",
              () ->
                  vision.isCameraConnected(cameraIndex) && !vision.isCameraCalibrated(cameraIndex));
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses, and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0 degrees when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is held
    controller.x().whileTrue(DriveCommands.stopWithX(drive));

    // Reset gyro to 0 degrees when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Brake/coast passthrough for {@link Robot}'s disabled-coast handling. */
  public void setDriveBrakeMode(boolean brake) {
    drive.setDriveBrakeMode(brake);
  }
}
