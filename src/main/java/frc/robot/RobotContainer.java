// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import bline.commands.FollowPath;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.FaultMonitor;
import frc.robot.autos.AutoPreview;
import frc.robot.autos.BLineAutos;
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
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

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

  // Controllers live in Triggers, not here. This class binds triggers to commands; it does not
  // own input devices and never touches a controller object directly.
  // See .claude/rules/01-architecture.md.

  private final AutoPreview autoPreview;

  /** BLine routines by name, registered as Autonomous OpModes (see {@code OpModes}). */
  private final Map<String, Supplier<Command>> autoRoutines;

  /** Characterization and SysId routines by name, registered as Utility OpModes. */
  private final Map<String, Supplier<Command>> utilityRoutines = new LinkedHashMap<>();

  /** Auto Side captured when the auto starts, so every leg of one routine mirrors the same way. */
  private boolean mirrorAtAutoStart = false;

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
        Supplier<Pose2d> simCameraPose = drive::getPose;
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, simCameraPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, simCameraPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, simCameraPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3, simCameraPose));
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

    // ---- Autos ----
    //
    // Each BLine routine is an Autonomous OpMode, registered by OpModes from autoRoutines, once
    // per side (Left, Right). Each value is a SUPPLIER, built when the OpMode starts: the selected
    // routine loads only its own paths, and nothing is constructed for autos that never run.
    // Autos are drawn blue-left only. Alliance comes from the Driver Station (FMS station) and
    // flips the path; the Right OpMode mirrors it. See docs/autos.md.
    autoPreview = new AutoPreview();
    FollowPath.Builder bline =
        BLineAutos.createBuilder(drive, () -> mirrorAtAutoStart, autoPreview);

    // Bind BLine event triggers BEFORE building any routine, so the smoke test sees every key.
    // Bound commands run alongside the path, so they must NOT require the drive
    // (.claude/rules/03-commands.md).
    //
    //   BLineAutos.bind("Intake", IntakeCommands.deploy(intake));

    autoRoutines = BLineAutos.all(bline, drive);

    // Characterization and SysId routines — Utility OpModes. Each runs only while a Utility
    // OpMode is selected and enabled, so none is reachable in a match.
    utilityRoutines.put(
        "Drive Wheel Radius Characterization",
        () -> DriveCommands.wheelRadiusCharacterization(drive));
    utilityRoutines.put(
        "Drive Simple FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    utilityRoutines.put(
        "Drive SysId (Quasistatic Forward)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD));
    utilityRoutines.put(
        "Drive SysId (Quasistatic Reverse)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
    utilityRoutines.put(
        "Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.FORWARD));
    utilityRoutines.put(
        "Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.REVERSE));

    // Register fault conditions. These roll up into a single "Robot OK" dashboard boolean
    // plus a named fault list, so the pit does not have to check a dozen indicators
    // individually — see FaultMonitor for the 2026 failure that motivated this.
    // Register any new mechanism's health conditions here too.
    FaultMonitor.getInstance().register("Gyro disconnected", () -> !drive.isGyroConnected());
    FaultMonitor.getInstance().register("Pose off field", drive::isPoseOffField);
    FaultMonitor.getInstance().register("Swerve motor sticky fault", drive::hasStickyFault);
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
   * Binds triggers to commands.
   *
   * <p>Triggers come from {@link Triggers#getInstance()} and are never constructed here — see the
   * class javadoc on {@link Triggers} for why. This method should read as a list of "when this
   * happens, run that", with no input-device details and no game logic.
   */
  private void configureButtonBindings() {
    Triggers triggers = Triggers.getInstance();

    // Default command, normal field-relative drive.
    //
    // The suppliers already return robot convention (+X forward, +Y left, +rot CCW) — the
    // joystick sign flips live inside Triggers. Do not negate them again here.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, triggers::driveXSupplier, triggers::driveYSupplier, triggers::driveRotSupplier));

    // Hold a fixed heading while the driver keeps translation control.
    triggers
        .lockHeading()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, triggers::driveXSupplier, triggers::driveYSupplier, () -> Rotation2d.ZERO));

    // Wheels into an X so the robot resists being pushed.
    triggers.stopWithX().whileTrue(DriveCommands.stopWithX(drive));

    triggers.resetGyro().onTrue(DriveCommands.resetGyro(drive));
  }

  /**
   * Reads every subsystem's inputs. Call once per loop from {@code Robot.robotPeriodic()}, BEFORE
   * the scheduler runs, in this fixed order.
   *
   * <p><b>This is the Logger.processInputs() gate.</b> Commands V3 has no {@code
   * Subsystem.periodic()} — nothing calls a mechanism's {@code periodic()} for you — so each
   * subsystem, {@code Vision} included, is called here. A subsystem left out is silent: no inputs,
   * no replay, no error. Add every new mechanism's {@code periodic()} below; {@code
   * RobotContainerSmokeTest} proves each one runs.
   */
  public void periodic() {
    drive.periodic();
    vision.periodic();
  }

  /** BLine routines by name, for {@code OpModes} to register as Autonomous OpModes. */
  public Map<String, Supplier<Command>> autoRoutines() {
    return autoRoutines;
  }

  /** Characterization routines by name, for {@code OpModes} to register as Utility OpModes. */
  public Map<String, Supplier<Command>> utilityRoutines() {
    return utilityRoutines;
  }

  /** The BLine path files a routine runs, in order, for the preview. */
  public List<String> pathsOf(Supplier<Command> routine) {
    return BLineAutos.pathsOf(routine);
  }

  /**
   * Latches the Left/Right side. Call when the auto starts: the builder reads it at each path
   * command's start, and a later leg must not mirror differently from the first.
   */
  public void latchMirror(boolean mirror) {
    mirrorAtAutoStart = mirror;
  }

  /** Disabled loop: redraw the auto preview if the selection changed, check the start pose. */
  public void updateAutoPreview(List<String> pathNames, boolean mirror) {
    autoPreview.update(
        pathNames,
        AllianceFlipUtil.shouldFlip(),
        mirror,
        RobotState.getInstance().getEstimatedPose());
  }

  /** Auto loop: keep the robot moving on the preview widget. */
  public void showAutoRobot() {
    autoPreview.showRobot(RobotState.getInstance().getEstimatedPose());
  }

  /** Brake/coast passthrough for {@link Robot}'s disabled-coast handling. */
  public void setDriveBrakeMode(boolean brake) {
    drive.setDriveBrakeMode(brake);
  }
}
