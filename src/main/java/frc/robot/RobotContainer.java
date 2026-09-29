// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.mechanism.roller.RollerMechanism;
import frc.lib.mechanism.rotary.RotaryMechanism;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.FaultMonitor;
import frc.robot.Zones.ApproachingZoneX;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.FlywheelCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakePivotCommands;
import frc.robot.commands.IntakeRollerCommands;
import frc.robot.commands.PrestageCommands;
import frc.robot.commands.ShootSequences;
import frc.robot.commands.TransportCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodPosCalculator;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.IntakePivotConstants;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerConstants;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.lowerFeeder.LowerFeederConstants;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.prestage.PrestageConstants;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.transport.TransportConstants;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import frc.robot.subsystems.upperFeeder.UpperFeederConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOPhotonVision;
import frc.robot.subsystems.vision.io.VisionIOPhotonVisionSim;
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

  // Mechanisms
  private final Transport transport;
  private final UpperFeeder upperFeeder;
  private final LowerFeeder lowerFeeder;
  private final Prestage prestage;
  private final IntakeRoller intakeRoller;
  private final Flywheel flywheel;
  private final Hood hood;
  private final IntakePivot intakePivot;

  // Controllers live in Triggers, not here. This class binds triggers to commands; it does not
  // own input devices and never touches a controller object directly.
  // See .claude/rules/01-architecture.md.

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Double> driverPresetChooser;
  private final LoggedDashboardChooser<Boolean> driveControllerChooser;

  private static final String AUTO_DELAY_KEY = "Auto Delay";

  private final AutoPreview autoPreview;
  private final RobotModelVisualizer robotModelVisualizer;

  /** The three positions every hopper compress moves through. */
  private static final IntakePivotCommands.CompressPositions COMPRESS_POSITIONS =
      new IntakePivotCommands.CompressPositions(
          Constants.Setpoints.PIVOT_DOWN_POSITION,
          Constants.Setpoints.PIVOT_JOSTLE_FIRST_POSITION,
          Constants.Setpoints.PIVOT_JOSTLE_UP_POSITION);

  // ---- Cancellation flags (see .claude/rules/03-commands.md) ----
  //
  // compressCancelled: set by intake in/out/compress, so the automatic compress does not
  //   reschedule for the rest of this shoot press. Cleared on shoot-button release.
  // xCancelled: set by the auto-X override. Only the SIM bindings read it (real alignOrXForShoot
  //   does not), as in 2026. Cleared on shoot-button release.
  // doubleCompress: toggled by the operator override so compress runs double. Cleared on
  //   shoot-button release (real bindings only).
  private boolean compressCancelled = false;
  private boolean xCancelled = false;
  private boolean doubleCompress = false;

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
        transport =
            new Transport(
                RollerMechanism.real(TransportConstants.CONFIG, TransportConstants.SETTINGS));
        upperFeeder =
            new UpperFeeder(
                RollerMechanism.real(UpperFeederConstants.CONFIG, UpperFeederConstants.SETTINGS));
        lowerFeeder =
            new LowerFeeder(
                RollerMechanism.real(LowerFeederConstants.CONFIG, LowerFeederConstants.SETTINGS));
        prestage =
            new Prestage(
                RollerMechanism.real(PrestageConstants.CONFIG, PrestageConstants.SETTINGS));
        intakeRoller =
            new IntakeRoller(
                RollerMechanism.real(IntakeRollerConstants.CONFIG, IntakeRollerConstants.SETTINGS));
        flywheel =
            new Flywheel(
                RollerMechanism.real(FlywheelConstants.CONFIG, FlywheelConstants.SETTINGS));
        hood = new Hood(RotaryMechanism.real(HoodConstants.CONFIG, HoodConstants.SETTINGS));
        intakePivot =
            new IntakePivot(
                RotaryMechanism.real(IntakePivotConstants.CONFIG, IntakePivotConstants.SETTINGS));
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
        transport =
            new Transport(
                RollerMechanism.sim(
                    TransportConstants.CONFIG,
                    TransportConstants.SETTINGS,
                    TransportConstants.SIM));
        upperFeeder =
            new UpperFeeder(
                RollerMechanism.sim(
                    UpperFeederConstants.CONFIG,
                    UpperFeederConstants.SETTINGS,
                    UpperFeederConstants.SIM));
        lowerFeeder =
            new LowerFeeder(
                RollerMechanism.sim(
                    LowerFeederConstants.CONFIG,
                    LowerFeederConstants.SETTINGS,
                    LowerFeederConstants.SIM));
        prestage =
            new Prestage(
                RollerMechanism.sim(
                    PrestageConstants.CONFIG, PrestageConstants.SETTINGS, PrestageConstants.SIM));
        intakeRoller =
            new IntakeRoller(
                RollerMechanism.sim(
                    IntakeRollerConstants.CONFIG,
                    IntakeRollerConstants.SETTINGS,
                    IntakeRollerConstants.SIM));
        flywheel =
            new Flywheel(
                RollerMechanism.sim(
                    FlywheelConstants.CONFIG, FlywheelConstants.SETTINGS, FlywheelConstants.SIM));
        hood =
            new Hood(
                RotaryMechanism.sim(
                    HoodConstants.CONFIG, HoodConstants.SETTINGS, HoodConstants.SIM));
        intakePivot =
            new IntakePivot(
                RotaryMechanism.sim(
                    IntakePivotConstants.CONFIG,
                    IntakePivotConstants.SETTINGS,
                    IntakePivotConstants.SIM));
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
        transport =
            new Transport(
                RollerMechanism.replay(TransportConstants.CONFIG, TransportConstants.SETTINGS));
        upperFeeder =
            new UpperFeeder(
                RollerMechanism.replay(UpperFeederConstants.CONFIG, UpperFeederConstants.SETTINGS));
        lowerFeeder =
            new LowerFeeder(
                RollerMechanism.replay(LowerFeederConstants.CONFIG, LowerFeederConstants.SETTINGS));
        prestage =
            new Prestage(
                RollerMechanism.replay(PrestageConstants.CONFIG, PrestageConstants.SETTINGS));
        intakeRoller =
            new IntakeRoller(
                RollerMechanism.replay(
                    IntakeRollerConstants.CONFIG, IntakeRollerConstants.SETTINGS));
        flywheel =
            new Flywheel(
                RollerMechanism.replay(FlywheelConstants.CONFIG, FlywheelConstants.SETTINGS));
        hood = new Hood(RotaryMechanism.replay(HoodConstants.CONFIG, HoodConstants.SETTINGS));
        intakePivot =
            new IntakePivot(
                RotaryMechanism.replay(IntakePivotConstants.CONFIG, IntakePivotConstants.SETTINGS));
        break;
    }

    // Wire the drivetrain into PathPlanner. The configuration lives in Drive.java; only the
    // timing is decided here, because it mutates PathPlanner's global state and a constructor
    // is the wrong place to do that. Must come before buildAutoChooser() below — without it
    // that call throws AutoBuilderException and takes out the whole chooser.
    drive.configureAutoBuilder();

    // The trajectory visualizer draws the shot at the hood's measured angle. A supplier keeps
    // Flywheel and Hood decoupled.
    flywheel.setHoodAngleSupplier(hood::getPosition);

    // "Is the flywheel spun up" is a trigger, and triggers live in Triggers — which cannot hold a
    // subsystem. Hand it the predicate instead.
    Triggers.getInstance().setFlywheelSpunUpSupplier(flywheel::isSpunUp);

    // 3D robot model for AdvantageScope. Suppliers only — no subsystem cross-references.
    robotModelVisualizer =
        new RobotModelVisualizer(
            intakePivot::getPosition, hood::getPosition, flywheel::getFlywheelAngle);

    // IMPORTANT: register PathPlanner named commands and event triggers BEFORE building the
    // auto chooser. AutoBuilder.buildAutoChooser() parses the .auto files and resolves named
    // commands at build time — anything registered after this line is silently ignored.
    registerNamedCommands();
    registerEventTriggers();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption(
        Constants.Autos.DEFAULT_AUTO_NAME, new PathPlannerAuto(Constants.Autos.DEFAULT_AUTO_NAME));

    // Set up SysId routines. Commented out in 2026; kept from the template for shop testing.
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

    // Which driver is driving — sets the rotation input exponent, latched at teleopInit.
    driverPresetChooser = new LoggedDashboardChooser<>("Driver Preset");
    driverPresetChooser.addDefaultOption("Parker", Constants.Controllers.PARKER_ROTATION_EXPONENT);
    driverPresetChooser.addOption("Christian", Constants.Controllers.CHRISTIAN_ROTATION_EXPONENT);

    // Which controller drives — latched at teleopInit. See Triggers.latchDriveController.
    driveControllerChooser = new LoggedDashboardChooser<>("Drive controller");
    driveControllerChooser.addDefaultOption(Constants.Controllers.FLIGHTSTICK_OPTION, false);
    driveControllerChooser.addOption(Constants.Controllers.XBOX_OPTION, true);

    autoPreview = new AutoPreview(autoChooser::get);
    SmartDashboard.putNumber(AUTO_DELAY_KEY, Constants.Waits.DEFAULT_AUTO_DELAY_SECONDS);

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

    // Mechanism health: disconnected, rebooted, over temperature, hardware fault.
    transport.registerFaultMonitors();
    upperFeeder.registerFaultMonitors();
    lowerFeeder.registerFaultMonitors();
    prestage.registerFaultMonitors();
    intakeRoller.registerFaultMonitors();
    flywheel.registerFaultMonitors();
    hood.registerFaultMonitors();
    intakePivot.registerFaultMonitors();

    // Configure the button bindings. The 2026 robot had two separate sets: the real robot's, and
    // a keyboard-driven set for simulation that differs in places (see configureSimBindings).
    if (Robot.isReal()) {
      configureRealBindings();
    } else if (Robot.isSimulation()) {
      configureSimBindings();
    }
  }

  // ==============================================================================================
  // PATHPLANNER
  // ==============================================================================================

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "DeployIntake",
        IntakePivotCommands.setPivotPosition(intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION));

    NamedCommands.registerCommand(
        "RetractIntake",
        IntakePivotCommands.setPivotPosition(intakePivot, Constants.Setpoints.PIVOT_UP_POSITION));

    NamedCommands.registerCommand(
        "RunIntake",
        IntakeRollerCommands.setRollerVoltage(
            intakeRoller, Constants.Setpoints.INTAKE_ROLLER_VOLTAGE));

    // Hold the shooter on the hub, stationary, while the shot sequence runs.
    NamedCommands.registerCommand(
        "Shoot",
        DriveCommands.joystickDriveAtAngle(
                drive, () -> 0, () -> 0, () -> RobotState.getInstance().getAngleToAllianceHub())
            .alongWith(
                ShootSequences.autoShootToHub(
                    flywheel,
                    prestage,
                    hood,
                    upperFeeder,
                    lowerFeeder,
                    transport,
                    intakeRoller,
                    intakePivot)));

    NamedCommands.registerCommand(
        "stopAll",
        ShootSequences.stopAll(
            flywheel, prestage, hood, upperFeeder, lowerFeeder, transport, intakeRoller));

    NamedCommands.registerCommand(
        "HoodDownNamed", HoodCommands.setHoodPos(hood, Constants.Setpoints.HOOD_DOWN_POSITION));
  }

  /**
   * Event markers in the paths.
   *
   * <p>Event trigger commands must NOT declare subsystem requirements that overlap with any command
   * in the auto group — a SequentialCommandGroup requires the union of its children, so the
   * scheduler would interrupt the whole auto. These call the subsystem directly without requiring
   * it, as 2026 did. See .claude/rules/03-commands.md.
   */
  private void registerEventTriggers() {
    new EventTrigger("DeployIntake")
        .onTrue(
            Commands.runOnce(
                () -> intakePivot.setPosition(Constants.Setpoints.PIVOT_DOWN_POSITION)));

    new EventTrigger("RetractIntake")
        .onTrue(
            Commands.runOnce(() -> intakePivot.setPosition(Constants.Setpoints.PIVOT_UP_POSITION)));

    // A zoned marker (start and end), so whileTrue.
    new EventTrigger("RunIntake")
        .whileTrue(
            Commands.startEnd(
                () -> intakeRoller.setVoltage(Constants.Setpoints.INTAKE_ROLLER_VOLTAGE),
                () -> intakeRoller.setVoltage(Volts.of(0))));

    // KNOWN 2026 ISSUE, carried over unchanged: this one DOES require the hood, against the rule
    // above. Any auto that places a "HoodDown" marker inside a group that also requires the hood
    // (the "Shoot" named command does) would be interrupted by it.
    new EventTrigger("HoodDown")
        .onTrue(HoodCommands.setHoodPos(hood, Constants.Setpoints.HOOD_DOWN_POSITION));
  }

  // ==============================================================================================
  // REAL-ROBOT BINDINGS — Rebuilt2026 configureStateBindings()
  // ==============================================================================================

  private void configureRealBindings() {
    Triggers triggers = Triggers.getInstance();

    // ---- Default commands ----

    // The suppliers already return robot convention (+X forward, +Y left, +rot CCW) — the
    // joystick sign flips live inside Triggers. Do not negate them again here.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, triggers::driveXSupplier, triggers::driveYSupplier, triggers::driveRotSupplier));
    flywheel.setDefaultCommand(
        FlywheelCommands.flywheelIdle(flywheel, Constants.Setpoints.FLYWHEEL_IDLE_VELOCITY));
    hood.setDefaultCommand(HoodCommands.hoodIdle(hood, Constants.Setpoints.HOOD_DOWN_POSITION));
    intakeRoller.setDefaultCommand(
        IntakeRollerCommands.intakeRollerIdle(
            intakeRoller, Constants.Setpoints.INTAKE_ROLLER_AGITATE_VOLTAGE));

    // ---- Overrides ----

    triggers.allianceWinFlipper().onTrue(HubShiftUtil.flipWinner());
    triggers.allianceWinDisabler().onTrue(HubShiftUtil.disableHubShiftUtil());
    // Sets xCancelled, which only the sim bindings read — as in 2026.
    triggers.autoXOverride().onTrue(Commands.runOnce(() -> xCancelled = true));
    triggers
        .doubleCompressOverride()
        .onTrue(Commands.runOnce(() -> doubleCompress = !doubleCompress));

    // ---- Drivetrain ----

    // Shoot in our zone: aim at the hub at half speed, X once aligned and the stick is centered.
    triggers
        .shootButton()
        .and(triggers.isShootSafeZone)
        .whileTrue(
            DriveCommands.alignOrXForShoot(
                drive,
                () -> triggers.driveXSupplier() * 0.5,
                () -> triggers.driveYSupplier() * 0.5,
                () -> RobotState.getInstance().getAngleToAllianceHub()));

    // Shoot outside our zone (and not demo): aim at the pass target at half speed.
    triggers
        .shootButton()
        .and(triggers.isShootSafeZone.negate())
        .and(triggers.isDemoMode.negate())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> triggers.driveXSupplier() * 0.5,
                () -> triggers.driveYSupplier() * 0.5,
                () ->
                    RobotState.getInstance()
                        .getShooterAngleToTarget(
                            RobotState.getInstance().getPassTarget().toTranslation2d())));

    triggers
        .trenchAlignButton()
        .whileTrue(
            DriveCommands.joystickDriveAlignForTrench(
                drive, triggers::driveXSupplier, triggers::driveYSupplier));

    // ---- Upper shooter (flywheel + prestage) ----

    // Shoot while clear (our zone, hub active): flywheel tracks the hub distance.
    triggers
        .shootButton()
        .and(triggers.isShootClear)
        .and(triggers.isShotTuningMode.negate())
        .whileTrue(
            FlywheelCommands.setVelocityForHub(flywheel)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, Constants.Setpoints.PRESTAGE_VELOCITY)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage));

    // Shoot outside our zone: flywheel tracks the pass distance.
    triggers
        .shootButton()
        .and(triggers.isShootSafeZone.negate())
        .and(triggers.isShotTuningMode.negate())
        .whileTrue(
            FlywheelCommands.setVelocityForPassing(flywheel)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, Constants.Setpoints.PRESTAGE_VELOCITY)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage));

    // Pass button: a fixed pass, feeding after a fixed wait.
    triggers
        .passButton()
        .whileTrue(fixedShot(Constants.Setpoints.FLYWHEEL_PASS_BUTTON_VELOCITY))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // ---- Lower shooter (feeders + transport) ----
    //
    // Feed when the shoot button is held, not shot-tuning, not in our zone with the hub inactive,
    // and loosely aligned — then wait for spin-up (bounded) before feeding.
    triggers
        .shootButton()
        .and(triggers.isShotTuningMode.negate())
        .and(triggers.isHubInactiveInZone.negate())
        .and(triggers.isAlignedLooser)
        .whileTrue(
            Commands.sequence(
                Commands.waitUntil(triggers.isFlywheelSpunUp.and(triggers.isAlignedLooser))
                    .withTimeout(Constants.Waits.SPIN_UP_TIMEOUT_SECONDS),
                FeederCommands.setUpperFeederVelocity(
                        upperFeeder, Constants.Setpoints.FEEDER_VELOCITY)
                    .alongWith(
                        FeederCommands.setLowerFeederVelocity(
                            lowerFeeder, Constants.Setpoints.FEEDER_VELOCITY))
                    .alongWith(
                        TransportCommands.setTransportVelocity(
                            transport, Constants.Setpoints.TRANSPORT_VELOCITY))))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // ---- Full shooter: fixed shots with constant waits, no alignment or spin-up check ----

    triggers
        .shootFromTowerButton()
        .whileTrue(fixedShot(Constants.Setpoints.FLYWHEEL_TOWER_VELOCITY))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    triggers
        .shootButton()
        .and(triggers.isShotTuningMode)
        .whileTrue(fixedShot(Constants.Setpoints.FLYWHEEL_TUNING_VELOCITY))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    triggers
        .demoDistanceShot()
        .and(triggers.isDemoMode)
        .whileTrue(fixedShot(Constants.Setpoints.FLYWHEEL_TUNING_VELOCITY))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // ---- Intake roller ----

    triggers
        .intakeRollerButton()
        .whileTrue(
            IntakeRollerCommands.setRollerVoltage(
                intakeRoller, Constants.Setpoints.INTAKE_ROLLER_VOLTAGE))
        .onFalse(IntakeRollerCommands.stopIntakeRoller(intakeRoller));

    // Agitate while feeding, under the same conditions as the feed binding.
    triggers
        .shootButton()
        .and(triggers.isShotTuningMode.negate())
        .and(triggers.isHubInactiveInZone.negate())
        .and(triggers.isAlignedLooser)
        .whileTrue(
            IntakeRollerCommands.setRollerVoltage(
                intakeRoller, Constants.Setpoints.INTAKE_ROLLER_AGITATE_VOLTAGE))
        .onFalse(IntakeRollerCommands.stopIntakeRoller(intakeRoller));

    // ---- Intake pivot ----
    //
    // Retract, deploy and manual compress all cancel the automatic compress for the rest of this
    // shoot press (compressCancelled, cleared when the shoot button is released).

    triggers
        .intakeInButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.setPivotPosition(
                intakePivot, Constants.Setpoints.PIVOT_UP_POSITION));

    triggers
        .intakeOutButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.setPivotPosition(
                intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION));

    triggers
        .intakeCompressButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.compressPivot(
                intakePivot, COMPRESS_POSITIONS, () -> doubleCompress))
        .onFalse(
            IntakePivotCommands.setPivotPosition(
                intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION));

    // Automatic compress: when feeding (and not cancelled this press), or on a fixed tower or
    // pass shot. Single unless the operator toggled double.
    triggers
        .shootButton()
        .and(() -> !compressCancelled)
        .and(triggers.isHubInactiveInZone.negate())
        .and(triggers.isAlignedLooser)
        .or(triggers.shootFromTowerButton())
        .or(triggers.passButton())
        .whileTrue(
            Commands.sequence(
                Commands.waitUntil(triggers.isFlywheelSpunUp)
                    .withTimeout(Constants.Waits.SPIN_UP_TIMEOUT_SECONDS),
                IntakePivotCommands.compressPivot(
                    intakePivot, COMPRESS_POSITIONS, () -> doubleCompress)))
        .onFalse(
            IntakePivotCommands.setPivotPosition(
                intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION));

    // ---- Hood ----

    triggers
        .shootButton()
        .and(triggers.isShootClear)
        .and(triggers.isShotTuningMode.negate())
        .whileTrue(HoodCommands.setHoodPosForHub(hood));

    triggers
        .shootFromTowerButton()
        .whileTrue(HoodCommands.setHoodPos(hood, Constants.Setpoints.HOOD_TOWER_POSITION));

    triggers
        .shootButton()
        .and(triggers.isShootSafeZone.negate())
        .and(triggers.isShotTuningMode.negate())
        .or(triggers.passButton())
        .whileTrue(HoodCommands.setHoodPos(hood, Constants.Setpoints.HOOD_PASS_POSITION));

    triggers
        .shootButton()
        .and(triggers.isShotTuningMode)
        .whileTrue(HoodCommands.setHoodPos(hood, Constants.Setpoints.HOOD_TUNING_POSITION));

    triggers
        .demoDistanceShot()
        .and(triggers.isDemoMode)
        .whileTrue(HoodCommands.setHoodPos(hood, Constants.Setpoints.HOOD_DEMO_POSITION));

    // ---- Odometry (added at WVROX) ----
    //
    // Reset the pose to centered in front of the tower, facing 0°.
    //
    // KNOWN 2026 ISSUE, carried over unchanged: this is a RunCommand requiring the drive, so it
    // never ends. After one press the default drive command stays interrupted — the robot will not
    // respond to the sticks — until another drive command (a shoot or trench-align press) takes
    // the drivetrain.
    triggers
        .wvroxOdometryReset()
        .onTrue(
            Commands.run(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                AllianceFlipUtil.applyX(Constants.Setpoints.TOWER_RESET_X_METERS),
                                AllianceFlipUtil.applyY(Constants.Setpoints.TOWER_RESET_Y_METERS),
                                Rotation2d.kZero)),
                    drive)
                .withName("Drive_TowerOdometryReset"));

    // ---- Cancellation flags, cleared when the shoot button is released ----

    triggers
        .shootButton()
        .onFalse(Commands.runOnce(() -> compressCancelled = false))
        .onFalse(Commands.runOnce(() -> xCancelled = false))
        .onFalse(Commands.runOnce(() -> doubleCompress = false));
  }

  /**
   * A fixed shot: flywheel at a set speed and prestage running, then every feed stage after the
   * fixed spin-up wait. No alignment or spin-up check. Used by the tower, pass, tuning and demo
   * shots.
   */
  private Command fixedShot(AngularVelocity flywheelVelocity) {
    return FlywheelCommands.setFlywheelVelocity(flywheel, flywheelVelocity)
        .alongWith(
            PrestageCommands.setPrestageVelocity(prestage, Constants.Setpoints.PRESTAGE_VELOCITY))
        .alongWith(
            FeederCommands.setUpperVelocityAfterWait(
                upperFeeder, Constants.Setpoints.FEEDER_VELOCITY))
        .alongWith(
            FeederCommands.setLowerVelocityAfterWait(
                lowerFeeder, Constants.Setpoints.FEEDER_VELOCITY))
        .alongWith(
            TransportCommands.setVelocityAfterWait(
                transport, Constants.Setpoints.TRANSPORT_VELOCITY));
  }

  // ==============================================================================================
  // SIMULATION BINDINGS — Rebuilt2026 configureSimBindings()
  // ==============================================================================================
  //
  // Driven from a keyboard joystick. These are NOT a copy of the real bindings — 2026 let them
  // diverge: sim uses the tight alignment trigger rather than the loose one, pathfinds to a
  // defensive shot near the trench, uses shoot-on-the-move, the pass-distance hood map rather than
  // the fixed pass angle, and has no intake-roller default command.

  private void configureSimBindings() {
    Triggers triggers = Triggers.getInstance();

    // ---- Default commands ----

    // Raw keyboard axes, clamped and passed through unnegated, as 2026 did.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> MathUtil.clamp(triggers.simXSupplier(), -1.0, 1.0),
            () -> MathUtil.clamp(triggers.simYSupplier(), -1.0, 1.0),
            () -> MathUtil.clamp(triggers.simRotationSupplier(), -1.0, 1.0)));
    flywheel.setDefaultCommand(
        FlywheelCommands.flywheelIdle(flywheel, Constants.Setpoints.FLYWHEEL_IDLE_VELOCITY));
    hood.setDefaultCommand(HoodCommands.hoodIdle(hood, Constants.Setpoints.HOOD_DOWN_POSITION));

    // ---- Overrides ----

    triggers.simAllianceWinFlipper().onTrue(HubShiftUtil.flipWinner());
    triggers.allianceWinDisabler().onTrue(HubShiftUtil.disableHubShiftUtil());
    triggers.autoXOverride().onTrue(Commands.runOnce(() -> xCancelled = true));

    // ---- Drivetrain ----

    // Aim at the hub, unless approaching the alliance trench (then the defensive shot below).
    triggers
        .simShootButton()
        .and(triggers.isShootClear)
        .and(
            () ->
                RobotState.getInstance()
                        .getApproachingZoneX(RobotState.getInstance().getEstimatedPose())
                    != ApproachingZoneX.APPROACHING_ALLIANCE_TRENCH)
        .or(triggers.simShootFromTowerButton())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                triggers::simXSupplier,
                triggers::simYSupplier,
                () -> RobotState.getInstance().getAngleToAllianceHub()));

    triggers
        .simShootButton()
        .and(triggers.isShootClear)
        .and(
            () ->
                RobotState.getInstance()
                        .getApproachingZoneX(RobotState.getInstance().getEstimatedPose())
                    == ApproachingZoneX.APPROACHING_ALLIANCE_TRENCH)
        .whileTrue(DriveCommands.alignForDefenseShot(drive));

    triggers
        .simShootButton()
        .and(triggers.isShootSafeZone.negate())
        .or(triggers.simPassButton())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                triggers::simXSupplier,
                triggers::simYSupplier,
                () ->
                    RobotState.getInstance()
                        .getShooterAngleToTarget(
                            RobotState.getInstance().getPassTarget().toTranslation2d())));

    // X when shooting and aligned (unless cancelled), or on the X-wheels button.
    triggers
        .simShootButton()
        .and(triggers.isShootClear)
        .and(triggers.isAlignedForCurrentShot)
        .and(() -> !xCancelled)
        .or(triggers.xWheels())
        .whileTrue(DriveCommands.stopWithX(drive));

    triggers
        .simTrenchAlignButton()
        .whileTrue(
            DriveCommands.joystickDriveAlignForTrench(
                drive, triggers::simXSupplier, triggers::simYSupplier));

    triggers
        .simBumpAlignButton()
        .whileTrue(
            DriveCommands.joystickDriveAlignForBump(
                drive, triggers::simXSupplier, triggers::simYSupplier));

    // ---- Upper shooter ----

    // Shoot-on-the-move. The hood angle is read ONCE, here, at startup — as in 2026.
    triggers
        .simShootButton()
        .and(triggers.isShootClear)
        .and(triggers.isShotTuningMode.negate())
        .whileTrue(
            FlywheelCommands.shootOnTheMove(
                    flywheel, HoodPosCalculator.getInstance().getHoodPosForHub().magnitude())
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, Constants.Setpoints.PRESTAGE_VELOCITY)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage));

    triggers
        .simShootButton()
        .and(triggers.isShootSafeZone.negate())
        .and(triggers.isShotTuningMode.negate())
        .or(triggers.simPassButton())
        .whileTrue(
            FlywheelCommands.setVelocityForPassing(flywheel)
                .alongWith(
                    PrestageCommands.setPrestageVelocity(
                        prestage, Constants.Setpoints.PRESTAGE_VELOCITY)))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage));

    // ---- Lower shooter ----

    triggers
        .simShootButton()
        .and(triggers.isShootClear)
        .and(triggers.isShotTuningMode.negate())
        .and(triggers.isAlignedForCurrentShot)
        .whileTrue(feedAll())
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    triggers
        .simShootButton()
        .and(triggers.isShootSafeZone.negate())
        .and(triggers.isShotTuningMode.negate())
        .and(triggers.isAlignedForCurrentShot)
        .whileTrue(feedAll())
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // ---- Full shooter ----

    triggers
        .simShootFromTowerButton()
        .whileTrue(fixedShot(Constants.Setpoints.FLYWHEEL_TOWER_VELOCITY))
        .onFalse(FlywheelCommands.stop(flywheel))
        .onFalse(PrestageCommands.stop(prestage))
        .onFalse(FeederCommands.stopLower(lowerFeeder))
        .onFalse(FeederCommands.stopUpper(upperFeeder))
        .onFalse(TransportCommands.stop(transport));

    // ---- Intake roller ----

    triggers
        .simIntakeRollerButton()
        .whileTrue(
            IntakeRollerCommands.setRollerVoltage(
                intakeRoller, Constants.Setpoints.INTAKE_ROLLER_VOLTAGE))
        .onFalse(IntakeRollerCommands.stopIntakeRoller(intakeRoller));

    triggers
        .simShootButton()
        .or(triggers.simPassButton())
        .or(triggers.simShootFromTowerButton())
        .whileTrue(
            IntakeRollerCommands.setVoltageAfterWait(
                intakeRoller,
                Constants.Setpoints.INTAKE_ROLLER_AGITATE_VOLTAGE,
                triggers.isAlignedForCurrentShot))
        .onFalse(IntakeRollerCommands.stopIntakeRoller(intakeRoller));

    // ---- Intake pivot ----

    triggers
        .simIntakeInButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.setPivotPosition(
                intakePivot, Constants.Setpoints.PIVOT_UP_POSITION));

    triggers
        .simIntakeOutButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(
            IntakePivotCommands.setPivotPosition(
                intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION));

    triggers
        .simIntakeCompressButton()
        .onTrue(Commands.runOnce(() -> compressCancelled = true))
        .whileTrue(IntakePivotCommands.manualPivotCompress(intakePivot, COMPRESS_POSITIONS))
        .onFalse(
            IntakePivotCommands.setPivotPosition(
                intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION));

    triggers
        .simShootButton()
        .and(() -> !compressCancelled)
        .and(triggers.isHubInactiveInZone.negate())
        .and(triggers.isAlignedForCurrentShot)
        .whileTrue(IntakePivotCommands.compressPivot(intakePivot, COMPRESS_POSITIONS))
        .onFalse(
            IntakePivotCommands.setPivotPosition(
                intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION));

    // ---- Hood ----

    triggers
        .simShootButton()
        .or(triggers.simShootFromTowerButton())
        .and(triggers.isShootClear)
        .and(triggers.isShotTuningMode.negate())
        .whileTrue(HoodCommands.setHoodPosForHub(hood));

    triggers
        .simShootButton()
        .and(triggers.isShootSafeZone.negate())
        .and(triggers.isShotTuningMode.negate())
        .or(triggers.simPassButton())
        .whileTrue(HoodCommands.setPosForPassing(hood));

    // ---- Cancellation flags ----
    //
    // Cleared on release of the REAL shoot button, not the sim one — as in 2026.
    triggers
        .shootButton()
        .onFalse(Commands.runOnce(() -> compressCancelled = false))
        .onFalse(Commands.runOnce(() -> xCancelled = false));
  }

  /** Every feed stage at feeding speed, immediately. */
  private Command feedAll() {
    return FeederCommands.setUpperFeederVelocity(upperFeeder, Constants.Setpoints.FEEDER_VELOCITY)
        .alongWith(
            FeederCommands.setLowerFeederVelocity(lowerFeeder, Constants.Setpoints.FEEDER_VELOCITY))
        .alongWith(
            TransportCommands.setTransportVelocity(
                transport, Constants.Setpoints.TRANSPORT_VELOCITY));
  }

  // ==============================================================================================
  // ROBOT-LEVEL HOOKS — called from Robot
  // ==============================================================================================

  /** The selected auto, after the dashboard "Auto Delay". Proxied, as in 2026. */
  public Command getAutonomousCommand() {
    double delay =
        SmartDashboard.getNumber(AUTO_DELAY_KEY, Constants.Waits.DEFAULT_AUTO_DELAY_SECONDS);
    Command auto = autoChooser.get().asProxy();
    return Commands.sequence(Commands.waitSeconds(delay), auto);
  }

  /** Scheduled at teleopInit: stop every shooter mechanism left running by auto. */
  public Command getAutoStopCommand() {
    return ShootSequences.stopAll(
        flywheel, prestage, hood, upperFeeder, lowerFeeder, transport, intakeRoller);
  }

  /**
   * Scheduled at teleopInit: run the intake roller at intake voltage. It is a startEnd, so it runs
   * until something else takes the intake roller.
   */
  public Command getIntakeRollerCommand() {
    return IntakeRollerCommands.setRollerVoltage(
        intakeRoller, Constants.Setpoints.INTAKE_ROLLER_VOLTAGE);
  }

  /** Scheduled at teleopInit: deploy the intake. */
  public Command getIntakePivotCommand() {
    return IntakePivotCommands.setPivotPosition(
        intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION);
  }

  public double getDriverPreset() {
    return driverPresetChooser.get();
  }

  /** Currently selected drive controller. Latched into Triggers at teleopInit. */
  public boolean isXboxDriveSelected() {
    return driveControllerChooser.get();
  }

  public boolean isFlywheelSpunUp() {
    return flywheel.isSpunUp();
  }

  public boolean isDriveXed() {
    return drive.areWheelsXed;
  }

  public void updateRobotModelVisualizer() {
    robotModelVisualizer.update();
  }

  public AutoPreview autoPreview() {
    return autoPreview;
  }

  /** Brake/coast passthrough for {@link Robot}'s disabled-coast handling. */
  public void setDriveBrakeMode(boolean brake) {
    drive.setDriveBrakeMode(brake);
  }
}
