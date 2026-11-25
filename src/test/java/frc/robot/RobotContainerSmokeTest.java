// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import bline.commands.FollowPath;
import bline.path.Path;
import frc.robot.autos.AutoPreview;
import frc.robot.autos.BLineAutos;
import frc.robot.commands.DriveCommands;
import frc.robot.opmodes.AutoOpMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.RoboRioSim;

/**
 * Builds a real {@link RobotContainer} and asserts the wiring came out intact.
 *
 * <p>Everything {@code RobotContainer} does happens in a constructor: subsystems are built, the
 * BLine {@code FollowPath.Builder} is created, fault conditions are registered, the OpMode routine
 * maps are assembled, and every button binding is made. None of that is exercised by a unit test of
 * any individual class, and all of it runs exactly once — at robot boot, on the field. A null
 * supplier, a subsystem constructed in the wrong order, or an event key nothing was bound to throws
 * or silently misbehaves there and nowhere else.
 *
 * <p>This is the guardrail on {@code /add-subsystem} in particular. That skill scaffolds wiring
 * into all three mode branches; this test is what proves the scaffold it produced actually
 * constructs.
 *
 * <p><b>The processInputs test is the one that matters most.</b> Commands V3 has no {@code
 * Subsystem.periodic()}, so a subsystem that {@code RobotContainer.periodic()} forgets to call
 * simply never runs {@code Logger.processInputs()}: no inputs, no replay, no error. {@link
 * #everySubsystemProcessesItsInputsEveryLoop} fails when that happens.
 *
 * <p><b>Runs in SIM.</b> {@code Constants.currentMode} resolves to {@code SIM} off a robot, so the
 * physics-sim IO implementations are built — including four {@code VisionIOPhotonVisionSim}
 * cameras. The {@code REAL} branch cannot be covered here; it constructs Phoenix device objects
 * against hardware that is not present.
 *
 * <p><b>Cleanup matters.</b> {@code Scheduler.getDefault()} is JVM-wide and shared with every other
 * test in this module, and {@code RobotContainer} installs default commands on it. {@link
 * #cancelEverything()} cancels what this class started.
 */
class RobotContainerSmokeTest {

  private static RobotContainer container;

  @BeforeAll
  static void buildContainer() {
    // JUnit's assertTrue, deliberately not the Java `assert` keyword. It always runs and fails
    // right here if HAL.initialize returns false. An `assert` statement is skipped entirely
    // without -ea, which would skip the initialization and fail later inside the HAL instead.
    assertTrue(HAL.initialize(), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);
    DriverStationSim.setEnabled(false);

    // Declare both controllers present. configureButtonBindings() polls flight-stick buttons
    // through Triggers, and an undeclared joystick makes the DriverStation warn that the button
    // is unavailable. On a real robot both controllers are plugged in for every match — see the
    // note on Constants.Controllers about verifying ports in the DS USB tab — so modelling them
    // as present is what matches reality. Counts are generous rather than exact so that adding a
    // binding does not silently reintroduce the warning.
    DriverStationSim.setJoystickButtonsMaximumIndex(Constants.Controllers.FLIGHT_STICK_PORT, 16);
    DriverStationSim.setJoystickButtonsAvailable(Constants.Controllers.FLIGHT_STICK_PORT, 0xFFFFL);
    DriverStationSim.setJoystickAxesMaximumIndex(Constants.Controllers.FLIGHT_STICK_PORT, 6);
    DriverStationSim.setJoystickAxesAvailable(Constants.Controllers.FLIGHT_STICK_PORT, 0x3F);
    DriverStationSim.setJoystickButtonsMaximumIndex(Constants.Controllers.XBOX_PORT, 16);
    DriverStationSim.setJoystickButtonsAvailable(Constants.Controllers.XBOX_PORT, 0xFFFFL);
    DriverStationSim.setJoystickAxesMaximumIndex(Constants.Controllers.XBOX_PORT, 6);
    DriverStationSim.setJoystickAxesAvailable(Constants.Controllers.XBOX_PORT, 0x3F);
    DriverStationSim.notifyNewData();

    // Logger.start() normally exits the JVM unless the main class extends a Logged robot base, and
    // Vision.periodic()/Drive.periodic() call Logger.processInputs unconditionally.
    TestLogging.start();

    container = new RobotContainer();
  }

  @AfterAll
  static void cancelEverything() {
    Scheduler.getDefault().cancelAll();
    // Alert ids are unique per process in 2027; release Vision's so a later test can build its own.
    findSubsystem(Vision.class).close();
  }

  @Test
  void containerConstructsWithoutThrowing() {
    // Reaching @BeforeAll's last line is the assertion; this makes the intent visible in the
    // report rather than leaving it as a side effect of some other test happening to run.
    assertNotNull(container, "RobotContainer failed to construct in SIM mode");
  }

  @Test
  void everySubsystemProcessesItsInputsEveryLoop() throws InterruptedException {
    // THE V3 GATE. One loop exactly as Robot.robotPeriodic() runs it: AdvantageKit's before-user
    // hook, RobotContainer.periodic(), then AdvantageKit's after-user hook, which hands the
    // completed table to the receiver above. Every subsystem the container holds must have put
    // its inputs under "/<ClassName>/..." (AdvantageKit 27 logs inputs at the root; outputs go
    // under /RealOutputs) — the convention each subsystem follows when it calls
    // Logger.processInputs. A subsystem missing here is one RobotContainer.periodic()
    // does not call.
    TestLogging.keys().clear();
    Logger.AdvancedHooks.invokePeriodicBeforeUser();
    container.periodic();
    Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
    // The receiver runs on AdvantageKit's thread; wait briefly for the table to arrive.
    for (int i = 0;
        i < 150 && TestLogging.keys().stream().noneMatch(key -> key.startsWith("/Drive/"));
        i++) {
      Thread.sleep(20);
    }

    List<String> missing = new ArrayList<>();
    for (Object subsystem : wiredSubsystems()) {
      String prefix = "/" + subsystem.getClass().getSimpleName() + "/";
      if (TestLogging.keys().stream().noneMatch(key -> key.startsWith(prefix))) {
        missing.add(subsystem.getClass().getSimpleName());
      }
    }
    assertTrue(
        missing.isEmpty(),
        () ->
            "These subsystems logged no inputs in a loop: "
                + missing
                + "\n\nCommands V3 calls no periodic() for you. Add each one to"
                + " RobotContainer.periodic() — without it Logger.processInputs() never runs,"
                + " and replay is silently broken (.claude/rules/00-safety.md).");
  }

  @Test
  void everyOpModeRoutineBuildsACommand() {
    // Each routine is a supplier called when its OpMode is enabled. A supplier that returns null
    // or throws is only found out when the drive team selects that OpMode.
    List<String> broken = new ArrayList<>();
    container
        .autoRoutines()
        .forEach((name, routine) -> checkRoutine("Autonomous " + name, routine, broken));
    container
        .utilityRoutines()
        .forEach((name, routine) -> checkRoutine("Utility " + name, routine, broken));
    assertTrue(broken.isEmpty(), () -> "These OpMode routines did not build a command: " + broken);
  }

  private static void checkRoutine(String label, Supplier<Command> routine, List<String> broken) {
    try {
      if (routine.get() == null) {
        broken.add(label + " (returned null)");
      }
    } catch (RuntimeException e) {
      broken.add(label + " (" + e + ")");
    }
  }

  @Test
  void autoPreviewSurvivesNoAutoAndAMissingPathFile() {
    // Runs every disabled loop on the robot: neither an empty selection nor a path file that is
    // not deployed may throw, or the drive team loses the start-pose check.
    container.updateAutoPreview(List.of(), false);
    new AutoPreview().update(List.of("no-such-path"), false, true, new Pose2d());
  }

  @Test
  void deselectingAutoBeforeEnableClearsPreview() throws ReflectiveOperationException {
    Field field = RobotContainer.class.getDeclaredField("autoPreview");
    field.setAccessible(true);
    AutoPreview preview = (AutoPreview) field.get(container);
    preview.showActivePath(new Translation2d[] {new Translation2d(1.0, 2.0)});
    assertFalse(preview.getField().getObject("activePath").getPoses().isEmpty());

    // WPILib closes a deselected OpMode even if it never called start() or end().
    new AutoOpMode(null, container, "Preview test", () -> null, false).close();

    assertTrue(preview.getField().getObject("activePath").getPoses().isEmpty());
  }

  @Test
  void gyroResetWorksDisabledAndInterruptsDriveWhenEnabled() {
    Drive drive = findSubsystem(Drive.class);
    Command reset = DriveCommands.resetGyro(drive);
    Command owner = drive.run(co -> co.park()).named("Test_Drive_Owner");
    Scheduler scheduler = Scheduler.getDefault();
    Pose2d startingPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(40.0));
    try {
      drive.setPose(startingPose);
      scheduler.schedule(reset);
      scheduler.run();
      assertEquals(startingPose.getTranslation(), drive.getPose().getTranslation());
      assertEquals(0.0, drive.getPose().getRotation().getDegrees(), 1e-9);

      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      drive.setPose(startingPose);
      scheduler.schedule(owner);
      scheduler.run();
      assertTrue(scheduler.isScheduledOrRunning(owner));

      scheduler.schedule(reset);
      scheduler.run();
      assertFalse(scheduler.isScheduledOrRunning(owner));
      assertEquals(startingPose.getTranslation(), drive.getPose().getTranslation());
      assertEquals(0.0, drive.getPose().getRotation().getDegrees(), 1e-9);
    } finally {
      scheduler.cancelAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      drive.setPose(Pose2d.ZERO);
    }
  }

  @Test
  void blineLoggingReachesThePreviewWidget() {
    // createBuilder is the one place BLine's logging consumers are installed. Running a path for
    // one cycle makes FollowPath publish FollowPath/pathTranslations, which must reach the widget;
    // if a consumer were never set, the active path would stay empty and nothing would log.
    Drive drive = findSubsystem(Drive.class);
    AutoPreview preview = new AutoPreview();
    FollowPath.Builder builder = BLineAutos.createBuilder(drive, () -> false, preview);
    Command follow =
        BLineAutos.follow(
            builder,
            new Path(new Path.Waypoint(new Translation2d(2.0, 2.0), 0.0, Rotation2d.ZERO)));

    // V3 will not schedule a command that requires Drive while disabled (Drive must not be
    // controllableDuringDisabled), so enable the sim Driver Station for this one cycle.
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Scheduler scheduler = Scheduler.getDefault();
    try {
      scheduler.schedule(follow);
      scheduler.run();
      scheduler.cancel(follow);
    } finally {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }

    assertTrue(
        !preview.getField().getObject("activePath").getPoses().isEmpty(),
        "FollowPath's pathTranslations never reached the Auto Preview widget — a BLine logging"
            + " consumer is not installed in BLineAutos.createBuilder.");
  }

  @Test
  void driveHasItsDefaultCommand() {
    Drive drive = findSubsystem(Drive.class);
    Command defaultCommand = drive.getDefaultCommand();

    assertNotNull(
        defaultCommand,
        "Drive has no default command. Without one the drivetrain does not respond to the sticks"
            + " at all whenever no other command requires it — see .claude/rules/03-commands.md,"
            + " which names joystickDrive as non-removable.");
    assertEquals(
        "Drive_Joystick",
        defaultCommand.name(),
        "Drive's default command is not the joystick drive. Whatever is there now will run"
            + " whenever nothing else requires the drivetrain, including the moment teleop"
            + " starts.");
  }

  @Test
  void everyDefaultCommandYieldsToOrdinaryCommands() {
    // V3 interrupts a running command only when the newcomer's priority is EQUAL OR HIGHER. A
    // default command at or above DEFAULT_PRIORITY blocks every ordinary command from ever taking
    // its mechanism. Defaults sit below.
    List<String> blocking =
        wiredMechanisms().stream()
            .map(Mechanism::getDefaultCommand)
            .filter(command -> command != null)
            .filter(command -> command.priority() >= Command.DEFAULT_PRIORITY)
            .map(Command::name)
            .toList();

    assertTrue(
        blocking.isEmpty(),
        () ->
            "These default commands are at or above Command.DEFAULT_PRIORITY and will block every"
                + " ordinary command on their mechanism: "
                + blocking
                + "\n\nGive them .withPriority(Command.LOWEST_PRIORITY).");
  }

  @Test
  void everyBLineEventKeyIsBound() {
    Set<String> referenced = BLineAssets.eventKeysReferencedByPaths();
    List<String> unbound =
        referenced.stream().filter(name -> !BLineAutos.boundEvents().contains(name)).toList();

    assertTrue(
        unbound.isEmpty(),
        () ->
            "BLine paths use event keys nothing is bound to: "
                + unbound
                + "\n\nThis does not throw on the robot. BLine-Lib reaches the trigger, finds no"
                + " registration, and only warns — the auto follows its path on schedule while"
                + " the mechanism sits still, which looks like a broken mechanism, not a wiring"
                + " mistake."
                + "\n\nBind it with BLineAutos.bind(...) in RobotContainer"
                + " (.claude/rules/01-architecture.md).");
  }

  /**
   * Every subsystem {@link RobotContainer} holds a field for: anything with a public {@code
   * periodic()}, which is the contract {@code RobotContainer.periodic()} calls.
   *
   * <p>Read by reflection so a subsystem added to {@code RobotContainer} is covered here
   * automatically — nothing in this file names {@code Drive} or {@code Vision}.
   */
  private static List<Object> wiredSubsystems() {
    List<Object> subsystems = new ArrayList<>();
    for (Field field : RobotContainer.class.getDeclaredFields()) {
      if (!hasPublicPeriodic(field.getType())) {
        continue;
      }
      field.setAccessible(true);
      try {
        Object subsystem = field.get(container);
        if (subsystem != null) {
          subsystems.add(subsystem);
        }
      } catch (IllegalAccessException e) {
        throw new AssertionError("Could not read RobotContainer." + field.getName(), e);
      }
    }
    return subsystems;
  }

  private static boolean hasPublicPeriodic(Class<?> type) {
    try {
      Method periodic = type.getMethod("periodic");
      return periodic.getParameterCount() == 0;
    } catch (NoSuchMethodException e) {
      return false;
    }
  }

  /** The wired subsystems that are V3 mechanisms ({@code Vision} is not one). */
  private static List<Mechanism> wiredMechanisms() {
    return wiredSubsystems().stream()
        .filter(Mechanism.class::isInstance)
        .map(Mechanism.class::cast)
        .toList();
  }

  /** Finds the single wired subsystem of the given type, failing if it is not there. */
  private static <T> T findSubsystem(Class<T> type) {
    return wiredSubsystems().stream()
        .filter(type::isInstance)
        .map(type::cast)
        .findFirst()
        .orElseThrow(
            () ->
                new AssertionError(
                    "RobotContainer has no " + type.getSimpleName() + " field, or it was null"));
  }
}
