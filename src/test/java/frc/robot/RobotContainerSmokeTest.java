// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Builds a real {@link RobotContainer} and asserts the wiring came out intact.
 *
 * <p>Everything {@code RobotContainer} does happens in a constructor: subsystems are built, {@code
 * AutoBuilder} is configured, fault conditions are registered, the auto chooser is assembled, and
 * every button binding is made. None of that is exercised by a unit test of any individual class,
 * and all of it runs exactly once — at robot boot, on the field. A null supplier, a subsystem
 * constructed in the wrong order, or a chooser built before its named commands were registered
 * throws or silently misbehaves there and nowhere else.
 *
 * <p>This is the guardrail on {@code /add-subsystem} in particular. That skill scaffolds wiring
 * into all three mode branches; this test is what proves the scaffold it produced actually
 * constructs.
 *
 * <p><b>Runs in SIM.</b> {@code Constants.currentMode} resolves to {@code SIM} off a roboRIO, so
 * the physics-sim IO implementations are built — including four {@code VisionIOPhotonVisionSim}
 * cameras. The {@code REAL} branch cannot be covered here; it constructs Phoenix device objects
 * against hardware that is not present.
 *
 * <p><b>Cleanup matters.</b> {@code CommandScheduler} is a JVM-wide singleton shared with every
 * other test in this module, and {@code SubsystemBase}'s constructor registers into it. Left alone,
 * the {@code Drive} and {@code Vision} built here would keep running {@code periodic()} during
 * {@code DriveToPoseSimTest} and friends. {@link #unregisterSubsystems()} is what prevents that —
 * see the same warning on {@code DriveToPoseSimTest}.
 */
class RobotContainerSmokeTest {

  private static RobotContainer container;

  @BeforeAll
  static void buildContainer() {
    // See GainSweepTest's javadoc for why this must not be inside a Java assert.
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    // Logger.start() normally exits the JVM unless the main class extends LoggedRobot, and
    // Vision.periodic()/Drive.periodic() call Logger.processInputs unconditionally.
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();

    container = new RobotContainer();
  }

  @AfterAll
  static void unregisterSubsystems() {
    CommandScheduler.getInstance().cancelAll();
    // Only the subsystems this class created. unregisterAllSubsystems() would also drop any a
    // later test builds for itself, which is not this class's to decide.
    CommandScheduler.getInstance().unregisterSubsystem(wiredSubsystems().toArray(new Subsystem[0]));
  }

  @Test
  void containerConstructsWithoutThrowing() {
    // Reaching @BeforeAll's last line is the assertion; this makes the intent visible in the
    // report rather than leaving it as a side effect of some other test happening to run.
    assertNotNull(container, "RobotContainer failed to construct in SIM mode");
  }

  @Test
  void autonomousCommandIsAvailableAfterOneDashboardCycle() {
    // LoggedDashboardChooser resolves its selection in periodic(), and its two-argument
    // constructor calls periodic() BEFORE copying the options across from the SendableChooser it
    // was handed. So a freshly constructed chooser holds a null selection and get() returns null
    // until something pumps it again. On the robot that happens on the first loop, inside
    // Logger.periodicBeforeUser() — which is package-private and cannot be called from here.
    // Pumping the chooser directly is the same code path the robot takes.
    autoChooser().periodic();

    Command auto = container.getAutonomousCommand();
    assertNotNull(
        auto,
        "The auto chooser yielded no command after a dashboard cycle."
            + " AutoBuilder.buildAutoChooser() always installs a \"None\" default that maps to"
            + " Commands.none(), so null here means the chooser was never built or its options"
            + " were not copied — and Robot.autonomousInit would schedule nothing.");
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
        defaultCommand.getName(),
        "Drive's default command is not the joystick drive. Whatever is there now will run"
            + " whenever nothing else requires the drivetrain, including the moment teleop"
            + " starts.");
  }

  @Test
  void everyDefaultCommandIsNamed() {
    // Command names are what appear in the AdvantageKit command log. A command that never got a
    // .withName() logs under its class name, which for a factory-built command is an anonymous
    // inner class like "frc.robot.commands.DriveCommands$1" — see .claude/rules/03-commands.md.
    List<String> unnamed =
        wiredSubsystems().stream()
            .map(CommandScheduler.getInstance()::getDefaultCommand)
            .filter(command -> command != null)
            .map(Command::getName)
            .filter(name -> name.contains("$") || name.isBlank())
            .toList();

    assertTrue(
        unnamed.isEmpty(),
        () -> "These default commands were never given a .withName(): " + unnamed);
  }

  @Test
  void everyNamedCommandReferencedByAnAutoIsRegistered() {
    Set<String> referenced = PathPlannerAssets.namedCommandsReferencedByAutos();
    List<String> unregistered =
        referenced.stream().filter(name -> !NamedCommands.hasCommand(name)).toList();

    assertTrue(
        unregistered.isEmpty(),
        () ->
            "Autos reference named commands that were never registered: "
                + unregistered
                + "\n\nThis does not throw on the robot. NamedCommands.getCommand() logs a warning"
                + " to the driver station and substitutes Commands.none(), so the auto drives its"
                + " paths on schedule while the mechanism does nothing — the failure looks like a"
                + " broken mechanism, not a wiring mistake."
                + "\n\nRegister it in RobotContainer BEFORE AutoBuilder.buildAutoChooser(): the"
                + " chooser resolves named commands at build time, and anything registered after"
                + " that line is ignored (.claude/rules/01-architecture.md).");
  }

  /**
   * Every subsystem {@link RobotContainer} holds a field for.
   *
   * <p>Read by reflection rather than from the {@code CommandScheduler}, which exposes no accessor
   * for its registered subsystems. Reading the container's own fields is also the more meaningful
   * set: it is exactly what the wiring layer built, with no leftovers from another test class.
   *
   * <p>A subsystem added to {@code RobotContainer} is covered here automatically — nothing in this
   * file names {@code Drive} or {@code Vision}.
   */
  private static List<Subsystem> wiredSubsystems() {
    List<Subsystem> subsystems = new ArrayList<>();
    for (Field field : RobotContainer.class.getDeclaredFields()) {
      if (!Subsystem.class.isAssignableFrom(field.getType())) {
        continue;
      }
      field.setAccessible(true);
      try {
        Subsystem subsystem = (Subsystem) field.get(container);
        if (subsystem != null) {
          subsystems.add(subsystem);
        }
      } catch (IllegalAccessException e) {
        throw new AssertionError("Could not read RobotContainer." + field.getName(), e);
      }
    }
    return subsystems;
  }

  /** {@link RobotContainer}'s auto chooser, which it does not expose. */
  @SuppressWarnings("unchecked")
  private static LoggedDashboardChooser<Command> autoChooser() {
    return (LoggedDashboardChooser<Command>) readField("autoChooser");
  }

  private static Object readField(String name) {
    try {
      Field field = RobotContainer.class.getDeclaredField(name);
      field.setAccessible(true);
      return field.get(container);
    } catch (NoSuchFieldException | IllegalAccessException e) {
      throw new AssertionError("Could not read RobotContainer." + name, e);
    }
  }

  /** Finds the single wired subsystem of the given type, failing if it is not there. */
  private static <T extends Subsystem> T findSubsystem(Class<T> type) {
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
