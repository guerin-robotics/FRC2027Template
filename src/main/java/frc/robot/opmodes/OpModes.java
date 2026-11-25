package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import org.wpilib.hardware.hal.RobotMode;

/**
 * Registers every OpMode the robot offers. Called once from the {@code Robot} constructor.
 *
 * <p>2027 selects the robot program on the Driver Station, not on a dashboard chooser:
 *
 * <ul>
 *   <li><b>Teleoperated "Drive"</b> — the only teleop OpMode. Bindings are global (made in {@code
 *       RobotContainer}), so it only does the on-enable work.
 *   <li><b>Autonomous</b> — one OpMode per BLine routine <i>per side</i>: {@code "<routine>
 *       (Left)"} and {@code "<routine> (Right)"}. Paths are authored blue-left; the Right OpMode
 *       mirrors them. This replaces the old "Auto Choices" and "Auto Side" dashboard choosers. The
 *       selection is logged by AdvantageKit's OpMode support, so it replays.
 *   <li><b>Utility</b> — characterization and SysId routines. Nothing here is reachable from
 *       Teleoperated or Autonomous, which is what the old held-button test-mode gate protected.
 * </ul>
 */
public final class OpModes {

  private OpModes() {}

  public static void register(Robot robot, RobotContainer container) {
    robot.addOpMode(RobotMode.TELEOPERATED, "Drive", () -> new TeleopOpMode(robot));

    container
        .autoRoutines()
        .forEach(
            (name, routine) -> {
              robot.addOpMode(
                  RobotMode.AUTONOMOUS,
                  name + " (Left)",
                  "Auto",
                  () -> new AutoOpMode(robot, container, name, routine, false));
              robot.addOpMode(
                  RobotMode.AUTONOMOUS,
                  name + " (Right)",
                  "Auto",
                  () -> new AutoOpMode(robot, container, name, routine, true));
            });

    container
        .utilityRoutines()
        .forEach(
            (name, routine) ->
                robot.addOpMode(
                    RobotMode.UTILITY,
                    name,
                    "Characterization",
                    () -> new CommandOpMode(robot, name, routine)));
  }
}
