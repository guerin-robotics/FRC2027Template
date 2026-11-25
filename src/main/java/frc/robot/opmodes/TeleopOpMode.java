package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.opmode.OpMode;

/**
 * The only teleop OpMode. Driving is bound globally in {@code RobotContainer} (the default drive
 * command and the Triggers), so all that is left for the OpMode is the work every enable needs.
 */
public class TeleopOpMode implements OpMode {
  private final Robot robot;

  public TeleopOpMode(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    robot.onEnable();
  }
}
