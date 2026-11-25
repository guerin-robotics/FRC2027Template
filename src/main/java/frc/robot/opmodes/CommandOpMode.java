package frc.robot.opmodes;

import frc.robot.Robot;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.OpMode;

/**
 * An OpMode that runs one command from enable to disable. The command is built when the OpMode is
 * enabled, so only the selected routine loads, and is canceled when the OpMode ends.
 *
 * <p>Used directly for Utility routines; {@link AutoOpMode} extends it for BLine autos.
 */
public class CommandOpMode implements OpMode {
  protected final Robot robot;
  protected final String name;
  private final Supplier<Command> routine;
  protected Command running;

  public CommandOpMode(Robot robot, String name, Supplier<Command> routine) {
    this.robot = robot;
    this.name = name;
    this.routine = routine;
  }

  @Override
  public void start() {
    robot.onEnable();
    running = routine.get();
    if (Scheduler.getDefault().schedule(running)
        instanceof Scheduler.ScheduleResult.Failure failure) {
      Logger.recordOutput("OpMode/ScheduleFailure", name + ": " + failure);
      onScheduleRejected();
    }
  }

  protected void onScheduleRejected() {}

  @Override
  public void end() {
    if (running != null) {
      Scheduler.getDefault().cancel(running);
    }
  }
}
