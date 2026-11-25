package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.SchedulerEvent;
import org.wpilib.system.Timer;

/**
 * An Autonomous OpMode: one BLine routine on one side. Carries what {@code Robot} used to do in its
 * autonomous hooks — the Left/Right latch, the auto preview, and the auto-duration log.
 *
 * <p>The duration log matters: the 2026 routines overran the auto period and the last path was
 * truncated in every match; nobody measured it until the season was over.
 */
public class AutoOpMode extends CommandOpMode {
  private final RobotContainer container;
  private final Supplier<Command> routine;
  private final boolean mirror;
  private final Timer autoTimer = new Timer();
  private boolean durationLogged = false;

  public AutoOpMode(
      Robot robot,
      RobotContainer container,
      String name,
      Supplier<Command> routine,
      boolean mirror) {
    super(robot, name, routine);
    this.container = container;
    this.routine = routine;
    this.mirror = mirror;
  }

  /** While this OpMode is selected and the robot is disabled: draw the auto as it will run. */
  @Override
  public void disabledPeriodic() {
    container.updateAutoPreview(container.pathsOf(routine), mirror);
  }

  @Override
  public void start() {
    // Latch the side before the routine is built; the builder reads it at each path's start.
    container.latchMirror(mirror);
    autoTimer.restart();
    durationLogged = false;
    robot.trackAuto(this);
    super.start();
  }

  @Override
  public void periodic() {
    container.showAutoRobot();
  }

  @Override
  protected void onScheduleRejected() {
    logDuration("Rejected", false);
  }

  public void onSchedulerEvent(SchedulerEvent event) {
    if (durationLogged) {
      return;
    }
    switch (event) {
      case SchedulerEvent.Completed e when e.command() == running ->
          logDuration("Completed", false);
      case SchedulerEvent.CompletedWithError e when e.command() == running ->
          logDuration("Failed", false);
      case SchedulerEvent.Interrupted e when e.command() == running ->
          logDuration("Interrupted", false);
      case SchedulerEvent.Canceled e when e.command() == running -> logDuration("Canceled", false);
      default -> {}
    }
  }

  @Override
  public void end() {
    if (!durationLogged) {
      // WPILib does not provide an end reason. Report that the OpMode ended with work pending.
      logDuration("OpModeEndedWhileRunning", true);
    }
    robot.trackAuto(null);
    super.end();
  }

  @Override
  public void close() {
    // Also called when the selection changes before this auto has been enabled.
    container.updateAutoPreview(List.of(), false);
  }

  private void logDuration(String outcome, boolean overran) {
    durationLogged = true;
    double seconds = autoTimer.get();
    Logger.recordOutput("Auto/DurationSeconds", seconds);
    Logger.recordOutput("Auto/Overran", overran);
    Logger.recordOutput("Auto/Outcome", outcome);
    Logger.recordOutput("Auto/Name", name + (mirror ? " (Right)" : " (Left)"));
    System.out.println("Auto " + outcome + " after " + String.format("%.2f", seconds) + " s");
  }
}
