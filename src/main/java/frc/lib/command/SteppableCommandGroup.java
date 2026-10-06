package frc.lib.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Runs commands one at a time, advancing only when told to — for walking an auto through in the
 * pit, one waypoint per button press.
 *
 * <p>Each step runs until it finishes on its own; the group then waits. A rising edge on {@code
 * forward} starts the next step (interrupting the current one if it is still running); a rising
 * edge on {@code back} restarts the previous one. Stepping forward past the last step ends the
 * group.
 *
 * <p>Like {@code ContinuousConditionalCommand}, this is library infrastructure, not game logic, so
 * it is a class. Build it from a factory and name it there.
 */
public class SteppableCommandGroup extends Command {

  private final List<Command> steps;
  private final BooleanSupplier forward;
  private final BooleanSupplier back;

  private int index = 0;
  private boolean stepRunning = false;
  private boolean lastForward = false;
  private boolean lastBack = false;
  private boolean done = false;

  /**
   * @param forward Rising edge advances. Pass a trigger from {@code Triggers.java}.
   * @param back Rising edge goes back one step.
   * @param steps At least one.
   */
  public SteppableCommandGroup(BooleanSupplier forward, BooleanSupplier back, List<Command> steps) {
    if (steps.isEmpty()) {
      throw new IllegalArgumentException("SteppableCommandGroup needs at least one step");
    }
    this.steps = List.copyOf(steps);
    this.forward = forward;
    this.back = back;
    CommandScheduler.getInstance().registerComposedCommands(this.steps.toArray(new Command[0]));
    for (Command step : this.steps) {
      addRequirements(step.getRequirements());
    }
  }

  @Override
  public void initialize() {
    index = 0;
    done = false;
    // Seed the edge detectors from the current state, so a button already held when the group
    // starts does not count as a press.
    lastForward = forward.getAsBoolean();
    lastBack = back.getAsBoolean();
    start(0);
  }

  @Override
  public void execute() {
    boolean forwardNow = forward.getAsBoolean();
    boolean backNow = back.getAsBoolean();
    boolean forwardPressed = forwardNow && !lastForward;
    boolean backPressed = backNow && !lastBack;
    lastForward = forwardNow;
    lastBack = backNow;

    if (forwardPressed) {
      stop(true);
      if (index + 1 >= steps.size()) {
        done = true;
        return;
      }
      start(index + 1);
    } else if (backPressed) {
      stop(true);
      start(Math.max(0, index - 1));
    }

    if (stepRunning) {
      Command step = steps.get(index);
      step.execute();
      if (step.isFinished()) {
        step.end(false);
        stepRunning = false;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    stop(true);
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  /** Which step is current, zero-based — worth logging from the factory that builds this. */
  public int currentStep() {
    return index;
  }

  private void start(int newIndex) {
    index = newIndex;
    steps.get(index).initialize();
    stepRunning = true;
  }

  private void stop(boolean interrupted) {
    if (stepRunning) {
      steps.get(index).end(interrupted);
      stepRunning = false;
    }
  }
}
