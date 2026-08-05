package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Every {@link Trigger} in the robot, in one file.
 *
 * <p>{@code RobotContainer} reads from {@link #getInstance()} and never constructs a trigger
 * itself. Two things come out of that: button logic is auditable in one place, and state triggers
 * can be composed without a subsystem holding a reference to another subsystem. The 2026 season
 * caught real timing bugs purely because every condition was visible on one screen.
 *
 * <p>See {@code .claude/rules/01-architecture.md}.
 *
 * <h2>Controller layout</h2>
 *
 * <p><b>Flight stick drives. Xbox operates.</b> Both are plugged in for every match. Ports are in
 * {@link Constants.Controllers} and must be checked in the DS USB tab before the first match.
 *
 * <h2>Why the controllers are private</h2>
 *
 * <p>Nothing outside this class may touch a controller object. Every axis read goes through {@link
 * #driveXSupplier()} / {@link #driveYSupplier()} / {@link #driveRotSupplier()}, and every button
 * goes through a named accessor below.
 *
 * <p>This is not style. It is what makes the dashboard controller swap addable later by editing
 * only this file — and it prevents the failure that actually bit in 2026, where one command read a
 * controller directly while everything else used the suppliers. That command then followed a stick
 * nobody was holding. The symptom was an align command drifting on its own while ordinary driving
 * worked fine: hours to diagnose, trivial to prevent.
 *
 * <h2>Naming</h2>
 *
 * <p>Accessors are named for <b>what the robot does</b>, not for the button. {@code resetGyro()},
 * not {@code bButton()}. When the drive team asks to move a function to a different button, you
 * change one line here and every call site keeps working — and reviewing a binding does not require
 * a controller diagram.
 */
public class Triggers {

  private static Triggers instance;

  public static Triggers getInstance() {
    if (instance == null) {
      instance = new Triggers();
    }
    return instance;
  }

  // ============================================================================================
  // CONTROLLERS — private on purpose. See the class javadoc.
  // ============================================================================================

  /** The driver's flight stick: translation and rotation. */
  private final CommandJoystick flightStick =
      new CommandJoystick(Constants.Controllers.FLIGHT_STICK_PORT);

  /** The operator's Xbox controller: mechanisms and overrides. */
  private final CommandXboxController xbox =
      new CommandXboxController(Constants.Controllers.XBOX_PORT);

  private Triggers() {}

  // ============================================================================================
  // DRIVE AXES
  // ============================================================================================
  //
  // These return values in ROBOT convention, already sign-corrected:
  //
  //   driveX   +1 = forward (away from your alliance wall)
  //   driveY   +1 = left
  //   driveRot +1 = counter-clockwise
  //
  // The sign flips live here rather than at the call site on purpose. A flight stick reports
  // pushed-forward as NEGATIVE Y, which is the opposite of the robot's +X-forward convention,
  // and every call site that re-derives that flip is a chance to get one of them backwards.
  // Having it in one place means a command written by someone who has never thought about
  // joystick sign conventions is still correct.
  //
  // NO DEADBAND IS APPLIED HERE. DriveCommands applies it to the translation magnitude, which
  // is what makes the deadband a circle rather than a plus-sign. Deadbanding per-axis here as
  // well would reshape the response near center — a real bug, not a redundancy.
  //
  // Adding the dashboard controller swap means changing only these three methods and the button
  // accessors below. See docs/drive-controller-mode.md.

  /** Forward/backward. +1 is away from your alliance wall. */
  public double driveXSupplier() {
    return -flightStick.getY();
  }

  /** Left/right. +1 is left. */
  public double driveYSupplier() {
    return -flightStick.getX();
  }

  /** Rotation. +1 is counter-clockwise. */
  public double driveRotSupplier() {
    return -flightStick.getTwist();
  }

  // ============================================================================================
  // DRIVER BUTTONS — flight stick
  // ============================================================================================

  /**
   * Re-zeroes the gyro to the current heading.
   *
   * <p>Field-relative driving is only as good as this. Expect to use it after a collision that
   * moves the robot without the wheels turning.
   */
  public Trigger resetGyro() {
    return flightStick.button(2);
  }

  /** Holds a fixed heading while the driver still controls translation. */
  public Trigger lockHeading() {
    return flightStick.button(3);
  }

  /**
   * Points the wheels into an X so the robot resists being pushed.
   *
   * <p>Worth binding somewhere reachable — it is the only defense against being shoved off a
   * scoring position.
   */
  public Trigger stopWithX() {
    return flightStick.button(4);
  }

  // ============================================================================================
  // OPERATOR BUTTONS — Xbox
  // ============================================================================================
  //
  // Add one accessor per robot function as mechanisms land. Name them for the action, not the
  // button. Analog triggers take Constants.Controllers.TRIGGER_THRESHOLD rather than being read
  // as raw booleans — they rest near zero but not at it, and a resting hand produces spurious
  // presses otherwise.
  //
  //   public Trigger intake() {
  //     return xbox.leftTrigger(Constants.Controllers.TRIGGER_THRESHOLD);
  //   }
  //
  //   public Trigger score() {
  //     return xbox.rightTrigger(Constants.Controllers.TRIGGER_THRESHOLD);
  //   }
  //
  //   public Trigger stow() {
  //     return xbox.b();
  //   }

  // ============================================================================================
  // STATE TRIGGERS
  // ============================================================================================
  //
  // Conditions built from robot state rather than from a button — "is the mechanism ready", "is
  // the robot in a legal position to score", "has the game piece been held long enough to be
  // seated".
  //
  // Use LoggedTrigger rather than Trigger for these. It publishes the condition to the log every
  // time it is polled, which is the difference between a match review that can answer "why
  // didn't it fire" and one that cannot. Buttons do not need it — the DS already logs joystick
  // state.
  //
  //   public final LoggedTrigger readyToScore =
  //       new LoggedTrigger("Triggers/ReadyToScore", () -> shooter.isAtVelocity());
  //
  // Compose rather than duplicate. A trigger that repeats another's condition inline will drift
  // out of sync with it the first time one is edited:
  //
  //   public final LoggedTrigger clearToFire =
  //       new LoggedTrigger("Triggers/ClearToFire", readyToScore.and(inScoringZone));
  //
  // Fields, not methods, so composition happens once at construction instead of allocating a new
  // Trigger on every call.
}
