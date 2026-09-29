package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.LoggedTrigger;
import frc.robot.Zones.BroadZone;
import java.util.function.BooleanSupplier;

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
 * <h2>This branch: the 2026 layout</h2>
 *
 * <p>Ported from Rebuilt2026 {@code Triggers}. By default the flight stick (a Thrustmaster) drives
 * and the Xbox is the override controller. The "Drive controller" dashboard chooser swaps them
 * between matches; {@link #latchDriveController} applies the choice once, at teleopInit, and it is
 * not re-read until the next enable. See {@code docs/drive-controller-mode.md}.
 *
 * <p>Driver functions move from the flight stick to the Xbox when the Xbox drives; override
 * functions move the opposite way. {@link #sourced} polls exactly one device per trigger, so the
 * swap costs one boolean read per trigger per loop.
 *
 * <h2>Why the controllers are private</h2>
 *
 * <p>Nothing outside this class may touch a controller object. Every axis read goes through {@link
 * #driveXSupplier()} / {@link #driveYSupplier()} / {@link #driveRotSupplier()}, and every button
 * goes through a named accessor below. That is what makes the swap possible at all, and it prevents
 * the 2026 bug where one command read a controller directly and followed a stick nobody was
 * holding.
 *
 * <h2>Naming</h2>
 *
 * <p>Accessors are named for <b>what the robot does</b>, not for the button — {@code
 * shootButton()}, not {@code button1()}. The names are the 2026 ones, so a binding reads the same
 * as it did in Rebuilt2026.
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

  /** The flight stick (Thrustmaster). Drives by default. */
  private final CommandJoystick flightStick =
      new CommandJoystick(Constants.Controllers.FLIGHT_STICK_PORT);

  /** The Xbox controller. Overrides by default; drives when selected. */
  private final CommandXboxController xbox =
      new CommandXboxController(Constants.Controllers.XBOX_PORT);

  /** The keyboard joystick the simulation bindings read. */
  private final CommandJoystick simKeyboard =
      new CommandJoystick(Constants.Controllers.SIM_KEYBOARD_PORT);

  private Triggers() {}

  // ============================================================================================
  // DRIVE-SOURCE GATING
  // ============================================================================================

  /** Latched at teleopInit. False: flight stick drives. True: Xbox drives. */
  private boolean xboxDrives = false;

  /**
   * Applies the dashboard's drive-controller selection. Call only from {@code teleopInit} — the
   * whole point of latching is that a mid-match dashboard change does nothing.
   */
  public void latchDriveController(boolean xboxDrives) {
    this.xboxDrives = xboxDrives;
  }

  public boolean isXboxDriving() {
    return xboxDrives;
  }

  /**
   * Routes a function to the control it lives on in each mode. The parameters are MODE-based, not
   * device-based: drive functions move flight stick to Xbox, override functions the other way.
   */
  private Trigger sourced(Trigger whenFlightStickDrives, Trigger whenXboxDrives) {
    return new Trigger(
        () -> xboxDrives ? whenXboxDrives.getAsBoolean() : whenFlightStickDrives.getAsBoolean());
  }

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
  // 2026 negated these at every call site in RobotContainer (-getDriveY(), -getDriveX(),
  // -getDriveRot()); the template does it here, once. Same values.
  //
  // NO DEADBAND IS APPLIED HERE. DriveCommands applies it to the translation magnitude.

  /** Forward/backward. +1 is away from your alliance wall. */
  public double driveXSupplier() {
    return xboxDrives ? -xbox.getLeftY() : -flightStick.getY();
  }

  /** Left/right. +1 is left. */
  public double driveYSupplier() {
    return xboxDrives ? -xbox.getLeftX() : -flightStick.getX();
  }

  /** Rotation. +1 is counter-clockwise. */
  public double driveRotSupplier() {
    return xboxDrives ? -xbox.getRightX() : -flightStick.getTwist();
  }

  // ============================================================================================
  // DRIVER BUTTONS — flight stick by default, Xbox when it drives
  // ============================================================================================

  /** Shoot at the hub in our zone, pass outside it. Xbox: right trigger. */
  public Trigger shootButton() {
    return sourced(
        flightStick.button(1), xbox.rightTrigger(Constants.Controllers.TRIGGER_THRESHOLD));
  }

  /** Snap heading for driving through the trench. Xbox: X. */
  public Trigger trenchAlignButton() {
    return sourced(flightStick.button(2), xbox.x());
  }

  /** Retract the intake. Xbox: left bumper. */
  public Trigger intakeInButton() {
    return sourced(flightStick.button(3), xbox.leftBumper());
  }

  /** Deploy the intake. Xbox: right bumper. */
  public Trigger intakeOutButton() {
    return sourced(flightStick.button(4), xbox.rightBumper());
  }

  /** Run the intake roller. Xbox: left trigger. */
  public Trigger intakeRollerButton() {
    return sourced(
        flightStick.button(5), xbox.leftTrigger(Constants.Controllers.TRIGGER_THRESHOLD));
  }

  /** Manual hopper compress. No Xbox-mode home — auto-compress on shoot still works there. */
  public Trigger intakeCompressButton() {
    return flightStick.button(6);
  }

  /** Fixed tower shot: spins everything up, compresses, sets the hood. Xbox: Y. */
  public Trigger shootFromTowerButton() {
    return sourced(flightStick.button(10), xbox.y());
  }

  /** Fixed pass: spins everything up, sets the hood. Xbox: D-pad up. */
  public Trigger passButton() {
    return sourced(flightStick.button(9), xbox.povUp());
  }

  /** Reset odometry to the spot in front of the tower (added at WVROX). Xbox: D-pad down. */
  public Trigger wvroxOdometryReset() {
    return sourced(flightStick.button(8), xbox.povDown());
  }

  /** Demo shot. Only bound while demo mode is on. */
  public Trigger demoDistanceShot() {
    return flightStick.button(7);
  }

  // ============================================================================================
  // OVERRIDE BUTTONS — Xbox by default, flight stick when the Xbox drives
  // ============================================================================================

  /**
   * Flip which alliance won auto (the hub-shift schedule). In Xbox drive mode, flight stick buttons
   * 3 and 4 are BOTH mapped here, per drive team request.
   */
  public Trigger allianceWinFlipper() {
    return sourced(xbox.a(), flightStick.button(3).or(flightStick.button(4)));
  }

  /** Turn the hub-shift timer off entirely. */
  public Trigger allianceWinDisabler() {
    return sourced(xbox.y(), flightStick.button(2));
  }

  /** Cancel the automatic X while shooting, for the rest of this shoot press. */
  public Trigger autoXOverride() {
    return flightStick.button(12);
  }

  /**
   * Toggle double compress for this shoot press. Only live while the flight stick drives; in Xbox
   * drive mode B is deliberately dead.
   */
  public Trigger doubleCompressOverride() {
    return sourced(xbox.b(), new Trigger(() -> false));
  }

  // ============================================================================================
  // SIMULATION BUTTONS — keyboard joystick, plus two Xbox buttons 2026 borrowed
  // ============================================================================================

  public Trigger simShootButton() {
    return simKeyboard.button(1);
  }

  public Trigger simTrenchAlignButton() {
    return simKeyboard.button(2);
  }

  public Trigger simIntakeInButton() {
    return simKeyboard.button(3);
  }

  public Trigger simIntakeOutButton() {
    return simKeyboard.button(4);
  }

  public Trigger simIntakeRollerButton() {
    return simKeyboard.button(5);
  }

  public Trigger simIntakeCompressButton() {
    return simKeyboard.button(6);
  }

  public Trigger simPassButton() {
    return simKeyboard.button(7);
  }

  public Trigger simAllianceWinFlipper() {
    return simKeyboard.button(8);
  }

  public Trigger simBumpAlignButton() {
    return xbox.y();
  }

  public Trigger simShootFromTowerButton() {
    return xbox.x();
  }

  /** X the wheels. Xbox X — the same button as {@link #simShootFromTowerButton()}, as in 2026. */
  public Trigger xWheels() {
    return xbox.x();
  }

  /** Raw keyboard axes, NOT sign-corrected — 2026 passed them to the drive unnegated. */
  public double simXSupplier() {
    return simKeyboard.getRawAxis(0);
  }

  public double simYSupplier() {
    return simKeyboard.getRawAxis(1);
  }

  public double simRotationSupplier() {
    return simKeyboard.getRawAxis(2);
  }

  // ============================================================================================
  // STATE TRIGGERS
  // ============================================================================================
  //
  // LoggedTrigger, so every condition is in the match log. The keys are the 2026 keys, so 2026
  // AdvantageScope layouts still find them. Fields, not methods: built once, polled every loop.

  /** In our alliance zone, so a shot can score from here. */
  public final LoggedTrigger isShootSafeZone =
      new LoggedTrigger(
          "isShootSafeZone",
          () -> RobotState.getInstance().getBroadZone() == BroadZone.ALLIANCE_ZONE);

  /** Our hub is active now (with the flight-time fudge), or the timer is off, or demo mode. */
  public final LoggedTrigger isShootSafeTime =
      new LoggedTrigger(
          "isShootSafeTime",
          () ->
              HubShiftUtil.getShiftedShiftInfo().active()
                  || HubShiftUtil.disabled
                  || ShotModes.isDemo());

  /** Both: in our zone while our hub is active. */
  public final LoggedTrigger isShootClear =
      new LoggedTrigger("isShootClear", isShootSafeTime.and(isShootSafeZone));

  /**
   * In our zone while our hub is inactive — the one case the real-robot feed bindings refuse to
   * shoot in. 2026 wrote this inline in each binding as {@code !(isShootSafeZone &&
   * !isShootSafeTime)}.
   */
  public final LoggedTrigger isHubInactiveInZone =
      new LoggedTrigger(
          "isHubInactiveInZone",
          () -> isShootSafeZone.getAsBoolean() && !isShootSafeTime.getAsBoolean());

  /** Shot-tuning mode is on. See {@link ShotModes}. */
  public final LoggedTrigger isShotTuningMode =
      new LoggedTrigger("isShotTuningMode", ShotModes::isShotTuning);

  /** Demo mode is on. See {@link ShotModes}. */
  public final LoggedTrigger isDemoMode = new LoggedTrigger("isDemoMode", ShotModes::isDemo);

  /** Tight alignment to whichever target this zone shoots at, held 0.3 s. */
  public final LoggedTrigger isAlignedForCurrentShot =
      new LoggedTrigger(
              "isAlignedForCurrentShot",
              () ->
                  RobotState.getInstance().getBroadZone() == BroadZone.ALLIANCE_ZONE
                      ? RobotState.getInstance().isAlignedToHub()
                      : RobotState.getInstance().isAlignedToPass())
          .debounce(0.3, DebounceType.kRising);

  /** Loose alignment, held 0.3 s. What the real-robot bindings feed on. */
  public final LoggedTrigger isAlignedLooser =
      new LoggedTrigger(
              "isAlignedLooser",
              () ->
                  RobotState.getInstance().getBroadZone() == BroadZone.ALLIANCE_ZONE
                      ? RobotState.getInstance().isAlignedToHubLoose()
                      : RobotState.getInstance().isAlignedToPassLoose())
          .debounce(0.3, DebounceType.kRising);

  // ---- Flywheel ----
  //
  // In 2026 this trigger was a field on the Flywheel subsystem. Triggers live here, and Triggers
  // cannot hold a subsystem, so RobotContainer hands over the predicate once at construction.

  private BooleanSupplier flywheelSpunUp = () -> false;

  /** Wiring only. Called once, from RobotContainer. */
  public void setFlywheelSpunUpSupplier(BooleanSupplier spunUp) {
    this.flywheelSpunUp = spunUp;
  }

  public final LoggedTrigger isFlywheelSpunUp =
      new LoggedTrigger("isFlywheelSpunUp", () -> flywheelSpunUp.getAsBoolean());
}
