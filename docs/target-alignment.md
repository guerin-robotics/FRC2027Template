# Target Alignment: joystickDriveAtAngle and driveToPose

How to point the robot at something, or drive it to a specific spot, using the two
general-purpose building blocks in `DriveCommands.java`. Both are game-agnostic — nothing here
changes until you plug in real field geometry, which is the other half of this doc.

---

## The two commands, and when to reach for which

| | `joystickDriveAtAngle` | `driveToPose` |
|---|---|---|
| Controls | Heading only | Translation **and** heading |
| Driver input | Still drives X/Y | None — fully autonomous |
| Use it for | "Keep facing the target while I drive" | "Go park at this exact spot" |
| Underlying control | One `LoggedTunableProfiledPID` (`angleController`) | Two — `angleController` (shared) + `driveController` |
| Obstacle avoidance | N/A | None — see `Drive.pathfindToPose` if the route isn't always clear |

Both live in `src/main/java/frc/robot/commands/DriveCommands.java`, next to `joystickDrive`. Read
that file's class javadoc first — it has the worked example for building a game-specific command
on top of `joystickDriveAtAngle`, and this doc extends the same idea to `driveToPose`.

---

## joystickDriveAtAngle

Field-relative translation from the joysticks, heading held by PID. This is what every 2026
"aim while driving" command (align-to-goal, snap-to-trench, snap-to-tower, ...) was built from.

```java
public static Command aimAtSomething(Drive drive, DoubleSupplier x, DoubleSupplier y) {
  return DriveCommands.joystickDriveAtAngle(
      drive, x, y,
      () -> RobotState.getInstance().getAngleToTarget(SOME_TARGET_TRANSLATION));
}
```

The heading supplier is re-read every loop, so a moving target (nearest scoring location, a
tracked game piece) works without any extra plumbing. `RobotState.getAngleToTarget` already
handles the "shortest rotation direction" case — `angleController` has continuous input enabled,
so aiming at 170° from -170° turns 20°, not 340°.

**If the mechanism doing the aiming faces the back of the robot**, add the offset at the call
site, not inside `RobotState`:

```java
RobotState.getInstance().getAngleToTarget(target).plus(Rotation2d.kPi)
```

See `getAngleToTarget`'s javadoc in `RobotState.java` for why that offset does not belong baked
into the method itself.

---

## driveToPose

Straight-line PID to a fixed field pose — translation and heading both, no driver input, no
pathfinding. Reach for this when the destination is a specific spot and the path there is always
clear: a pick/place station, a fixed scoring position. If it might not be clear, use
`Drive.pathfindToPose` instead, which runs a PathPlanner path around configured obstacles.

```java
Pose2d target = AllianceFlipUtil.apply(someFieldConstantsPose);
controller.a().whileTrue(DriveCommands.driveToPose(drive, () -> target));
```

Like the heading supplier above, `targetPoseSupplier` is read every loop — a dynamic target (e.g.
"nearest scoring face") is just a supplier that recomputes which pose to return.

**Not alliance-flipped internally.** `driveToPose` takes an absolute, blue-alliance-origin pose
and drives to exactly that. Flip it yourself with `AllianceFlipUtil.apply(...)` before handing it
over — see the next section for why that has to happen at the call site rather than inside
`FieldConstants`.

**Gating a follow-up action on arrival:** `driveToPose` runs until interrupted, the same as
`joystickDriveAtAngle` — it does not finish on its own. Build an "is aligned" check using
`drive.getPose()` against the same target pose, and use the Ready → Align → Act pattern from
`.claude/rules/03-commands.md`:

```java
Commands.sequence(
    Commands.waitUntil(mechanism::isReady).withTimeout(Waits.mechanismReadySeconds),
    Commands.waitUntil(
            () -> drive.getPose().getTranslation().getDistance(target.getTranslation()) < 0.05)
        .withTimeout(Waits.totalTimeoutSeconds - Waits.mechanismReadySeconds),
    doTheThing())
```

`AutoAim/DriveToPose/DistanceErrorMeters` and `AutoAim/DriveToPose/AngleErrorRad` are logged every
loop the command runs, so a match log can confirm exactly when (or whether) it actually converged.

**Before binding this to a button:** `driveController`'s gains are an untuned placeholder — see
the `TODO(2027)` comment on that field in `DriveCommands.java`. Run a `/pid-tune` (or in-sim)
session on the real chassis first. `DriveToPoseSimTest` (below) proves the command *converges*
against a generic sim model; it is not evidence the gains are *good* on hardware.

---

## Where the target poses come from: the FieldConstants pattern

Neither command knows anything about the game — they both take a pose (or a heading) as a
parameter. Producing that pose from "which reef face" or "which coral station" is a separate,
game-specific problem, and 6328 (Mechanical Advantage) solves it the same way every season: derive
scoring/pickup poses from `AprilTagFieldLayout` tag poses rather than hand-measuring field
coordinates.

`frc/lib/FieldConstants.java` already tells you to do this in its start-of-season checklist
("Add game-element geometry ... following the 2026 pattern: dimensions first, then reference
points derived from tag poses"). The 2026 content that checklist refers to was stripped for the
template, so **the worked example now lives at
[`template/src/main/java/frc/lib/ExampleFieldConstants.java`](../template/src/main/java/frc/lib/ExampleFieldConstants.java)**
— a fictional 4-face structure showing the full pattern end to end, including a `faceScoringPose`
method that returns exactly the `Pose2d` `driveToPose` wants.

Three rules, in more detail in that file's javadoc:

1. **Derive from tag poses, never hand-measure field coordinates.** A pose built as an offset from
   `FieldConstants.aprilTagLayout.getTagPose(id)` stays correct if the layout is ever corrected; a
   typed-in coordinate does not.
2. **Everything is blue-alliance origin — flip once, at the call site.** Same rule `AllianceFlipUtil`
   already enforces everywhere else in this codebase. Baking a red-alliance mirror into a constant
   creates a second flip point, which is how a coordinate ends up flipped twice (or not at all).
3. **Return a `Pose2d`, not a `Translation2d`.** `driveToPose` drives heading too — a location that
   doesn't also say which way to face when the robot gets there is half a target.

When the 2027 game is known, do not edit `ExampleFieldConstants.java` in place. Copy the pattern
into a real, game-named class (2025's was `Reef`, 2024's was `Speaker`) inside
`src/main/java/frc/lib/FieldConstants.java`, per its own instructions.

---

## Testing an alignment command in sim before wiring it up

Per `.claude/rules/04-build.md`, verify command-logic changes in simulation before calling them
done. Two existing tests are the pattern to copy for a new game-specific alignment command:

- [`DriveToPoseSimTest`](../src/test/java/frc/robot/commands/DriveToPoseSimTest.java) — schedules
  `driveToPose` against a physics-sim `Drive` (`ModuleIOSim`, the same wiring `RobotContainer` uses
  for `SIM`) and asserts the pose converges on a target.
- [`JoystickDriveAtAngleSimTest`](../src/test/java/frc/robot/commands/JoystickDriveAtAngleSimTest.java)
  — same idea for the heading-only command.

Both classes' javadoc explains two things worth reading before copying them: why `ModuleIOSim`
needs no wall-clock sleeping to produce real odometry samples, and — more important — why
`CommandScheduler.getInstance().cancelAll()` in `@AfterEach` is mandatory rather than optional.
`CommandScheduler` is a JVM-wide singleton and Gradle runs every test class in one JVM, so a
command left scheduled at the end of one test keeps running during every later test. That bug is
what caused `JoystickDriveAtAngleSimTest` to fail the first time it was added: a stale
`driveToPose` command was still calling into the shared static `angleController` and corrupting
its state. Any new `CommandScheduler.schedule(...)`-based test needs the same teardown.

[`DriveOdometrySimTest`](../src/test/java/frc/robot/subsystems/drive/DriveOdometrySimTest.java) is
a level below either command — it commands raw `ChassisSpeeds` directly and checks that `+X`/`+Y`/
`+omega` odometry moves the way WPILib's convention says it should. Re-run it (or its pattern)
after any change to `TunerConstants`, module inversion, or `Drive`'s kinematics — those are exactly
the failure classes `.claude/rules/00-safety.md` calls out as high-risk and hard to notice.

### Loop timing

A new alignment command runs `angleController` (and maybe `driveController`) every cycle, so it
adds to the 20 ms budget.

[`DrivePeriodicBudgetTest`](../src/test/java/frc/robot/subsystems/drive/DrivePeriodicBudgetTest.java)
measures `Drive.periodic()`, the largest known contributor, and fails if its median jumps by more
than an order of magnitude. Copy this pattern for any 2027 subsystem whose `periodic()` does real
work.

`LoopTimeMonitor` itself — the watchdog that reports a sustained over-budget loop and raises an
alert — is no longer unit tested. Its thresholds (a 20 ms budget against a 25 ms alert) are worth
re-reading in `frc/lib/LoopTimeMonitor.java` before trusting them.

Neither replaces watching `LoopTiming/AverageMs` on the actual robot from the first day it drives —
a dev laptop is not a roboRIO. They catch structural regressions early, which is when they are
cheap to fix.
