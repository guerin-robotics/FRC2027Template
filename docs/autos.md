# Autos

Autos are **BLine paths**, selected as Driver Station OpModes. The follower is BLine's `FollowPath`, configured in
`BLineAutos.createBuilder`.

| Authored in | How the robot moves | Use it when |
|---|---|---|
| BLine Web → `src/main/deploy/autos/paths/*.json`; routine in `robot/autos/BLineAutos.java` | `FollowPath`: PID on remaining polyline distance, plus cross-track and heading PIDs, capped by the path's velocity and acceleration constraints | Always — passing waypoints, speed through a gap, mechanism actions mid-path |

**Removed for now:** point-to-point autos (`PointToPointAutos`, `WaypointAuto`, the pit stepper)
and the PID-to-pose finish after a path (`thenAlign`, `driveToPoseWithin`, `driveToWaypoint`).
`DriveCommands.driveToPose` stays for teleop alignment. They are in git history and can come back as
layers; nothing else depended on them.

**BLine is not a trajectory.** A BLine path is a polyline of waypoints and targets, followed by
physical progress rather than by a clock: the robot hands off to the next point when it is close
enough (or has projected past it), and the path ends when it is within tolerance of the final pose.
Nothing times it, so a blocked robot never finishes — every path command needs a timeout
(`Constants.Waits.PATH_TIMEOUT_SECONDS`). Speed comes from the velocity ceilings in
`deploy/autos/config.json` and from per-path constraints set in BLine Web.

---

## How top teams do it

Researched from public repos, October 2026:

| Team | Autos authored in | Route | PID-to-pose | PID-to-pose finishes on |
|---|---|---|---|---|
| **1114** (2026, Simbot Tim) | PathPlanner GUI | PathPlanner's own PID path follower (`PPHolonomicDriveController`) | `DriveToPose` class: profiled distance PID **seeded with current speed**, optional "approach strength" lane-curving, profiled heading PID. Present but **not used by any 2026 auto** | `atGoal()` on both PIDs (5 cm, 2.75°) |
| **6328** (2025) | Code + Choreo | Choreo trajectory, then `DriveToPose` for the approach | **Velocity-seeded** profile, **setpoint feedforward faded by distance**, driver-override blend, auto/teleop constraint sets, all tunable | Never self-finishes; callers gate on `withinTolerance` |
| **254** (2025) | Code | `AutoAlignToPoseCommand` | Profiled PID + **setpoint feedforward × ffScaler** | 4 cm / 2° |
| **1678** (2025) | Choreo | Choreo trajectory, then `PIDToPoseCommand` | Profiled PID | Tolerance |
| **3467** W8-Library | Code | Chain of `DriveToPose` per waypoint | Profiled PID on distance, angular PID, output clamps | Optional distance/angle tolerance |
| **Us** | BLine Web + code | BLine paths only, for now | **Velocity-seeded** profiles + **setpoint feedforward** faded inside `DRIVE_TO_POSE_FF_MIN_RADIUS_METERS`–`DRIVE_TO_POSE_FF_MAX_RADIUS_METERS` | `driveToPose` (teleop only; never finishes) |

**What 1114 actually does.** "PID with the PathPlanner GUI" means PathPlanner's path follower,
which is PID on the path — the same thing BLine's follower does. Their PID-to-pose is
a separate tool. The 6328/1678 hybrid — draw the route, PID the finish — is the removed
PID-to-pose finish.

---

## BLine paths

Draw paths in BLine Web and export the autos folder to `src/main/deploy/autos/` (`config.json` and
`paths/`). Set the robot's tested limits in BLine Web's Path Defaults, not just in code. Then, in
`BLineAutos`:

```java
// Import org.wpilib.command3.Command and static org.wpilib.units.Units.Seconds.
public static Command twoPiece(FollowPath.Builder builder) {
  Path toFirst = new Path("left-to-first");   // deploy/autos/paths/left-to-first.json
  Path toScore = new Path("first-to-score");

  // Reset odometry from the first path only. Both helpers apply the selected mirror.
  Command first = followFirst(builder, toFirst);
  Command second = follow(builder, toScore);

  return Command.sequence(
          first.withTimeout(Seconds.of(Constants.Waits.PATH_TIMEOUT_SECONDS)),
          second.withTimeout(Seconds.of(Constants.Waits.PATH_TIMEOUT_SECONDS)))
      .named("Auto_TwoPiece");
}
```

and list it in `BLineAutos.all(...)`:

```java
add(routines, "Two Piece", () -> twoPiece(builder),
    "left-to-first", "first-to-score");
```

Paths are flipped for red by BLine-Lib (`withDefaultShouldFlip`); author them blue-origin. The
follower logs its internals under `FollowPath/` (remaining distance, controller outputs,
cross-track error, `pathTranslations` for the active path).

### Left and right from one blue-left path

Draw every auto **blue-left only**. Two things happen when the auto starts:

- **Alliance** — `withDefaultShouldFlip()` reads the Driver Station alliance, which is the FMS
  station on the field, and flips the path rotationally for red. Nothing to select.
- **Side** — the Driver Station's Left/Right autonomous OpMode supplies `withShouldMirror`, which reflects the
  path across the field centerline for Right. Flip is applied first, then mirror; they commute, so
  all four combinations (blue/red x left/right) come from the one path.

The pose reset uses the same transform, so the robot's start pose follows both choices.

Before trusting a mirrored path, check: a side-mounted intake or shooter faces the wrong way when
mirrored (use separate authored paths if the mechanism is not symmetric); the field is symmetric
the way BLine assumes (`FlippingUtil` defaults to 16.54 x 8.07 m, the 2026 field — update
`fieldSizeX/Y` when the 2027 field is published); and the selected side and alliance on blocks, in
the planned and live pose.

### Logging

BLine publishes through four process-wide consumers (double, boolean, pose, translation list), all
no-ops until set. `BLineAutos.createBuilder` sets all four, once, to `Logger.recordOutput` — so
everything lands under `RealOutputs/FollowPath/` in AdvantageKit. The two geometry keys also feed the
Auto Preview widget. Plot `remainingPathDistanceMeters`, the `*TranslationControllerOutput` keys,
`crossTrackError` and `pathTranslations` / `robotTranslations` when tuning. Nothing is logged until a
path runs. About 45 keys publish per loop while one does, so watch `LoopTiming/` on the first run.

### Auto preview and start-pose check

`autos/AutoPreview` draws the selected routine's paths on the "Auto Preview" widget while disabled,
flipped and mirrored the way they will run, and logs `Auto/StartCheck/DistanceInches`,
`RotationDiffDegrees`, `PositionOK` and `RotationOK` (tolerances `Constants.Thresholds.START_POSE_*`,
6 in and 5°). During auto it shows the active leg and the closest point on it. It reloads when the
auto, the alliance or the side changes — the alliance often arrives from the FMS after the auto is
picked. Inside `BLineAutos.all`, register a routine with `add(routines, name, routine, pathFiles...)`, not
`routines.put`, or it has no preview.

### Event triggers

An event placed in BLine Web carries a `lib_key`. It fires whatever is registered under that key:

```java
// RobotContainer, BEFORE any routine is built
BLineAutos.bind("Intake", IntakeCommands.deploy(intake));
```

- A bound command must **not** require the drive — it runs beside the path, which does.
- It is scheduled through the V3 `Scheduler` independently of the path. Give it an end condition.
- If an event command shares a requirement with a later step of the same routine, compose the
  routine with a requirements-free V3 coroutine and use `co.await(...)` for each path or action.
  A group that reserves the event mechanism can be canceled by the event.
- `RobotContainerSmokeTest` fails if a path uses a `lib_key` nothing is bound to.

## Tuning

**Before trusting it on carpet:** the BLine follower gains in `BLineAutos` are carried over from
BLine's documented defaults (translation 2.0, rotation 1.0, cross-track 0.2), deliberately left at
the library values; none are tuned on a robot. They are not the old PathPlanner gains (10 / 7.1): PathPlanner's translation gain corrected position error on top of
feedforward, while BLine's translation
output is the whole speed command, `kP x remaining distance`, so kP must stay at or below
`2 x max acceleration / max velocity` (4.0 for the shipped 4.0 m/s and 8.0 m/s²), or the stop
overshoots. Tune all of them with `/bline-auto` on the real chassis.

---

## Driver Station selection

Each routine registered in `BLineAutos.all(...)` appears on the Driver Station as
`<routine> (Left)` and `<routine> (Right)`. AdvantageKit logs the selected OpMode for replay.
The selected routine is built on enable. Its preview clears when the selection is released.

`Auto/Outcome` distinguishes completion, rejection, interruption, cancellation, failure, and
an OpMode ending with the routine still running. The last case sets `Auto/Overran` and can
also occur on an early disable since WPILib does not supply the reason the OpMode ended.

---

## Time budget

In 2026 every auto overran the auto period and its last path was truncated. Time each auto in sim
before an event. The worst case adds every path's timeout (`Constants.Waits.PATH_TIMEOUT_SECONDS`); keep the total
inside the auto period, or accept on purpose that the last paths may be skipped.
