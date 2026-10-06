# Prompt: Create or Modify an Autonomous Routine

Use this prompt when adding or changing an auto. There are three kinds (see
[docs/autos.md](../../docs/autos.md)); pick the template that matches.

---

## Adding a Choreo Auto (trajectory exists)

```
Add a Choreo auto routine called "[AUTO NAME]" in robot/autos/ChoreoAutos.java.

Trajectories, in order (already drawn in the Choreo GUI and saved to src/main/deploy/choreo/):
- "[TrajectoryName]" — [what happens on it]
- [repeat]

PID-to-pose finish after: [none / "[TrajectoryName]" — use ChoreoAutos.thenAlign]

Event markers used in these trajectories:
- "[MarkerName]" — [what it should do, e.g.: "Deploy intake and run rollers"]
- [repeat]

Bind any new markers with ChoreoAutos.bind(...) in RobotContainer BEFORE any routine is
built. A bound command must not require the drive.
List the routine in ChoreoAutos.all(...).
Do not edit any .traj or .chor file.
```

---

## Adding a Point-to-Point Auto (no trajectory)

```
Add a point-to-point auto called "[AUTO NAME]" in robot/autos/PointToPointAutos.java.

Waypoints (blue-origin meters, degrees), in order — the first is the starting pose:
- ([x], [y], [heading])
- [repeat]

Use the default tolerances and timeout from Constants unless stated: [overrides].
List it in PointToPointAutos.all().
```

---

## Binding a New Event Marker

```
Bind a command to the Choreo event marker "[MARKER NAME]".

What it should do: [description]
Subsystems involved: [list — must not include the drive]
The command should be: [e.g. "parallel: deploy intake + run rollers, ends when intake is up"]

Bind it in RobotContainer with ChoreoAutos.bind(...), before any routine is built.
Run ./gradlew compileJava after.
```

---

## Modifying Auto Behavior (not the trajectory, just the command logic)

```
Modify the behavior of the command bound to "[MARKER NAME]" (or step "[N]" of "[AUTO NAME]").

Current behavior: [describe what it does now]
Desired behavior: [describe what it should do]
Reason: [why the change is needed]

Only change the command factory method. Do not change any trajectory file.
Do not change any other bindings.
```

---

## Safety Rules for Auto Changes

**Never edit `.traj` or `.chor` files in code.** Trajectories come out of the Choreo GUI's
optimiser; editing the JSON produces a path the robot was never shown to be able to follow.
Re-draw and re-export instead.

**Bind before building.** A routine only sees event-marker bindings that existed when it was
created. `RobotContainerSmokeTest` fails the build if a `.traj` uses a marker nothing is bound to.

**Bound commands never require the drive.** They run alongside the trajectory, which does. See
`.claude/rules/03-commands.md`.

**Every PID-to-pose step finishes.** Use `driveToPoseWithin` / `driveToWaypoint` (tolerance or
timeout), never the open-ended `driveToPose`, inside an auto.

**Time the auto.** In 2026 the routines overran the auto period and the last path was
truncated in every match. Run the auto in simulation and check the elapsed time against
the auto period before calling it done.

---

## Auto Preview & Start Pose Check (not in the template — worth rebuilding)

The 2026 robot published the selected auto's path to a `Field2d` widget during
`disabledPeriodic()` and reported how far the robot was from the path's starting pose in
inches and degrees. It caught mis-placed robots on the field before the match started.

This was deliberately left out of the template because it is straightforward to rewrite
and the 2026 version was entangled with that season's dashboard. If you rebuild it for Choreo:

1. Load the selected routine's first trajectory with `Choreo.loadTrajectory(name)`
2. Use `getInitialPose(flip)` for the start pose, flipping when on red
3. Only reload when the chooser selection changes — not every loop
4. Compare `RobotState.getInstance().getEstimatedPose()` against that pose and publish the
   deltas; for a point-to-point auto, compare against `WaypointAuto.startingPose()` instead
