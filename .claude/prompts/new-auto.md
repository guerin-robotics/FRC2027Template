# Prompt: Create or Modify an Autonomous Routine

Use this prompt when adding or changing an auto. There are three kinds (see
[docs/autos.md](../../docs/autos.md)); pick the template that matches.

---

## Adding a BLine Auto (path exists)

```
Add a BLine auto routine called "[AUTO NAME]" in robot/autos/BLineAutos.java.

Paths, in order (already drawn in BLine Web and exported to src/main/deploy/autos/paths/):
- "[path-name]" — [what happens on it]
- [repeat]

Event triggers (lib_key) used in these paths:
- "[lib_key]" — [what it should do, e.g.: "Deploy intake and run rollers"]
- [repeat]

Bind any new keys with BLineAutos.bind(...) in RobotContainer BEFORE any routine is
built. A bound command must not require the drive.
Author blue-left only (alliance flip and the Auto Side mirror are applied by the builder).
Reset odometry on the first path only (builder.withPoseReset(drive::setPose), then clear it).
Give every path command a timeout from Constants.Waits.
List the routine in BLineAutos.all(...).
Do not edit any file under deploy/autos by hand.
```

---

## Binding a New Event Trigger

```
Bind a command to the BLine event trigger "[LIB KEY]".

What it should do: [description]
Subsystems involved: [list — must not include the drive]
The command should be: [e.g. "parallel: deploy intake + run rollers, ends when intake is up"]

Bind it in RobotContainer with BLineAutos.bind(...), before any routine is built.
Run ./gradlew compileJava after.
```

---

## Modifying Auto Behavior (not the path, just the command logic)

```
Modify the behavior of the command bound to "[LIB KEY]" (or step "[N]" of "[AUTO NAME]").

Current behavior: [describe what it does now]
Desired behavior: [describe what it should do]
Reason: [why the change is needed]

Only change the command factory method. Do not change any file under deploy/autos.
Do not change any other bindings.
```

---

## Safety Rules for Auto Changes

**Never edit files under `deploy/autos` by hand.** Paths come out of BLine Web with generated
radii and velocity caps the editor tracks; hand-edited JSON drifts from what the editor and the
preview show. Re-draw and re-export instead.

**Bind before building.** `RobotContainerSmokeTest` fails the build if a path uses a `lib_key` nothing is bound to.

**Bound commands never require the drive.** They run alongside the path, which does. See
`.claude/rules/03-commands.md`.

**Never use the open-ended `driveToPose` inside an auto.** It never finishes. Point-to-point autos
and the PID-to-pose finish were removed for now; if they come back, they need a tolerance or
timeout.

**Time the auto.** In 2026 the routines overran the auto period and the last path was
truncated in every match. Run the auto in simulation and check the elapsed time against
the auto period before calling it done.

---

## Auto Preview & Start Pose Check (in the template for BLine routines)

`autos/AutoPreview` draws the selected routine's paths on the "Auto Preview" `Field2d` while
disabled, transformed exactly as they will run (alliance flip, then Left/Right mirror), and logs
`Auto/StartCheck/*` — distance and heading error from the start pose, with OK flags against
`Constants.Thresholds.START_POSE_*`. It redraws when the auto, the alliance or the side changes.

A routine only gets a preview if it is registered inside `BLineAutos.all` with `add(routines, name, supplier,
"path-1", "path-2", ...)`.
