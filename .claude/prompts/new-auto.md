# Prompt: Create or Modify an Autonomous Routine

Use this prompt when adding a new auto path or modifying an existing one.

---

## Adding a New Auto (PathPlanner path exists)

```
Add a new autonomous routine called "[AUTO NAME]".

The PathPlanner path file "[filename].auto" already exists in deploy/pathplanner/autos/.

Named commands used in this auto:
- "[CommandName]" — [what it should do, e.g.: "Deploy intake and run rollers"]
- [repeat for each named command]

Register any new named commands in RobotContainer before buildAutoChooser().
Do not change any existing named command registrations.
Do not modify any existing auto paths.
```

---

## Adding a New Named Command

```
Register a new named command "[COMMAND NAME]" for use in PathPlanner autos.

What it should do: [description]

Subsystems involved: [list]

The command should be: [describe — e.g., "parallel: deploy intake + run rollers, ends when intake is up"]

Register it in RobotContainer alongside the existing named commands.
Run ./gradlew compileJava after.
```

---

## Modifying Auto Behavior (not the path, just the command logic)

```
Modify the behavior of the "[COMMAND NAME]" named command used in auto.

Current behavior: [describe what it does now]
Desired behavior: [describe what it should do]
Reason: [why the change is needed]

Only change the command factory method. Do not change the path file.
Do not change any other named commands.
```

---

## Safety Rules for Auto Changes

**Never modify `.auto` path files in code.** PathPlanner paths are edited in the
PathPlanner GUI, not by hand. Editing the JSON directly corrupts waypoints.

**Named command registration order matters.** Commands must be registered
before `AutoBuilder.buildAutoChooser()`. If they're registered after, PathPlanner
silently ignores them and the path runs without the command.

**Event triggers need no subsystem requirements.** See `.claude/rules/03-commands.md`.

**Time the auto.** In 2026 the routines overran the auto period and the last path was
truncated in every match. Run the auto in simulation and check the elapsed time against
the auto period before calling it done.

---

## Auto Preview & Start Pose Check (not in the template — worth rebuilding)

The 2026 robot published the selected auto's path to a `Field2d` widget during
`disabledPeriodic()` and reported how far the robot was from the path's starting pose in
inches and degrees. It caught mis-placed robots on the field before the match started.

This was deliberately left out of the template because it is straightforward to rewrite
and the 2026 version was entangled with that season's dashboard. If you rebuild it:

1. Load the selected auto's paths with `PathPlannerAuto.getPathGroupFromAutoFile(name)`
2. Flip for alliance with `AllianceFlipUtil` before drawing
3. Only reload when the chooser selection changes — not every loop
4. Compare `RobotState.getInstance().getEstimatedPose()` against
   `paths.get(0).getStartingHolonomicPose()` and publish the deltas
