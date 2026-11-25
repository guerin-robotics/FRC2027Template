---
name: bline-auto
description: Create or tune a BLine autonomous routine — author paths blue-left in BLine Web, export them, wire the routine, events, preview and timeouts in BLineAutos, then tune the follower's translation, rotation and cross-track PIDs. Use when adding or changing an auto, when a path misbehaves (orbits a point, overshoots, finishes while moving, wrong on red or right), or when tuning BLine gains. Sim-first; real-robot gain changes always require explicit user confirmation.
---

# BLine Autos — Create and Tune

Read `docs/autos.md` and `.claude/rules/01-architecture.md` ("BLine Wiring") first. They are the
source of truth for how this repo is wired; this skill is the procedure.

**Hard rules (from CLAUDE.md):**

- Follower gains (`TRANSLATION_KP`, `ROTATION_KP`, `CROSS_TRACK_KP` in `BLineAutos`) are **High
  risk**. Propose a change with its evidence; the user confirms before it is applied or deployed.
  Everything the skill does to a gain happens in simulation until they do.
- **Never edit `deploy/autos` JSON by hand.** Paths come out of BLine Web. If a path is wrong,
  re-draw and re-export.
- Never raise a speed or acceleration ceiling to make a path pass. A ceiling the robot cannot
  reach makes paths it cannot follow.
- Every path command has a timeout from `Constants.Waits`. Never remove one.
- Do not change a CAN ID, an encoder offset or a current limit to fix a path.
- One logical change per response. Creating an auto and retuning gains are two.

Ask which mode, if the user did not say: **A. create or change an auto**, or **B. tune the
follower**.

---

## Mode A — Create or change an auto

### 1. Get the plan from the user (ask if missing)

- What the auto does, in order: where it starts, each stop, what mechanism acts where.
- Which side it is drawn on. **Every auto is authored blue-left only.** Red comes from the Driver
  Station alliance; Right comes from selecting the `(Right)` autonomous OpMode. Do not draw separate paths for them
  unless the field or a mechanism is genuinely asymmetric.
- Which mechanism actions fire mid-path (they become event triggers) and which happen between
  paths (they become steps in the routine).

### 2. Author in BLine Web (the user does this; you instruct)

Give them a checklist, per path:

- Waypoints with heading at the start and end. Intermediate **translation targets** for shaping;
  rotation targets only where heading matters.
- Event triggers: a non-empty `lib_key`, `t_ratio` in [0,1], same-segment events in increasing
  order. The key is case-sensitive and must match the `BLineAutos.bind` call exactly.
- Run **Generate** for handoff radii and velocity caps, then review them. Add a lower
  max-velocity ranged constraint over the **final** translation ordinals: BLine finishes when the
  robot is inside tolerance, with no final-velocity check, so a fast arrival ends the command
  while the robot is still moving.
- Put a velocity cap before any bump or tight clearance; keep anchors off the obstacle itself.
- Editor ordinals are one-based, runtime JSON ordinals are zero-based. Do not hand-translate.
- Set the project's Path Defaults to the robot's **tested** limits (they must match
  `deploy/autos/config.json`).
- Export the autos folder (browser Save does not write the robot project) and copy it to
  `src/main/deploy/autos/`. Review the whole diff: `config.json`, `paths/*.json`, `project.json`.

### 3. Verify the files before touching code

- `python -m json.tool` each new path parses; every path has `path_elements`; events have `lib_key`.
- Each `lib_key` is listed for step 4.
- `config.json` limits are the ones the user intends. **Do not edit it.** If it is wrong, say so.

### 4. Wire it (in `BLineAutos`, `RobotContainer`)

- **Bind events** with `BLineAutos.bind("key", command)` in `RobotContainer`, before any routine
  is built. A bound command must **not** require the drive, and it is not cancelled with the
  routine: give it an end condition and leave the mechanism safe if the path is interrupted.
  An unbound key only logs a warning on the field. The smoke test catches it once the path exists.
- **Add the routine** inside `BLineAutos.all(...)` with `add(routines, "Name", supplier,
  "path-1", "path-2", ...)`. List the path files in order — the auto preview draws from that list,
  and a routine added with `routines.put` instead has no preview and a false start check.
- **Compose the routine** as in the example at the bottom of `BLineAutos`:
  - Reset odometry on the **first path only** with `BLineAutos.followFirst(builder, first)`.
    Use `BLineAutos.follow(builder, path)` for the rest. Never `drive.setPose(path.getStartPose())`.
  - Every path command gets `.withTimeout(...)`. BLine never times out on its own. Size each value
    from the path's measured duration in sim, with margin, and keep the routine's total inside the
    auto period. The 3.0 s `Constants.Waits.PATH_TIMEOUT_SECONDS` placeholder only covers about a
    5 m leg at the shipped limits; give a longer path its own value in `Constants.Waits`.
  - If an event command shares a requirement with another step of the same routine, use a requirements-free V3 coroutine
    and `co.await(...)` for each path or action. A group that reserves the event mechanism can be
    canceled by the event.
  - Name the routine: `"Auto_<Name>"`.
- **Never** put `driveToPose` in a routine. It does not finish.
- Do not also call `Path.flip()` or `mirror()` on a path you give the builder.

### 5. Verify

Run in order and report each result:

1. `./gradlew spotlessApply compileJava`
2. `./gradlew build` — includes the event-key check against the new paths.
3. `./gradlew simulateJava`, then in AdvantageScope / Elastic:
   - Select the Left and Right versions of the auto on the Driver Station and try both alliances
     (Driver Station sim). The **Auto Preview** widget must show the path on the correct side each
     time, and `Auto/StartCheck/*` must report the start pose.
   - Run auto. `FollowPath/pathTranslations` should match the preview. Check elapsed time against
     the auto period (the 2026 routines overran every match) and the sum of path timeouts.
4. Report what was **not** checked: nothing here proves it on carpet.

### 6. Hand-off checklist for the user (on the robot, in this order)

Robot on blocks first; someone on disable. Then: pushed-by-hand pose check, slow first run at
reduced limits, then the intended limits. Pre-orient the modules along the first segment for an
aggressive start (BLine's `Path.getInitialModuleDirection`; this repo does not do it for you).
Check the mirrored run too — a side-mounted intake or shooter faces the wrong way when mirrored.

---

## Mode B — Tune the follower

Tuning is a **sim-first loop**, then a user-confirmed change. The gains live in `BLineAutos`.
Hand the generic loop mechanics to `/pid-tune` (it owns harness, logging and the confirmation
step); this section is what is specific to BLine.

### 1. Read the current state, do not assume it

- Current gains and where they came from (comment above them in `BLineAutos`). They are BLine
  starting values from a trial branch and **have not been tuned on a robot**.
- The shipped gains are BLine's own defaults: translation 2.0, rotation 1.0, cross-track 0.2. They
  are untuned. If a recorded gain is far above those, treat it as the first suspect for hunting,
  and say so to the user.
- `deploy/autos/config.json` limits, and whether they are measured for this robot.
- Translation output is **speed = kP x remaining path distance**. The raw request leaves the
  velocity ceiling at about `vmax / kP` from the end. Overshoot when `kP > 2 x accel / vmax`
  (`2 x 8 / 4.0 = 4.0` at the shipped limits). Tune at the limits the auto will really run at; a
  lower acceleration later changes the response.

### 2. Preconditions — stop and report if any fail

- Pose moves the right distance and direction when the robot is pushed (sim: odometry test).
- Gyro follows WPILib convention; speed supplier and consumer are robot-relative (they are:
  `Drive.getChassisSpeeds` and `Drive.runVelocity`).
- Module drive and steer control is stable and the wheel radius is right. A controller cannot fix
  a wrong wheel radius, gear ratio or noisy pose.
- No minimum-velocity ranged constraints on the test path.
- Wire the evidence: `FollowPath/*` is already logged by `createBuilder`. BLine does **not** log
  measured chassis speed; use `SwerveChassisSpeeds/Measured`.

### 3. Tune one controller at a time, in this order

Use the same start pose, path, limits and axes every run. Record each gain set tried and the
result, so a known-good set can be restored.

**Translation** — straight ~5 m path, no intermediate targets, equal start and end heading,
rotation kP and cross-track kP **zero**, I and D zero. Plot `remainingPathDistanceMeters`,
`rawTranslationControllerOutput`, `clampedTranslationControllerOutput`,
`translationControllerOutput`, `maxTranslationVelocityMetersPerSec`, measured speed,
`finishedTranslationAtSetpoint`.

| Pattern | Meaning | Change |
|---|---|---|
| Raw drops below ceiling early, long tail to the end | kP too low, or modules do not follow low-speed requests | Raise kP in small steps |
| Holds ceiling, leaves it smoothly, reaches tolerance once | Good | Repeat from other distances |
| Enters and leaves tolerance, command reverses | kP too high, accel limit delaying decel, or noisy pose | Lower kP; check measured vs commanded speed |
| Speed changes long after the raw request falls | Acceleration limit or drivetrain response dominates | Test at the intended acceleration first |

**Rotation** — restore the chosen translation gain, cross-track zero. Straight path, start 0°,
end a visible heading (90°). Run once with a non-profiled end rotation target (a step), once with
`profiled_rotation`. Plot `targetRotationDeg`, measured heading, `rotationErrorDeg`,
`rotationPidOutputRadPerSec`, `outputOmegaRadPerSec`, `finishedRotationAtSetpoint`. For a profiled
target, compare against `targetRotationDeg`, not the final heading.

**Cross-track** — last. Use a path with a bend. Start low and raise slowly. Plot
`crossTrackError` and `crossTrackControllerOutput`. Keep I at zero: its state can persist between
path commands in BLine-Lib v0.9.3.

Start with P only. Add D only when a repeatable plot shows it is needed.

### 4. Symptom -> first thing to check

| Symptom | Check first |
|---|---|
| Orbits an intermediate target | Speed at the handoff: add a lower max-velocity range there; confirm t-ratio handoffs are on (`createBuilder` enables them). Not cross-track |
| Endpoint chatters | Translation kP, tolerance tighter than pose noise, entry velocity, minimum-velocity baseline, pose jumps |
| Finishes while still moving | No final-velocity criterion: add a max-velocity cap over the last ordinals; tightening tolerance alone does not help |
| Right shape, wrong direction | Frame contract (robot-relative in and out), no driver-perspective transform on auto output |
| Works on blue, fails on red | A transform applied zero or two times: do not also call `Path.flip()`; first path must reset pose through the builder |
| Wrong side when mirrored | Mirror applied twice, or a side-mounted mechanism; check the selected autonomous OpMode |
| Event does not fire | Exact case-sensitive `lib_key`, `t_ratio` in [0,1], increasing order, key bound before routines were built |
| Event cancels the whole auto | Event command shares a requirement with another step: compose with `BLineCommands`; it must not require the drive |
| Never finishes when blocked | Timeout missing: every path command needs one |
| Drives toward (0,0) first | Pose captured at startup: build runtime paths in the V3 coroutine when that step starts |
| JSON constraint has no effect | Velocity/acceleration constraints must be arrays of ranged objects in v0.9.3 |

### 5. Report, then ask

Give the user: the gains tried, the plots or logged numbers behind each, the proposed set, the
limits and conditions it was tuned at, and **what is sim-only**. State the failure mode if wrong
(overshoot at the endpoint, a hunting robot, an auto that ends early). Do not apply the new gains
to the code until the user confirms. After they confirm, change only the three constants and the
comment above them, update `docs/autos.md` and `docs/robot-spec.md` so they state the new values,
and say the values are sim-validated until a robot run confirms them.

---

## Stop and ask when

- The change needs a CAN ID, offset, inversion, current-limit or PID change outside the three
  follower gains.
- The path needs a speed or acceleration above `config.json`'s ceilings.
- A mirrored path would put a mechanism on the wrong side and there is no asymmetric alternative.
- BLine-Lib's behavior is unclear: read the source jar (Gradle cache, `BLine-Lib-*-sources.jar`)
  rather than guessing.
- The user asks for something that removes a timeout.
