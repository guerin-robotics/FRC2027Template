# FRC 2027 Template — Guerin Robotics (Team 10021)

Season-start template carried over from the 2026 competition robot (`Rebuilt2026`).

It is the AdvantageKit Tuner X swerve template **plus** the drivetrain, vision and
infrastructure work from two competition seasons — with every 2026 mechanism and every
piece of 2026 game logic stripped out.

Build it, deploy it, and the robot drives with field-relative swerve, AprilTag pose
estimation, Choreo trajectory following and point-to-point autos, and full AdvantageKit logging
and replay. Nothing else.

---

## Why This Repo Exists

Every January the team rebuilt the same drivetrain, the same vision pipeline and the same
logging setup, and spent kickoff week doing it instead of working on the game. This repo
ends that. Four goals:

1. **The robot drives on day one.** Field-relative swerve, AprilTag pose estimation and the
   auto framework (Choreo plus PID-to-pose) are already here and already work. Kickoff week goes to the game.
2. **Keep what was earned; drop what expires.** Vision rejection thresholds, current limits
   and instrumentation came out of real match logs and stay. Mechanisms, game logic and 2026
   field geometry are gone — they do not transfer and pretending they do is worse than an
   empty file.
3. **Instrument the failures that already cost us matches.** Brownouts, loop overruns,
   truncated autos, a camera that died mid-event, a motor that rebooted — 2026 had all of
   these and none of them were visible in a log. Each one now has a channel and an alert.
   See the instrumentation table below.
4. **The rules travel with the code.** `CLAUDE.md`, `.claude/rules/` and `docs/` encode the
   architecture decisions, the hard stops and the bring-up order. A new student — or an
   agent — gets the same guardrails the code was written under.

---

## Starting A 2027 Robot Repo — Use This Template, Do Not Fork

This is a **GitHub template repository**. Every robot repo — competition bot, practice bot,
offseason bot — is generated as a *new, independent repository* from this one. **Never fork
it.**

### Creating a robot repo

In the browser, from
[guerin-robotics/FRC2027Template](https://github.com/guerin-robotics/FRC2027Template):

1. Click the green **Use this template** → **Create a new repository**.
2. Owner: **guerin-robotics**. Name it for the robot, not the season template —
   e.g. `Rebuilt2027`, `PracticeBot2027`.
3. Visibility: **Private** while the season is in progress.
4. Leave **Include all branches** unchecked. You want `main` only.

Or from the command line:

```bash
gh repo create guerin-robotics/Rebuilt2027 \
  --private \
  --template guerin-robotics/FRC2027Template \
  --clone
```

Because this template is private, only members of the org with read access to it can
generate from it. That is intentional.

### Why not a fork

A fork looks like the same thing and behaves differently in ways that bite mid-season:

| | Fork | Template |
|---|---|---|
| Pull requests | Default to **this** repo, not the robot repo. A student opening a PR on the robot lands it on the template unless they notice the dropdown | Default to the robot repo, correctly |
| How many | GitHub allows the org **one** fork of a given repo. Comp bot *and* practice bot is impossible | Unlimited — generate as many as the season needs |
| History | Carries this repo's full history, and every robot commit shows up in the template's network graph | Starts clean at one commit; robot history is the robot's |
| Visibility | Tied to the parent; you cannot independently manage it | Set freely at creation |
| Deleting the parent | Rewires or orphans the forks | No effect — there is no link |

The cost of a template over a fork: **there is no upstream link**, so improvements do not
flow automatically in either direction. That is a deliberate trade, and it has one rule
attached — see below.

### If you improve the shared layer, port it back here

While working on a robot repo you will sometimes fix something that is not robot-specific:
a bug in `frc/lib`, a drive or vision improvement, a rule in `.claude/rules/`, a doc that was
wrong. Those belong back in this template, or next season starts from the broken version
again.

Copy the change into a branch of this repo and open a PR here. It is a manual copy — the
robot repo is not a fork and cannot send one. Robot-specific work (mechanisms, game logic,
autos, tuned gains) stays in the robot repo and never comes back.

### First hour in a new robot repo

Do these before writing any mechanism code:

1. Verify the team number in `.wpilib/wpilib_preferences.json`.
2. Confirm `./gradlew build` passes on a fresh clone.
3. Rewrite this README to describe the robot instead of the template — delete this section
   and the one above it. A README that still says "template" in March is a README nobody
   reads.
4. Work [template/NEW_SEASON_CHECKLIST.md](template/NEW_SEASON_CHECKLIST.md) top to bottom.
   It is ordered by dependency; out of order you get numbers that look plausible and are
   wrong.
5. Read [Before This Drives A Real Robot](#before-this-drives-a-real-robot) below and treat
   it as blocking.

---

## Using This With Claude Code

The governance lives in the repo, so it applies wherever the agent runs — terminal, IDE, or
[claude.ai/code](https://claude.ai/code) in the browser against the GitHub repo.

- **`CLAUDE.md`** is loaded automatically at session start. It defines the hard stops (CAN
  IDs, encoder offsets, gains, inversions, `Logger.processInputs()`), the safety hierarchy
  for edits, and when the agent must ask before acting.
- **`.claude/rules/`** carries the architecture, hardware, command, build and git rules that
  `CLAUDE.md` pulls in.
- **`.claude/skills/`** — `/add-subsystem` scaffolds a mechanism (the two files, CAN IDs,
  setpoints, three-mode wiring, triggers); `/debug-match-log` root-causes a field problem from an
  AdvantageKit log; `/pid-tune` runs a sim-first tuning loop.
- **`.claude/prompts/`** holds task templates for the common jobs.
- **`.github/copilot-instructions.md`** and `.github/agents/` configure GitHub Copilot and the
  PR review agent. They **point at** `.claude/rules/` rather than restating it — a restated
  copy is what drifted in 2026. Keep them as pointers; the hard-stop list is the only
  deliberate inline copy.

Working in the browser, the useful shape is one branch and one PR per logical change: the
CI workflow (`spotlessCheck` then `build`) runs on every PR, and the review agent config in
`.github/` reviews against these same rules. Nothing agent-related requires a local
machine — but nothing agent-related can test on hardware either, so treat anything that
moves the robot as needing a human at the driver station.

---

## What's In Here

**Subsystems**

- `subsystems/drive` — full AdvantageKit swerve: `Drive`, `Module`, `PhoenixOdometryThread`,
  and IO implementations for TalonFX, TalonFXS, sim, and replay. Gyro IO for Pigeon2 and
  NavX. Owns the single `SwerveDrivePoseEstimator` and the Choreo trajectory follower
  (`followTrajectory`).
  Logs torque current on both drive and steer motors alongside stator and supply current, so
  module load is readable straight from a match log.
- `subsystems/vision` — multi-camera PhotonVision AprilTag pose estimation with the
  rejection filters tuned against real 2026 match logs (ambiguity, tag distance, a stricter
  single-tag distance limit, angular velocity, pitch/roll, field bounds), per-observation
  rejection-reason logging, and distance/tag-count std dev scaling.

**Mechanism library** — `frc/lib/mechanism`: one shared `MotorIO` schema, `MotorIOTalonFX` /
`MotorIOTalonFXSim`, the `MotorConfig` builder that refuses to build without current limits,
ratios and gains — plus travel bounds on a position mechanism — and three mechanism kinds — `roller/`, `rotary/`, `linear/` —
each with its subsystem, command factories, physics sim and (for the position kinds) a
visualizer. A new mechanism is two files on top of it. In the build; the config builder is under
test (`MotorConfigTest`).

**Device library** — `frc/lib/device`: CANcoder absolute encoder, CANrange and LaserCAN
distance sensors, DIO and LaserCAN beam breaks, CANdle lights — each a wrapper a subsystem owns
plus real, sim and replay IOs — and `CanIdRegistry`, which stops boot on a duplicate CAN ID. See
[docs/library.md](docs/library.md) for how `frc/lib` is laid out and how it relates to 3467's
W8-Library.

**Autos** — three layers in one chooser, all built on one `driveToPose`: Choreo trajectories
(`frc/robot/autos/ChoreoAutos`), a Choreo trajectory with a PID-to-pose finish
(`ChoreoAutos.thenAlign`), and code-only point-to-point autos (`PointToPointAutos`, built from
`frc/lib/auto` waypoints). See [docs/autos.md](docs/autos.md).

**Commands** — `commands/DriveCommands`: `joystickDrive`, `joystickDriveLimited`,
`joystickDriveAtAngle`, `driveToPose`, `driveToWaypoint`, `alignForScore`, `stopWithX`,
`feedforwardCharacterization`, `wheelRadiusCharacterization`.

**Bindings** — `Triggers.java` owns both controllers (flight stick drives, Xbox operates) and
every trigger; `RobotContainer` only reads from it.

**Core** — `Main`, `Robot`, `RobotContainer`, `Constants` in AdvantageKit template shape.
`RobotState` as a game-agnostic singleton (pose, velocity, distance/bearing helpers).

**Instrumentation** — things 2026 did not have, each one closing a failure that went
unnoticed for a whole season:

| What | Channels | The 2026 failure it closes |
|---|---|---|
| Active command logging | `Commands/Active`, `Commands/All/<name>`, `<Subsystem>/CurrentCommand` | No way to tell "never scheduled" from "scheduled and interrupted", or which command owned a mechanism when it misbehaved |
| Loop-time watchdog | `LoopTiming/*` + Alert | Ran ~30 Hz against a 20 ms budget all season, found post-season |
| CAN bus health | `CANBus/Canivore/*` + Alerts | Signal frequency decisions had no visible consequence |
| Match metadata | `Match/*` | Logs identified only by timestamp filename |
| Consolidated health | `RobotHealth/OK`, `RobotHealth/ActiveFaults` | A camera died after q13 and nobody noticed |
| Brownout counting | `BatteryLogger/BrownoutCount`, `MinVoltage` | 491 brownouts across 23 matches, never surfaced in-event |
| Pose divergence detection | `Odometry/OffField`, `PoseJumpCount` | Nothing caught a bad pose estimate |
| Auto duration | `Auto/DurationSeconds`, `Auto/Overran` | Autos overran and truncated the last path every match |
| Motor sticky faults | `Drive/Module*/Sticky*`, `Drive/AnyStickyFault` | A motor that rebooted mid-match left no trace at all |
| Motor temperature | `Drive/HottestMotorCelsius` + Alert | Krakens thermally limit before faulting; nothing logged temp |
| CTRE hoot logging | `.hoot` files on the USB drive | Device-internal state AdvantageKit never sees; required by Tuner X SysId |
| Vision health | `Vision/Camera*/LatencySeconds`, `HasCalibration` | An uncalibrated camera contributes nothing, silently |

**Utilities**

All in `frc/lib/util/`.

| File | Purpose |
|---|---|
| `lib/BatteryLogger` | Per-subsystem current accounting, brownout counting, min voltage |
| `lib/CommandLogger` | Publishes which commands are running |
| `lib/LoopTimeMonitor` | Loop duration, peak, overrun count, over-budget alert |
| `lib/CANBusMonitor` | Bus utilization and error counts |
| `lib/FaultMonitor` | Rolls all fault conditions into one "Robot OK" indicator |
| `lib/MatchMetadataLogger` | Event and match identity, for triaging logs |
| `lib/PhoenixSignalLogger` | CTRE hoot logging, started on enable and stopped on disable |
| `lib/PhoenixUtil` | `tryUntilOk` — retries CTRE config until it sticks |
| `lib/LoggedTrigger` | Trigger wrapper that logs its state |
| `lib/Elastic` | Elastic dashboard notifications and tab switching |
| `lib/ThrowingRunnable` | Functional interface for throwing config calls |
| `lib/AllianceFlipUtil` | Alliance mirroring, cached once per loop |
| `lib/ContinuousConditionalCommand` | Conditional that re-evaluates while running |
| `lib/FieldConstants` | Field dimensions + AprilTag layout |
| `lib/GeomUtil` | Pose/Transform/Twist conversions |
| `lib/PointInPolygon` | Ray-casting zone containment |
| `lib/EdgeDetector` | Rising/falling edges, and counting edges in a window |
| `lib/LoggedTunableNumber` | Dashboard-adjustable constant, gated on `tuningMode` |
| `lib/LoggedTunableBoolean` | Same, for booleans |
| `lib/LoggedTunableProfiledPID` | Tunable `ProfiledPIDController` gains and constraints |
| `lib/MotorSpecs` | Motor free speeds from WPILib's `DCMotor`; computes top speed and the sim gearbox |

Everything shared lives in `frc/lib` — there is no `frc/robot/util`. A utility either applies
to any robot, in which case it goes there, or it belongs to a subsystem, in which case it goes
in that subsystem's package.

**Governance** — `CLAUDE.md` and `.claude/` carry the team's AI rules, prompt templates
and skills forward.

**Docs** — `docs/` carries the team documentation, genericized. The workflow docs
(`ai-development-handbook.md`, `ai-development-playbook.md`, `review-checklist.md`,
`change-classification.md`) are ready to use; the robot-describing ones are skeletons with
banners saying what to fill in. See [docs/README.md](docs/README.md).

**Style reference** — `template/` holds three scaffolds of the two-file mechanism pattern (the
subsystem and its constants, on top of `frc/lib/mechanism`): `exampleRoller` (velocity), `exampleArm` (rotary position), `exampleLift`
(linear position). Plus [GUIDE.md](template/GUIDE.md) and a
[new-season checklist](template/NEW_SEASON_CHECKLIST.md). Not part of the Gradle build.

**Tooling** — `tools/` holds ClaudeScope (`.wpilog` / NT querying with `/scope` and
`/simulate` skills), log-sync (roboRIO → Google Drive between matches), and
wpilib-agent-tools (Python CLI for sim, NT4 recording, log analysis). See
[tools/README.md](tools/README.md).

---

## What Was Removed

All 2026 mechanisms (flywheel, hood, prestage, upper/lower feeder, transport, intake pivot,
intake roller), their commands and sequences, `HardwareConstants` (folded into `Constants`), the
2026 `Triggers` bindings (`Triggers.java` is back, rebuilt around the drive controls), `HubShiftUtil`, `RobotModelVisualizer`, the 2026 field geometry in `FieldConstants`, the
2026 PathPlanner autos and paths (PathPlanner itself was later replaced by Choreo), and the `ALPHA`/`COMP` robot-type switch —
`COMP_TunerConstants` is now simply `TunerConstants`.

---

## Before This Drives a Real Robot

Everything carried over describes the **2026** robot. In rough order:

1. **`generated/TunerConstants.java`** — regenerate with CTRE Tuner X against the real
   drivetrain. CAN IDs, encoder offsets, gear ratios, wheel radius, module positions and
   gains are all 2026 values. The encoder offsets in particular were set by physically
   zeroing each module and cannot be guessed.
2. **`.wpilib/wpilib_preferences.json`** — verify the team number.
3. **`subsystems/vision/VisionConstants.java`** — camera names must match the coprocessor
   config, and `robotToCameraN` are identity placeholders that must be measured. Delete
   cameras the robot doesn't have — the count must match in all three `RobotContainer`
   branches (REAL, SIM, REPLAY).
4. **`lib/FieldConstants.java`** — update `aprilTagLayout` when WPILib ships the 2027 field.
5. **`src/main/deploy/choreo/`** — create the Choreo project with the 2027 robot's mass, MOI,
   wheel COF and bumpers (2026 values are in its README), then tune the follower gains in
   `Drive.java` (`trajectoryX/Y/HeadingController`).
6. **`commands/DriveCommands.java`** — `ANGLE_KP` / `ANGLE_KD` for heading hold.

Vision filter thresholds are geometry-independent and were earned from real match logs.
Keep them.

---

## Build & Run

```bash
./gradlew build            # compile + tests
./gradlew compileJava      # quick compile check
./gradlew spotlessApply    # format (also runs automatically before compileJava)
./gradlew simulateJava     # run in simulation
```

Deploy with the WPILib VS Code extension, or `./gradlew deploy`. Every deploy runs
`checkConstantsDeploy` first, which refuses one with `tuningMode` on from an `event*` branch or a
detached checkout. See [docs/testing.md](docs/testing.md) for that gate and the three test classes.

Logs are written to `/U/logs` on the robot's USB drive. Replay a log with
`./gradlew replayWatch`, or set `Constants.simMode = Mode.REPLAY`.

---

## Notes Carried Forward From 2026

Worth knowing before they bite again:

- **Loop timing.** The 2026 robot ran nearer 30 Hz than 50 Hz, with Drive and Vision
  dominating `robotPeriodic`. Unused `@AutoLogOutput` methods contributed — AdvantageKit
  calls them every loop whether anything reads them or not. Watch loop time from day one.
- **Brownouts.** Hundreds across a single 2026 event under peak draw. The current limits
  in `TunerConstants` were tuned against that data.
- **Vision failure mode.** Every catastrophically wrong accepted pose in the 2026 logs was
  a *single-tag* solve at long range, some with near-zero ambiguity. Hence
  `maxSingleTagDistanceMeters`, which is stricter than the multi-tag limit.
- **Pose estimator.** Divergence is now *detected* (`Odometry/OffField`, `PoseJumpCount`, an
  Alert) but not *guarded* — nothing rejects or corrects a bad estimate. Vision rejection
  filters are still the only thing keeping one from persisting. See `template/GUIDE.md` § D.
- **Auto time budget.** 2026 routines overran the auto period; the final path was truncated
  in every match. Time autos in simulation before competition.
- **Log units.** AdvantageKit logs `Measure`-typed fields in SI base units — temperatures
  come out in Kelvin.
