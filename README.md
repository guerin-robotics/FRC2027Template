# FRC 2027 Template — Guerin Robotics (Team 10021)

Season-start template carried over from the 2026 competition robot (`Rebuilt2026`).

It is the AdvantageKit Tuner X swerve template **plus** the drivetrain, vision and
infrastructure work from two competition seasons — with every 2026 mechanism and every
piece of 2026 game logic stripped out.

Build it, deploy it, and the robot drives with field-relative swerve, AprilTag pose
estimation, PathPlanner autos, and full AdvantageKit logging and replay. Nothing else.

---

## What's In Here

**Subsystems**

- `subsystems/drive` — full AdvantageKit swerve: `Drive`, `Module`, `PhoenixOdometryThread`,
  and IO implementations for TalonFX, TalonFXS, sim, and replay. Gyro IO for Pigeon2 and
  NavX. Owns the single `SwerveDrivePoseEstimator` and the PathPlanner `AutoBuilder` wiring.
- `subsystems/vision` — multi-camera PhotonVision AprilTag pose estimation with the
  rejection filters tuned against real 2026 match logs (ambiguity, tag distance, a stricter
  single-tag distance limit, angular velocity, pitch/roll, field bounds), per-observation
  rejection-reason logging, and distance/tag-count std dev scaling.

**Commands** — `commands/DriveCommands`: `joystickDrive`, `joystickDriveLimited`,
`joystickDriveAtAngle`, `stopWithX`, `feedforwardCharacterization`,
`wheelRadiusCharacterization`.

**Core** — `Main`, `Robot`, `RobotContainer`, `Constants` in AdvantageKit template shape.
`RobotState` as a game-agnostic singleton (pose, velocity, distance/bearing helpers).

**Utilities**

| File | Purpose |
|---|---|
| `util/BatteryLogger` | Per-subsystem current accounting for brownout analysis |
| `util/PhoenixUtil` | `tryUntilOk` — retries CTRE config until it sticks |
| `util/CANUpdateThread` | Async CAN device configuration with retry |
| `util/LocalADStarAK` | Replay-safe PathPlanner pathfinder |
| `util/LoggedTrigger` | Trigger wrapper that logs its state |
| `util/Elastic` | Elastic dashboard notifications and tab switching |
| `util/ThrowingRunnable` | Functional interface for throwing config calls |
| `lib/AllianceFlipUtil` | Alliance mirroring, cached once per loop |
| `lib/ContinuousConditionalCommand` | Conditional that re-evaluates while running |
| `lib/FieldConstants` | Field dimensions + AprilTag layout |

**Governance** — `CLAUDE.md` and `.claude/` carry the team's AI rules, prompt templates
and skills forward.

**Docs** — `docs/` carries the team documentation, genericized. The workflow docs
(`ai-development-handbook.md`, `ai-development-playbook.md`, `review-checklist.md`,
`change-classification.md`) are ready to use; the robot-describing ones are skeletons with
banners saying what to fill in. See [docs/README.md](docs/README.md).

**Style reference** — `template/` holds the five-file subsystem pattern every mechanism
must follow, plus [GUIDE.md](template/GUIDE.md) and a
[new-season checklist](template/NEW_SEASON_CHECKLIST.md). Not part of the Gradle build.

**Tooling** — `tools/` holds ClaudeScope (`.wpilog` / NT querying with `/scope` and
`/simulate` skills), log-sync (roboRIO → Google Drive between matches), and
wpilib-agent-tools (Python CLI for sim, NT4 recording, log analysis). See
[tools/README.md](tools/README.md).

---

## What Was Removed

All 2026 mechanisms (flywheel, hood, prestage, upper/lower feeder, transport, intake pivot,
intake roller), their commands and sequences, `HardwareConstants`, `Triggers`,
`HubShiftUtil`, `RobotModelVisualizer`, the 2026 field geometry in `FieldConstants`, the
2026 PathPlanner autos and paths, and the `ALPHA`/`COMP` robot-type switch —
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
5. **`subsystems/drive/Drive.java`** — `ROBOT_MASS_KG`, `ROBOT_MOI` and `WHEEL_COF` in
   `PP_CONFIG`, plus the PathPlanner PID gains.
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

Deploy with the WPILib VS Code extension, or `./gradlew deploy`.

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
- **Pose estimator.** There is no off-field divergence guard. Vision rejection filters are
  the only thing keeping a bad estimate from persisting.
- **Auto time budget.** 2026 routines overran the auto period; the final path was truncated
  in every match. Time autos in simulation before competition.
- **Log units.** AdvantageKit logs `Measure`-typed fields in SI base units — temperatures
  come out in Kelvin.
