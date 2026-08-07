# Guerin Robotics — Robot Specification

> **THIS IS A SPEC SKELETON.** The 2026 version of this document was a complete
> ~1450-line specification of that robot: every constant, every subsystem, every command,
> every binding. It was the single best artifact for onboarding someone (or an agent)
> onto the codebase fast.
>
> That content was all 2026-specific, so it was not carried over. What remains is the
> **structure**, plus the sections that describe code that actually shipped in this
> template (§2–§6, §9, §11, §14–§17).
>
> Fill the TODO sections in as the 2027 robot is built. Keeping this current is what makes
> the rest of the AI workflow work — see `docs/ai-development-handbook.md` §7.

**Status:** template skeleton — not yet a description of any robot
**Last updated:** when this template was created

---

## 1. Game Context

TODO — after kickoff, describe:

- The game, in a paragraph
- Scoring elements and where they score
- Match phases and their durations
- What this robot is designed to do, and explicitly what it is *not*
- Alliance/field asymmetry that affects code (mirrored vs rotated field)

The 2026 version of this section was what let an agent reason about *why* a sequence
existed, not just what it did. It is worth the half hour.

---

## 2. Software Stack

| Layer | Choice |
|---|---|
| Framework | WPILib Command-Based |
| Logging / replay | AdvantageKit (`LoggedRobot`, IO layer, `@AutoLog`) |
| Motor control | CTRE Phoenix 6 (TalonFX, TalonFXS, CANcoder, Pigeon2) |
| Vision | PhotonVision over NetworkTables |
| Path following | PathPlannerLib (`AutoBuilder`, `LocalADStarAK` pathfinder) |
| Dashboard | Elastic + AdvantageScope |
| Formatting | Spotless / Google Java Format (runs on `compileJava`) |
| Java | 17 |

Versions live in `vendordeps/` — see `docs/hardware-layout.md` for the table.

---

## 3. Package Structure

```
frc/lib/                     ALL shared utilities — see docs/subsystem-ownership.md
frc/robot/                   Main, Robot, RobotContainer, RobotState, Constants, BuildConstants
frc/robot/generated/         TunerConstants (Tuner X generated — never hand-edit)
frc/robot/subsystems/drive/  Drive, Module, Gyro/Module IO, PhoenixOdometryThread
frc/robot/subsystems/vision/ Vision, VisionConstants, io/
frc/robot/commands/          DriveCommands (+ one file per mechanism)
```

Tests mirror this structure under `src/test/java`. See [testing.md](testing.md) for what each
layer covers, and — just as important — what it does not.

TODO — add mechanism packages as they are created.

---

## 4. Architecture Rules

Summarized here; authoritative version is `.claude/rules/01-architecture.md`.

- Every subsystem wraps hardware behind an `XxxIO` interface; hardware objects exist only
  in `IOReal`. This is what makes log replay work.
- No subsystem holds a reference to another subsystem. Shared state goes through
  `RobotState`; one-off values go through a `Supplier` passed at construction.
- Commands are static factory methods, never `extends Command` classes. Every one calls
  `.withName()`.
- Exactly one `SwerveDrivePoseEstimator`, inside `Drive`. `RobotState` delegates to it.
- `TunerConstants` is the sole source of truth for drivetrain geometry and gains.
- Never call `DriverStation.getAlliance()` in a hot path — use `AllianceFlipUtil`.
- Every `waitUntil` has a `withTimeout`.

**None of these are checked automatically.** They are enforced by review, using
[review-checklist.md](review-checklist.md). An ArchUnit test enforced them briefly and was
removed to keep the suite small.

---

## 5. Robot.java — Lifecycle

| Phase | What happens |
|---|---|
| Constructor | Record build metadata; configure AdvantageKit receivers per `Constants.currentMode`; `Logger.start()`; register `RobotState` with `AutoLogOutputManager`; construct `RobotContainer` |
| `robotPeriodic` | `AllianceFlipUtil.refresh()` → `CommandScheduler.run()` → `batteryLogger` update |
| `autonomousInit` | Pull selected auto from the chooser and schedule it |
| `teleopInit` | Cancel the auto command |
| `testInit` | `CommandScheduler.cancelAll()` |

`AllianceFlipUtil.refresh()` must run **before** the scheduler — commands read the cached
value during their own execution.

TODO — 2026 also did hub-shift init, driver-preset latching, controller-mode latching, and
dashboard publishing here. Add whatever 2027 equivalents appear, and keep this table current.

---

## 6. Constants.java

Runtime mode only.

- `simMode` — `SIM` or `REPLAY`, chosen at compile time
- `currentMode` — `REAL` on a roboRIO, otherwise `simMode`
- `disableHAL` — set true in unit tests so `AllianceFlipUtil` skips DriverStation access

The 2026 `RobotType` enum (COMP/ALPHA) was removed along with the dual `TunerConstants`.
If a practice robot needs different constants in 2027, prefer auto-detecting it (CAN
device ping, jumper pin) over a hand-edited flag — deploying the wrong constants under
competition pressure was a known 2026 risk.

---

## 7. Where Constants Live

**One constants file.** 2026 had both `Constants` and `HardwareConstants`, and the split meant
remembering which of the two a value lived in while a match clock was running. They are merged.

```
Constants
├── Mode / simMode / currentMode   runtime mode selection
├── disableHAL                     unit-test escape hatch
├── tuningMode                     enables dashboard-adjustable constants; FALSE for competition
├── CanIds                         every non-swerve CAN ID, each with // CANivore or // RIO CAN
├── Setpoints                      voltages, velocities, positions a mechanism is commanded to
├── Waits                          every command timeout — nothing inline
└── Thresholds                     alignment tolerances, readiness bands
```

The rule: **if you would change it in the pit between matches, it goes in `Constants`.**

Everything describing how a mechanism is built or characterized stays in that subsystem's own
`*Constants.java` — gains and their sim/real split, current limits, gear ratio, sim model
parameters, and interpolation maps. A distance-to-setpoint table is structure rather than a
knob, and putting it in `Constants` would bury the values people actually reach for.

Swerve is outside both: its IDs, ratios, gains and limits are Tuner X output in
`generated/TunerConstants.java` and must not be hand-edited.

---

## 8. CAN Bus Layout

See `docs/hardware-layout.md` — that is the source of truth. Do not duplicate the table
here; the 2026 spec did and the two drifted.

---

## 9. Drive Subsystem

**Files:** `subsystems/drive/` + `commands/DriveCommands.java` + `generated/TunerConstants.java`

**Structure.** Four `Module`s (FL, FR, BL, BR), each a `ModuleIO` + closed-loop control,
plus a `GyroIO`. `PhoenixOdometryThread` samples drive position, steer position, and gyro
yaw at 250 Hz on CAN FD (100 Hz otherwise) into timestamped queues. `Drive.periodic()`
drains those queues under `odometryLock` and feeds `poseEstimator.updateWithTime()`.

**Pose estimation.** One `SwerveDrivePoseEstimator`, owned by `Drive`. Vision enters via
`addVisionMeasurement()`. `RobotState` reads it through `poseSupplier`, set in the `Drive`
constructor. If the gyro disconnects, heading falls back to integrating module deltas via
`kinematics.toTwist2d()` and an `Alert` is raised.

**PathPlanner.** `AutoBuilder.configure()` lives in the `Drive` constructor. Path-following
gains and `PP_CONFIG` (mass, MOI, wheel COF) are there too — all 2026 values, all need
re-measuring.

**Commands.** `joystickDrive`, `joystickDriveLimited`, `joystickDriveAtAngle`, `stopWithX`,
`feedforwardCharacterization`, `wheelRadiusCharacterization`. Every "align to X" command is
built by passing a heading supplier to `joystickDriveAtAngle`.

**Logged per module.** Connected flags, drive position/velocity/applied volts, stator
current and **torque current**, turn absolute/relative position, velocity, applied volts,
stator current and **torque current**, plus the odometry sample queues. Both motors run
`*TorqueCurrentFOC` requests, so torque current is the actual control signal — stator
current alone cannot distinguish a module fighting a stall from one spinning free.
Registered at 50 Hz in the same frame as stator current, so it adds no CAN traffic.
`ModuleIOSim` leaves both torque channels at zero rather than inventing a value.

**Gains.** Both drive and steer use `ClosedLoopOutputType.TorqueCurrentFOC`, so every
Slot0 gain is in amps. See `docs/characterization-and-tuning.md` for how to measure them.

**2026 performance notes.** Voltage-saturated near 3.8 m/s rather than current-limited.
Drive supply limit went 40 A → 60 A mid-season: ~12% more peak acceleration, 4× the
brownouts, no change in top speed.

TODO — add the 2027 alignment commands as they are written.

---

## 10. Mechanism Subsystems

**TODO — one section per mechanism.** Use this shape for each:

```
### [Mechanism Name]

**Files:** subsystems/x/… + commands/XCommands.java
**Hardware:** motor type, CAN ID(s), bus, followers (and whether they oppose), encoder
**Control mode:** VoltageOut / VelocityTorqueCurrentFOC / MotionMagic…
**Gains:** kP/kI/kD/kS/kV/kA and where they live
**Current limits:** supply / stator, and why those values
**Logged inputs:** the @AutoLog field list — voltage, stator amps, supply amps,
                   velocity, temperature, and torque current if any control request
                   is *TorqueCurrentFOC (see .claude/rules/02-hardware.md)
**State queries:** isAtVelocity(), isAtPosition(), … and their tolerances
**Commands:** every factory, what it does, what it names itself
**Default command:** what it idles to
**Failure modes:** what breaks if this is wrong
```

The 2026 spec had nine of these (flywheel, hood, intake pivot, intake roller, prestage,
upper feeder, lower feeder, transport, plus drive and vision). That level of detail is the
point — it is what let an agent make a correct change without reading every file.

---

## 11. Vision Subsystem

**Files:** `subsystems/vision/`

**Structure.** N cameras, each a `VisionIO`. `Vision.periodic()` pulls observations,
filters them, computes standard deviations, and hands survivors to the consumer wired to
`drive::addVisionMeasurement`.

**Rejection filters,** checked in priority order — first match wins and is logged to
`Vision/CameraN/RejectionReason`:

| Reason | Condition |
|---|---|
| `AngularVelocityTooHigh` | robot yaw rate > `maxAngularVelocityRadPerSec` (6.0) |
| `NoTags` | `tagCount == 0` |
| `InvalidTimestamp` | timestamp ≤ 0 |
| `HighAmbiguity` | single-tag and ambiguity > `maxAmbiguity` (0.35) |
| `BelowFloor` | pose Z < −`floorError` |
| `ZTooHigh` | pose Z > `maxZError` (2 m) |
| `TagsTooFar` | average tag distance > `maxDistanceMeters` (8.0) |
| `SingleTagTooFar` | single-tag and distance > `maxSingleTagDistanceMeters` (4.0) |
| `PitchRollTooLarge` | pitch or roll > 25° |
| `OutsideField` | pose outside the AprilTag layout bounds |

**Standard deviations.** Base × distance² / tagCount, then ×2 for single-tag, then the
per-camera factor. MegaTag2 observations halve linear std dev and set angular to infinity.

**Why the single-tag distance limit exists.** Every catastrophically wrong accepted pose in
the 2026 State and Worlds logs (2.5–9.6 m off) was a single-tag solve at ≥ 3.8 m, some with
near-zero ambiguity. The ambiguity filter alone could not catch them. Multi-tag solves at
4–6 m were never catastrophically wrong, so they keep the looser limit.

**Known gaps.** No pose-estimator divergence guard. Camera transforms are identity
placeholders. A camera died mid-event in 2026 (`RobotLeft`, after q13) and nothing alerted
beyond the disconnect `Alert`.

---

## 12. RobotState — API

Current surface (game-agnostic):

| Method | Returns |
|---|---|
| `getEstimatedPose()` | `Pose2d` from Drive's estimator |
| `getRotation()` | current heading |
| `getFieldRelativeVelocity()` | `ChassisSpeeds`, field frame |
| `getRobotRelativeVelocity()` | `ChassisSpeeds`, robot frame |
| `getDistanceToPoint(Translation2d)` | `Distance` |
| `getAngleToTarget(Translation2d)` | bearing for the robot **front** |
| `setPoseSupplier(Supplier<Pose2d>)` | wiring, called once by `Drive` |
| `updateModuleStates(SwerveModuleState[])` | called each loop by `Drive` |

TODO — add scoring targets, alignment predicates, and zone classification for 2027.

Note `getAngleToTarget` returns the true bearing. The 2026 version silently added 180°
because that shooter faced backward; the offset is now the caller's job.

---

## 13. Triggers.java

**TODO — does not exist yet.** Bindings live inline in `RobotContainer` while there are
only four. Create `Triggers.java` as soon as operator controls and state triggers appear;
`.claude/rules/01-architecture.md` explains why they belong in one file.

---

## 14. Command Implementations

See §9 for drive commands. TODO for mechanism commands.

Patterns that must survive the season:

- Static factories only, every one `.withName()`'d
- Every `waitUntil` has a `withTimeout`
- Ready → Align → Act, where phase 2 gets the *remaining* budget, not a fresh one
- `ContinuousConditionalCommand` when a mode can change mid-execution

---

## 15. RobotContainer — Binding Logic

Wiring layer. Constructs subsystems per `Constants.currentMode` (REAL / SIM / REPLAY),
registers PathPlanner named commands and event triggers **before**
`AutoBuilder.buildAutoChooser()`, builds the auto chooser, then configures bindings.

Current bindings: default `joystickDrive`, A = hold 0° heading, X = X-wheels, B = reset gyro.

No game logic here. Ever.

---

## 16. Utility Classes

| Class | Purpose |
|---|---|
| `AllianceFlipUtil` | Alliance mirroring; `refresh()` once per loop, `shouldFlip()` reads the cache |
| `ContinuousConditionalCommand` | Conditional that re-evaluates while running |
| `FieldConstants` | Field dimensions + AprilTag layout; gains game geometry each season |
| `BatteryLogger` | Per-subsystem current accounting for brownout analysis |
| `PhoenixUtil` | `tryUntilOk` — retries CTRE config until it sticks |
| `LocalADStarAK` | Replay-safe PathPlanner pathfinder |
| `LoggedTrigger` | Trigger wrapper that logs its own state |
| `Elastic` | Dashboard notifications and tab switching |
| `ThrowingRunnable` | Functional interface for throwing config calls |

---

## 17. Known Issues Carried From 2026

| Issue | Impact |
|---|---|
| Loop overruns — ~30 Hz actual vs 50 Hz budget, Drive + Vision dominant | Stale control, missed odometry samples |
| 491 brownouts across 23 matches; battery to ~6.3 V at 390 A p95 | Motor cutouts mid-match |
| No pose-estimator divergence guard | A bad estimate persists until good vision arrives |
| `maxPoseJumpMeters` filter written but never tuned or enabled | One less defense against bad poses |
| No jam/stall detection pattern on open-loop mechanisms | Jams are silent |
| Autos overran the auto period; last path truncated every match | Lost auto points |
| `Measure`-typed log fields record in SI base units | Temperatures read as Kelvin |

---

## 18. Robot Physical Specifications

TODO — see `docs/hardware-layout.md`. Weigh the robot, estimate MOI, measure real top
speed rather than trusting the configured value.
