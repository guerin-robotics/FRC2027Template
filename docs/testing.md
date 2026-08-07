# Testing

What is under test, what each layer catches, and what to add when you build a mechanism.

Everything here runs on `./gradlew build` and in CI on every PR and push to main. No test
touches hardware; the sim-backed ones use the HAL simulator and the physics `IOSim`
implementations.

---

## The layers

| Layer | Runs | Catches |
|---|---|---|
| **Pure logic** | Instantly, no HAL | Geometry, filters, math helpers |
| **Config validation** | Instantly, no HAL | CAN ID collisions, broken PathPlanner assets |
| **Architecture** | Instantly, bytecode only | The rules in `.claude/rules/` being broken |
| **Wiring** | HAL sim | `RobotContainer` failing to construct or resolve |
| **Simulation** | HAL sim + physics | Commands not converging, loop budget regressions |

---

## What exists

### Pure logic — no HAL, no sim

| Test | Covers |
|---|---|
| `frc/lib/AllianceFlipUtilTest` | Mirroring, and the per-loop cache |
| `frc/lib/PointInPolygonTest` | Ray-casting zone containment |
| `frc/lib/MotorSpecsTest` | Motor curve lookups |
| `frc/robot/RobotStateGeometryTest` | `getAngleToTarget` / `getDistanceToPoint` — the two calls every alignment command builds on |
| `frc/robot/subsystems/vision/VisionFilterTest` | Every branch of the pose-rejection ladder, both directions |

`VisionFilterTest` reads its thresholds from `VisionConstants` rather than restating them.
Retuning a threshold does not break it; deleting or reordering a filter does. That split is
deliberate — the values were earned from real 2026 match logs and are the source of truth.

### Config validation — catches the failure with no symptom

| Test | Covers |
|---|---|
| `frc/robot/CanIdUniquenessTest` | Duplicate `(bus, id)`, IDs outside 0–62, mechanism IDs reusing the swerve block |
| `frc/robot/PathPlannerAssetsTest` | Unparseable `.auto`/`.path`, files hidden in subdirectories, dangling path references, off-field waypoints, zeroed constraints |

Both read the real sources — `TunerConstants` and `src/main/deploy/pathplanner` — so a device
or routine added later is covered without touching the test.

> **The PathPlanner checks currently pass vacuously.** `autos/` and `paths/` are empty, which
> is correct for the template. `reportDiscoveredAssets()` prints the counts on every run so
> that shows as `0 auto(s), 0 path(s)` rather than a silent green tick. They start doing work
> with the first 2027 auto.

### Architecture — `.claude/rules/` as build failures

`frc/robot/ArchitectureRulesTest` uses ArchUnit to enforce nine rules. Each test names the
rule file it comes from:

| Rule | Source |
|---|---|
| Hardware only behind `*IO*` classes | 01-architecture |
| IO interface methods all `default` (keeps `new ModuleIO() {}` compiling) | 01-architecture |
| Every subsystem calls `Logger.processInputs` | 00-safety |
| No subsystem holds another subsystem | 01-architecture |
| `getAlliance()` only via `AllianceFlipUtil` | 01-architecture |
| Controller objects private to `Triggers` | 01-architecture |
| No `Command` subclasses in `frc.robot.commands` | 03-commands |
| `frc.lib` free of robot code except `Constants` | layering |
| No console output from a subsystem | 05-git |

Two exemptions, both documented at the rule with why each is legitimate:
`MatchMetadataLogger` (runs once at match start) and `Drive` (PathPlanner's `shouldFlipPath`,
evaluated in `initialize()` rather than `execute()` — verified against PathplannerLib 2026.1.2,
recheck on a major upgrade).

**If you change a rule, change it in three places:** `.claude/rules/`,
`.github/instructions/default.instructions.md`, and here. The first two drift silently; this
one fails loudly.

### Wiring

`frc/robot/RobotContainerSmokeTest` builds a real `RobotContainer` in SIM and asserts the
constructor completes, the auto chooser yields a command, `Drive` keeps its
`Drive_Joystick` default, every default command is named, and every named command an auto
references was actually registered.

That last one matters more than it looks. An unregistered named command **does not throw** —
PathPlanner substitutes `Commands.none()`, so the auto drives its paths on schedule while the
mechanism does nothing. It reads as a broken mechanism, not a wiring mistake.

Subsystems are found by reflecting `RobotContainer`'s own fields, so a new mechanism is
covered automatically — nothing in that file names `Drive` or `Vision`.

### Simulation and performance

| Test | Covers |
|---|---|
| `commands/DriveToPoseSimTest` | `driveToPose` converges against the physics sim |
| `commands/JoystickDriveAtAngleSimTest` | Heading hold converges |
| `subsystems/drive/DriveOdometrySimTest` | Odometry integrates correctly |
| `subsystems/drive/DrivePeriodicBudgetTest` | `Drive.periodic()` staying inside the loop budget |
| `frc/lib/LoopTimeMonitorTest` | The watchdog that reports an over-budget loop |
| `frc/robot/GainSweepTest` | The gain-sweep harness `/pid-tune` drives |

`DrivePeriodicBudgetTest` measures wall-clock time on a dev laptop or CI runner, not a
roboRIO. A RIO is far slower, so passing is necessary, not sufficient. What it reliably
catches is a *structural* regression — an accidental O(n²), a blocking call added to the hot
path, a heavyweight object rebuilt every cycle. Those show up as multiples, not percentages.
The real check is watching `LoopTiming/AverageMs` from the first day the robot drives.

---

## Adding a mechanism — what to write

`/add-subsystem` scaffolds the six files. Tests are yours:

1. **Setpoints and CAN IDs** — nothing to write. `CanIdUniquenessTest` picks up the new IDs
   automatically, and `RobotContainerSmokeTest` covers the wiring.
2. **The architecture rules apply immediately.** If the scaffold puts a `TalonFX` in the
   subsystem rather than the IO impl, or forgets `Logger.processInputs`, or holds a reference
   to another subsystem, `ArchitectureRulesTest` fails. Run `./gradlew test` before you go
   looking for what you did wrong.
3. **Pure logic gets a unit test** — interpolation tables, readiness bands, zone math. Use
   `.claude/prompts/write-test.md`; the known-correct cases come from measurement, not from
   reading the code.
4. **Closed-loop mechanisms get a sim convergence test.** Copy `DriveToPoseSimTest`'s shape.
   It needs a physics `IOSim` — tuning or testing against stub IO is meaningless.

### The cleanup rule for sim tests

`CommandScheduler` is a JVM-wide singleton and Gradle runs every test class in one JVM. A
command left scheduled, or a subsystem left registered, keeps running during every later
test. Every sim test here has teardown that cancels and unregisters; give any new one the
same. `DriveToPoseSimTest`'s javadoc explains the failure mode in full.

---

## Gaps worth knowing

- **No replay determinism test.** The gold standard for an AdvantageKit codebase: check in a
  short `.wpilog`, replay it, assert outputs match a golden file. Needs a real log, so it is
  a mid-season addition rather than a template one.
- **No auto time-budget test.** `.claude/rules/03-commands.md` records that 2026 autos overran
  their budget and had the last path truncated in *every* match. A sim harness asserting a
  routine completes inside the auto period would regression-test that directly. Waiting on
  2027 autos to exist.
- **`DrivePeriodicBudgetTest` has no `Vision` equivalent.** Drive and Vision were the two
  dominators of `robotPeriodic` in 2026; only one of them is guarded.
