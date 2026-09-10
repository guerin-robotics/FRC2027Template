# Testing

What is under test, what each layer catches, and what to add when you build a mechanism.

Everything here runs on `./gradlew build` and in CI on every PR and push to main. No test
touches hardware; the sim-backed ones use the HAL simulator and the physics `IOSim`
implementations.

The suite is deliberately small — 39 tests across 9 classes. It covers the things that fail
*silently*, and leaves everything else to review. Adding a test is cheap; maintaining one that
nobody trusts is not.

---

## The layers

| Layer | Runs | Catches |
|---|---|---|
| **Config validation** | Instantly, no HAL | CAN ID collisions |
| **Wiring** | HAL sim | `RobotContainer` failing to construct or resolve |
| **Filter logic** | HAL sim, no physics | Vision pose rejection |
| **Simulation** | HAL sim + physics | Commands not converging, loop budget regressions |

---

## What exists

### Config validation — the failure with no symptom

**`frc/robot/CanIdUniquenessTest`** — duplicate `(bus, id)` pairs, IDs outside 0–62, and
mechanism IDs reusing the swerve block.

A duplicate CAN ID has no symptom: nothing errors, nothing logs, one device wins arbitration
and the other is silently never heard from. The test reads `TunerConstants` and reflects over
`Constants.CanIds`, so a device added later is covered without touching it.

It cannot check whether a constant matches the ID actually flashed into the device. That lives
in Phoenix Tuner X and `docs/hardware-layout.md`.

### Wiring

**`frc/robot/RobotContainerSmokeTest`** builds a real `RobotContainer` in SIM and asserts the
constructor completes, the auto chooser yields a command, `Drive` keeps its `Drive_Joystick`
default, every default command is named, and every named command an auto references was
actually registered.

That last one matters more than it looks. An unregistered named command **does not throw** —
PathPlanner substitutes `Commands.none()`, so the auto drives its paths on schedule while the
mechanism does nothing. It reads as a broken mechanism, not a wiring mistake.

Subsystems are found by reflecting `RobotContainer`'s own fields, so a new mechanism is
covered automatically — nothing in that file names `Drive` or `Vision`.

`PathPlannerAssets.java` beside it is test support, not a test: it scans
`src/main/deploy/pathplanner` for the named commands the autos reference. Those directories are
empty in the template, so that assertion passes vacuously until the first 2027 auto exists.

### Filter logic

**`frc/robot/subsystems/vision/VisionFilterTest`** — 17 cases covering every branch of the
pose-rejection ladder in `Vision.periodic()`, in both directions.

Vision failures are invisible from the driver station. A filter that stops rejecting bad poses
feeds garbage to the estimator and the robot teleports mid-auto; one that starts rejecting good
poses just looks like drift. Thresholds are read from `VisionConstants` rather than restated, so
retuning a value does not break the tests — deleting or reordering a filter does. That split is
deliberate: the values were earned from real 2026 match logs and are the source of truth.

### Simulation and performance

| Test | Covers |
|---|---|
| `commands/DriveToPoseSimTest` | `driveToPose` converges against the physics sim |
| `commands/JoystickDriveAtAngleSimTest` | Heading hold converges |
| `subsystems/drive/DriveOdometrySimTest` | Odometry integrates correctly |
| `subsystems/drive/DrivePeriodicBudgetTest` | `Drive.periodic()` staying inside the loop budget |
| `frc/lib/util/LoopTimeMonitorTest` | The watchdog that reports an over-budget loop |
| `frc/robot/GainSweepTest` | The gain-sweep harness `/pid-tune` drives |

`LoopTimeMonitorTest` drives timing with `SimHooks.pauseTiming()`/`stepTiming()`, so its
assertions about the 20 ms budget and the 25 ms alert threshold are exact rather than
machine-dependent. `DrivePeriodicBudgetTest` is the opposite: it measures wall-clock time on a
dev laptop or CI runner, not a roboRIO. A RIO is far slower, so passing is necessary, not sufficient. What it reliably
catches is a *structural* regression — an accidental O(n²), a blocking call added to the hot
path, a heavyweight object rebuilt every cycle. Those show up as multiples, not percentages.
The real check is watching `LoopTiming/AverageMs` from the first day the robot drives.

---

## What is **not** tested — review these by hand

The architecture rules in `.claude/rules/` are enforced by **review, not by the build**. There
is no automated check for any of them. When reviewing, look for:

- Hardware (`TalonFX`, `CANcoder`, `SparkMax`) anywhere outside an `*IO*` class
- A subsystem `periodic()` missing `Logger.processInputs()`
- A subsystem holding a reference to another subsystem
- `DriverStation.getAlliance()` called outside `AllianceFlipUtil`
- A controller object outside `Triggers.java`
- `extends Command` in `frc.robot.commands`
- `frc.lib` depending on anything in `frc.robot` except `Constants`
- A command factory without `.withName()`, or a `waitUntil()` without `.withTimeout()`
- A setpoint inlined at a binding instead of coming from `Constants.Setpoints`

`docs/review-checklist.md` is the working version of that list. These were briefly enforced by
an ArchUnit test; it was removed to keep the suite small, so the checklist is now the only
thing standing between the codebase and these regressions. Read it before merging.

---

## Adding a mechanism — what to write

`/add-subsystem` scaffolds the two files. Tests are yours:

1. **CAN IDs** — nothing to write. `CanIdUniquenessTest` picks up new IDs automatically.
2. **Wiring** — nothing to write. `RobotContainerSmokeTest` reflects over `RobotContainer`'s
   fields, so a new subsystem is covered the moment it is wired in.
3. **Closed-loop mechanisms get a sim convergence test.** Copy the one for the matching kind
   in `src/test/java/frc/lib/mechanism/` — `RollerMechanismSimTest`, `RotaryMechanismSimTest` or
   `LinearMechanismSimTest` — rather than `DriveToPoseSimTest`, which is a different harness. A
   mechanism built on `frc/lib/mechanism` always has physics, and the configured gains run on a
   simulated Talon, so what the test measures is what the robot will do. See the harness rules
   below before you write one; two of them will otherwise waste an afternoon.
4. **Pure logic gets a unit test only when it is worth one** — an interpolation table, a
   readiness band, zone math. Use `.claude/prompts/write-test.md`; the known-correct cases come
   from measurement, not from reading the code. A test that just restates the implementation is
   worse than none.

### Sim tests on a Phoenix device — two rules

These apply to any test that drives a mechanism from `frc/lib/mechanism`. Both failures look
exactly like broken physics, which is why they are written down rather than left to be rediscovered.

**Let wall-clock time pass; do not step the FPGA clock.** Phoenix's device simulation advances on
real time. `SimHooks.stepTiming()` does nothing for it, so a tight loop leaves the simulated
Talon's control loop never ticking and the mechanism sits at exactly zero. Sleep the loop period
each iteration:

```java
mechanism.periodic();
Thread.sleep(20);
```

`ModuleIOSim` needs none of this because it models everything in Java and never touches a Phoenix
device. That is the difference to look for: if your test constructs a `*MechanismSim`, it needs to
sleep.

**Wait for a condition to hold; do not run a fixed number of loops.** The physics steps a fixed
20 ms per call while the device advances on wall-clock time, so how much control the device gets
per physics step depends on how loaded the machine is. A fixed loop count that passes on an idle
laptop fails on a busy CI runner. Copy the `settles()` helper from any of the three tests — it
requires the condition to hold for ten consecutive loops, which also stops a mechanism overshooting
its goal from counting as having arrived.

**If the test schedules commands, point the time source at the FPGA clock first:**

```java
RobotController.setTimeSource(RobotController::getFPGATime);
```

WPILib's `Timer` reads `RobotController.getTime()`, which `IterativeRobotBase` advances once per
loop. There is no robot base in a unit test, so without this every `WaitCommand` and every
`withTimeout` waits forever — `LinearMechanismSimTest` does it for the zeroing routine.

### The cleanup rule for sim tests

`CommandScheduler` is a JVM-wide singleton and Gradle runs every test class in one JVM. A
command left scheduled, or a subsystem left registered, keeps running during every later
test. Every sim test here has teardown that cancels and unregisters; give any new one the
same. `DriveToPoseSimTest`'s javadoc explains the failure mode in full.

### Global state and the scheduler

Two JVM-wide singletons will bite a new test if you let them. `CommandScheduler` is one, above.
PathPlanner's `AutoBuilder` is the other: it used to be configured from `Drive`'s constructor,
so every test that built a `Drive` rebound it and PathPlanner reported an error on every run.
It now lives in `Drive.configureAutoBuilder()`, which only `RobotContainer` calls — so a sim
test gets a `Drive` that has not touched PathPlanner global state. If you write a test that
needs path following, call `configureAutoBuilder()` on your own `Drive` explicitly.
