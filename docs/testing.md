# Testing

What is under test, what each check catches, and what to add when you build a mechanism.

The suite is deliberately minimal, modelled on 6328's public robot code (one
`RobotContainerTest`, plus constant checks wired into deploy and PRs): **3 test classes**, all
fast, none touching hardware. It covers the failures that are *silent* — a robot that will not
construct, two devices on one CAN ID, a motor config missing a safety value — and leaves
behaviour to simulation, logs and practice time. Adding a test is cheap; maintaining one that
nobody trusts is not.

`./gradlew build` runs them, and CI runs `spotlessCheck`, `checkConstantsPullRequest` and
`build` on every PR and push to main.

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

### Mechanism library

| Test | Covers |
|---|---|
| `MotorConfigTest` | The builder refuses to build without a current limit, ratio, gains or — on a position mechanism — travel bounds, and derives the torque clamp and follower config correctly |

---

## Constant gates — not tests, but run by Gradle

Two `JavaExec` tasks read `Constants` and fail the build. They live as nested classes at the
bottom of `Constants.java`.

| Task | Runs | Fails when |
|---|---|---|
| `checkConstantsDeploy` | Before every `deploy` | `tuningMode` is on **and** the branch starts with `event` |
| `checkConstantsPullRequest` | In CI, on every PR and push to main | `tuningMode` is on |

Deploying with `tuningMode` on from an ordinary branch is allowed on purpose — that is how a
tuning session gets tunables onto the robot. What is blocked is taking it to an event, and
merging it.

---

## Removed tests — where to find them

The template used to carry about 95 tests. They were cut to this minimum; recover any of them
with `git show <commit>:<path>` if a mechanism or season needs one back.

| Test | What it covered | Last present at |
|---|---|---|
| `frc/robot/subsystems/vision/VisionFilterTest` | Every branch of the vision pose-rejection ladder | `7b81e48` |
| `frc/robot/commands/DriveToPoseSimTest`, `JoystickDriveAtAngleSimTest` | `driveToPose` and heading hold converge in sim | `7b81e48` |
| `frc/robot/subsystems/drive/DriveOdometrySimTest` | +X/+Y/+ω commands move odometry the right way | `7b81e48` |
| `frc/robot/GainSweepTest` | Gain-sweep harness for `/pid-tune` | `7b81e48` |
| `frc/lib/util/LoopTimeMonitorTest` | The loop-overrun watchdog | `7b81e48` |
| `frc/lib/mechanism/MotorIOReplayTest` | Follower and encoder groups survive replay | `7b81e48` |
| `frc/lib/mechanism/linear/LinearGeometryTest`, `LinearZeroingSoftLimitTest` | Drum/stage math; zeroing drops and restores the reverse bound | `7b81e48` |
| `frc/lib/mechanism/{roller,rotary,linear}/*MechanismSimTest`, `rotary/RotaryGravityFrameTest`, `MotorIOTalonFXSimEncoderTest` | Phoenix device-sim convergence, arm gravity frame, CANcoder seeding | `f9b09f9` |
| `frc/robot/subsystems/drive/DrivePeriodicBudgetTest` | Wall-clock cost of `Drive.periodic()` | `f9b09f9` |

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
- Any change to the vision rejection ladder in `Vision.periodic()` — `VisionFilterTest` used to
  pin every branch; now only review does

`docs/review-checklist.md` is the working version of that list. These were briefly enforced by
an ArchUnit test; it was removed to keep the suite small, so the checklist is now the only
thing standing between the codebase and these regressions. Read it before merging.

---

## Adding a mechanism — what to write

`/add-subsystem` scaffolds the two files. Tests are mostly already done:

1. **CAN IDs** — nothing to write. `CanIdUniquenessTest` picks up new IDs automatically.
2. **Wiring** — nothing to write. `RobotContainerSmokeTest` reflects over `RobotContainer`'s
   fields, so a new subsystem is covered the moment it is wired in.
3. **Config** — nothing to write. `MotorConfig.builder(...)` throws at startup on a missing
   safety value, and the smoke test constructs it.
4. **Anything else is optional.** Pure logic worth pinning — an interpolation table, zone math —
   gets a unit test via `.claude/prompts/write-test.md`, with known-correct cases from
   measurement rather than from reading the code. For a sim convergence test, recover the
   matching `*MechanismSimTest` from the table above rather than writing a harness from scratch.

### If you bring a sim test back — harness rules

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
laptop fails on a busy CI runner. Copy the `settles()` helper from the recovered mechanism tests — it
requires the condition to hold for ten consecutive loops, which also stops a mechanism overshooting
its goal from counting as having arrived.

**If the test schedules commands, point the time source at the FPGA clock first:**

```java
RobotController.setTimeSource(RobotController::getFPGATime);
```

WPILib's `Timer` reads `RobotController.getTime()`, which `IterativeRobotBase` advances once per
loop. There is no robot base in a unit test, so without this every `WaitCommand` and every
`withTimeout` waits forever.

### The cleanup rule for sim tests

`CommandScheduler` is a JVM-wide singleton and Gradle runs every test class in one JVM. A
command left scheduled, or a subsystem left registered, keeps running during every later
test. Every sim test needs teardown that cancels and unregisters — `RobotContainerSmokeTest`'s
`unregisterSubsystems()` is the pattern.

### Global state and the scheduler

Two JVM-wide singletons will bite a new test if you let them. `CommandScheduler` is one, above.
PathPlanner's `AutoBuilder` is the other: it used to be configured from `Drive`'s constructor,
so every test that built a `Drive` rebound it and PathPlanner reported an error on every run.
It now lives in `Drive.configureAutoBuilder()`, which only `RobotContainer` calls — so a sim
test gets a `Drive` that has not touched PathPlanner global state. If you write a test that
needs path following, call `configureAutoBuilder()` on your own `Drive` explicitly.
