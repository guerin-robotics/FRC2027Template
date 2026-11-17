# Testing

What is under test, what each check catches, and what to add when you build a mechanism.

The suite is deliberately minimal, modelled on 6328's public robot code (one
`RobotContainerTest`, plus constant checks wired into deploy and PRs): **3 test classes**, all
fast, none touching hardware. It covers the failures that are *silent* — a robot that will not
construct, two devices on one CAN ID, an event marker nothing is bound to — and leaves
behaviour to simulation, logs and practice time. Adding a test is cheap; maintaining one that
nobody trusts is not.

`./gradlew build` runs them, and CI runs `spotlessCheck` and `build` on every PR and push to
main.

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

**At runtime, `frc/lib/device/CanIdRegistry` is the second net.** Every CTRE or Grapple IO —
each mechanism's `*IOReal` (motor, follower, encoder) and each real IO under `frc/lib/device` — claims
its ID when constructed, and a second claim on the same bus stops robot code at boot naming both
owners. That covers what the test cannot see: an ID typed into a constructor, or two mechanisms
built from one constant. Swerve devices do not claim; the test covers them.

### Wiring

**`frc/robot/RobotContainerSmokeTest`** builds a real `RobotContainer` in SIM and asserts the
constructor completes, the auto chooser yields a command, `Drive` keeps its `Drive_Joystick`
default, every default command is named, and every Choreo event marker a trajectory uses has a
command bound to it.

That last one matters more than it looks. An unbound event marker **does not throw** — ChoreoLib
fires the marker, finds nothing, and the auto drives its trajectory on schedule while the mechanism
does nothing. It reads as a broken mechanism, not a wiring mistake.

Subsystems are found by reflecting `RobotContainer`'s own fields, so a new mechanism is
covered automatically — nothing in that file names `Drive` or `Vision`.

`ChoreoAssets.java` beside it is test support, not a test: it scans `src/main/deploy/choreo/*.traj`
for event-marker names. There are no trajectories in the template, so that assertion passes
vacuously until the first 2027 trajectory exists.

---


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
| `frc/lib/mechanism/**` (`MotorConfigTest`, `*MechanismSimTest`, `LinearZeroingSoftLimitTest`, `RotaryGravityFrameTest`, …) | The removed W8-style mechanism library. Not applicable to the per-mechanism scaffolds, but the zeroing and gravity-frame tests show what to assert if you test those paths | `f9b09f9` (sim tests), `4dcafdb^` (`MotorConfigTest`) |
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

`/add-subsystem` scaffolds the six files. Tests are mostly already done:

1. **CAN IDs** — nothing to write. `CanIdUniquenessTest` picks up new IDs automatically.
2. **Wiring** — nothing to write. `RobotContainerSmokeTest` reflects over `RobotContainer`'s
   fields, so a new subsystem is covered the moment it is wired in.
3. **Config** — nothing automated. The scaffolds fail to compile until the unguessable values are
   supplied, but a current limit or soft limit edited to something wrong afterward is caught
   only in review. Check `getFXConfig()` against the scaffold you copied.
4. **Anything else is optional.** Pure logic worth pinning — an interpolation table, zone math —
   gets a unit test via `.claude/prompts/write-test.md`, with known-correct cases from
   measurement rather than from reading the code. For a sim convergence test, recover
   `DriveToPoseSimTest` from the table above as the shape to copy.

### If you bring a sim test back — harness rules

The scaffold `*IOSim`s model everything in Java with WPILib physics classes, so they advance on the
FPGA sim clock and `SimHooks.stepTiming()` drives them. **An IO that drives Phoenix's device
simulation (`TalonFXSimState`) does not**: the simulated Talon advances on wall-clock time, so a
tight loop leaves its control loop never ticking and the mechanism sits at exactly zero. Sleep the
loop period each iteration in that case.

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

`CommandScheduler` is a JVM-wide singleton that will bite a new test if you let it — see above.
`ChoreoAutos.boundEvents()` is static too: it records every `ChoreoAutos.bind` call so the smoke
test can check markers, and it is never cleared. A test that builds a second `RobotContainer` sees
the first one's bindings as well; that only ever makes the marker check more lenient, never
stricter. (PathPlanner's global `AutoBuilder`, which caused the old version of this warning, is
gone with PathPlanner.)
