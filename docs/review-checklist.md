# Review Checklist

Run through this before submitting or merging any change.
Faster checks first.

---

## 1. Build (required for every change)

```bash
./gradlew build                 # Compiles, checks formatting, and runs the test suite
```

That single command is what CI runs. Use the narrower ones only to iterate faster —
`compileJava` and `spotlessCheck` alone will not catch a rule violation or a broken test, and
a change that passes them can still fail CI.

For changes to `@AutoLog` inputs classes:
```bash
./gradlew clean generateSources # Regenerates AutoLogged classes
./gradlew build
```

**The build covers very little of what follows.** The suite checks CAN ID collisions,
`RobotContainer` wiring and Choreo event-marker bindings — see [testing.md](testing.md). A
mechanism's config — current limits, soft limits, signal rates — is checked by nobody but you;
compare each new `*IOReal` and `getFXConfig()` against the scaffold it was copied from. Vision
filtering and sim convergence are no longer tested, so a change to either is reviewed by hand.
The architecture rules below are enforced by *this checklist* and nothing else, so read them
rather than assuming a green build means they hold.

---

## 2. Scope Check

- [ ] Only the requested files were changed
- [ ] No reformatting of unrelated lines
- [ ] No renamed variables outside the scope of the task
- [ ] No new imports in files that weren't otherwise modified
- [ ] No debug `System.out.println()` left in

---

## 3. Architecture Check

- [ ] No hardware calls (TalonFX, CANcoder, etc.) inside a subsystem class
- [ ] No subsystem-to-subsystem references outside `RobotContainer` or the sequences layer
- [ ] No new command classes (`extends Command`) — use static factories only
- [ ] No controller object outside `Triggers.java`
- [ ] `DriverStation.getAlliance()` not called outside `AllianceFlipUtil`
- [ ] Nothing in `frc.lib` depends on `frc.robot` except `Constants`
- [ ] Every new command factory method calls `.withName()`
- [ ] Every new `waitUntil()` has a `.withTimeout()`

---

## 4. Logging Check

- [ ] Every new subsystem `periodic()` calls `Logger.processInputs()`
- [ ] Every new subsystem `periodic()` calls `Robot.batteryLogger.reportCurrentUsage()`
- [ ] Every new motor logs voltage, stator amps, supply amps, velocity and temperature
- [ ] Any motor driven by a `*TorqueCurrentFOC` request also logs `getTorqueCurrent()`,
      registered at 50 Hz next to stator current
- [ ] New `StatusSignal`s are cached in the constructor and added to the existing batched
      `BaseStatusSignal.refreshAll(...)`, not fetched inside `updateInputs()`
- [ ] Sim IO does not invent values for signals it cannot model (e.g. torque current)
- [ ] New state values use `Logger.recordOutput()` or `@AutoLogOutput`
- [ ] No `Logger.processInputs()` calls removed

---

## 5. Hardware Check (only for changes touching hardware config)

- [ ] No CAN IDs changed *(collisions and out-of-range IDs are caught by `CanIdUniquenessTest`; a valid ID pointing at the wrong device is not)*
- [ ] Any new CAN device class calls `CanIdRegistry.claim(...)` before constructing the device *(the runtime duplicate check only covers devices that claim)*
- [ ] No motor inversion flags changed
- [ ] No swerve encoder offsets changed
- [ ] All new TalonFX configurations use `PhoenixUtil.tryUntilOk(5, ...)`
- [ ] All new signal frequencies are set with `BaseStatusSignal.setUpdateFrequencyForAll()`
- [ ] `motor.optimizeBusUtilization()` called for new motors

---

## 6. Safety Interlock Check (only for changes touching command logic)

- [ ] All `waitUntil()` calls have a `withTimeout()`
- [ ] Timeout constants come from a constants class, not inline
- [ ] Interlock/zone checks not removed from the triggers that gate mechanism motion
- [ ] Any composite "safe to act" trigger logic left intact

---

## 7. Auto Check (only for auto changes)

- [ ] Choreo event markers bound with `ChoreoAutos.bind` before any routine is built
- [ ] No bound command or trajectory-trigger command requires the drive (`thenAlign` on `done()`
      is the one exception)
- [ ] Every PID-to-pose step in an auto finishes — `driveToPoseWithin` / `driveToWaypoint`, never
      bare `driveToPose`
- [ ] No `.traj` / `.chor` files edited in code (use the Choreo GUI)
- [ ] Auto still fits inside the auto period (time it — 2026 overran every match)

---

## 8. Constants Check (only for constant changes)

- [ ] New constants are in the correct constants file (not inline)
- [ ] Magic numbers that were previously inline have been named
- [ ] No constants duplicated across files

---

## 9. Commit Message Check

- [ ] Message explains WHY, not WHAT
- [ ] Message does not start with "Changed" or "Updated" (too vague)
- [ ] If behavioral change: message names the change
- [ ] Commit does not mix unrelated changes

---

## 10. NASA/JPL "Power of 10" Check (every code change)

Gerard Holzmann's [Power of 10 rules](https://spinroot.com/gerard/pdf/P10.pdf) are NASA JPL's
baseline for safety-critical flight software. They were written for C, so some do not carry over
to Java on a roboRIO. The table says how each one applies here. Use it as a second lens on a diff:
a rule this codebase already enforces is listed with the local rule that enforces it, so a
violation is usually a breach of both.

| # | Power of 10 rule | How it applies here | Check |
|---|---|---|---|
| 1 | Simple control flow: no `goto`, `setjmp`, or recursion | Applies as written. No recursion; no exceptions used for control flow | - [ ] No recursion; no `try`/`catch` steering normal logic |
| 2 | Every loop has a fixed upper bound | Every `while`/`for` is bounded, and so is every *wait*: `waitUntil` always has `.withTimeout()`, retries are bounded (`tryUntilOk(5, ...)`). An unbounded wait is this rule's command-based form (`03-commands.md`) | - [ ] Every loop, retry and wait has a stated bound |
| 3 | No dynamic allocation after initialization | Java always allocates, so the target is the 20 ms loop: no `new` in `periodic()`, `updateInputs()` or a command body when it can be built once. Cache `StatusSignal`s, control requests and colors (`04-build.md`, `02-hardware.md`) | - [ ] Nothing allocated per loop that could be built once |
| 4 | Functions fit on one page (about 60 lines) | Applies as written. Long `periodic()` or factory methods split into named helpers | - [ ] No new method over about 60 lines without a reason in the PR |
| 5 | At least two assertions per function, checking things that should never happen | Java `assert` is off on the robot, so check at **construction**: fail fast with a named exception when an invariant is broken (the arm's ratio-product check, `CanIdRegistry`). Runtime anomalies are logged and raised as `FaultMonitor` alerts rather than thrown | - [ ] New invariants are checked at startup, not assumed |
| 6 | Declare data at the smallest possible scope | Applies as written. `private` by default, `final` wherever possible, no mutable `public static` state | - [ ] No wider visibility or mutability than needed |
| 7 | Check every non-void return value, and validate parameters | Every Phoenix `StatusCode` goes through `tryUntilOk` or is checked; inputs that can be out of range are clamped or rejected, and the clamp is logged (`GoalWasClamped`) | - [ ] No ignored `StatusCode`; out-of-range inputs handled visibly |
| 8 | Limit the preprocessor | No preprocessor in Java. The nearest equivalent is compile-time flags that change behavior, which this repo avoids (there is no tuning-mode flag) | - [ ] No new behavior-changing compile-time flag |
| 9 | Restrict pointers; no function pointers | Not applicable as written: lambdas and suppliers are idiomatic WPILib. The spirit is traceable control flow, so keep a lambda short and give a command a `.withName()` | - [ ] Lambdas are short; commands are named |
| 10 | Compile with all warnings on and zero warnings; run static analysis | Zero compiler warnings, never suppressed (a Hard Stop in `CLAUDE.md`), `spotlessCheck` clean, and `./gradlew build` green | - [ ] No new warnings, no `@SuppressWarnings` |

A rule marked "applies as written" is a hard requirement for new code. For the rest, the adapted
check is the requirement; the original rule is context.

---

## Quick Reference: Where Things Belong

| What | Where |
|---|---|
| Swerve CAN IDs, gains, geometry, current limits | `generated/TunerConstants.java` (Tuner X generated) |
| Other CAN IDs | `Constants.CanIds` |
| Robot-wide setpoints / timeouts / alignment thresholds | `Constants.Setpoints` / `.Waits` / `.Thresholds` — never inline |
| Mechanism position / velocity tolerances | `[Subsystem]Constants.java` |
| PID gains (mechanisms) | `[Subsystem]Constants.java` or IO implementation |
| Trajectory-following gains | `Drive.java` (`trajectoryX/Y/HeadingController`) |
| Drive-to-pose gains / FF radii | `DriveCommands.java` (`driveController`, `DRIVE_TO_POSE_FF_*_RADIUS_METERS`) |
| Heading-hold gains | `DriveCommands.java` (`angleController` (heading kP/kD)) |
| Vision thresholds | `subsystems/vision/VisionConstants.java` |
| Field dimensions / AprilTag layout | `frc/lib/util/FieldConstants.java` |
| Game geometry, scoring targets, zones | `RobotState.java` + `FieldConstants` |
| Button objects | `Triggers.java` — never in `RobotContainer` |
| Subsystem wiring | `RobotContainer.java` |
| Multi-subsystem sequences | a dedicated sequences file |
| Choreo event-marker bindings | `RobotContainer.java` (`ChoreoAutos.bind`, before any routine is built) |
| Auto routines | `robot/autos/ChoreoAutos.java`, `robot/autos/PointToPointAutos.java` |
