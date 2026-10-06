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
`RobotContainer` wiring and `MotorConfig` safety values — see [testing.md](testing.md). Vision
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

## Quick Reference: Where Things Belong

| What | Where |
|---|---|
| Swerve CAN IDs, gains, geometry, current limits | `generated/TunerConstants.java` (Tuner X generated) |
| Other CAN IDs | `Constants.CanIds` |
| Setpoints / timeouts / tolerances | `Constants.Setpoints` / `.Waits` / `.Thresholds` — never inline |
| PID gains (mechanisms) | `[Subsystem]Constants.java` or IO implementation |
| Trajectory-following gains | `Drive.java` (`trajectoryX/Y/HeadingController`) |
| Drive-to-pose gains / FF radii | `DriveCommands.java` (`driveController`, `Drive/ToPose/FF*`) |
| Heading-hold gains | `DriveCommands.java` (`ANGLE_KP` / `ANGLE_KD`) |
| Vision thresholds | `subsystems/vision/VisionConstants.java` |
| Field dimensions / AprilTag layout | `frc/lib/util/FieldConstants.java` |
| Game geometry, scoring targets, zones | `RobotState.java` + `FieldConstants` |
| Button objects | `Triggers.java` — never in `RobotContainer` |
| Subsystem wiring | `RobotContainer.java` |
| Multi-subsystem sequences | a dedicated sequences file |
| Choreo event-marker bindings | `RobotContainer.java` (`ChoreoAutos.bind`, before any routine is built) |
| Auto routines | `robot/autos/ChoreoAutos.java`, `robot/autos/PointToPointAutos.java` |
