# Review Checklist

Run through this before submitting or merging any change.
Faster checks first.

---

## 1. Build (required for every change)

```bash
./gradlew compileJava          # Must pass
./gradlew spotlessCheck         # Must pass (or run spotlessApply first)
```

For changes to `@AutoLog` inputs classes:
```bash
./gradlew clean generateSources # Regenerates AutoLogged classes
./gradlew compileJava
```

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

- [ ] No CAN IDs changed
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

## 7. PathPlanner Check (only for auto changes)

- [ ] Named commands registered before `AutoBuilder.buildAutoChooser()`
- [ ] Event triggers have no subsystem requirements
- [ ] No `.auto` files edited in code (use PathPlanner GUI)
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
| Path-following gains | `Drive.java` (`AutoBuilder.configure`) |
| Heading-hold gains | `DriveCommands.java` (`ANGLE_KP` / `ANGLE_KD`) |
| Vision thresholds | `subsystems/vision/VisionConstants.java` |
| Field dimensions / AprilTag layout | `frc/lib/FieldConstants.java` |
| Game geometry, scoring targets, zones | `RobotState.java` + `FieldConstants` |
| Button objects | `Triggers.java` — never in `RobotContainer` |
| Subsystem wiring | `RobotContainer.java` |
| Multi-subsystem sequences | a dedicated sequences file |
| Named commands | `RobotContainer.java` (registered before `buildAutoChooser`) |
