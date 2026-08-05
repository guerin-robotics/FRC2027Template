# Change Classification Guide

Every change to this codebase falls into one of five risk levels.
Classify before acting. When uncertain, go one level higher.

> **Template note:** the file names in the examples below are the ones that exist
> today. As 2027 mechanisms are added, extend the examples rather than replacing
> them — the five levels themselves are game-independent.

---

## Level 0 — Safe (Proceed)

No physical robot behavior changes. No review required.

Examples:
- Adding a `Logger.recordOutput()` call
- Adding `.withName()` to a command
- Fixing a compile error
- Updating a comment
- Formatting / import cleanup (via `spotlessApply`)
- Adding a new constant that isn't yet used
- Adding a `@SuppressWarnings` removal (fix the warning instead)

---

## Level 1 — Low Risk (Proceed, note what was added)

New code that doesn't modify existing behavior.

Examples:
- New subsystem files (IO interface, real, sim, subsystem class)
- New command factory method in an existing commands file
- New named command registration in `RobotContainer`
- New auto path registration
- Adding a new field to an `@AutoLog` inputs class

**Required:** State what was added in one sentence.

---

## Level 2 — Medium Risk (Proceed, describe behavioral change)

Modifying existing behavior in a bounded, predictable way.

Examples:
- Changing a command's internal logic (not its trigger conditions)
- Modifying a wait sequence (adding/removing a phase)
- Adding a new button binding
- Changing demo or tuning mode behavior
- Adding a new `LoggedTrigger`
- Changing a constant that affects non-critical behavior (idle speeds, etc.)

**Required:** "After this change, the robot will [X] instead of [Y] when [condition]."

---

## Level 3 — High Risk (Ask for confirmation, name failure mode)

Changes that affect motion, safety interlocks, or sensor fusion.
Do not proceed until the user acknowledges the risk.

Examples:
- Changing PID or feedforward gains
- Changing vision filter thresholds
- Modifying alignment tolerances or zone boundaries in `RobotState`
- Changing timeout values in a scoring or handoff sequence
- Modifying a composite "safe to act" trigger
- Changing the composition of a multi-subsystem sequence
- Any change to `Drive.java` or `Vision.java`
- Modifying `PhoenixOdometryThread`
- Changing auto configuration in `AutoBuilder.configure()`
- Changing wait/timeout constants

**Required before proceeding:**
```
"This changes [X] from [old] to [new].
Behavioral consequence: [what robot does differently].
Failure mode if wrong: [what goes wrong]."
```
Then wait for acknowledgment.

---

## Level 4 — Blocked (Hard Stop — explicit confirmation required)

Changes that can cause incorrect physical motion, destroy calibration data,
or break the logging system. Must not proceed without the user explicitly
saying "confirm" or equivalent.

Examples:
- Any CAN ID change
- Motor inversion flag changes
- Swerve encoder offset changes
- Current limit changes (especially raises)
- Removing `Logger.processInputs()` from any `periodic()`
- Removing a safety timeout from a `waitUntil()` chain
- Removing `AllianceFlipUtil` usage and replacing with `DriverStation.getAlliance()`
- Directly modifying `.auto` PathPlanner files
- Removing `BatteryLogger.reportCurrentUsage()` calls
- Changes to `PhoenixUtil.tryUntilOk()` retry logic

---

## Classification Flowchart

```
Does the change affect physical robot motion?
  No → Level 0 or 1
  Yes →
    Is it a new mechanism / new code path that doesn't modify existing?
      Yes → Level 1
      No →
        Does it affect safety interlocks, gains, or sensor fusion?
          No → Level 2
          Yes →
            Is it a Hard Stop item (CAN IDs, inversions, processInputs)?
              Yes → Level 4 (Blocked)
              No → Level 3 (Ask first)
```

---

## Competition vs. Off-Season Standard

**During competition (robot at event):**
- All Level 3 changes require driver/lead engineer sign-off
- No Level 4 changes without head mentor present
- All changes must pass `./gradlew compileJava` before deployment
- Keep a list of every change made between matches

**Off-season / development:**
- Level 3 changes: confirm with one other team member
- Level 4 changes: still require explicit "confirm" before proceeding
- Write a commit message that explains why, not just what
