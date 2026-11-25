# Build & Verification Rules

---

## Before Reporting a Task Complete

Run these checks in order. Do not skip.

### 1. Compile Check
```bash
./gradlew compileJava
```
Every change must compile. Do not deliver code with compile errors.

### 2. AdvantageKit Annotation Processor
If you added or modified an `@AutoLog` inputs class, check that the annotation
processor generated the updated `*AutoLogged` class:
```bash
./gradlew generateSources   # or: ./gradlew build -x test
```

### 3. Code Formatting
```bash
./gradlew spotlessCheck
# If formatting issues found:
./gradlew spotlessApply
```
The repo uses Google Java Format via Spotless. Non-formatted code will fail CI.

### 4. Full Build (before any commit that touches hardware or command logic)
```bash
./gradlew build
```

---

## What Each Check Catches

| Check | What it catches |
|---|---|
| `compileJava` | Type errors, missing methods, wrong imports |
| `generateSources` | `@AutoLog` changes not reflected in AutoLogged class |
| `spotlessCheck` | Formatting inconsistencies |
| Full `build` | Everything above + any test failures |

---

## Simulation Verification

For changes to command logic, trigger logic, or RobotState calculations —
verify in simulation before declaring complete:

```bash
./gradlew simulateJava
```

Check in Advantage Scope (connected to sim via NT4):
- Subsystem inputs are logging correctly
- Commands activate and deactivate as expected
- RobotState values update correctly

---

## Things That Don't Replace Build Verification

- "It looks right" — does not catch type errors
- "I only changed a constant" — constants have types; wrong type breaks build
- "The logic is correct" — logic is irrelevant if it doesn't compile
- "I've done this before" — always verify

---

## Annotation Processor Note

AdvantageKit uses annotation processing to generate `*AutoLogged` classes.
If you add a field to an `@AutoLog` class (e.g., `VisionIOInputs`),
the `VisionIOInputsAutoLogged` class is regenerated at build time.

Do not manually edit `*AutoLogged` files — they are overwritten on every build.

If a compile error says `*AutoLogged` is missing a field you just added:
```bash
./gradlew clean generateSources
```

---

## Deploy Verification (physical robot only)

Before declaring a change ready to deploy:

1. Compile locally: `./gradlew compileJava`
2. Check `Constants.currentMode` resolves to `REAL` on the robot — in particular that
   `simMode` was not left on `REPLAY` after a debugging session
3. Check that no tuning or demo behavior is reachable outside enabled Test mode (there are no flags; `tune*` commands sit behind `Triggers.tuningEnabled`, and in 2027 are `@Utility` OpModes)
4. Verify the USB drive is present for log writing

### First deploy of the season — extra gate

The first time this code drives a real robot, before enabling:

- [ ] `./gradlew build` passes (not just `compileJava`)
- [ ] `generated/TunerConstants.java` has been **regenerated with Tuner X** for this robot.
      The values in the template are the 2026 robot's and are wrong for anything else
- [ ] Phoenix Tuner device count matches what the code expects. A missing device shows up
      as a mechanism that silently does nothing
- [ ] CANivore bus utilization is sane; no CAN error frames
- [ ] `Constants.currentMode` resolves to `REAL`
- [ ] No tuning or demo command reachable outside Test mode (there are no flags to check)
- [ ] USB drive present — no USB means no match log, and the first drive session is exactly
      when you want one
- [ ] Working tree is clean and committed. `BuildConstants.GIT_DIRTY` should read
      "All changes committed" so the log can be matched back to source
- [ ] Someone on the driver station with a hand on disable, robot on blocks for the first
      enable

The full event-day version of this lives in
[docs/pre-match-checklist.md](../../docs/pre-match-checklist.md).

---

## Toolchain, Deploy Target and JVM Options

**Toolchain (2027 alpha):** GradleRIO `2027.0.0-alpha-8` (plugin id `org.wpilib.GradleRIO`),
**Gradle 9.4.1**, **Java 25**. The WPILib 2027 install lives in `C:\Users\Public\wpilib\2027_alpha7`
(its folder name predates alpha-8); `settings.gradle` and `.wpilib/wpilib_preferences.json` point
there. Run Gradle with `JAVA_HOME` on a JDK 25 — Gradle 8 cannot run on it.

**Deploy target:** `systemcore` (`getTargetTypeClass('SystemCore')`, `WPILibJavaArtifact`), deploying
`src/main/deploy` to `/home/systemcore/deploy`. The roboRIO is gone; the JNI configurations are
`systemcoreDebug` / `systemcoreRelease`, and the main class is wired with the `application` plugin.

**The JVM flags are not set yet.** 2026 ran `-Xmx100M -Xms100M -XX:+AlwaysPreTouch
-XX:+UseSerialGC` (sized for a RIO 2 with two cores). Those were dropped from `build.gradle` for
Systemcore rather than copied: the machine, its memory and its core count are different. Measure loop
timing (`LoopTiming/`, `BatteryLogger/LoopSeconds`) on real Systemcore hardware, then decide the heap
and collector, and put them back in `deployArtifact` `jvmArgs` with the numbers that justify them.
Until then the defaults apply and there is **no pause-time guarantee**.

The 2026 notes still hold: `GCTimeRatio` and `MaxGCPauseMillis` do nothing under SerialGC (they were
inherited inert from the WPILib template), and an earlier version of this document claimed a "50 ms
max pause target" that was never true.

**Pause behaviour comes from not allocating, not from a flag.** The lever that actually
works is keeping garbage out of the 20 ms loop: build control requests and collections
once and reuse them, do not allocate per-loop inside `periodic()`, and watch
`LoopTiming/` and `BatteryLogger/LoopSeconds` for the pauses you do get.

Do not change JVM flags without understanding the tradeoff.
