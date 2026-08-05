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
3. Check any tuning/demo mode flags are `false` (unless intentional)
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
- [ ] Tuning and demo flags `false`
- [ ] USB drive present — no USB means no match log, and the first drive session is exactly
      when you want one
- [ ] Working tree is clean and committed. `BuildConstants.GIT_DIRTY` should read
      "All changes committed" so the log can be matched back to source
- [ ] Someone on the driver station with a hand on disable, robot on blocks for the first
      enable

The full event-day version of this lives in
[docs/pre-match-checklist.md](../../docs/pre-match-checklist.md).

---

## Build Flags and JVM Options

The robot's `build.gradle` sets:

| Flag | What it actually does |
|---|---|
| `-Xmx100M` / `-Xms100M` | Fixes the heap at 100 MB. Equal min and max means the JVM never resizes it mid-match |
| `-XX:+AlwaysPreTouch` | Commits every heap page at startup, so the first touch of a page during a match is not a page fault |
| `-XX:+UseSerialGC` | Single-threaded stop-the-world collector |
| `-XX:GCTimeRatio=5` | **No effect under SerialGC** — adaptive-sizing input for ParallelGC |
| `-XX:MaxGCPauseMillis=50` | **No effect under SerialGC** — pause goal for ParallelGC's adaptive sizing and G1's pause predictor |

**There is no pause-time guarantee.** SerialGC has no pause-goal mechanism; it collects
when it needs to and takes as long as it takes. The last two flags above are inherited
from the WPILib template and are inert with the collector we select — they are left in
place only because removing them is a build change with no upside, not because they do
anything. An earlier version of this document claimed a "50 ms max pause target," which
was never true.

SerialGC is still the right choice here. On a 100 MB heap the concurrent collectors cost
more in overhead and footprint than they save, and a single-threaded collector on a
two-core RIO leaves the other core alone.

**Pause behaviour comes from not allocating, not from a flag.** The lever that actually
works is keeping garbage out of the 20 ms loop: build control requests and collections
once and reuse them, do not allocate per-loop inside `periodic()`, and watch
`LoopTiming/` and `BatteryLogger/LoopSeconds` for the pauses you do get.

Do not change JVM flags without understanding the tradeoff.
