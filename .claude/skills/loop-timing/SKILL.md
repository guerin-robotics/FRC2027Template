---
name: loop-timing
description: Find what is making the robot loop run slower than 50 Hz — read LoopTiming and BatteryLogger/LoopSeconds from a log or sim, then audit periodic() paths for per-loop allocation and expensive @AutoLogOutput methods. Use when the loop-overrun alert fires, when LoopTiming/OverrunFraction is nonzero, or before the first drive session of the season. Read-only diagnosis; fixes are proposed separately.
---

# Loop Timing

The 2026 robot ran closer to 30 Hz than 50 Hz, with Drive and Vision dominating `robotPeriodic`.
This skill finds the cause. It is **read-only**: report findings and propose fixes; make each fix as
its own reviewed change.

## Step 1 — Measure

Source, in order of preference: a real match or practice log, then a sim run
(`./gradlew simulateJava`) with AdvantageScope on NT4.

Read these (`LoopTimeMonitor`):

| Channel | Meaning |
|---|---|
| `LoopTiming/CurrentMs`, `AverageMs`, `PeakMs` | Loop duration |
| `LoopTiming/BudgetMs` | The budget (50 Hz = 20 ms) |
| `LoopTiming/OverrunCount`, `OverrunFraction` | How often it ran over |
| `BatteryLogger/LoopSeconds` | Independent measurement of the same loop |
| `LoggedTracer/<Section>MS` | Where the loop went: milliseconds since the previous record, so read adjacent channels. `Drive`, `Vision`, `Scheduler`, `Logging`, `Diagnostics` ship; a mechanism adds `LoggedTracer.record("Name")` at the end of its `periodic()` |

Classify the pattern before looking at code:

| Pattern | Likely cause |
|---|---|
| Average near budget, peak not much higher | Too much steady work per loop |
| Average fine, rare huge peaks | GC pause (allocation) or a one-off blocking call |
| Overruns only at enable, or only in auto | Command construction in the loop, auto loading paths |
| Overruns only with vision targets in view | Vision processing per frame |
| Sim fine, real slow | CAN reads blocking, or Phoenix signals not batched |

**Sim loop time is not robot loop time.** A desktop is far faster than a roboRIO. Use sim only to
find *relative* hot spots and allocation; judge the budget from a real log.

## Step 2 — Audit the usual suspects

Grep and read, in this order. Each is a known cost, from `.claude/rules/`:

1. **`@AutoLogOutput` methods** — AdvantageKit calls every one each loop whether or not anything
   reads it. List them (`rg "@AutoLogOutput"`), find any that are expensive or have no caller, and
   report them. Do not delete in this skill.
2. **Allocation inside `periodic()` / `updateInputs()`** — `new` objects, streams, lambdas
   capturing state, `List`/`ArrayList` creation, string concatenation for log keys, `.of(...)` unit
   measures in hot paths. Build control requests, collections and keys once and reuse them.
3. **Phoenix reads not batched** — `motor.getX()` inside `updateInputs()` allocates a signal every
   loop. Signals must be cached in the constructor and refreshed with one
   `BaseStatusSignal.refreshAll(...)`.
4. **`Drive` and `Vision`** — the two that dominated in 2026. Per-camera loops that allocate
   `Pose3d` arrays every frame are the first thing to check in `Vision.periodic()`.
5. **`DriverStation.getAlliance()` in a hot path** — must use `AllianceFlipUtil.shouldFlip()`.
6. **Logging volume** — large arrays or structs logged every loop.

For each finding report: file and line, why it costs, how often it runs, and the proposed change.
Rank by expected saving.

## Step 3 — Prove it

A fix is not done until the number moves. After a proposed change is applied (separately), re-run
and compare `AverageMs`, `PeakMs` and `OverrunFraction` against Step 1. State the before and after.
If nothing moved, the diagnosis was wrong — say so and go back to Step 1.

## Hard limits

- Do not change JVM flags (`-Xmx`, `-XX:+UseSerialGC`, ...) — see `.claude/rules/04-build.md`.
  Pause behavior comes from not allocating, not from a flag.
- Do not remove `Logger.processInputs()` or `BatteryLogger.reportCurrentUsage()` to save time.
  Removing either is a Hard Stop.
- Do not lower log rates on safety-relevant channels without asking.

## Related

- `docs/skills.md` — how to run this skill in sim and on the robot
- `.claude/rules/04-build.md` — "Pause behavior comes from not allocating"
