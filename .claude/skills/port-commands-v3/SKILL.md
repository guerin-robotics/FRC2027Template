---
name: port-commands-v3
description: Port this repo's Commands V2 code to WPILib 2027 Commands V3, following docs/commands-v3.md. Use only when the team has decided to do the port, after the 2027 platform bump works on V2 (the order in docs/2027-migration.md). Works one subsystem or one commands class at a time and treats the processInputs wiring as the gate.
---

# Port Commands to V3

**Do not start this unprompted.** The code is V2 on purpose. `docs/2027-migration.md` orders the work:
port the **platform** on V2 first, V3 second. If the platform bump has not built and run on V2, stop
and say so.

`docs/commands-v3.md` is the specification — it was written before the port so nobody decides under
a deadline. Read it fully. It was checked against allwpilib `v2027.0.0-alpha-7` and `main` of
2026-09-27; **re-diff the signatures against the current release before relying on any of them**.
Report any difference rather than coding around it. Do not mix V3 idioms into V2 code.

## Rules for this skill

- **One logical change per response.** Port one mechanism or one `*Commands` class at a time,
  scaffolds first, so every later mechanism is copied from V3 code. A multi-subsystem port needs
  the plan confirmed first.
- Drive, vision and BLine-related commands are High risk. State the behavior and the failure mode
  for each before changing it.
- A port must preserve behavior. No opportunistic refactors.
- Rewrite `.claude/rules/03-commands.md` from `docs/commands-v3.md` **in the same commit** as the
  port it describes.

## Step 0 — Gate: does `Logger.processInputs()` still run?

V3 has **no `Subsystem.periodic()`**. Nothing calls a mechanism's `periodic()`, so a naive port
silently stops `Logger.processInputs()`: no inputs, no replay, no error. This is a Hard Stop.

Before porting any mechanism, pick one wiring and apply it to **every** subsystem, `Vision`
included (it requires nothing and is not a mechanism):

- call each `periodic()` from `Robot.robotPeriodic()` before `Scheduler.getDefault().run()`, in a
  fixed order; **or**
- `Scheduler.getDefault().addPeriodic(sub::periodic)` at construction.

Then add the test: construct the container, run one loop, assert every subsystem's inputs were
processed. Do not port further until it passes.

## Step 1 — Port each factory

Use the factory table in `docs/commands-v3.md`. The key mappings:

| V2 | V3 |
|---|---|
| `Commands.startEnd(start, end, sub)` | `mech.run(co -> { start.run(); co.park(); }).whenCanceled(end).named(...)` |
| `Commands.run(action, sub)` | `mech.runRepeatedly(action).named(...)` |
| `Commands.runOnce(action, sub)` | `mech.run(co -> action.run()).named(...)` |
| `.withName(x)` | `.named(x)` — required by the compiler |
| `.withTimeout(double)` | `.withTimeout(Seconds.of(...))` |
| `Commands.waitUntil(c).withTimeout(t)` | `coroutine.waitUntil(c, Seconds.of(t))` — returns `timedOut()` |
| `finallyDo(end)` | alpha-7: `whenCanceled(end)` **plus** `end` at the end of the body; `whenExited` only if the release ships it |

Rules that survive and get sharper:

- **No untimed wait.** Both `Command.waitUntil(cond)` and `coroutine.waitUntil(cond)` without a
  timeout are forbidden. Check every `WaitResult`, and log the timeout as an event.
- Factories stay in `*Commands`, not on the mechanism, even though `mechanism.run(...)` invites it.
- Every command is named; do not leave a bound command on `withAutomaticName()`.
- Naming convention `Subsystem_ActionVerb_Param` stays.
- Any loop that does not yield **freezes the whole robot program** — there is no preemption. Check
  every `while` in a coroutine body.

## Step 2 — Subsystems, defaults, triggers

- Subsystems implement `org.wpilib.command3.Mechanism`; override `getName()` instead of `setName`.
- Default commands: priority **below** `Command.DEFAULT_PRIORITY`, safe idle, never end. A mechanism
  with no default gets `idle()` — a motor mechanism still needs an explicit stop/hold default.
- Triggers: `whileTrue` is not restarted if the command ends while held; use `retryWhileTrue` only
  where V2's behavior must be kept. `debounce` takes a `Time`.
- Gyro reset uses a requirement-free command while disabled. While enabled, dispatch to a
  command that requires Drive so scheduler ownership is preserved. Keep Drive unavailable to
  motion commands while disabled.
- Cancellation flags: reach for priorities or scoped triggers first. If a flag stays, document when
  it is set and cleared.

## Step 3 — `frc/lib`

Per the verdict table in `docs/commands-v3.md`: `ContinuousConditionalCommand` probably deleted,
`LoggedTrigger` ported (`debounce` to `Time`), `CommandLogger` rewritten on
`Scheduler.addEventListener`.

## Step 4 — BLine events: do not port blindly

BLine-Lib's current release targets the V2 scheduler. Keep the V2 rule — a bound event command must
not require the drive — until BLine-Lib ships V3 support, then re-derive it from what its path
command owns. If it has not shipped, leave autos on V2 and say so.

## Step 5 — Verify

```
./gradlew compileJava
./gradlew spotlessCheck
./gradlew build
./gradlew simulateJava
```

Then: `RobotContainerSmokeTest` rewritten for the V3 `Scheduler` (default commands, names, the
processInputs assertion); sim run of each ported command for activation, release, and timeout;
a replay of an old log to confirm inputs still process. Report anything that behaves differently
from V2 — and do not call it done while the processInputs test is missing.

## Tuning and demo behavior under OpModes

2027 OpModes replace the V2 stand-ins. Each `tune*` command becomes a `@Utility` OpMode, and
`Triggers.tuningEnabled` goes away — **keep the held dead-man button inside the OpMode**, and keep
the rule that match code never reads a dashboard tunable. See `docs/tunables.md`.

## Related

- `docs/commands-v3.md`, `docs/2027-migration.md`, `.claude/rules/03-commands.md`
- `docs/skills.md` — how to run this skill in sim and on the robot
