# Commands V3 — What Our Rules Become

> **Status: implemented.** The robot code is Commands V3 (`org.wpilib.command3`) as of the 2027
> port, checked against allwpilib **`2027.0.0-alpha-8`** (read from the `commandsv3-java` sources
> jar, not from memory). `.claude/rules/03-commands.md` was rewritten from this document in the
> same change and is now the rule to follow; this file is the design record — why each rule became
> what it did — and the list of alpha-8 differences from the alpha-7 text it was first written
> against. Re-diff the signatures at beta.
>
> **Nothing here has run on Systemcore hardware.** It has been built and unit-tested on a desktop,
> and the V3 scheduler has run against `Drive` and a BLine `FollowPath` in `RobotContainerSmokeTest`.

## What alpha-8 changed from the alpha-7 text

- **`whenExited(...)` exists on the builder.** It runs on any exit — completion, cancel or
  interruption — so `finallyDo` now has a direct equivalent. Alpha-7 had only `whenCanceled`, and
  this doc's earlier advice to "clean up at the bottom of the body as well" is obsolete. Use
  `whenExited`; `whenCanceled` remains for the cancel-only case.
- **`Command` is an interface** with `name()`, `requirements()`, `run(Coroutine)`, `priority()`,
  `onCancel()` and `onExit()`. A command is read with `command.name()`, not `getName()`.
- **`Scheduler.addPeriodic(Runnable)` exists**, but the repo does not use it: see
  [processInputs](#loggerprocessinputs--the-hard-stop-that-v3-makes-easy-to-break).
- **`Coroutine.waitUntil(cond, Time)` returns a `WaitResult`** with `timedOut()`, as described.
- **`SysIdRoutine.Config` no longer accepts `null`** (use `Config.DEFAULT_*`), `SysIdRoutine.Mechanism`
  is `SysIdMechanism`, and `Direction` is `FORWARD` / `REVERSE`.
- **Trigger, controller and Gamepad types** moved to `org.wpilib.command3.Trigger` and
  `org.wpilib.command3.button.*`. The Xbox Start button is `menu()` and Back is `view()`;
  `CommandJoystick` has no `button(int)` — go through `getHID().button(int)`.
- **Scheduler timestamps are nanoseconds** (`SchedulerEvent.timestampNanos`).

---

## The five things that change

1. **Commands are functions, not lifecycles.** A command body is one `Consumer<Coroutine>`: do
   setup, loop with `coroutine.yield()`, clean up. No `initialize` / `execute` / `isFinished` /
   `end` split. Any loop that forgets to yield **freezes the whole robot program** — there is no
   preemption.
2. **Names are required by the type system.** Every builder ends in `.named(...)` (or
   `withAutomaticName()` for groups), and a builder cannot produce a `Command` without one. Our
   "every command gets `.withName()`" rule becomes a compile error instead of a review item.
3. **`Mechanism` replaces `Subsystem`.** `org.wpilib.command3.Mechanism` is an interface with
   default methods — `run`, `runRepeatedly`, `idle`, `setDefaultCommand`. **It has no
   `periodic()`**, and nothing calls one for you.
4. **Priorities replace interrupt behavior.** A scheduled command interrupts a running one that
   shares a mechanism only if its priority is **equal or higher**. Default commands should sit
   below `Command.DEFAULT_PRIORITY`.
5. **Child commands are "proxied" by default.** A command body that `await`s or `fork`s another
   command does not own that command's mechanisms except while the child runs. The built-in
   `Command.sequence(...)` / `Command.parallel(...)` groups keep V2 behavior and own everything for
   their full duration.

---

## Rule by rule

### Static command factories — keep

The rule survives unchanged in spirit: commands are built by static methods in `*Commands` classes,
never by `implements Command` classes, never by command methods on the subsystem.

V3 makes `mechanism.run(...)` the builder's entry point, which will tempt people to put factories on
the mechanism. Keep them in `*Commands`. The reasons were never about the API — one file per verb
set, testable without a subsystem subclass, and every mechanism's verbs in one place to review.

```java
// V2 — today
public static Command runAtVelocity(Intake roller, AngularVelocity velocity) {
  return Commands.startEnd(() -> roller.setVelocity(velocity), roller::stop, roller)
      .withName(roller.getName() + "_Velocity");
}

// V3 — the same factory
public static Command runAtVelocity(Intake roller, AngularVelocity velocity) {
  return roller
      .run(coroutine -> {
        roller.setVelocity(velocity);
        coroutine.park(); // hold until canceled
      })
      .whenExited(roller::stop) // park() never returns, so cancel is the only way out
      .named(roller.getName() + "_Velocity");
}
```

**Cleanup hooks.** `whenCanceled(...)` runs **only** when the command is canceled — not on natural
completion. `whenExited(...)` (present in alpha-8) runs on any exit. A body that parks forever (like
the one above) can only exit by cancel, so either works there; use `whenExited` everywhere so a body
that later gains an exit path is still cleaned up.

### The factory table

| V2 today | V3 |
|---|---|
| `Commands.runOnce(action, sub)` | `mech.run(co -> action.run()).named(...)` |
| `Commands.run(action, sub)` | `mech.runRepeatedly(action).named(...)` |
| `Commands.startEnd(start, end, sub)` | `mech.run(co -> { start.run(); co.park(); }).whenExited(end).named(...)` — `park()` only exits by cancel |
| `cmd.finallyDo(end)` | `.whenExited(end)`, which runs on completion, cancel or interrupt |
| `Commands.sequence(a, b, c)` | `Command.sequence(a, b, c).named(...)`, or `a.andThen(b).andThen(c).named(...)` |
| `Commands.parallel(a, b)` | `Command.parallel(a, b).named(...)` — ends when **all** finish |
| `Commands.race(a, b)` | `Command.race(a, b).named(...)` — ends when **any** finishes |
| `Commands.deadline(a, b)` | `new ParallelGroupBuilder().requiring(a).optional(b).named(...)` — `b` is canceled when `a` ends |
| `cmd.until(cond)` | `cmd.until(cond).named(...)` |
| `cmd.withTimeout(seconds)` | `cmd.withTimeout(Seconds.of(...))` — takes a `Time`, and names itself `"<name> [<t> timeout]"` |
| `Commands.waitSeconds(s)` | `Command.waitFor(Seconds.of(s)).named(...)`, or `co.wait(...)` inside a body |
| `Commands.none()` | none needed — an empty body, `Command.noRequirements(co -> {}).named(...)` |
| `.ignoringDisable(true)` | See [Disabled](#disabled) |
| `new ScheduleCommand(c)` / `ProxyCommand` | No equivalent needed — `co.fork(c)` or `co.await(c)`. Children cannot outlive their parent |

### Timeouts on every wait — keep, and it gets sharper

This is a competition safety rule and it survives exactly. What changes is where the untimed form
hides.

V3 has **two** untimed waits, and both are forbidden:

```java
Command.waitUntil(mechanism::isReady).named("Wait")  // WRONG — no timeout, blocks forever
coroutine.waitUntil(mechanism::isReady);             // WRONG — same, inside a command body
```

and two timed ones:

```java
Command.waitUntil(mechanism::isReady).named("Wait").withTimeout(Seconds.of(Waits.READY))
var result = coroutine.waitUntil(mechanism::isReady, Seconds.of(Waits.READY));
if (result.timedOut()) { /* log it, then decide: act anyway, or give up */ }
```

`coroutine.waitUntil(condition, timeout)` is new in alpha-7 and is the better form: it **returns
whether it timed out**, so the command can log a timeout as the distinct event it is rather than
proceeding silently. Our V2 code cannot tell "ready" from "gave up"; V3 code should always record it.

The review-checklist item becomes: *every `waitUntil` has a timeout argument or a `.withTimeout`,
and every `WaitResult` is checked.*

### Ready → Align → Act — keep the budget, write it straight

The budget arithmetic is unchanged: phase 2 gets the total minus phase 1's budget, so the robot
always acts within the total. The coroutine form reads top to bottom and can log which phase ran
out:

```java
public static Command score(Shooter shooter, Feeder feeder, BooleanSupplier isAligned) {
  return Command.noRequirements(coroutine -> {
        coroutine.fork(ShooterCommands.spinUp(shooter, Constants.Setpoints.SHOT_VELOCITY));

        var ready = coroutine.waitUntil(
            shooter::isAtVelocity, Seconds.of(Waits.MECHANISM_READY_SECONDS));
        var aligned = coroutine.waitUntil(
            isAligned, Seconds.of(Waits.TOTAL_TIMEOUT_SECONDS - Waits.MECHANISM_READY_SECONDS));
        Logger.recordOutput("Score/ReadyTimedOut", ready.timedOut());
        Logger.recordOutput("Score/AlignTimedOut", aligned.timedOut());

        coroutine.await(FeederCommands.feed(feeder, Constants.Setpoints.FEED_VOLTAGE));
      })
      .named("Score");
}
```

Two V3 behaviors to know before copying that:

- **A forked command does not outlive its parent.** Cancel `Score` and the spin-up is canceled too.
  That is what we want here, and it is different from V2's `ScheduleCommand`, which let the child
  run on.
- **`noRequirements` + `fork`/`await` means the parent owns nothing between children.** A default
  command can take a mechanism back in the gap. If a mechanism must stay commanded across phases,
  either make the parent require it (`Command.requiring(shooter, feeder).executing(...)`) or use
  `Command.sequence(...)`, which owns everything for its whole duration. **Verify this against the
  scheduler at the port** — whether the gap is zero cycles or one decides which form is safe.

### Where command values live — unchanged

Setpoints are factory parameters, supplied from `Constants.Setpoints`; timeouts from
`Constants.Waits`. V3 takes `Time` where V2 took `double` seconds, so `Waits` can become `Time`
constants at the port. Keep the rule: no bare numbers at the binding site.

### Naming — enforced, keep the convention

`"Subsystem_ActionVerb_OptionalParam"` stays. The compiler now guarantees a name exists; review
still has to check it is a *useful* one. `withAutomaticName()` on groups produces
`"A -> B -> C"` / `"A | B"`, which is acceptable for a throwaway composition and poor for anything
bound to a button or to a BLine event trigger. Name those.

### Default commands — keep, add a priority

Still safe idle states, still must not end, still set in `RobotContainer`. Two additions:

- Give them a priority **below** `Command.DEFAULT_PRIORITY`. The scheduler treats the default
  command's priority as the floor for that mechanism, so a default at `DEFAULT_PRIORITY` blocks
  every ordinary command.
- A mechanism with no default command is **not** unowned — V3 gives it `mechanism.idle()`, which
  owns it and does nothing. For a motor mechanism that means the last control request keeps
  running. Every motor mechanism still needs an explicit stop/hold default.

Drive's default stays `DriveCommands.joystickDrive`.

V3 also scopes default commands: one set inside a command or an OpMode reverts when that scope
exits. Keep setting ours in `RobotContainer` (global scope) unless there is a reason not to.

### `Logger.processInputs()` — the hard stop that V3 makes easy to break

V2's `CommandScheduler` calls every registered `Subsystem.periodic()`. **V3 calls nothing** —
`Mechanism` has no `periodic()`. Port `Drive`, `Vision` and every mechanism subsystem
naively and `processInputs()` simply never runs: no inputs, no replay, no error.

**Decision (made at the port):** every subsystem's `periodic()` is called explicitly from
**`RobotContainer.periodic()`**, which `Robot.robotPeriodic()` calls **before**
`Scheduler.getDefault().run()`, in a fixed order (`Drive`, then `Vision`). This includes `Vision`,
which requires nothing and is not a mechanism at all. It was chosen over
`Scheduler.addPeriodic(sub::periodic)` because it is one visible list, it is testable without a
scheduler, and registering periodics at construction would accumulate across the many instances a
test suite builds (the default scheduler has no way to remove one).

`RobotContainerSmokeTest.everySubsystemProcessesItsInputsEveryLoop` proves it. It drives one loop
the way `robotPeriodic` does — AdvantageKit's before-user hook, `RobotContainer.periodic()`, the
after-user hook — captures every logged key with a data receiver, and asserts each subsystem found in
`RobotContainer` (by reflection: anything with a public `periodic()`) logged inputs under
`/<ClassName>/`. Removing `vision.periodic()` makes it fail; that was checked. A new subsystem is
covered automatically.

### Triggers — keep the rules, check the semantics

`Triggers.java`, private controllers, action-named accessors — none of that depends on the framework.
V3 ships `org.wpilib.command3.Trigger` and `org.wpilib.command3.button.CommandXboxController` /
`CommandJoystick` / `CommandGamepad`.

Binding methods, and the ones that differ:

| Method | V3 behavior |
|---|---|
| `onTrue`, `onFalse` | Same as V2 |
| `whileTrue`, `whileFalse` | Start on the edge, cancel on the opposite edge. **Not** restarted if the command ends while still held |
| `retryWhileTrue`, `retryWhileFalse` | **New.** Like `whileTrue`, but restarts the command if it ends while still held |
| `ifTrue` | **New.** Schedules on *every poll* while true (no edge) |
| `toggleOnTrue`, `toggleOnFalse` | Same as V2 |
| `debounce(Time)` | Takes a `Time`, not `double` seconds |

**Trigger scoping is new.** A binding created inside a running command or OpMode is removed when
that scope ends. Bindings made in `RobotContainer` are global and behave like V2. Scoped bindings
are useful — an auto that only wants `atScoringPosition.onTrue(score())` while it runs — but they
are a second place a binding can live. If we use them, they go in the command-composition layer,
never scattered through subsystems.

### Cancellation flags — mostly replaced

The 2026 `compressCancelled` / `xCancelled` / `doubleCompress` flags existed because a V2 command
could not easily be told "the driver overrode you". Under V3, most of those become either
**priorities** (the override runs at a higher priority and the automatic behavior cannot interrupt
it back) or **a scoped trigger** inside the automatic command. Reach for those before adding a flag.
If a flag is still needed, the V2 rule stands: write down when it is set and when it is cleared.

### Disabled

V2's `.ignoringDisable(true)` is per command. V3 uses each mechanism's
`controllableDuringDisabled()` permission. The gyro-reset binding dispatches on the button
edge. While disabled it resets only the odometry heading. While enabled it awaits a command
that requires `Drive`, preserving normal interruption and priority rules.

---

## Our `frc/lib` classes under V3

| Class | What happened |
|---|---|
| `ContinuousConditionalCommand` | **Deleted** (it had no callers). V3's body that loops, re-checks and forks the matching command, or a `StateMachine` with a transition each way, replaces it |
| `LoggedTrigger` | **Ported.** Same wrapping-supplier approach; the `debounce` overrides take a `Time`. Scoped bindings do not affect it |
| `CommandLogger` | **Rewritten on `Scheduler.addEventListener`.** It counts `Scheduled` against `Completed` / `CompletedWithError` / `Canceled`, and logs `Interrupted(command, interrupter)` as `Commands/LastInterrupt` — the "interrupted by what?" answer V2 could not give. `recordCurrentCommand` uses `mechanism.getRunningCommands()`. The V3 `Scheduler` is also protobuf-serializable; logging that beside ours is open |
| Mechanism subsystems and their `*Commands` (the `template/` scaffolds) | **Ported:** the subsystem `implements Mechanism`, its `periodic()` is public and called from `RobotContainer.periodic()`, and each `*Commands` class uses the factory table. Spin-up waits are `coroutine.waitUntil(..., Time)` and log whether they timed out. The scaffolds still fail to compile on exactly the values they owe (gear ratio, CAN IDs, soft limits, encoder split, drum geometry) and nothing else |
| `DriveCommands` | **Ported** verb by verb. `driveToPose` seeds its profiles at the top of the body; the math is unchanged. The two characterization commands are a `Command.sequence` and a `Command.parallel` of named parts |

---

## Tests under V3

- **Scheduler isolation.** `Scheduler.getDefault()` is JVM-wide. The smoke test cancels everything
  it started in `@AfterAll`. `Scheduler.createIndependentScheduler()` could end that, but a
  `Mechanism` reports its scheduler through `getRegisteredScheduler()`, which defaults to the default
  scheduler, so each mechanism would need to accept a scheduler at construction. Not done yet.
- **Time.** AdvantageKit sets `RobotController.setTimeSource(Logger::getTimestamp)`, and in 2027
  that source, `Timer`, and the scheduler's events are **nanoseconds**. `Timer.getTimestamp()` is the
  seconds-valued clock; the repo uses it wherever 2026 used `Timer.getFPGATimestamp()`.
- **Disabled.** V3 will not schedule a command that requires `Drive` while disabled, so the test that
  runs a BLine `FollowPath` enables the sim Driver Station for that one cycle.
- **`RobotContainerSmokeTest`** was rewritten for the V3 `Scheduler`: default commands (present, and
  below `DEFAULT_PRIORITY`), every OpMode routine builds a command, the BLine logging reaches the
  preview widget, every BLine event key is bound, and the processInputs gate above.

---

## Things V3 gives us that we should use

- **`coroutine.waitUntil(..., timeout)` returning `timedOut()`** — log every timeout as an event.
- **`Interrupted(command, interrupter)` events** — the 2026 "why did that stop?" question, answered
  in the log.
- **Priorities** — driver overrides that the automatic behavior cannot steal back, without a flag.
- **`StateMachine`** — the explicit readiness state machine that `template/GUIDE.md` § D.4 says
  2026 lacked. Consider it for the 2027 scoring flow if it is as complex as 2026's.
- **`Scheduler.createIndependentScheduler()`** — test isolation, once mechanisms can be handed a
  scheduler.

---

## Sources

- `org.wpilib.command3` source: `Command`, `Coroutine`, `Mechanism`, `Scheduler`, `Trigger`,
  `StateMachine`, `ParallelGroupBuilder`, `SchedulerEvent` —
  [allwpilib/commandsv3](https://github.com/wpilibsuite/allwpilib/tree/main/commandsv3)
- [Commands V3 design doc](https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/commands-v3.md)
- [Commands V3 state machines](https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/commands-v3-state-machines.md)
- [alpha-7 release notes](https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-7)
