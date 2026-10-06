# Commands V3 — What Our Rules Become

> **Status: doctrine, not code.** The robot code is Commands V2 and stays V2 until the port in
> [2027-migration.md](2027-migration.md). `.claude/rules/03-commands.md` describes the code that
> exists and is still the rule to follow today.
>
> This document is the target for that port: for every command rule in this repo, what it becomes
> under V3, written before anyone has to decide it under a deadline. When the port happens,
> `03-commands.md` gets rewritten from this, and this file is deleted or kept as the V2 → V3
> history.
>
> *Checked against allwpilib `v2027.0.0-alpha-7` and `main` of 2026-09-27, package
> `org.wpilib.command3`. The package name has held since alpha-5; the API inside it has not
> (`Mechanism` became an interface in alpha-7). Re-diff the signatures below at beta.*

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
set, testable without a subsystem subclass, and the library's `RollerCommands` / `RotaryCommands` /
`LinearCommands` cover every mechanism of a kind.

```java
// V2 — today
public static Command runAtVelocity(RollerSubsystem roller, AngularVelocity velocity) {
  return Commands.startEnd(() -> roller.setVelocity(velocity), roller::stop, roller)
      .withName(roller.getName() + "_Velocity");
}

// V3 — the same factory
public static Command runAtVelocity(RollerSubsystem roller, AngularVelocity velocity) {
  return roller
      .run(coroutine -> {
        roller.setVelocity(velocity);
        coroutine.park(); // hold until canceled
      })
      .whenCanceled(roller::stop) // park() never returns, so cancel is the only way out
      .named(roller.getName() + "_Velocity");
}
```

**Cleanup hooks differ between alpha-7 and `main`.** Alpha-7's builder has only
`whenCanceled(...)`, which runs **only** when the command is canceled — not on natural completion.
`whenExited(...)`, which runs on any exit, was added on `main` after alpha-7. A body that parks
forever (like the one above) can only exit by cancel, so `whenCanceled` is enough. A body that can
finish on its own must also clean up at its end, or use `whenExited` once a release ships it.

### The factory table

| V2 today | V3 |
|---|---|
| `Commands.runOnce(action, sub)` | `mech.run(co -> action.run()).named(...)` |
| `Commands.run(action, sub)` | `mech.runRepeatedly(action).named(...)` |
| `Commands.startEnd(start, end, sub)` | `mech.run(co -> { start.run(); co.park(); }).whenCanceled(end).named(...)` — `park()` only exits by cancel |
| `cmd.finallyDo(end)` | alpha-7: `.whenCanceled(end)` **and** `end` at the bottom of the body — `whenCanceled` does not run on natural completion. `main` (post-alpha-7): `.whenExited(end)`, which runs on completion, cancel or interrupt |
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
bound to a button or to a Choreo event marker. Name those.

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
`Mechanism` has no `periodic()`. Port `Drive`, `Vision` and the library `*Subsystem` classes
naively and `processInputs()` simply never runs: no inputs, no replay, no error.

At the port, choose one of these and apply it to every subsystem, including `Vision`, which
requires nothing and is not a mechanism at all:

- call each `periodic()` explicitly from `Robot.robotPeriodic()`, **before**
  `Scheduler.getDefault().run()`, in a fixed order; or
- register each once with `Scheduler.getDefault().addPeriodic(sub::periodic)` at construction —
  periodic callbacks run before triggers are polled and before commands run, which is the order we
  need.

Then make `RobotContainerSmokeTest` prove it: construct the container, run one loop, and assert every
subsystem's inputs were processed. This is the one V3 change that can silently disable replay, so
it gets a test, not a checklist line.

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

V2's `.ignoringDisable(true)` is per command. V3 decides per **mechanism**:
`Mechanism.controllableDuringDisabled()` defaults to `false`, and a command requiring a
non-controllable mechanism cannot be scheduled while disabled — and is canceled on disable.

Commands that require **no** mechanism are unaffected. So our re-zero-gyro binding, which today
requires `drive` and sets `ignoringDisable(true)`, becomes a `Command.noRequirements(...)` command:
resetting a pose does not need to own the drivetrain. Do **not** mark `Drive`
`controllableDuringDisabled` to get the same effect — that would let any drive command be scheduled
while disabled.

### Choreo event markers and trajectory triggers — open

The V2 rule — a command bound to an event marker or started from `traj.atTime`/`traj.done` must not
require the drive, or it interrupts the trajectory — exists because the bound command and the
trajectory command are scheduled side by side and conflict on the drive requirement. V3's
mechanism ownership and priorities may change its form.

**This depends on ChoreoLib's 2027 command story.** Its 2027 alpha targets `org.wpilib.command2`
(V2), not V3. Keep the V2 rule until ChoreoLib ships V3 support; then re-derive it from what its V3
trajectory command actually owns.

---

## Our `frc/lib` classes under V3

| Class | V3 verdict |
|---|---|
| `ContinuousConditionalCommand` | **Probably delete.** It exists because V2's `ConditionalCommand` checks its condition once. In V3, a body that loops, re-checks the condition and forks the matching command does the same thing in a few lines — or a `StateMachine` with a transition each way (`a.switchTo(b).when(() -> !cond)`, `b.switchTo(a).when(cond)`). Check that the state machine's fixed initial state does not run the wrong branch for a cycle before relying on it |
| `LoggedTrigger` | **Port.** V3 `Trigger` is still a subclassable `BooleanSupplier` with a `(BooleanSupplier)` constructor, so the wrapping-supplier approach carries over. The `debounce` overrides change to take `Time`. Scoped bindings do not affect it — logging is in the supplier |
| `CommandLogger` | **Rewrite on `Scheduler.addEventListener`.** V3 emits typed `SchedulerEvent`s — `Scheduled`, `Mounted`, `Completed`, `CompletedWithError`, `Canceled`, `Interrupted(command, interrupter)`, `ForkFailure`. `Interrupted` carries **who** interrupted, which is exactly the "scheduled and interrupted — by what?" question this class was written to answer. `recordCurrentCommand` becomes `mechanism.getRunningCommands()`. V3's `Scheduler` is also protobuf-serializable (the command tree, with per-command runtimes); decide whether to log that instead of, or beside, ours |
| `RollerCommands` / `RotaryCommands` / `LinearCommands` | **Port verb by verb** using the factory table above. `spinUpTo` becomes a body with `coroutine.waitUntil(roller::isAtVelocity, timeout)` — and can now log whether it timed out |
| `RollerSubsystem` / `RotarySubsystem` / `LinearSubsystem` | **Implement `org.wpilib.command3.Mechanism`** instead of extending `SubsystemBase`. Wire `periodic()` per the processInputs section. `setName` goes away — override `getName()` |
| `SteppableCommandGroup` | **Rewrite as a body.** V3 makes it a loop: run step *i* as a child, `co.waitUntil` a rising edge on forward/back, cancel and move. No custom `Command` subclass is needed, and the edge detection stays in the body |
| `DriveCommands.driveToPoseWithin` / `driveToWaypoint` | **Port** with the factory table: `driveToPose` becomes a body that runs until the arrival check passes, under a `withTimeout(Time)`. The seeding and setpoint feedforward are plain math and carry over unchanged |
| `frc.lib.mechanism.Mechanism` | **Rename.** Collides with `org.wpilib.command3.Mechanism`, which the `*Subsystem` classes will implement. `MotorMechanism` says what it is — the motor-backed core the three kinds extend — and keeps `RollerMechanism` / `RotaryMechanism` / `LinearMechanism` reading naturally. Decide once, before the V3 branch starts |

---

## Tests under V3

- **Scheduler isolation.** Today every sim test cancels and unregisters from the JVM-wide
  `CommandScheduler`. V3 offers `Scheduler.createIndependentScheduler()`, which can end the
  teardown ritual — but a `Mechanism` reports its scheduler through `getRegisteredScheduler()`,
  which defaults to `Scheduler.getDefault()`. For a test scheduler to own a mechanism, the library
  `*Subsystem` classes need to accept the scheduler at construction and return it there. Design
  that in during the port rather than retrofitting it.
- **Time.** V3's `coroutine.wait` / `waitUntil(…, timeout)` use WPILib's `Timer`, which reads
  `RobotController.getTime()` — the same clock V2 used. So the `RobotController.setTimeSource`
  workaround in `docs/testing.md` should carry over; confirm it on the first V3 sim test. Note
  alpha-7's raw timestamps are nanoseconds.
- **`RobotContainerSmokeTest`** needs rewriting: default-command and naming checks move to the V3
  `Scheduler` API, and it gains the processInputs assertion above.

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
