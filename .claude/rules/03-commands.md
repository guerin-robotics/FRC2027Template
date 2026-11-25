# Command Rules

> **These rules describe Commands V3** (`org.wpilib.command3`), which is what the code runs. The
> package and API were checked against allwpilib `2027.0.0-alpha-8`. How each rule was derived from
> the old V2 rule, and what to re-check at beta, is in [docs/commands-v3.md](../../docs/commands-v3.md).
> Do not mix V2 idioms (`Commands.*`, `.withName`, `finallyDo`, `SubsystemBase`) into V3 code.

---

## Factory Method Pattern (Mandatory)

```java
// Every command method:
public static Command doThing(MyMechanism mech, SomeParam param) {
    return mech.run(coroutine -> { /* body */ })
        .named("MyMechanism_DoThing");   // the compiler refuses a command without a name
}
```

No exceptions. Never `implements Command` for game logic, and never put command factories on the
mechanism (`mechanism.run(...)` invites it) — they live in `XxxCommands` classes.

**A body that loops must `coroutine.yield()` every pass.** V3 has no preemption: a loop that never
yields freezes the whole robot program. Check every `while` in a command body.

---

## Choosing the Right Factory

| Scenario | Factory to use |
|---|---|
| "Run while button held, stop on release" | `mech.run(co -> { start; co.park(); }).whenExited(stop).named(...)` |
| "Do this once and finish" | `mech.run(co -> action).named(...)` |
| "Run every loop until interrupted" | `mech.runRepeatedly(action).named(...)` |
| "Run A, then B, then C" | `Command.sequence(A, B, C).named(...)` |
| "Run A and B simultaneously" | `Command.parallel(A, B).named(...)` — ends when **all** finish |
| "Run A and B, stop when either finishes" | `Command.race(A, B).named(...)` |
| "Run A until condition, then B" | `Command.sequence(A.until(cond).named(...), B).named(...)` |
| "Run both, stop when A finishes" | `new ParallelGroupBuilder().requiring(A).optional(B).named(...)` |
| "Do something with no mechanism" | `Command.noRequirements(co -> ...).named(...)` |

`whenExited(...)` runs on **any** exit — completion, cancel or interruption. `whenCanceled(...)`
runs only on cancel, so a body that can finish must use `whenExited`. A body that parks can only be
canceled, so either works there; prefer `whenExited`.

---

## Timeouts Are Mandatory on Every Wait

V3 has **two** untimed waits, and both are forbidden:

```java
Command.waitUntil(mechanism::isReady).named("Wait")   // WRONG — blocks forever
coroutine.waitUntil(mechanism::isReady);              // WRONG — blocks forever
```

The timed forms:

```java
Command.waitUntil(mechanism::isReady).named("Wait").withTimeout(Seconds.of(Waits.READY))
var result = coroutine.waitUntil(mechanism::isReady, Seconds.of(Waits.READY));
if (result.timedOut()) { /* log it, then decide: act anyway, or give up */ }
```

`coroutine.waitUntil(condition, timeout)` is the better form: it **returns whether it timed out**,
so a log can tell "ready" from "gave up". **Check every `WaitResult` and log the timeout as an
event.** This is a competition safety rule: a robot that hangs mid-sequence scores zero.

Every path command in an auto has a timeout as well.

---

## The Ready → Align → Act Pattern

```java
roller.run(co -> {
    roller.setVelocity(velocity);
    var ready = co.waitUntil(roller::isAtVelocity, Seconds.of(Waits.MECHANISM_READY_SECONDS));
    var aligned = co.waitUntil(isAligned,
        Seconds.of(Waits.TOTAL_TIMEOUT_SECONDS - Waits.MECHANISM_READY_SECONDS));
    Logger.recordOutput("Roller/WhenReady/ReadyTimedOut", ready.timedOut());
    Logger.recordOutput("Roller/WhenReady/AlignTimedOut", aligned.timedOut());
    co.await(act);                 // act must be finite and must NOT require the roller
}).whenExited(roller::stop).named("Roller_WhenReady");
```

Phase 2 timeout = total budget minus phase 1. This guarantees the robot always acts within the
total budget even if alignment never arrives. All timeout constants live in a constants class — not
inline.

Two V3 behaviors to know:

- **A forked or awaited child does not outlive its parent.** Cancel the parent and the child is
  canceled too. (V2's `ScheduleCommand` let the child run on.)
- **Keep a child off the mechanism its parent owns.** The scheduler leaves a command's parent chain
  out of its conflict checks, so such a child is not rejected, but who owns the mechanism while the
  child runs, and in the gap after it, is not something this repo has verified. Put the owning
  mechanism on the parent and give children other mechanisms, or use `Command.sequence(...)`, which
  owns everything for its whole duration. `Command.noRequirements(...)` with `fork`/`await` owns
  nothing between children, so a default command can take a mechanism back in the gap — verify
  against the scheduler at beta whether that gap is zero cycles or one.

---

## Where Command Values Live

Command factories take setpoints as **parameters**. They never read a setpoint themselves, and they
never hardcode one.

**Exception — test-only setpoint tuning.** A scaffold's `tune*` factory may read its own
`LoggedTunableNumber` to discover a velocity or position setpoint. It must require the mechanism,
run only while Utility mode is enabled, and end as soon as Utility mode ends. Match commands must
never read dashboard tunables; they use compiled values from `Constants.Setpoints`. See
[docs/tunables.md](../../docs/tunables.md).

The caller — almost always `RobotContainer` — supplies the value from `Constants.Setpoints`:

```java
// CORRECT
triggers.intake().whileTrue(IntakeCommands.runAtVelocity(intake, Constants.Setpoints.INTAKE_VELOCITY));

// WRONG — inline magic number, invisible in the pit
triggers.intake().whileTrue(IntakeCommands.runAtVelocity(intake, RotationsPerSecond.of(30)));

// ALSO WRONG — a private constant in RobotContainer is still scattered
private static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(30);
```

**When you build a new mechanism, every value you bind to a button or use in a sequence goes into
`Constants.Setpoints` as part of that same task.** Not into the mechanism's own constants file, and
not into `RobotContainer` as a private field.

| Value | Where |
|---|---|
| What the mechanism is commanded to — velocities, positions, voltages | `Constants.Setpoints` |
| How the mechanism is built or characterized — gains, ratios, current limits, soft limits, sim model | `MyMechanismConstants.java` |
| How long a command waits | `Constants.Waits` |
| How close counts as "there" | `MyMechanismConstants.java` (it is a property of the mechanism) |

V3 takes `Time` where V2 took `double` seconds (`Seconds.of(Waits.X)`); `Waits` is still `double`
seconds and is wrapped at the call site.

The test: **if you would change it in the pit between matches, it belongs in `Constants`.**
`RobotContainer` is the wiring layer. When it is finished it should contain no bare numbers at all.

---

## Default Commands

Default commands run when nothing else requires the mechanism. Rules:

- Safe idle states (stop, stow, hold, low-speed idle).
- They must not end (use `runRepeatedly`, not a one-shot).
- **Priority below `Command.DEFAULT_PRIORITY`** — `.withPriority(Command.LOWEST_PRIORITY)`. V3 lets
  a new command interrupt a running one only at **equal or higher** priority, so a default at or
  above `DEFAULT_PRIORITY` blocks every ordinary command. `RobotContainerSmokeTest` fails if one
  does.
- A mechanism with no default command is **not** unowned: V3 gives it `mechanism.idle()`, which owns
  it and does nothing. For a motor, that means the **last control request keeps running** — so every
  motor mechanism still needs an explicit stop/hold default.
- Set in `RobotContainer` via `mechanism.setDefaultCommand()`.
- Drive's default is `DriveCommands.joystickDrive` — do not remove it.

---

## Subsystems, `periodic()` and `Logger.processInputs()`

A V3 mechanism `implements org.wpilib.command3.Mechanism` — there is no `SubsystemBase`. It has
**no `periodic()` that anything calls.** Every subsystem's `periodic()` (which calls
`Logger.processInputs()`) is therefore called explicitly from **`RobotContainer.periodic()`**, which
`Robot.robotPeriodic()` calls **before** `Scheduler.getDefault().run()`, in a fixed order. This
includes `Vision`, which requires nothing and is not a mechanism at all.

**Adding a subsystem means adding its `periodic()` call there.** A subsystem left out is silent: no
inputs, no replay, no error. `RobotContainerSmokeTest.everySubsystemProcessesItsInputsEveryLoop`
fails when one is missing — it finds every subsystem in `RobotContainer` by reflection and checks its
inputs were logged under `/<ClassName>/`. This is a Hard Stop in `00-safety.md`.

---

## Command Naming Convention

`.named()` format: `"SubsystemName_ActionVerb_OptionalParam"`

Examples:

```
"Drive_Joystick"
"Drive_JoystickAtAngle"
"Drive_StopWithX"
"Mechanism_Velocity_2000RPM"
```

The compiler now guarantees a name exists; review still checks it is a *useful* one.
`withAutomaticName()` produces `"A -> B"` / `"A | B"`: acceptable for a throwaway composition and
poor for anything bound to a button or a BLine event trigger. Name those.

---

## Triggers and Bindings

`Triggers.java` rules are unchanged (see `01-architecture.md`). V3 semantics that differ from V2:

| Method | V3 behavior |
|---|---|
| `onTrue`, `onFalse` | Same as V2 |
| `whileTrue`, `whileFalse` | Start on the edge, cancel on the opposite edge. **Not** restarted if the command ends while still held |
| `retryWhileTrue`, `retryWhileFalse` | **New.** Like `whileTrue`, but restarts the command if it ends while still held |
| `ifTrue` | **New.** Schedules on *every poll* while true (no edge) |
| `toggleOnTrue`, `toggleOnFalse` | Same as V2 |
| `debounce(Time)` | Takes a `Time`, not `double` seconds |

A binding created inside a running command is **scoped** and removed when that command ends.
Bindings made in `RobotContainer` are global. If scoped bindings are used they go in the
command-composition layer, never scattered through subsystems.

---

## Disabled

V2's `.ignoringDisable(true)` is per command. V3 decides per **mechanism**:
`Mechanism.controllableDuringDisabled()` defaults to `false`, and a command requiring a
non-controllable mechanism cannot be scheduled while disabled and is canceled on disable.

Commands that require **no** mechanism are unaffected. The gyro reset updates odometry
directly while disabled. While enabled, it awaits a command that requires `Drive`, so an
active drive command must yield ownership before the heading changes. Leave
`Drive.controllableDuringDisabled()` at its default to keep motion commands disabled.

---

## OpModes: Autonomous, Teleop and Utility

The robot program is selected on the Driver Station, not on a dashboard chooser. `Robot` extends
AdvantageKit's `LoggedOpModeRobot` and `frc.robot.opmodes.OpModes` registers:

- **Teleoperated `Drive`** — the only teleop OpMode; bindings are global.
- **Autonomous** — one OpMode per BLine routine per side, `"<routine> (Left)"` and `"<routine>
  (Right)"`. Paths are authored blue-left; the Right OpMode mirrors them. Each OpMode builds its
  routine when it starts, so only the selected routine loads.
- **Utility** — characterization and SysId routines. Tuning and demo behavior is reachable only here,
  never from Teleoperated or Autonomous.

`Robot.onEnable()` runs from every OpMode's `start()`. Adding an auto means adding a routine to
`BLineAutos.all(...)`; `OpModes` registers both sides automatically.

---

## BLine Event Triggers

Bind in `RobotContainer`, before any routine is built:

```java
BLineAutos.bind("Intake", IntakeCommands.deploy(intake));
```

A bound command must NOT require the drive:

```java
BLineAutos.bind("Intake", IntakeCommands.deploy(intake));   // requires intake — fine
BLineAutos.bind("Stop", DriveCommands.stopWithX(drive));    // requires drive — WRONG
```

**Why:** BLine-Lib schedules a bound command on the default `Scheduler` **independently of** the path
command, on the next event-loop poll. The path command requires the drive, and an equal-priority
command that also requires it interrupts the path. Requiring a mechanism is fine — the path does not
use it.

A bound command also outlives the path unless it ends itself, and it is not canceled with the
routine ("Completion does not cancel it; the robot application owns cleanup"). Give it a termination
condition, and leave the mechanism safe if the path is interrupted.

---

## No Open-Ended PID-to-Pose in Autos

Never put the non-finishing `DriveCommands.driveToPose` in an auto sequence, or the sequence stops
there forever. Point-to-point autos and the PID-to-pose finish were removed for now; if they come
back, they need a tolerance or timeout (`withTimeout(Time)`).

---

## Conditional Re-evaluation

V3 deleted the need for a `ContinuousConditionalCommand` (it was removed from `frc.lib`). Write the
body as a loop that re-checks the condition and forks or awaits the matching command, or use a
`StateMachine` with a transition each way. Check that a state machine's fixed initial state does not
run the wrong branch for one cycle before relying on it.

---

## Cancellation Flags

Reach for **priorities** (the driver override runs at a higher priority and the automatic behavior
cannot interrupt it back) or a **scoped trigger** inside the automatic command before adding a
boolean flag in `RobotContainer`. If a flag is still needed, write down when it is set and when it
is cleared, and check every binding that reads it when you change any of them.

---

## Auto Time Budget

Autos must fit the auto period with margin. In 2026 the routines overran their budget and the last
path was truncated in *every* match, with roughly a third of auto spent stationary waiting on
mechanisms. `AutoOpMode` logs `Auto/DurationSeconds` and `Auto/Overran`. Time each auto in
simulation and against real logs before competition — not at competition.
