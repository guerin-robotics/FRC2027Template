# Command Rules

---

## Factory Method Pattern (Mandatory)

```java
// Every command method:
public static Command doThing(MySubsystem sub, SomeParam param) {
    return Commands.<something>(...)
        .withName("MySubsystem_DoThing");   // ALWAYS name commands
}
```

No exceptions. Never `new SomeCommand(...)` for game logic.

---

## Choosing the Right Factory

| Scenario | Factory to use |
|---|---|
| "Run while button held, stop on release" | `Commands.startEnd(onStart, onEnd, subsystem)` |
| "Do this once and finish" | `Commands.runOnce(action, subsystem)` |
| "Run every loop until interrupted" | `Commands.run(action, subsystem)` |
| "Run A, then B, then C" | `Commands.sequence(A, B, C)` |
| "Run A and B simultaneously" | `Commands.parallel(A, B)` |
| "Run A until condition, then B" | `Commands.sequence(A.until(cond), B)` |
| "Run both, stop when A finishes" | `Commands.deadline(A, B)` |

---

## Timeouts Are Mandatory on `waitUntil`

```java
// CORRECT — always has a timeout escape
Commands.waitUntil(mechanism::isReady).withTimeout(1.5)

// WRONG — hangs forever if condition never becomes true
Commands.waitUntil(mechanism::isReady)
```

This is a competition safety rule. A robot that hangs mid-sequence scores zero points.

---

## The Ready → Align → Act Pattern

Use this pattern for any action that requires mechanism readiness + robot alignment:

```java
Commands.sequence(
    // Phase 1: wait for mechanism readiness (hard timeout)
    Commands.waitUntil(mechanism::isReady)
        .withTimeout(Waits.mechanismReadySeconds),

    // Phase 2: wait for alignment OR give up after budget
    Commands.waitUntil(isAligned)
        .withTimeout(Waits.totalTimeoutSeconds - Waits.mechanismReadySeconds),

    // Phase 3: act
    runTheAction()
)
```

Phase 2 timeout = total budget minus phase 1. This guarantees the robot always
acts within the total budget even if alignment never arrives.

All timeout constants must live in a constants class — not inline.

---

## Where Command Values Live

Command factories take setpoints as **parameters**. They never read a setpoint
themselves, and they never hardcode one.

The caller — almost always `RobotContainer` — supplies the value from
`Constants.Setpoints`:

```java
// CORRECT
controller.rightBumper()
    .whileTrue(IntakeCommands.runAtVelocity(intake, Constants.Setpoints.INTAKE_VELOCITY));

// WRONG — inline magic number, invisible in the pit
controller.rightBumper()
    .whileTrue(IntakeCommands.runAtVelocity(intake, RotationsPerSecond.of(30)));

// ALSO WRONG — a private constant in RobotContainer is still scattered
private static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(30);
```

**When you build a new subsystem, every value you bind to a button or use in a
sequence goes into `Constants.Setpoints` as part of that same task.** Not into the
subsystem's own constants file, and not into `RobotContainer` as a private field.

The split, restated:

| Value | Where |
|---|---|
| What the mechanism is commanded to — velocities, positions, voltages | `Constants.Setpoints` |
| How the mechanism is built or characterized — gains, ratios, current limits, soft limits, sim model | `MyMechanismConstants.java` |
| How long a command waits | `Constants.Waits` |
| How close counts as "there" | `MyMechanismConstants.java` (it is a property of the mechanism) |

The test: **if you would change it in the pit between matches, it belongs in
`Constants`.** A driver asking for "a bit more intake speed" should send you to one
file, not to a hunt across three.

`RobotContainer` is the wiring layer. When it is finished it should contain no bare
numbers at all — if a unit import like `RotationsPerSecond` is still needed there, a
setpoint has been left behind.

---

## Default Commands

Default commands run when nothing else requires the subsystem. Rules:

- Default commands should be safe idle states (stop, stow, low-speed idle)
- They must not end (use `Commands.run()`, not `Commands.runOnce()`)
- Set in `RobotContainer` via `subsystem.setDefaultCommand()`
- Drive's default is `DriveCommands.joystickDrive` — do not remove it

---

## Command Naming Convention

`.withName()` format: `"SubsystemName_ActionVerb_OptionalParam"`

Examples:

```
"Drive_Joystick"
"Drive_JoystickAtAngle"
"Drive_StopWithX"
"Mechanism_Velocity_2000RPM"
```

Names appear in AdvantageKit's command log. Bad names make debugging impossible.

---

## Named Commands (PathPlanner)

Register in `RobotContainer` before `AutoBuilder.buildAutoChooser()`:

```java
NamedCommands.registerCommand("Score", ScoringSequences.autoScore(...));
```

Event triggers must NOT declare subsystem requirements:

```java
new EventTrigger("DeployIntake")
    .whileTrue(Commands.runOnce(() -> intake.setPosition(down)));
    //                          ^^ no subsystem arg — intentional
```

**Why:** A `SequentialCommandGroup`'s requirements are the UNION of all its sub-commands'
requirements. An event trigger that shares any requirement with the auto group will cause
WPILib's scheduler to interrupt the entire auto to resolve the conflict. Dropping the
requirement lets both run — the subsystem method is still called, so the hardware moves.

---

## ContinuousConditionalCommand

Use this (from `frc.lib`) when a command's mode must be re-evaluated while running:

```java
new ContinuousConditionalCommand(commandIfTrue, commandIfFalse, conditionSupplier)
```

WPILib's `ConditionalCommand` evaluates the condition only at schedule time.
Use `ContinuousConditionalCommand` when the driver can toggle a mode mid-execution.

---

## Cancellation Flags

When a driver override needs to suppress an automatic behavior for the rest of a button
press, use a boolean flag in `RobotContainer` set by the override and cleared on release.

The 2026 robot had three of these (`compressCancelled`, `xCancelled`, `doubleCompress`).
They worked, but resetting a flag at the wrong time caused auto-behaviors to re-engage
unexpectedly. If you add one, write down when it is set and when it is cleared, and check
every binding that reads it when you change any of them.

---

## Auto Time Budget

Autos must fit the auto period with margin. In 2026 the routines overran their budget and
the last path was truncated in *every* match, with roughly a third of auto spent
stationary waiting on mechanisms. Time each auto in simulation and against real logs
before competition — not at competition.
