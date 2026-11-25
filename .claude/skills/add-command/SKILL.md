---
name: add-command
description: Add a command to an existing mechanism or to the drive, following the repo's rules — static factory in the XxxCommands class, named, every wait timed out, setpoints in Constants.Setpoints, timeouts in Constants.Waits, binding in Triggers.java and RobotContainer. Use for any new command, button binding or command sequence on an existing subsystem. For a brand-new mechanism use /add-subsystem.
---

# Add a Command

Commands are **static factory methods**, never classes (`.claude/rules/01-architecture.md`,
`03-commands.md`). This skill produces one that follows those rules and compiles.

## Classify first

| Change | Level |
|---|---|
| New command factory, not bound | Safe |
| New binding to an unused button | Medium — state what the button now does |
| Changes an existing binding or default command | **High** — ask first, name the failure mode |
| Touches drive, vision, or anything that moves the robot on the field | Ask before acting |

If the change spans more than one subsystem, stop and confirm the plan first.

## Step 1 — Gather

Ask for, or infer from the code:

1. Which subsystem, and what the command does (hold-to-run, one-shot, sequence).
2. Setpoints it uses (value, unit).
3. What it requires and what must be true to start (readiness, alignment).
4. How it is triggered (a button, a state, an auto event).

## Step 2 — Write the factory

In `commands/XxxCommands.java`, not on the subsystem:

```java
public static Command runAtVelocity(Xxx xxx, AngularVelocity velocity) {
  return xxx.run(co -> {
        xxx.setVelocity(velocity);
        co.park();
      })
      .whenExited(xxx::stop)
      .named("Xxx_Velocity");
}
```

Checklist — all mandatory:

- [ ] `public static Command`, no `extends Command`, no `new SomeCommand(...)`.
- [ ] `.named("Subsystem_ActionVerb_Param")`. Names appear in the log.
- [ ] Use the mechanism's factory so the command requires the mechanism it drives.
- [ ] Setpoints are **parameters**. The factory never reads or hardcodes one.
- [ ] **Every `waitUntil` has an explicit timeout**, from `Constants.Waits`, not inline.
- [ ] Ready → Align → Act sequences: phase 2 timeout = total budget minus phase 1.
- [ ] The command leaves the mechanism safe when interrupted (the `end` action stops or holds).
- [ ] Default commands use `mechanism.runRepeatedly(...)` at `Command.LOWEST_PRIORITY` and are safe idle states.
- [ ] No hardware calls — everything goes through the subsystem's methods and IO.
- [ ] A command bound to a BLine event trigger **must not require the drive**, and must end itself.

Use `run` for a coroutine, `runRepeatedly` for an every-loop action, and `Command.sequence` or
`Command.parallel` for composition. See [the command rules](../../rules/03-commands.md). Cross-subsystem sequencing goes in a
dedicated command-composition class (the 2027 equivalent of 2026's `ShootSequences`), never in
`RobotContainer` and never as one subsystem calling another.

A `tune*` command that reads a `LoggedTunableNumber` is the one exception to "no dashboard values":
it must require the mechanism, run only in enabled Utility mode with the held-button gate, and end when either gate closes. Match
commands use compiled values from `Constants.Setpoints`.

## Step 3 — Put values where they belong

| Value | Where |
|---|---|
| What the mechanism is commanded to (velocity, position, voltage) | `Constants.Setpoints` |
| How long a command waits | `Constants.Waits` |
| How the mechanism is built or characterized | `XxxConstants.java` |

If you would change it in the pit between matches, it belongs in `Constants`. `RobotContainer`
should end up with no bare numbers.

## Step 4 — Wire it

1. Add the accessor in `Triggers.java`, named for the **action**, not the button (`runIntake()`,
   not `aButton()`). Controllers are private to that file. A state trigger is a `LoggedTrigger`
   field there.
2. Bind in `RobotContainer` from `Triggers.getInstance()`. RobotContainer is wiring only.
3. BLine event trigger: `BLineAutos.bind("Key", command)` before any routine is built. The smoke
   test fails if a path uses a key nothing is bound to.
4. Update `docs/driver-controls-card.md` for any new binding, and `docs/robot-spec.md` if behavior
   changed — a doc that describes the old behavior is a trap.

## Step 5 — Verify

```
./gradlew compileJava
./gradlew spotlessCheck     # spotlessApply if it fails
./gradlew build
./gradlew simulateJava      # for command, trigger or state-calculation changes
```

In sim, confirm the command activates and deactivates, the name appears in the log, the timeout
fires when the condition never becomes true (force it), and the mechanism stops on release.

Report: what you added, the binding, and any behavioral consequence. Do not change anything you
were not asked to.

## Related

- `.claude/rules/03-commands.md`, `docs/commands-v3.md` — V2 rules, and what each becomes under V3
- `docs/skills.md` — how to run this skill in sim and on the robot
