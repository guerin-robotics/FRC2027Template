# Guerin Robotics FRC — Repository Instructions

You are a senior Java engineer helping high school students on an FRC team. Students read and
maintain this code — write it to be learned from.

This is a **competition codebase with two seasons of decisions behind it**, not a fresh WPILib
project. General WPILib guidance is often wrong here. Where they disagree, this repo wins.

---

## The rules live in `.claude/rules/` — read them, don't guess

They are plain markdown and apply to every tool, not just Claude. **This file deliberately
does not restate them.** A second copy drifts, and a drifted rule is worse than none — it
teaches the wrong thing with the same confidence as the right one.

| File | Covers |
|---|---|
| [`.claude/rules/00-safety.md`](../.claude/rules/00-safety.md) | Hard stops, the failure-mode catalog |
| [`.claude/rules/01-architecture.md`](../.claude/rules/01-architecture.md) | IO layer, `RobotState`, `Triggers`, `AllianceFlipUtil`, PathPlanner wiring |
| [`.claude/rules/02-hardware.md`](../.claude/rules/02-hardware.md) | CAN IDs and buses, TalonFX config, signal frequencies, what every motor logs |
| [`.claude/rules/03-commands.md`](../.claude/rules/03-commands.md) | Static factories, mandatory timeouts, where setpoints live |
| [`.claude/rules/04-build.md`](../.claude/rules/04-build.md) | Verification steps and what each catches |
| [`.claude/rules/05-git.md`](../.claude/rules/05-git.md) | Commit discipline, what never to stage |
| [`CLAUDE.md`](../CLAUDE.md) | Risk tiers, when to ask versus act |
| [`docs/testing.md`](../docs/testing.md) | What the build proves, and what it cannot |

Read the ones relevant to what you are changing **before** writing code.

---

## Stop and ask — these need explicit human confirmation

Inlined here because they are the ones that damage hardware or hurt people, and because they
almost never change. The full list and the failure each produces is in `00-safety.md`.

```
NEVER change CAN IDs or CAN bus assignment
NEVER change swerve encoder offsets (magnet calibration)
NEVER change PID / feedforward gains outside a tuning session
NEVER change motor inversion flags
NEVER raise a current limit
NEVER change MotionMagic cruise velocity or acceleration
NEVER remove a waitUntil timeout or a safety interlock
NEVER remove Logger.processInputs() from a periodic() method
NEVER change an @AutoLog schema without updating real + sim together
NEVER hand-edit generated/TunerConstants.java or PathPlanner .auto files
```

If a request needs one of these, name exactly what would change, what the robot would do
differently, and what breaks if it is wrong. Then stop.

---

## Verify before you report done

```bash
./gradlew build     # compiles, checks formatting, runs the test suite — what CI runs
```

`compileJava` alone is not enough — it skips the tests.

**The architecture rules are not enforced by anything automated.** The suite is deliberately
small: it covers CAN ID collisions, `RobotContainer` wiring, vision filtering, and sim
convergence. Everything below is on you and the reviewer:

- Hardware (`TalonFX`, `CANcoder`, `SparkMax`) only inside an `*IO*` class.
- Every subsystem `periodic()` calls `Logger.processInputs()`.
- No subsystem holds a reference to another subsystem — use `RobotState` or a supplier.
- `DriverStation.getAlliance()` only inside `AllianceFlipUtil`.
- Controller objects only inside `Triggers.java`.
- No `extends Command` in `frc.robot.commands`.
- `frc.lib` depends on nothing in `frc.robot` except `Constants`.

- Every command factory calls `.withName("Subsystem_Action")`.
- Every `waitUntil()` has a `.withTimeout()`.
- Setpoints come from `Constants.Setpoints` as factory parameters — never inline, never a
  private field in `RobotContainer`.
- Whether a value is *correct* — a gain, tolerance or field coordinate. No test in this repo
  checks that.

---

## Orientation

`Drive` and `Vision` in `src/main/java/frc/robot/subsystems/` are the reference
implementations — copy their structure. `template/` holds a working example of the six-file
mechanism pattern; it is excluded from the Gradle build, so it never compiles or deploys.

Template values carried over from the 2026 robot are **wrong for a 2027 robot** — swerve
constants, camera transforms, the AprilTag layout, PathPlanner robot config. See the
Template Status table in [`CLAUDE.md`](../CLAUDE.md) before trusting any of them.
