# Claude Code — Guerin Robotics FRC

*AI governance for a competition robot codebase. Read completely before acting.*

---

## Identity

You are assisting an FRC robotics team that competed at Worlds.

This repository is the **2027 season template**, carried over from the 2026 competition
robot (`Rebuilt2026`). It contains the drivetrain, vision, and supporting infrastructure —
the parts that are not game-specific — with every 2026 mechanism and every piece of 2026
game logic removed.

**What that means for how you work here:**

- Early in the season, this repo is *meant* to grow. Adding subsystems, commands and
  game logic is the expected work, not a risk to be minimized.
- The code that is already here has been through two competition seasons. Treat it as
  working code: preserve its behavior, and be skeptical of restructuring it.
- Once the robot is real and driving, the default stance flips to the one this codebase
  has always had: preserve working behavior, make targeted and reviewable changes.

---

## Template Status — Read This First

The values carried over from 2026 are **not** correct for a 2027 robot. Before this code
drives anything, these must be revisited:

| Item | File | What's needed |
|---|---|---|
| Swerve CAN IDs, encoder offsets, geometry, gains | `generated/TunerConstants.java` | Regenerate with CTRE Tuner X against the real robot |
| Camera names and `robotToCameraN` transforms | `subsystems/vision/VisionConstants.java` | Identity placeholders — measure on the real robot |
| AprilTag layout | `frc/lib/FieldConstants.java` | Points at the 2026 field; update when WPILib ships 2027 |
| PathPlanner robot config (mass, MOI, wheel COF) | `subsystems/drive/Drive.java` | `ROBOT_MASS_KG`, `ROBOT_MOI`, `WHEEL_COF` are 2026 values |
| Heading-hold gains | `commands/DriveCommands.java` | `ANGLE_KP` / `ANGLE_KD` tuned on the 2026 chassis |
| Team number | `.wpilib/wpilib_preferences.json` | Verify |

Vision filter thresholds in `VisionConstants` are geometry-independent and were tuned
against real 2026 match logs. Those are worth keeping — do not reset them to defaults.

---

## Hard Stops — Never Do These Without Explicit Instruction

These actions can destroy working robot behavior or cause physical harm:

```
NEVER change CAN IDs
NEVER change swerve encoder offsets (magnet calibration)
NEVER change PID / feedforward gains outside a tuning session
NEVER remove Logger.processInputs() from any periodic() method
NEVER add hardware calls (TalonFX, CANcoder, etc.) inside a subsystem class
NEVER modify PathPlanner .auto files by hand
NEVER change motor inversion flags
NEVER disable or remove a safety timeout
NEVER change an AdvantageKit IO interface (@AutoLog fields) without updating real + sim
NEVER silence a compiler warning by suppressing it — fix the root cause
```

If a user request would require one of the above, **stop and ask for explicit
confirmation** before proceeding. Name exactly what would change and why.

---

## Safety Hierarchy for Edits

Classify every proposed change before making it:

| Level | Description | Behavior |
|---|---|---|
| **Safe** | Comment, rename, log addition, new command factory | Proceed |
| **Low risk** | New subsystem (no existing wiring), new auto named command | Proceed, note what was added |
| **Medium risk** | Modifying existing command logic, changing wait timeouts, tuning thresholds | Proceed with explicit summary of behavioral change |
| **High risk** | Changing PID gains, swerve constants, vision filter thresholds, button bindings | Ask for confirmation, describe the failure mode |
| **Blocked** | Any Hard Stop above | Stop. Ask explicitly. |

When uncertain which level, treat as one level higher.

---

## Change Discipline

**One logical change per response.** If a task requires touching 3 subsystems, confirm
the plan first.

**Scope creep is a bug.** If fixing a command and you notice a style issue elsewhere —
leave it. Do not fix things you weren't asked to fix.

**No opportunistic refactors.** Do not rename variables, reorganize imports, or
restructure classes unless that is the explicit task.

**Small diffs are correct.** A 5-line change that fixes the issue is better than a
50-line change that also "cleans up" the file.

---

## Architecture Rules

@.claude/rules/01-architecture.md

---

## Hardware & CAN Rules

@.claude/rules/02-hardware.md

---

## Command Rules

@.claude/rules/03-commands.md

---

## Build & Verification

@.claude/rules/04-build.md

---

## Git Discipline

@.claude/rules/05-git.md

---

## Subsystem Ownership Model

Every subsystem has a single owner file. Changes to a subsystem must not leak into other
subsystem files. If a fix requires touching two subsystems, that is a design problem —
surface it rather than patching across boundaries.

**Allowed cross-subsystem paths:**

- `RobotState` — read-only access via singleton (no setters except `setPoseSupplier`)
- A dedicated command-composition layer (in 2026 this was `ShootSequences.java` /
  `SpitSequences.java`) — create the 2027 equivalent when sequences span subsystems
- `RobotContainer` — wiring only; no logic

If you find yourself writing game logic inside `RobotContainer`, it belongs in a command
or in `RobotState`.

---

## When to Ask vs When to Act

**Act without asking:**

- Adding a log line
- Adding a `.withName()` call to a command
- Fixing a compile error
- Writing a new subsystem from scratch following the vision/drive IO pattern
- Adding a new named command to `RobotContainer`

**Ask before acting:**

- Any change to a file that controls physical robot motion
- Any change to a timeout, threshold, or tuning constant
- Any change that affects auto behavior
- Any change that removes existing functionality
- Any change to drive, vision, or swerve configuration
- When the right approach is unclear

**Always explain behavioral changes.** "I changed the alignment timeout from 1.0 s to
0.8 s" is not enough — say "this means the robot will act 0.2 s sooner if alignment
hasn't been achieved, which reduces hang time but may increase misses."

---

## Response Format

- State what you changed and why in one or two sentences
- If a change has a behavioral consequence, name it explicitly
- If you identified a risk, flag it even if you didn't address it
- Do not summarize the diff — the user can read it
- Do not add "let me know if you need anything else"

---

## Reusable Prompts

Common task templates are in `.claude/prompts/`:

- [add-subsystem.md](.claude/prompts/add-subsystem.md) — scaffold a new mechanism subsystem
- [tune-constants.md](.claude/prompts/tune-constants.md) — update PID/FF/threshold values
- [new-auto.md](.claude/prompts/new-auto.md) — create or modify an autonomous routine
- [review-change.md](.claude/prompts/review-change.md) — review a proposed change for safety
- [write-test.md](.claude/prompts/write-test.md) — lock in a behavior or bug as a JUnit test

Team skills (slash commands) are in `.claude/skills/`:

- `/debug-match-log` — root-cause a field problem from an AdvantageKit log
- `/pid-tune` — sim-based PID/FF tuning loop; proposes gains, user confirms

---

## Documentation

`docs/` carried over from 2026, genericized. See [docs/README.md](docs/README.md) for the
index and the current state of each file.

Ready to use: `ai-development-handbook.md`, `ai-development-playbook.md`,
`review-checklist.md`, `change-classification.md`.

Need robot data filled in (each carries a banner saying what):
`robot-spec.md`, `hardware-layout.md`, `subsystem-ownership.md`,
`VISION_DEBUG_CHECKLIST.md`, `driver-controls-card.md`, `drive-controller-mode.md`.

**Keep `docs/robot-spec.md` current.** It is the artifact that lets a new team member or
an agent make a correct change without reading every file. A doc that describes the old
behavior after you changed it is worse than no doc — update it in the same commit.

---

## Style Reference

`template/` holds a working example of the team's subsystem pattern — the five files every
mechanism must have. It is **not** part of the Gradle build, so it is never compiled or
deployed; the examples deliberately reference classes that don't exist yet.

- [template/GUIDE.md](template/GUIDE.md) — the five-file pattern, what carried over, what debt is still open
- [template/NEW_SEASON_CHECKLIST.md](template/NEW_SEASON_CHECKLIST.md) — season startup sequence
- `template/src/` — copy these when scaffolding a mechanism

`subsystems/vision/` in `src/` is the same pattern in production form.

---

## Tooling

`tools/` holds season-agnostic developer tooling — see [tools/README.md](tools/README.md).
None of it is part of the robot build, and it is excluded from Spotless so upstream pulls
stay clean.

- **ClaudeScope** — `/scope` and `/simulate` skills for querying `.wpilog` and live NT
- **log-sync** — pulls match logs off the roboRIO to Google Drive between matches
- **wpilib-agent-tools** — Python CLI for sim, NT4 recording, and log analysis

`.agent/skills/` holds two harness-agnostic skills:

- `replay-testing.md` — how to replay a match log deterministically against current code
- `tools.md` — which of the above tools to reach for, and when

---

## CI

`.github/workflows/build.yml` runs `spotlessCheck` then `build` on every PR and push to
main. **If it fails, fix the code — do not weaken the workflow.**

`.github/instructions/default.instructions.md` is the GitHub Copilot instruction file;
`.github/agents/` and `.github/prompts/` hold the code-review agent configs. They restate
the same architecture rules as `.claude/rules/` for a different harness — if you change a
rule in one place, change it in both or they will drift.
