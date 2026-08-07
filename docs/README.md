# docs/

Team documentation carried forward from the 2026 season into the 2027 template.

Everything here has been made season- and robot-agnostic. Where a document described the
2026 robot specifically, the **structure** was kept and the data was replaced with TODOs —
those files carry a banner at the top saying exactly what needs filling in.

---

## Ready to use as-is

Process and workflow docs. Nothing in them depends on a particular robot or game.

| Doc | What it's for |
|---|---|
| [ai-development-handbook.md](ai-development-handbook.md) | How to actually work with Claude Code on this codebase — specs, context, the loop, testing. Start here. |
| [ai-development-playbook.md](ai-development-playbook.md) | The team-facing case for AI-assisted development: why, the concerns, the guardrails. For mentors and leads. |
| [review-checklist.md](review-checklist.md) | Run before submitting or merging any change |
| [change-classification.md](change-classification.md) | The five risk levels, with worked examples |
| [pre-match-checklist.md](pre-match-checklist.md) | Event-day checklist. Structure is ready; fill in the 2027 mechanism checks and device count, then print it |
| [characterization-and-tuning.md](characterization-and-tuning.md) | Running the FF/wheel-radius/SysId routines and turning their output into gains. Covers the TorqueCurrentFOC units trap, drive and steer specifically, then a general recipe split by velocity vs position control |
| [third-party-code.md](third-party-code.md) | Where borrowed code came from and what licence it carries. Two files are GPLv3 and the repo has no LICENSE — read before publishing |
| [tunables.md](tunables.md) | Changing values live instead of redeploying. RAM-side vs TalonFX Slot0 gains, what should and should not be tunable, where Phoenix Tuner X fits, and why gains typed into Tuner X get silently overwritten at the next boot |
| [new-mechanism-bringup.md](new-mechanism-bringup.md) | The ordered path from "the subsystem compiles" to "the mechanism is trusted": what to set before it moves, first open-loop motion, gains, motion profile, tolerances, and writing it all back |
| [target-alignment.md](target-alignment.md) | How to use `joystickDriveAtAngle` and `driveToPose`, the FieldConstants pattern for turning AprilTag poses into scoring targets, and the sim tests to copy before wiring a new one up |
| [testing.md](testing.md) | What is under test and what each layer catches — CAN ID validation, `RobotContainer` wiring, vision filtering, and sim convergence — plus the architecture rules that nothing checks automatically. Read before adding a mechanism |

## Procedure kept, data needs filling in

| Doc | What needs updating |
|---|---|
| [VISION_DEBUG_CHECKLIST.md](VISION_DEBUG_CHECKLIST.md) | Camera names (§2) and transforms (§5) are placeholders; vendordep version (§1) and AprilTag layout (§3, §15) are 2026 |
| [subsystem-ownership.md](subsystem-ownership.md) | Drive and Vision sections are accurate. Add a section per 2027 mechanism as you build it |
| [hardware-layout.md](hardware-layout.md) | Swerve section is 2026 — regenerate with Tuner X. Mechanism, camera and physical-spec tables are empty |

## Skeletons — describe no robot yet

| Doc | Status |
|---|---|
| [robot-spec.md](robot-spec.md) | Section structure kept; §2–§6, §9, §11, §14–§17 describe shipped code. The rest is TODO |
| [driver-controls-card.md](driver-controls-card.md) | Drive controls are real (flight stick); operator section is blank. Fill it in as mechanisms land, then print it for the pit |
| [drive-controller-mode.md](drive-controller-mode.md) | Design record for the dashboard controller swap. Not wired up, but `Triggers` already routes every axis read through suppliers so it can be added in one file |

---

## Related, outside this directory

- `CLAUDE.md` — AI governance, hard stops, risk tiers
- `.claude/rules/` — the detailed architecture, hardware, command, build and git rules
- `.claude/prompts/` — fill-in-the-blank templates for common tasks
- `template/GUIDE.md` — codebase structure, what carried over, what debt is still open
- `template/NEW_SEASON_CHECKLIST.md` — the season startup sequence
- `tools/README.md` — ClaudeScope, log-sync, wpilib-agent-tools

---

## Keeping these current

`robot-spec.md` is the one that pays for itself. It is what lets a new team member — or an
agent — make a correct change without reading every file first. The 2026 version was
~1450 lines and was worth every one of them.

A doc that lies is worse than no doc. If you change behavior and the spec still describes
the old behavior, you have created a trap. Update it in the same commit.
