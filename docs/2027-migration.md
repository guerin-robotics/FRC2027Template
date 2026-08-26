# WPILib 2027 / Commands V3 Migration

> **Status: not started, deliberately.** This repo is on WPILib 2026 (GradleRIO 2026.2.1, Java 17,
> `edu.wpi.first`, commands V2) and stays there until WPILib 2027 reaches **beta**. This document
> records the audit so the port does not have to re-derive it.
>
> *Audit date: 2026-08-26. Re-check the version table before acting on it.*

---

## Why we are waiting

WPILib 2027 was at **alpha-6** (released 2026-05-08) when this was written. No beta.

The blocking problem is not that it is alpha — it is that the API is still moving. The Commands V3
package was renamed **between alphas**:

| Alpha | Commands V3 package |
|---|---|
| 2027.0.0-alpha-3 | `org.wpilib.commands3` |
| 2027.0.0-alpha-5 | `org.wpilib.command3` |

Porting onto that means chasing renames rather than writing robot code, and every line of the port
would be re-done at beta anyway.

The second reason: **we have no Systemcore hardware.** 2027 is a control-system change, not a
library bump — the roboRIO is legal only through the 2026 season, and 2027 code targets Systemcore.
Without a unit, nothing we port can be tested beyond sim and unit tests, and `main` would stop being
deployable on the hardware the team actually has.

**Decision: build season-independent work (scaffolds, tests, docs) on 2026 now; do the 2027 port as
one clean pass at beta.**

---

## What the port actually involves

Not a version bump. In rough order of blast radius:

| Change | Impact here |
|---|---|
| **Systemcore replaces the roboRIO** | New deploy target, new Driver Station. `build.gradle`'s `deploy { targets { roborio ... } }` block is rewritten |
| **Java 17 → Java 25** | `build.gradle` toolchain, CI workflow, every developer's JDK |
| **`edu.wpi.first` → `org.wpilib`** | Every source file. The VS Code importer attempts this automatically |
| **Commands V2 → V3** | 17 files touch `wpilibj2.command` (9 in `src/main`, 6 in `src/test`, 2 in `template/`) |
| **SmartDashboard and Shuffleboard removed** | `FaultMonitor.java`, `Robot.java`, and both scaffold visualizers |
| **NetworkTables v3 removed** | We are already on NT4; expected to be a no-op. Verify |
| **Phoenix device constructors changed** | Mostly already done — see below |

### Commands V3 is a rewrite, not a port

V3 is not V2 with new names. It uses **coroutines** (Java 21+ continuations) and **mechanism
ownership** in place of `Subsystem` requirements, and it makes command names **required at
construction**.

That invalidates doctrine, not just imports:

- `.claude/rules/03-commands.md` — the static-factory pattern, the `Commands.startEnd` /
  `runOnce` / `sequence` table, and the mandatory-`.withName()` rule all need re-deriving. Command
  names being required by construction is a straight improvement; the rest needs thought.
- **`ContinuousConditionalCommand`** (`frc/lib`) exists because WPILib's `ConditionalCommand`
  evaluates its condition only at schedule time. Check whether V3 makes it unnecessary before
  porting it.
- **`LoggedTrigger`**, **`CommandLogger`** — both wrap V2 scheduler internals.
- The `waitUntil`-with-mandatory-timeout rule is a competition safety rule and must survive the
  port in whatever form V3 expresses it. Every scaffold command factory depends on it.

**Do not port these mechanically.** Read the
[commands-v3 design doc](https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/commands-v3.md)
first and decide what each one becomes.

---

## Vendor readiness — the real gate

Against WPILib 2027 alpha-5/6, as of the audit date:

| Vendor | 2027 status | Notes |
|---|---|---|
| CTRE Phoenix 6 | ✅ `26.50.0-alpha-1` | Breaking API changes, see below |
| PathPlannerLib | ✅ `2027.0.0-alpha-3` | Author is holding major changes for the Systemcore switch |
| AdvantageKit | ⚠️ `27.0.0-alpha-4` | **Alert logging disabled** — WPILib alpha-6 has no alert support |
| REVLib | ✅ `2027.0.0-alpha-2` | Not used here |
| ReduxLib | ✅ `2027.0.0-alpha-6` | Not used here |
| **PhotonVision** | ❌ **no vendordep** | CI artifacts off `branch:2027` only |
| ChoreoLib | ❌ no compatible release | Not used here |

Two of these land on us directly:

**PhotonVision has no official 2027 vendordep.** The `Vision` subsystem is one of the two things
this template exists to carry forward. Until PhotonVision ships a real release, a 2027 port means
either pinning an unversioned CI build or having no vision at all. **Check this first at beta** —
it can gate the whole port regardless of what WPILib does.

**AdvantageKit's alert logging is disabled on 2027 alpha.** This repo uses `edu.wpi.first.wpilibj.Alert`
in **7 files**, and alerts are how `FaultMonitor`, the loop-timing monitor and the pose-divergence
detector reach the pit. Losing them is a real capability regression, not a cosmetic one. Confirm it
is restored before porting, or plan a fallback path for surfacing faults.

---

## Phoenix 6: mostly already handled

Phoenix `26.50.0-alpha-1` **removed** the device constructors taking a CAN bus *string*, and those
taking a device ID with no bus at all.

This codebase is already clean on that point — a deliberate earlier decision, not luck:

- `Constants.CanIds.RIO_BUS` is a `CANBus` object, not `"rio"`.
- `ModuleIOTalonFX` and `GyroIOPigeon2` take `TunerConstants.kCANBus`.
- All three scaffold `*IOReal` classes pass `Constants.CanIds.RIO_BUS`.

**What does change: the bus model.** Systemcore has **five native CAN buses**, constructed with
`CANBus.systemcore(int)`. There is no `"rio"` bus. That makes the bus-assignment table in
[`.claude/rules/02-hardware.md`](../.claude/rules/02-hardware.md) wrong as written — the
`"Canivore"` vs `"rio"` split has to be re-expressed against the new bus numbering. CANivores still
work; Motioncore buses were not supported at alpha.

Rename `RIO_BUS` when that happens. A constant named for hardware that no longer exists is exactly
the kind of lie this codebase tries not to carry.

---

## Concrete checklist for the beta port

Do these in order. Steps 1–3 are gates; stop if one fails.

- [ ] **1.** Confirm WPILib 2027 is at beta and the Commands V3 package name has settled.
- [ ] **2.** Confirm PhotonVision ships a real 2027 vendordep.
- [ ] **3.** Confirm AdvantageKit alert logging is restored.
- [ ] **4.** Branch. Never do this on `main` while the team needs a deployable robot.
- [ ] **5.** `build.gradle`: GradleRIO 2027, Java 25 toolchain, Systemcore deploy target.
- [ ] **6.** Update the CI workflow's JDK. It runs `spotlessCheck` then `build` — do not weaken it.
- [ ] **7.** Pull every 2027 vendordep.
- [ ] **8.** `edu.wpi.first` → `org.wpilib` across `src/` **and** `template/`. Mechanical.
- [ ] **9.** Replace `SmartDashboard` in `FaultMonitor.java`, `Robot.java`, `ExampleArmVisualizer`,
      `ExampleLiftVisualizer`. Decide the replacement once and apply it everywhere.
- [ ] **10.** Commands V2 → V3, guided by the design doc. Rewrite `.claude/rules/03-commands.md` in
      the same pass — a rule that lies is worse than no rule.
- [ ] **11.** Re-derive `ContinuousConditionalCommand`, `LoggedTrigger`, `CommandLogger`.
- [ ] **12.** Re-express the CAN bus assignment rules for Systemcore; rename `RIO_BUS`.
- [ ] **13.** Get the test suite green. `RobotContainerSmokeTest` and the ArchUnit rules will need
      updating for V3 — they encode V2 assumptions.
- [ ] **14.** Re-run the template scaffold check (see `template/GUIDE.md`) and confirm the three
      scaffolds still fail with exactly their expected symbols and nothing else.
- [ ] **15.** Update `TunerConstants` for the real 2027 robot, and `FieldConstants` for the 2027
      field, per the Template Status table in [`CLAUDE.md`](../CLAUDE.md).

---

## Sources

- [WPILib 2027 changelog](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html)
- [allwpilib releases](https://github.com/wpilibsuite/allwpilib/releases)
- [Commands V3 design doc](https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/commands-v3.md)
- [SystemCoreTesting vendor compatibility matrix](https://github.com/wpilibsuite/SystemCoreTesting/blob/main/README.md)
- [The 2027 FIRST Driver Station](https://wpilib.org/blog/the-2027-first-driver-station)
