# WPILib 2027 / Commands V3 Migration

> **Status: not started, deliberately.** This repo is on WPILib 2026 (GradleRIO 2026.2.1, Java 17,
> `edu.wpi.first`, commands V2) and stays there until WPILib 2027 reaches **beta** and the team
> has Systemcore hardware. This document records the audit so the port does not have to re-derive
> it.
>
> *Audit date: 2026-09-28, against WPILib `2027.0.0-alpha-7` (tagged 2026-08-30, published 2026-09-01) and
> allwpilib `main` of 2026-09-27. Previous audit: 2026-08-26, alpha-6. Re-check the version tables
> before acting on them.*
>
> **What each command rule becomes under V3 is already written down** — see
> [commands-v3.md](commands-v3.md). This document is the platform port; that one is the doctrine.

---

## Why we are still waiting

WPILib 2027 is at **alpha-7**. No beta.

The API is still moving, and alpha-7 was the largest breaking release yet (see below). Porting onto
it means chasing renames rather than writing robot code, and every line of the port would be redone
at beta anyway.

The Commands V3 package, which moved between alpha-3 (`org.wpilib.commands3`) and alpha-5
(`org.wpilib.command3`), has **held still** since: alpha-5, alpha-6, alpha-7 and `main` all use
`org.wpilib.command3`. That was the biggest single reason to wait in August and it is now the
smallest. What is still moving is the API *inside* the package — alpha-7 turned `Mechanism` from a
class into an interface.

The second reason has not changed: **we have no Systemcore hardware.** 2027 is a control-system
change, not a library bump — the roboRIO is legal only through the 2026 season, and 2027 code
targets Systemcore. Without a unit, nothing we port can be tested beyond sim and unit tests, and
`main` would stop being deployable on the hardware the team actually has.

**Decision: build season-independent work (scaffolds, tests, docs, V3 doctrine) on 2026 now; do
the 2027 port as one clean pass at beta, once a Systemcore unit is in hand.**

---

## What the port actually involves

Not a version bump. In rough order of blast radius:

| Change | Impact here |
|---|---|
| **Systemcore replaces the roboRIO** | New deploy target, new Driver Station. `build.gradle`'s `deploy { targets { roborio ... } }` block is rewritten; so is the `roborioDebug` / `roborioRelease` dependency block and the RIO-specific JVM flags in `.claude/rules/04-build.md` |
| **Java 17 → Java 25** | `build.gradle` toolchain, `.github/workflows/build.yml` (it has a `TODO(2027)` for exactly this), every developer's JDK |
| **`edu.wpi.first` → `org.wpilib`** | Every source file. The VS Code importer attempts this automatically. Several classes also moved *within* the new namespace — geometry to a `shape` package, `Preferences` to `preferences`, the robot bases to `org.wpilib.framework` |
| **Constants renamed to `ALL_CAPS`** (alpha-7) | Every WPILib constant and enum value we reference — `Rotation2d.kZero`, `Rotation2d.kPi`, `DebounceType.kFalling`, `Alliance.Red` and the like. Mechanical, but it touches the drive, vision and every `frc/lib` file |
| **Commands V2 → V3** | 23 files import `wpilibj2.command` (15 in `src/main`, 8 in `src/test`); the scaffolds use it only through the library's `*Subsystem` and `*Commands`. See [the two-step plan](#commands-do-it-in-two-steps) below |
| **SmartDashboard and `SendableChooser` removed** (alpha-7) | Replaced by the **Telemetry** and **Tunables** APIs; the chooser equivalent is `org.wpilib.tunable.Selectable`. Touches `Robot.java` (pre-match checklist string), `FaultMonitor` (`Robot OK`, `Active Faults`), `RotaryVisualizer` and `LinearVisualizer` (`putData`), and the auto chooser — see [Auto selection](#auto-selection-opmodes-or-a-selectable) |
| **`Alert` moved to wpiutil** (alpha-7) | `org.wpilib.util.Alert`. Seven files import `edu.wpi.first.wpilibj.Alert`: `Drive`, `Module`, `Vision`, `BatteryLogger`, `CANBusMonitor`, `LoopTimeMonitor`, `PhoenixSignalLogger`. Import change only, as far as the alpha shows |
| **Integer timestamps are nanoseconds** (alpha-7) | Anything reading a raw `long` timestamp — `RobotController.getFPGATime()` in tests, odometry timestamps in `PhoenixOdometryThread`. NetworkTables and `.wpilog` files stay in microseconds, so replay of 2026 logs is unaffected. Audit every raw-timestamp arithmetic site; a factor of 1000 here compiles and runs |
| **Gamepad face buttons renamed, default deadband added** (alpha-7) | `Triggers.java` only — which is the point of `Triggers.java`. The new default deadband **stacks** with `DriveCommands`' radial deadband; see `Triggers`' comment on why per-axis deadbanding reshapes the response, and turn the default off rather than double-deadbanding |
| **AprilTag and CameraServer are vendordeps; `AprilTagFields` folded into `Fields`** (alpha-7) | `FieldConstants.aprilTagLayout` load path changes; add the AprilTag vendordep. PhotonVision depends on it too |
| **All WPILib CAN device classes take a `CANPort` enum** (alpha-7) | We use no WPILib CAN classes directly, so this is a note for anything added before the port |
| **NetworkTables v3 removed** | We are already on NT4; expected to be a no-op. Verify |
| **Phoenix device constructors changed** | Mostly already done — see below |

### Commands: do it in two steps

**2027 still ships V2**, as `org.wpilib.command2` (`commandsv2/` in allwpilib). V3 is a separate
library alongside it, not a replacement for it. That makes a two-step port possible, and it is the
recommended one:

1. **Platform port on V2.** Move to 2027 with `edu.wpi.first.wpilibj2.command` →
   `org.wpilib.command2`. Mechanical. Every test, `RobotContainerSmokeTest`, `CommandLogger`,
   `LoggedTrigger` and `ContinuousConditionalCommand` keep their current meaning, so a green suite
   actually proves the platform port.
2. **V2 → V3, as its own branch**, following [commands-v3.md](commands-v3.md). Only this step changes
   doctrine, and only this step needs the rules rewritten.

Doing both at once means a red test could be the platform or the framework, with no way to tell
which. Doing them apart means each is reviewable.

**Gate step 2 on PathPlannerLib.** `AutoBuilder`, `NamedCommands`, `EventTrigger` and every path
command are V2 `Command`s today. A V3 robot running PathPlanner's V2 commands needs either a
PathPlanner V3 build or a bridge between the two schedulers, and neither existed at this audit.

### Commands V3 is a rewrite, not a port

V3 uses **coroutines** (Java 21+ continuations), **mechanism ownership** in place of `Subsystem`
requirements, **priorities** in place of interrupt behavior, and makes command names **required at
construction**. The doctrine consequences — what happens to static factories, the mandatory
timeout, `ContinuousConditionalCommand`, `LoggedTrigger`, `CommandLogger`, default commands,
Ready → Align → Act and event triggers — are worked through in [commands-v3.md](commands-v3.md).

**One name collision to settle before step 2:** `frc.lib.mechanism.Mechanism` (the motor-mechanism
base class) and `org.wpilib.command3.Mechanism` (the V3 ownership interface) are both called
`Mechanism`, and under V3 our `*Subsystem` classes will implement the WPILib one. Pick the rename
once — the options are in commands-v3.md — rather than fully qualifying names everywhere.

### Auto selection: OpModes or a `Selectable`

2027 introduces **OpModes** — operator-selectable robot programs chosen on the Driver Station
itself — and removes `SendableChooser`. Today's auto chooser is a `LoggedDashboardChooser` fed by
`AutoBuilder.buildAutoChooser()`, and `RobotContainerSmokeTest` asserts it yields a command.

Two ways forward, decide once:

- **`Selectable`** — the closest analog to what exists. Needs AdvantageKit's equivalent of
  `LoggedDashboardChooser` so the selection still reaches the log; the log is how a post-match
  review knows which auto ran.
- **OpModes** — autos selected at the DS, no dashboard. Changes how the drive team works and how
  PathPlanner's chooser is built, so it waits on PathPlannerLib's 2027 story.

---

## Vendor readiness — the real gate

As of the audit date. The matrix lives in
[SystemCoreTesting](https://github.com/wpilibsuite/SystemCoreTesting/blob/main/README.md); the
PhotonVision and AdvantageKit rows were checked against their own tags.

| Vendor | alpha-5/6 | alpha-7 | Notes |
|---|---|---|---|
| CTRE Phoenix 6 | `26.50.0-alpha-1` | ✅ `26.70.0-alpha-2` | Breaking API changes, see below |
| AdvantageKit | `27.0.0-alpha-4` | ✅ `27.0.0-alpha-5` / `-6` | **Alert logging is back** — `AlertLogger` reads `org.wpilib.util.AlertDataJNI` in alpha-6 |
| PhotonVision | — | ⚠️ `v2027.0.0-alpha-2` | A real tagged release now, built against WPILib alpha-6. Not in the matrix; confirm against alpha-7 before relying on it |
| **PathPlannerLib** | `2027.0.0-alpha-3` | ❌ **not available** | Went backwards: there was an alpha-5/6 build and there is no alpha-7 one. Author is holding major changes for Systemcore |
| Studica (NavX) | ? | ? | Not in the matrix. Only `GyroIONavX` uses it; the robot runs a Pigeon2, so this gates nothing unless that changes |
| REVLib, ReduxLib, ChoreoLib | — | available | Not used here |

**Three of the August audit's gates, rechecked:**

1. *Commands V3 package settled* — **yes**, `org.wpilib.command3` since alpha-5.
2. *PhotonVision ships a real 2027 vendordep* — **mostly**. `v2027.0.0-alpha-2` exists; it
   trails WPILib by one alpha. Vision is one of the two things this template exists to carry
   forward, so check it first at beta.
3. *AdvantageKit alert logging restored* — **yes**, in `27.0.0-alpha-6`. `FaultMonitor`, the
   loop-timing monitor and the pose-divergence detector reach the pit through alerts; that path
   survives the port.

**And one new one: PathPlannerLib.** Without it there are no autos. It is now the vendor most
likely to gate the port.

---

## Phoenix 6: mostly already handled

Phoenix `26.50.0-alpha-1` **removed** the device constructors taking a CAN bus *string*, and those
taking a device ID with no bus at all.

This codebase is already clean on that point — a deliberate earlier decision, not luck:

- `Constants.CanIds.RIO_BUS` is a `CANBus` object, not `"rio"`.
- `ModuleIOTalonFX` and `GyroIOPigeon2` take `TunerConstants.kCANBus`.
- `MotorIOTalonFX` builds every mechanism's devices from the `CANBus` carried in its `MotorConfig`,
  and the three scaffolds pass `Constants.CanIds.RIO_BUS`.

**What does change: the bus model.** Systemcore has **five native CAN buses**, constructed with
`CANBus.systemcore(int)` as of `26.50`. There is no `"rio"` bus. That makes the bus-assignment table
in [`.claude/rules/02-hardware.md`](../.claude/rules/02-hardware.md) wrong as written — the
`"Canivore"` vs `"rio"` split has to be re-expressed against the new bus numbering. CANivores still
work. Re-check the `CANBus` factory against `26.70`, since WPILib's own CAN classes moved to a
`CANPort` enum in alpha-7 and Phoenix may follow.

Rename `RIO_BUS` when that happens. A constant named for hardware that no longer exists is exactly
the kind of lie this codebase tries not to carry. `CanIdUniquenessTest` keys on `(bus, id)`, so it
needs the new bus identity too.

---

## Concrete checklist for the beta port

Do these in order. Steps 1–4 are gates; stop if one fails.

- [ ] **1.** Confirm WPILib 2027 is at beta, and diff the Commands V3 API against
      [commands-v3.md](commands-v3.md) — the package has settled, the API inside it has not.
- [ ] **2.** Confirm a Systemcore unit is available to test on.
- [ ] **3.** Confirm PathPlannerLib ships a 2027 build for that WPILib version.
- [ ] **4.** Confirm PhotonVision ships a 2027 vendordep for that WPILib version, and that
      AdvantageKit's alert logging is still present.
- [ ] **5.** Branch. Never do this on `main` while the team needs a deployable robot.
- [ ] **6.** `build.gradle`: GradleRIO 2027, Java 25 toolchain, Systemcore deploy target and
      dependency configurations. Revisit the JVM flags in `.claude/rules/04-build.md` — they
      are sized for a RIO 2.
- [ ] **7.** Update the CI workflow's JDK. It runs `spotlessCheck` then `build` — do not weaken it.
- [ ] **8.** Pull every 2027 vendordep, including the new AprilTag vendordep.
- [ ] **9.** `edu.wpi.first` → `org.wpilib` across `src/` **and** `template/`, commands V2 to
      `org.wpilib.command2`, `ALL_CAPS` constants. Mechanical.
- [ ] **10.** Audit every raw integer timestamp for the microsecond → nanosecond change.
- [ ] **11.** Replace SmartDashboard in `FaultMonitor`, `Robot`, `RotaryVisualizer`,
      `LinearVisualizer` with the Telemetry API; decide `Selectable` vs OpModes for autos. Decide
      each replacement once and apply it everywhere.
- [ ] **12.** `Triggers.java`: new gamepad button names; disable the new default deadband.
- [ ] **13.** Re-express the CAN bus assignment rules for Systemcore; rename `RIO_BUS`.
- [ ] **14.** Get the test suite green **on V2**. This is the checkpoint that proves the platform
      port on its own. Merge it.
- [ ] **15.** New branch: Commands V2 → V3, following [commands-v3.md](commands-v3.md). Rewrite
      `.claude/rules/03-commands.md` and the command parts of `01-architecture.md` in the same pass —
      a rule that lies is worse than no rule. Settle the `Mechanism` name collision first.
- [ ] **16.** Re-derive `ContinuousConditionalCommand`, `LoggedTrigger`, `CommandLogger` per
      commands-v3.md, and rewrite `RobotContainerSmokeTest` and every sim test's scheduler cleanup
      for the V3 `Scheduler`.
- [ ] **17.** Re-run the template scaffold check (see `template/GUIDE.md`) and confirm the three
      scaffolds still fail with exactly their expected symbols and nothing else.
- [ ] **18.** Update `TunerConstants` for the real 2027 robot, and `FieldConstants` for the 2027
      field, per the Template Status table in [`CLAUDE.md`](../CLAUDE.md).
- [ ] **19.** Update every doc that names a 2026 API: `docs/robot-spec.md` §2, the
      `.claude/rules/`, the scaffolds' javadoc, and this file's status line.

---

## Sources

- [allwpilib releases](https://github.com/wpilibsuite/allwpilib/releases) — alpha-7 release notes
- [WPILib 2027 changelog](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html)
- [Commands V3 design doc](https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/commands-v3.md)
  and [state machines](https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/commands-v3-state-machines.md)
- [OpModes design doc](https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/opmodes.md)
- [Telemetry migration from 2026](https://github.com/wpilibsuite/allwpilib/blob/main/telemetry/doc/telemetry.md#migration-from-wpilib-2026)
  and [Tunables migration from 2026](https://github.com/wpilibsuite/allwpilib/blob/main/tunables/doc/tunables.md#migration-from-wpilib-2026)
- [SystemCoreTesting vendor compatibility matrix](https://github.com/wpilibsuite/SystemCoreTesting/blob/main/README.md)
- [The 2027 FIRST Driver Station](https://wpilib.org/blog/the-2027-first-driver-station)
