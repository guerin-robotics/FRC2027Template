# WPILib 2027 / Commands V3 Migration

> **Status: ported on branch `feature/2027-v3`; desktop-verified only.** The code is on WPILib
> **2027.0.0-alpha-8** (GradleRIO `org.wpilib.GradleRIO`), Gradle 9.4.1, Java 25, `org.wpilib`
> imports, Commands **V3**, and a Systemcore deploy target. `./gradlew build` (format check,
> compile, 28 tests) passes on a desktop. **Nothing has run on Systemcore hardware, nothing has run
> against a real CAN bus, and CI has not run.** This document keeps the audit and records what the
> port actually found.
>
> *Audit dates: 2026-09-28 against alpha-7; the port itself on 2026-11-25 against alpha-8, reading
> the jars in the local WPILib install and Gradle cache.*
>
> **What each command rule became under V3** is in [commands-v3.md](commands-v3.md), and
> `.claude/rules/03-commands.md` was rewritten from it.

---

## How it was done — and where it departed from the plan below

This file originally recommended two steps: port the platform on V2, get a green suite, then do
V2 → V3 on its own branch, so a red test could be blamed on one or the other. **The team chose one
combined branch** because the 2027 robot will run V3 from the start and there is no 2026 robot to
keep deployable. The cost is the one the plan predicted: a failure could have been either step.
In practice most breakage showed up at compile time, where the compiler names it, and the few that
did not are listed under *Silent breaks* below.

Four things in the original gates were resolved by decision, not by evidence:

- **Beta.** WPILib 2027 is still alpha (alpha-8 is the newest release found). The port targets the
  alpha knowingly; everything here must be re-checked at beta.
- **Systemcore hardware.** None is available to this repo yet. The deploy target is configured; it
  has not been deployed.
- **Auto selection.** Decided: **OpModes**. Autonomous routines and characterization are OpModes
  registered in `frc.robot.opmodes.OpModes`; there is no dashboard chooser. AdvantageKit 27 has no
  `LoggedDashboardChooser`, and the OpMode selection is logged by `LoggedOpModeRobot`.
- **maple-sim.** It has no 2027 build (the previous dependency was `v0.4.0-beta` for WPILib 2026), so the maple-sim
  drivetrain was moved to `template/deferred/maple-sim/` with a README on restoring it. Simulation
  uses the per-module `ModuleIOSim`. NavX was dropped (the robot uses a Pigeon2).

---

## AdvantageKit Drive and Vision templates

AdvantageKit's 2027 alpha ships a skeleton project. The official 2027 TalonFX(S) swerve
and Vision templates are pending. This branch ports the team's existing Tuner X module
IO, Phoenix odometry, PhotonVision IO and vision filters to the alpha stack. The existing
swerve values are for the 2026 chassis and require a fresh Tuner X export for the 2027 robot.
Compare both subsystems again when AdvantageKit publishes the 2027 templates.
[AdvantageKit installation](https://docs.advantagekit.org/getting-started/installation/) ·
[Systemcore alpha notes](https://github.com/wpilibsuite/SystemCoreTesting/blob/main/AdvantageKit.md)

---

## Silent breaks — compiled, would have run, and been wrong

These are the findings worth knowing about. Each was caught by reading a source jar, not by the
compiler.

| What | Why it was silent | Fix |
|---|---|---|
| AdvantageKit `Logger.getTimestamp()` is **nanoseconds** in 27 | `BatteryLogger` divided by `1.0e6`, so every loop time came out 1000× too large | `/ 1.0e9` |
| PhotonVision metadata timestamps are **nanoseconds** | `latencySeconds` was off by 1000×. The old field names no longer exist, so this one did fail to compile | `publishTimestampNanos`, `/ 1e9` |
| `SwerveModuleVelocity.optimize` / `cosineScale` **return new objects** (2026 mutated in place) | Ignoring the result drops the optimization. The `@NoDiscard` check caught it | `Module.runSetpoint` returns the optimized state and `Drive` logs that |
| `Translation2d.getAngle()` returns `Optional<Rotation2d>` | A zero vector has no angle; 2026 returned 0 | `.orElse(Rotation2d.ZERO)` |
| `DCMotor` has no `createDCMotorSystem` factory | Sim only | `Models.singleJointedArmFromPhysicalConstants`, which is the same two-state matrices |
| `TalonFX.setNeutralMode` became `configNeutralMode` | It is now a config write that may block on CAN for its timeout | Called only on enable and the post-disable coast, never per loop (comment in `ModuleIOTalonFX`) |
| `RobotController.getInputCurrent` is gone | Systemcore exposes none | `Controls/roboRIO` logs 0 A; see CLAUDE.md |
| `PubSubOption.sendAll(true)` etc. became constants | Compile error | `SEND_ALL`, `KEEP_DUPLICATES` |
| `Alert` takes a group, a unique id and a `Level` (`HIGH`/`MEDIUM`/`LOW`) | A duplicate id throws `AlertException` at construction | The text is used as the id; `Vision` is `AutoCloseable` so tests can rebuild it |
| `ChassisSpeeds` → `ChassisVelocities` (fields `vx`/`vy`/`omega`), `SwerveModuleState` → `SwerveModuleVelocity`, `fromFieldRelativeSpeeds` → instance `toRobotRelative` | Compile errors, but they touch the whole drive path | Renamed throughout; no value changed |

Handled but not verified on hardware: BLine's follower, `Drive`, Phoenix's 26.70 swerve and
odometry code, and the interaction of the 2027 `Telemetry` backend with `AutoPreview` (the smoke
test prints `no backend for path` because it has none configured).

---

## What the port actually involves

*(Kept from the audit. Every row below is done unless the *Open* list further down says otherwise.)*

Not a version bump. In rough order of blast radius:

| Change | Impact here |
|---|---|
| **Systemcore replaces the roboRIO** | New deploy target, new Driver Station. `build.gradle`'s `deploy { targets { roborio ... } }` block is rewritten; so is the `roborioDebug` / `roborioRelease` dependency block and the RIO-specific JVM flags in `.claude/rules/04-build.md` |
| **Java 17 → Java 25** | `build.gradle` toolchain, `.github/workflows/build.yml` (it has a `TODO(2027)` for exactly this), every developer's JDK |
| **`edu.wpi.first` → `org.wpilib`** | Every source file. The VS Code importer attempts this automatically. Several classes also moved *within* the new namespace — geometry to a `shape` package, `Preferences` to `preferences`, the robot bases to `org.wpilib.framework` |
| **Constants renamed to `ALL_CAPS`** (alpha-7) | Every WPILib constant and enum value we reference — `Rotation2d.kZero`, `Rotation2d.kPi`, `DebounceType.kFalling`, `Alliance.Red` and the like. Mechanical, but it touches the drive, vision and every `frc/lib` file |
| **Commands V2 → V3** | 15 files import `wpilibj2.command` (14 in `src/main`, 1 in `src/test`), plus 6 in the `template/` scaffolds — each mechanism subsystem and its `*Commands` class. See [the two-step plan](#commands-do-it-in-two-steps) below |
| **SmartDashboard and `SendableChooser` removed** (alpha-7) | Replaced by the **Telemetry** and **Tunables** APIs; the chooser equivalent is `org.wpilib.tunable.Selectable`. Touches `Robot.java` (pre-match checklist string), `FaultMonitor` (`Robot OK`, `Active Faults`), the scaffold visualizers `ExampleArmVisualizer` and `ExampleLiftVisualizer` (`putData`), and the auto chooser — see [Auto selection](#auto-selection-opmodes-or-a-selectable) |
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

**Gate step 2 on BLine-Lib.** `FollowPath.Builder` commands and event-trigger registrations are V2
`Command`s today. BLine-Lib's current release (`v0.9.3`, vendordep `frcYear` 2026) is the 2026 build; the 2027 beta (`2027.0.0-beta.3`) is built against
WPILib alpha-7's V2 package and has not been checked against V3. A V3 robot needs a BLine-Lib V3 build or a bridge
between the two schedulers. (This gate was PathPlannerLib's, then ChoreoLib's, before the switch to
BLine.)

### Commands V3 is a rewrite, not a port

V3 uses **coroutines** (Java 21+ continuations), **mechanism ownership** in place of `Subsystem`
requirements, **priorities** in place of interrupt behavior, and makes command names **required at
construction**. The doctrine consequences — what happens to static factories, the mandatory
timeout, `ContinuousConditionalCommand`, `LoggedTrigger`, `CommandLogger`, default commands,
Ready → Align → Act and event triggers — are worked through in [commands-v3.md](commands-v3.md).

**Watch the word `Mechanism`.** Under V3 every mechanism subsystem implements
`org.wpilib.command3.Mechanism`. The scaffolds use "mechanism" only in prose and in names like
`ExampleArm`, so there is no class collision today — keep it that way rather than naming a 2027
class `Mechanism`.

### Auto selection: OpModes or a `Selectable`

2027 introduces **OpModes** — operator-selectable robot programs chosen on the Driver Station
itself — and removes `SendableChooser`. Today's auto chooser is a
`LoggedDashboardChooser<Supplier<Command>>` of BLine routines and
characterization routines, and `RobotContainerSmokeTest` asserts it yields a command.

Two ways forward, decide once:

- **`Selectable`** — the closest analog to what exists. Needs AdvantageKit's equivalent of
  `LoggedDashboardChooser` so the selection still reaches the log; the log is how a post-match
  review knows which auto ran.
- **OpModes** — autos selected at the DS, no dashboard. Changes how the drive team works. BLine-Lib
  has no OpMode chooser, so the selection and its way into the AdvantageKit log would be ours to
  build.

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
| **BLine-Lib** | — | ⚠️ `2027.0.0-beta.3` | Our path library since the switch from Choreo. Beta.3 builds against WPILib `2027.0.0-alpha-7` and Java 25, so it cannot go in until the robot code is on 2027 GradleRIO; we stay on `v0.9.3` (2026) until then. Not checked: its V2/V3 status |
| Grapple (LaserCAN) | ? | ? | Not checked. Only `frc/lib/device` LaserCAN IOs use it; nothing gates on it until a robot wires one |
| Studica (NavX) | ? | ? | Not in the matrix. Only `GyroIONavX` uses it; the robot runs a Pigeon2, so this gates nothing unless that changes |
| REVLib, ReduxLib | — | available | Not used here. PathPlannerLib is no longer used either |

**Three of the August audit's gates, rechecked:**

1. *Commands V3 package settled* — **yes**, `org.wpilib.command3` since alpha-5.
2. *PhotonVision ships a real 2027 vendordep* — **mostly**. `v2027.0.0-alpha-2` exists; it
   trails WPILib by one alpha. Vision is one of the two things this template exists to carry
   forward, so check it first at beta.
3. *AdvantageKit alert logging restored* — **yes**, in `27.0.0-alpha-6`. `FaultMonitor`, the
   loop-timing monitor and the pose-divergence detector reach the pit through alerts; that path
   survives the port.

**And one new one: BLine-Lib.** Without it there are no path autos. Its 2027 beta exists (`2027.0.0-beta.3`) but is not in the build until the WPILib port; V3 is unchecked, so this gate is open.

---

## Phoenix 6: mostly already handled

Phoenix `26.50.0-alpha-1` **removed** the device constructors taking a CAN bus *string*, and those
taking a device ID with no bus at all.

This codebase is already clean on that point — a deliberate earlier decision, not luck:

- `Constants.CanIds.RIO_BUS` is a `CANBus` object, not `"rio"`.
- `ModuleIOTalonFX` and `GyroIOPigeon2` take `TunerConstants.kCANBus`.
- The three scaffold `*IOReal`s construct and claim their devices on `Constants.CanIds.RIO_BUS`.

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

## Checklist — status

Done on `feature/2027-v3`:

- [x] **5.** Branched (`feature/2027-v3`); `main` is untouched.
- [x] **6.** `build.gradle`: GradleRIO 2027 alpha-8, Java 25, Systemcore deploy target and
      dependency configurations, `application` plugin. Gradle wrapper 8.11 → 9.4.1.
- [x] **7.** CI workflow bumped to JDK 25. **Not run.**
- [x] **8.** 2027 vendordeps: AdvantageKit `27.0.0-alpha-7`, Phoenix 6 `26.70.0-alpha-2`, PhotonVision
      (a dev snapshot built for alpha-7, not a tagged release), BLine-Lib `2027.0.0-beta.3`, Grapple
      `27.0.0`, plus the WPILib `CommandsV3`, `AprilTag` and `CameraServer` vendordeps.
- [x] **9.** `edu.wpi.first` → `org.wpilib` across `src/` and `template/`; commands to V3; constants
      to `ALL_CAPS`.
- [x] **10.** Raw timestamps audited (see *Silent breaks*).
- [x] **11.** SmartDashboard replaced with `Telemetry.log` in `FaultMonitor`, `Robot`, `AutoPreview`
      and both scaffold visualizers; auto selection is OpModes.
- [x] **12.** `Triggers.java`: Start is `menu()`; flight-stick buttons go through `getHID()`. The
      default Gamepad deadband does not apply — the controllers used here are the flight stick and an
      Xbox, and `DriveCommands` still applies its own radial deadband, with no per-axis deadband
      added in `Triggers`.
- [x] **13.** CAN bus rules re-expressed for Systemcore; `RIO_BUS` replaced by `MECHANISM_BUS_A/B`
      (placeholders) — see `.claude/rules/02-hardware.md`.
- [x] **14–15.** Combined: V3 port done with the platform. `03-commands.md` and the command parts of
      `01-architecture.md` rewritten in the same change.
- [x] **16.** `LoggedTrigger` ported, `CommandLogger` rewritten, `ContinuousConditionalCommand`
      deleted, `RobotContainerSmokeTest` rewritten for the V3 `Scheduler`.
- [x] **17.** Scaffold check: the three `template/` scaffolds fail to compile on exactly the values
      they owe and nothing else (checked with `javac -proc:none` against the main classpath).
- [x] **19.** Docs updated (this file, `commands-v3.md`, `robot-spec.md`, rules, skills, the
      checklists).

**Open — needs hardware, a decision, or a release:**

- [ ] **Run it on Systemcore.** Deploy, enable, drive on blocks, check `LoopTiming/` and CAN
      utilisation. Until then every statement in this repo about 2027 behavior is from source and
      desktop tests.
- [ ] **Regenerate `TunerConstants` with Tuner X 2027** for the real robot. The file was edited only
      enough to compile (`new CANBus("Canivore")`, `.withNetwork(kCANBus)`); its values are 2026's.
- [ ] **JVM heap and GC flags** — dropped for Systemcore; measure, then set.
- [ ] **Decide which devices go on which Systemcore bus** and update `Constants.CanIds`,
      `docs/hardware-layout.md` and `CanIdUniquenessTest` (which treats all mechanism IDs as one bus).
- [ ] **Re-diff V3 and BLine at beta.** BLine `2027.0.0-beta.3` was built against alpha-7 and has not
      been run on alpha-8. PhotonVision is a dev snapshot.
- [ ] **maple-sim**: add back when it ships a 2027 build (`template/deferred/maple-sim/README.md`).
- [ ] **CI**: first run will likely need the alpha maven artifacts made resolvable (see the comment
      in `build.yml`).
- [ ] **Telemetry backend**: confirm the `Auto Preview` and `Visualizer` widgets actually appear on a
      dashboard; `Telemetry.log` has no backend in the unit tests.
- [ ] **Controller input current** for `BatteryLogger`: Systemcore exposes none.
- [ ] **FieldConstants / AprilTag layout** for the 2027 game when WPILib ships it, and PhotonVision's
      pose estimator against that `Field` type.

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
