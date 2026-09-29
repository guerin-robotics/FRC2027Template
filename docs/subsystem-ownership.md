# Subsystem Ownership

This document defines which files belong to each subsystem and the rules for
modifying them.

> **Branch `feature/rebuilt2026-port`:** Drive, Vision, the shared layers, and the eight
> 2026 mechanisms below are accurate for this branch.

---

## Ownership Model

Each subsystem is a vertical slice. A change to one subsystem should **never require
editing another subsystem's files.**

If a fix requires touching two subsystems, that is a design smell. Surface it and fix the
coupling rather than patching across boundaries.

---

## Drive

**Owner files:**

```
subsystems/drive/Drive.java
subsystems/drive/DriveConstants.java
subsystems/drive/Module.java
subsystems/drive/ModuleIO.java
subsystems/drive/ModuleIOTalonFX.java
subsystems/drive/ModuleIOTalonFXS.java
subsystems/drive/ModuleIOSim.java
subsystems/drive/GyroIO.java
subsystems/drive/GyroIOPigeon2.java
subsystems/drive/GyroIONavX.java
subsystems/drive/PhoenixOdometryThread.java
commands/DriveCommands.java
generated/TunerConstants.java
```

**Permitted cross-boundary interactions:**

- `Drive` calls `RobotState.getInstance().setPoseSupplier()` at construction — the designed wiring point
- `Vision` calls `drive.addVisionMeasurement()` via a consumer passed at construction
- `Drive.configureAutoBuilder()` wires PathPlanner; `RobotContainer` calls it once, before `AutoBuilder.buildAutoChooser()`
- `Drive.periodic()` calls `Robot.batteryLogger.reportCurrentUsage()` per module

**High-risk files:** all of them. Drive changes are Level 3+ (see
`change-classification.md`).

**Single source of truth:** `TunerConstants.java` owns module positions, gear ratios,
wheel radius, CAN IDs, gains and current limits. `DriveConstants` holds only what Tuner X
does not generate (joystick shaping, limited speed). Do not reintroduce a duplicate
module layout — the 2026 codebase had one and it had to be hand-synced.

---

## Vision

**Owner files:**

```
subsystems/vision/Vision.java
subsystems/vision/VisionConstants.java
subsystems/vision/io/VisionIO.java
subsystems/vision/io/VisionIOPhotonVision.java
subsystems/vision/io/VisionIOPhotonVisionSim.java
```

**Permitted cross-boundary interactions:**

- `Vision` calls a `VisionConsumer` passed at construction (wired to
  `drive::addVisionMeasurement` in `RobotContainer`)
- `Vision` reads `RobotState.getInstance().getFieldRelativeVelocity()` for the angular
  velocity rejection filter

**Known gaps carried from 2026:**

- Pose-estimator divergence is **detected** (`Odometry/OffField`, `PoseJumpCount`, an Alert)
  but not **guarded**. The rejection filters in `VisionConstants` are the only defense against
  a bad estimate persisting.
- The `maxPoseJumpMeters` filter was written and left disabled because it was never
  tuned against real logs. Either tune and enable it, or add an explicit field-bounds
  sanity check.
- Camera count is wired in **three** places in `RobotContainer` (REAL, SIM, REPLAY).
  Changing it in one branch and not the others breaks replay silently.

---

## 2026 Mechanisms (branch `feature/rebuilt2026-port`)

The eight 2026 mechanisms, each on `frc/lib/mechanism/`. The IO layer is not per-mechanism —
`MotorIO`, `MotorIOTalonFX` and `MotorIOTalonFXSim` are shared, so each mechanism owns only its
subsystem, its constants, and (where it has verbs of its own) a commands file.

| Mechanism | Kind | Owner files |
|---|---|---|
| Flywheel | Roller | `subsystems/flywheel/Flywheel.java`, `FlywheelConstants.java`, `ShotCalculator.java`, `FlywheelVisualizer.java`; `commands/FlywheelCommands.java` |
| Hood | Rotary | `subsystems/hood/Hood.java`, `HoodConstants.java`, `HoodPosCalculator.java`; `commands/HoodCommands.java` |
| Prestage | Roller | `subsystems/prestage/Prestage.java`, `PrestageConstants.java`; `commands/PrestageCommands.java` |
| Upper Feeder | Roller | `subsystems/upperFeeder/UpperFeeder.java`, `UpperFeederConstants.java`; `commands/FeederCommands.java` (shared) |
| Lower Feeder | Roller | `subsystems/lowerFeeder/LowerFeeder.java`, `LowerFeederConstants.java`; `commands/FeederCommands.java` (shared) |
| Transport | Roller | `subsystems/transport/Transport.java`, `TransportConstants.java`; `commands/TransportCommands.java` |
| Intake Roller | Roller | `subsystems/intakeRoller/IntakeRoller.java`, `IntakeRollerConstants.java`; `commands/IntakeRollerCommands.java` |
| Intake Pivot | Rotary | `subsystems/intakePivot/IntakePivot.java`, `IntakePivotConstants.java`; `commands/IntakePivotCommands.java` |

**Permitted cross-boundary interactions:**

- `Flywheel` takes a `Supplier<Angle>` for the hood angle (wired to `hood::getPosition` in
  `RobotContainer`), used only by the trajectory visualizer. It never holds a `Hood`.
- `Flywheel`, `Hood`, `ShotCalculator` and `HoodPosCalculator` read targets and distances from
  `RobotState`.
- `Triggers.isFlywheelSpunUp` reads `flywheel::isSpunUp` through a `BooleanSupplier` handed
  over once by `RobotContainer`.
- `RobotModelVisualizer` takes three `Supplier<Angle>`s (pivot, hood, flywheel), never the
  subsystems.

The per-mechanism command files keep 2026 semantics rather than using the library's
`RollerCommands` / `RotaryCommands` (velocity setters are `runOnce`; "stop" is a zero-velocity
setpoint on most mechanisms) because the 2026 bindings depend on both.

---

## Integration Layer (not owned by any single subsystem)

These files wire subsystems together. They are the only place cross-subsystem logic is
permitted.

```
RobotContainer.java   — subsystem wiring + command binding
RobotState.java       — shared state and field geometry (singleton)
Triggers.java         — all button and state trigger objects; owns the controllers
commands/ShootSequences.java, commands/SpitSequences.java
                      — composed multi-subsystem pipelines (the 2026 ones)
HubShiftUtil.java     — 2026 hub-shift schedule (static)
ShotModes.java        — 2026 shot-tuning / demo flags, FMS-guarded
AutoPreview.java      — pre-match auto path preview and start-pose check
RobotModelVisualizer.java — AdvantageScope 3D component poses
```

**Rule for `RobotContainer`:** wiring only. If you find yourself writing if-statements or
game logic inside `RobotContainer`, it belongs in a command factory or `RobotState`.

**Rule for `RobotState`:** anything annotated `@AutoLogOutput` runs every loop whether or
not your code reads it. Delete the annotation when a method loses its last caller.

---

## Utility (shared, no owner)

All shared utilities live in `frc/lib/` — the general-purpose helpers under
`frc/lib/util/`, the mechanism library under `frc/lib/mechanism/`. There is no
`frc/robot/util/` — utilities either belong to every robot, in which case they go here, or
they belong to a subsystem, in which case they go in that subsystem's package.

```
frc/lib/util/
├── Field and alliance
│   ├── AllianceFlipUtil.java        alliance mirroring, cached per loop
│   ├── FieldConstants.java          ← game-specific; extend each season
│   ├── GeomUtil.java                Pose/Transform/Twist conversions
│   └── PointInPolygon.java          zone containment
├── Commands and triggers
│   ├── ContinuousConditionalCommand.java
│   ├── LoggedTrigger.java
│   ├── EdgeDetector.java            rising/falling/count-in-window
│   └── CommandLogger.java           which commands are running
├── Hardware
│   ├── PhoenixUtil.java             tryUntilOk config retry
│   ├── PhoenixSignalLogger.java     CTRE hoot logging
│   ├── CANBusMonitor.java           bus utilization and errors
│   ├── MotorSpecs.java              motor free speeds and sim gearboxes, from DCMotor
│   └── ThrowingRunnable.java
├── Health and diagnostics
│   ├── BatteryLogger.java           power accounting, brownout counting
│   ├── FaultMonitor.java            one "Robot OK" from many conditions
│   ├── LoopTimeMonitor.java         loop duration and overrun alert
│   └── MatchMetadataLogger.java     event and match identity
├── Tuning
│   ├── LoggedTunableNumber.java     dashboard-adjustable, gated on tuningMode
│   ├── LoggedTunableBoolean.java
│   └── LoggedTunableProfiledPID.java  tunable ProfiledPIDController gains and constraints
└── Misc
    ├── Elastic.java                 dashboard notifications and tabs
    └── LocalADStarAK.java           replay-safe PathPlanner pathfinder
```

Everything here is season-agnostic except `FieldConstants`, which holds the field
dimensions and AprilTag layout and gains game geometry each year.

Four classes read `frc.robot.Constants` — `AllianceFlipUtil`, the two tunables, and
`PhoenixSignalLogger` — but only for global mode flags (`disableHAL`, `tuningMode`,
`currentMode`), never for robot structure. If you lift this package into another project,
those flags are the only thing you need to supply.

**One known exception:** `frc/lib/mechanism/Mechanism.java` imports `frc.robot.Robot` to reach
the static `Robot.batteryLogger`. It is the only thing tying the mechanism library to this
robot's `Robot` class; injecting the `BatteryLogger` instead would remove it. Worth doing at the
2027 port, when `Robot` is rewritten anyway.

Keep it that way otherwise: `frc.lib` may depend on `frc.robot.Constants` and nothing else under
`frc.robot`. Nothing checks this automatically, so a new `frc.lib` class reaching into a
subsystem or `RobotState` will compile happily — catch it in review.
