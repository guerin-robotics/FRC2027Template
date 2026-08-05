# Subsystem Ownership

This document defines which files belong to each subsystem and the rules for
modifying them.

> **NEEDS EXTENDING FOR THE 2027 ROBOT.** Drive, Vision, and the shared layers below are
> accurate as written. Every 2027 mechanism needs a section added following the same
> shape — do it when the subsystem is created, not later.

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
- `AutoBuilder.configure()` in `Drive.java` wires PathPlanner — the correct location
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

- The pose estimator has **no off-field divergence guard**. The rejection filters in
  `VisionConstants` are the only defense against a bad estimate persisting.
- The `maxPoseJumpMeters` filter was written and left disabled because it was never
  tuned against real logs. Either tune and enable it, or add an explicit field-bounds
  sanity check.
- Camera count is wired in **three** places in `RobotContainer` (REAL, SIM, REPLAY).
  Changing it in one branch and not the others breaks replay silently.

---

## [Your 2027 Mechanisms]

Add one section per mechanism, following this shape:

```
subsystems/myMechanism/MyMechanism.java
subsystems/myMechanism/MyMechanismConstants.java
subsystems/myMechanism/io/MyMechanismIO.java (+ Real, Sim)
commands/MyMechanismCommands.java
```

**Permitted cross-boundary interactions:** list them explicitly. If a mechanism needs a
value from another subsystem, it goes through `RobotState` or a `Supplier` passed at
construction — never a direct reference. The 2026 flywheel took a `hoodAngleSupplier`
rather than a `Hood` reference, and that is the pattern to copy.

---

## Integration Layer (not owned by any single subsystem)

These files wire subsystems together. They are the only place cross-subsystem logic is
permitted.

```
RobotContainer.java   — subsystem wiring + command binding
RobotState.java       — shared state and field geometry (singleton)
Triggers.java         — shared button/state trigger objects (create when needed)
[Sequences].java      — composed multi-subsystem pipelines (create when needed)
```

**Rule for `RobotContainer`:** wiring only. If you find yourself writing if-statements or
game logic inside `RobotContainer`, it belongs in a command factory or `RobotState`.

**Rule for `RobotState`:** anything annotated `@AutoLogOutput` runs every loop whether or
not your code reads it. Delete the annotation when a method loses its last caller.

---

## Utility (shared, no owner)

All shared utilities live in `frc/lib/`. There is no `frc/robot/util/` — utilities either
belong to every robot, in which case they go here, or they belong to a subsystem, in which
case they go in that subsystem's package.

```
frc/lib/
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
│   ├── CANUpdateThread.java         async CAN config with retry
│   └── ThrowingRunnable.java
├── Health and diagnostics
│   ├── BatteryLogger.java           power accounting, brownout counting
│   ├── FaultMonitor.java            one "Robot OK" from many conditions
│   ├── LoopTimeMonitor.java         loop duration and overrun alert
│   └── MatchMetadataLogger.java     event and match identity
├── Tuning
│   ├── LoggedTunableNumber.java     dashboard-adjustable, gated on tuningMode
│   └── LoggedTunableBoolean.java
└── Misc
    ├── Elastic.java                 dashboard notifications and tabs
    └── LocalADStarAK.java           replay-safe PathPlanner pathfinder
```

Everything here is season-agnostic except `FieldConstants`, which holds the field
dimensions and AprilTag layout and gains game geometry each year.

Three classes read `frc.robot.Constants` — `AllianceFlipUtil`, the two tunables, and
`PhoenixSignalLogger` — but only for global mode flags (`disableHAL`, `tuningMode`,
`currentMode`), never for robot structure. If you lift this package into another project,
those four flags are the only thing you need to supply.
