# Guerin Robotics — Robot Specification

> **BRANCH `feature/rebuilt2026-port`: THIS DESCRIBES THE 2026 ROBOT ON THE 2027 TEMPLATE.**
>
> This branch rebuilds the Rebuilt2026 competition code (its `main`, August 2026) on this
> template: the eight mechanisms on `frc/lib/mechanism`, the 2026 game logic, bindings and
> autos. It exists to show what the template does and does not carry, and it is never merged
> to `main` — on `main` this file is a 2027 skeleton.
>
> Every difference from how Rebuilt2026 behaved is listed in
> [rebuilt2026-port.md](rebuilt2026-port.md). Read that before trusting this robot on the floor.

**Status:** 2026 robot, ported — shop testing only, not for events
**Last updated:** 2026-09-29

---

## 1. Game Context

The 2026 FRC game **"Rebuilt"**: each alliance scores by launching fuel (balls) into its hub.
The robot intakes fuel off the floor, carries it through a hopper (transport, lower and upper
feeders, prestage) and launches it from a five-motor flywheel under an adjustable hood. The
shooter faces the **rear** of the robot.

**Hub shift.** The hub alternates between the alliances on a fixed 140 s teleop schedule, and
scoring only counts in your active window. The FMS game-specific message's first character
decides who starts active, inverted in code (`'R'` → blue starts active). `HubShiftUtil` owns
the schedule:

| Window | Time into teleop |
|---|---|
| TRANSITION | 0 – 10 s |
| SHIFT1 | 10 – 35 s |
| SHIFT2 | 35 – 60 s |
| SHIFT3 | 60 – 85 s |
| SHIFT4 | 85 – 110 s |
| ENDGAME | 110 – 140 s |

Active-first schedule `{T, T, F, T, F, T}`, the other `{T, F, T, F, T, T}`. Window edges are
shifted for flight time and count delay: opening −1.75 s, closing 0.0 s.

**What the robot does with a single trigger:** in our alliance zone while our hub is active it
aims the rear at the hub, spins up to a distance-mapped speed and hood angle, and feeds once
spun up and aligned; in our zone while the hub is inactive it aims and waits; outside our zone
it passes to a fixed target on our side. The field is **mirrored** (`AllianceFlipUtil`).

---

## 2. Software Stack

| Layer | Choice |
|---|---|
| Framework | WPILib Command-Based |
| Logging / replay | AdvantageKit (`LoggedRobot`, IO layer, `@AutoLog`) |
| Motor control | CTRE Phoenix 6 (TalonFX, TalonFXS, CANcoder, Pigeon2) |
| Vision | PhotonVision over NetworkTables |
| Path following | PathPlannerLib (`AutoBuilder`, `LocalADStarAK` pathfinder) |
| Dashboard | Elastic + AdvantageScope |
| Formatting | Spotless / Google Java Format (runs on `compileJava`) |
| Java | 17 |
| Commands | V2 (`edu.wpi.first.wpilibj2.command`) |

**2027 target:** Systemcore, Java 25, `org.wpilib`, Commands V3. Not ported yet — see
[2027-migration.md](2027-migration.md) for the gates and [commands-v3.md](commands-v3.md) for
what each command rule becomes. Update this table in the port commit.

Versions live in `vendordeps/` — see `docs/hardware-layout.md` for the table.

---

## 3. Package Structure

```
frc/lib/util/                ALL shared utilities — see docs/subsystem-ownership.md
frc/lib/mechanism/           motor abstraction and the three mechanism kinds
frc/robot/                   Main, Robot, RobotContainer, RobotState, Constants, BuildConstants
frc/robot/generated/         TunerConstants (Tuner X generated — never hand-edit)
frc/robot/subsystems/drive/  Drive, Module, Gyro/Module IO, PhoenixOdometryThread
frc/robot/subsystems/vision/ Vision, VisionConstants, io/
frc/robot/commands/          DriveCommands (+ one file per mechanism)
```

Tests mirror this structure under `src/test/java`. See [testing.md](testing.md) for what each
layer covers, and — just as important — what it does not.

This branch adds:

```
frc/robot/                   + HubShiftUtil, Zones, ShotModes, AutoPreview, RobotModelVisualizer
frc/robot/subsystems/flywheel/     Flywheel, FlywheelConstants, ShotCalculator, FlywheelVisualizer
frc/robot/subsystems/hood/         Hood, HoodConstants, HoodPosCalculator
frc/robot/subsystems/intakePivot/  IntakePivot, IntakePivotConstants
frc/robot/subsystems/intakeRoller/ IntakeRoller, IntakeRollerConstants
frc/robot/subsystems/prestage/     Prestage, PrestageConstants
frc/robot/subsystems/upperFeeder/  UpperFeeder, UpperFeederConstants
frc/robot/subsystems/lowerFeeder/  LowerFeeder, LowerFeederConstants
frc/robot/subsystems/transport/    Transport, TransportConstants
frc/robot/commands/          + one *Commands per mechanism (FeederCommands covers both feeders),
                               ShootSequences, SpitSequences
```

---

## 4. Architecture Rules

Summarized here; authoritative version is `.claude/rules/01-architecture.md`.

- Every subsystem wraps hardware behind an IO interface; hardware objects exist only in IO
  implementations (`MotorIOTalonFX` for every mechanism, `ModuleIOTalonFX` / `GyroIOPigeon2` /
  `VisionIOPhotonVision` for drive and vision). This is what makes log replay work.
- No subsystem holds a reference to another subsystem. Shared state goes through
  `RobotState`; one-off values go through a `Supplier` passed at construction.
- Commands are static factory methods, never `extends Command` classes. Every one calls
  `.withName()`.
- Exactly one `SwerveDrivePoseEstimator`, inside `Drive`. `RobotState` delegates to it.
- `TunerConstants` is the sole source of truth for drivetrain geometry and gains.
- Never call `DriverStation.getAlliance()` in a hot path — use `AllianceFlipUtil`.
- Every `waitUntil` has a `withTimeout`.

**None of these are checked automatically.** They are enforced by review, using
[review-checklist.md](review-checklist.md). An ArchUnit test enforced them briefly and was
removed to keep the suite small.

---

## 5. Robot.java — Lifecycle

| Phase | What happens |
|---|---|
| Constructor | Record build metadata; configure AdvantageKit receivers per `Constants.currentMode`; `Logger.start()`; register `RobotState` with `AutoLogOutputManager`; construct `RobotContainer` |
| `robotPeriodic` | `loopMonitor.begin()` → `AllianceFlipUtil.refresh()` → `CommandScheduler.run()` → `CommandLogger.periodic()` → `batteryLogger` update → CAN bus, match metadata and `FaultMonitor` → `loopMonitor.end()` |
| `disabledInit` / `disabledPeriodic` | Stop hoot logging; coast the drive `COAST_DELAY_SECONDS` (3 s) after disable |
| `autonomousInit` | Brake + start hoot logging; start the auto timer; schedule the chooser's auto |
| `autonomousExit` | Log `Auto/DurationSeconds` and `Auto/Overran` |
| `teleopInit` | Brake + start hoot logging; cancel the auto command |
| `testInit` | Brake + start hoot logging; `CommandScheduler.cancelAll()` |

`AllianceFlipUtil.refresh()` must run **before** the scheduler — commands read the cached
value during their own execution.

**Added on this branch (the 2026 behavior):**

| Phase | What happens |
|---|---|
| Constructor | Publish the "Robot Pose Field Map" `Field2d` and the 2026 pre-match checklist (29 devices) |
| `robotPeriodic` | After the battery logger: robot pose to the field map, 3D model poses, `ShotModes.update()`, log `driverPreset` and `driveController` |
| `disabledPeriodic` | `AutoPreview`: draw the selected auto, report distance from its start pose |
| `autonomousPeriodic` | Robot on the auto preview field; "Match Time" to the dashboard |
| `teleopInit` | Schedule stop-all; `HubShiftUtil.initialize()`; latch the drive controller; schedule intake roller at 12 V (runs until something else takes the roller) and pivot down; select the Elastic "Teleoperated" tab; latch the driver's rotation exponent |
| `teleopPeriodic` | Hub-shift time left, "Win Auto?", "Is Hub Active", match time, aligned / spun up / X'd to the dashboard; hub shift to the log |

---

## 6. Constants.java

Runtime mode only.

- `simMode` — `SIM` or `REPLAY`, chosen at compile time
- `currentMode` — `REAL` on a roboRIO, otherwise `simMode`
- `disableHAL` — set true in unit tests so `AllianceFlipUtil` skips DriverStation access

The 2026 `RobotType` enum (COMP/ALPHA) was removed along with the dual `TunerConstants`.
If a practice robot needs different constants in 2027, prefer auto-detecting it (CAN
device ping, jumper pin) over a hand-edited flag — deploying the wrong constants under
competition pressure was a known 2026 risk.

---

## 7. Where Constants Live

**One constants file.** 2026 had both `Constants` and `HardwareConstants`, and the split meant
remembering which of the two a value lived in while a match clock was running. They are merged.

```
Constants
├── Mode / simMode / currentMode   runtime mode selection
├── disableHAL                     unit-test escape hatch
├── tuningMode                     enables dashboard-adjustable constants; FALSE for competition
├── CanIds                         every non-swerve CAN ID, each with // CANivore or // RIO CAN
├── Setpoints                      voltages, velocities, positions a mechanism is commanded to
├── Waits                          every command timeout — nothing inline
├── ShotModeRequests               2026 shot-tuning / demo requests, FMS-guarded by ShotModes
├── Autos                          default auto name
└── Thresholds                     alignment tolerances, readiness bands, start-pose check
```

`Controllers` also carries the 2026 port map (flight stick on **2**, Xbox on 1, sim keyboard on
3), the driver-preset exponents and the drive-controller option labels.

The rule: **if you would change it in the pit between matches, it goes in `Constants`.**

Everything describing how a mechanism is built or characterized stays in that subsystem's own
`*Constants.java` — gains and their sim/real split, current limits, gear ratio, sim model
parameters, and interpolation maps. A distance-to-setpoint table is structure rather than a
knob, and putting it in `Constants` would bury the values people actually reach for.

Swerve is outside both: its IDs, ratios, gains and limits are Tuner X output in
`generated/TunerConstants.java` and must not be hand-edited.

---

## 8. CAN Bus Layout

See `docs/hardware-layout.md` — that is the source of truth. Do not duplicate the table
here; the 2026 spec did and the two drifted.

---

## 9. Drive Subsystem

**Files:** `subsystems/drive/` + `commands/DriveCommands.java` + `generated/TunerConstants.java`

**Structure.** Four `Module`s (FL, FR, BL, BR), each a `ModuleIO` + closed-loop control,
plus a `GyroIO`. `PhoenixOdometryThread` samples drive position, steer position, and gyro
yaw at 250 Hz on CAN FD (100 Hz otherwise) into timestamped queues. `Drive.periodic()`
drains those queues under `odometryLock` and feeds `poseEstimator.updateWithTime()`.

**Pose estimation.** One `SwerveDrivePoseEstimator`, owned by `Drive`. Vision enters via
`addVisionMeasurement()`. `RobotState` reads it through `poseSupplier`, set in the `Drive`
constructor. If the gyro disconnects, heading falls back to integrating module deltas via
`kinematics.toTwist2d()` and an `Alert` is raised.

**PathPlanner.** `AutoBuilder.configure()` lives in `Drive.configureAutoBuilder()`, which
`RobotContainer` calls once before `AutoBuilder.buildAutoChooser()`. It is a method rather than
constructor code because `AutoBuilder` is global state and a second `Drive` would rebind it.
Path-following gains and `PP_CONFIG` (mass, MOI, wheel COF) are in that method — all 2026
values, all need re-measuring.

**Commands.** `joystickDrive`, `joystickDriveLimited`, `joystickDriveAtAngle`, `driveToPose`,
`alignForScore`, `stopWithX`, `feedforwardCharacterization`, `wheelRadiusCharacterization`.
A heading-only align passes a heading supplier to `joystickDriveAtAngle`; a full-pose align
uses `driveToPose`. See [target-alignment.md](target-alignment.md).

**Logged per module.** Connected flags, drive position/velocity/applied volts, stator
current and **torque current**, turn absolute/relative position, velocity, applied volts,
stator current and **torque current**, plus the odometry sample queues. Both motors run
`*TorqueCurrentFOC` requests, so torque current is the actual control signal — stator
current alone cannot distinguish a module fighting a stall from one spinning free.
Registered at 50 Hz in the same frame as stator current, so it adds no CAN traffic.
`ModuleIOSim` leaves both torque channels at zero rather than inventing a value.

**Gains.** Both drive and steer use `ClosedLoopOutputType.TorqueCurrentFOC`, so every
Slot0 gain is in amps. See `docs/characterization-and-tuning.md` for how to measure them.

**2026 performance notes.** Voltage-saturated near 3.8 m/s rather than current-limited.
Drive supply limit went 40 A → 60 A mid-season: ~12% more peak acceleration, 4× the
brownouts, no change in top speed.

**2026 alignment commands (this branch).** All but two are `joystickDriveAtAngle` with a
heading supplier:

| Factory | Heading / behavior | Bound |
|---|---|---|
| `alignOrXForShoot` | Aim at the hub; X once `isAlignedForCurrentShot` and the stick is centered | Real: shoot in zone |
| `alignForDefenseShot` | Pathfinds to (3.5, 1.5 or 6.5) facing the shooter at the hub; finishes immediately | Sim only |
| `joystickDriveAlignForTrench` | ±90° by side of the field's center line | Real and sim |
| `joystickDriveAlignForBump` | Nearest diagonal | Sim only |
| `joystickDriveAlignForTower`, `…ForWall`, `…ForSweep`, `…ForSweepToAllianceZone` | 2026 variants | Unbound |

---

## 10. Mechanism Subsystems

Every mechanism is built on `frc/lib/mechanism`. That fixes, for all eight at once:

- **Control:** rollers use `MotionMagicVelocityTorqueCurrentFOC`, position mechanisms
  `MotionMagicTorqueCurrentFOC`; open loop is `VoltageOut` with FOC. Gains are in amps.
- **Logged inputs:** the shared `MotorIO` schema under the mechanism's name — applied volts,
  stator / supply / torque amps, position, velocity, temperature, closed-loop reference and
  error, four sticky faults — plus `/FollowerN` and `/Encoder` groups where present. 50 Hz
  control group, 10 Hz diagnostics, 4 Hz faults, `optimizeBusUtilization` last.
- **Health:** `registerFaultMonitors()` for disconnect, reboot, over-temperature, hardware fault.

Hardware, gains, limits and ratios per mechanism are in
[hardware-layout.md](hardware-layout.md#mechanism-configuration); this section covers behavior.

### Flywheel
**Files:** `subsystems/flywheel/` + `commands/FlywheelCommands.java`. Five Krakens, 36:24.
**At speed:** within 200 RPM (`isSpunUp()` = `isAtVelocity()`), feeds `Triggers.isFlywheelSpunUp`.
**Shot selection:** `setSpeedForHub` / `ForPassing` / `ForTarget` / `ForDistance` through
`ShotCalculator` and the `SPEED_MAP` / `PASSING_SPEED_MAP` tables, clamped 100–5600 RPM.
`shootDynamic` (shoot-on-the-move, sim-bound only) is ported with three known bugs, documented
in place.
**Default:** hold 1200 RPM idle.
**Commands:** velocity factories are `run` (hold until interrupted); `stop` sets 0 RPM.

### Hood
**Files:** `subsystems/hood/` + `commands/HoodCommands.java`. Fused CANcoder, 0–62°.
**Aim:** `setHoodPosForHub` / `ForPass` through `HoodPosCalculator` and `ANGLE_MAP` /
`PASSING_ANGLE_MAP`. Setpoints are clamped to the soft limits by the library.
**Default:** hold 0° (`hoodIdle`). Aiming commands stow to 0° when they end.

### Intake Pivot
**Files:** `subsystems/intakePivot/` + `commands/IntakePivotCommands.java`. Remote CANcoder;
0 = deployed, 0.3 rot = retracted.
**Compress:** lift to 0.25 rot after 0.5 s (single) or 0.115 → 0 → 0.25 (double), chosen live
by `ContinuousConditionalCommand`. Auto uses its own single compress.
**Default:** none. `setPivotPosition` is `runOnce`; the device holds the goal.
**Soft limits:** effectively off, as 2026 ran — see the constants file.

### Intake Roller
**Files:** `subsystems/intakeRoller/` + `commands/IntakeRollerCommands.java`. Two motors.
**Default:** 3 V agitate (real robot only). Intake is 12 V while held; stop is 0 RPM.

### Prestage
**Files:** `subsystems/prestage/` + `commands/PrestageCommands.java`. Two motors.
Runs 3000 RPM with the flywheel; stop is 0 RPM. No default command (commented out in 2026).

### Upper and Lower Feeder, Transport
**Files:** one package each + `commands/FeederCommands.java`, `commands/TransportCommands.java`.
Feeders run −3000 RPM, transport −1800 RPM, set once (`runOnce`) and left running until a stop.
"After wait" variants wait 0.5 s, then for alignment within the remaining 1.0 s. No defaults.

---

## 11. Vision Subsystem

**Files:** `subsystems/vision/`

**Structure.** N cameras, each a `VisionIO`. `Vision.periodic()` pulls observations,
filters them, computes standard deviations, and hands survivors to the consumer wired to
`drive::addVisionMeasurement`.

**Rejection filters,** checked in priority order — first match wins and is logged to
`Vision/CameraN/RejectionReason`:

| Reason | Condition |
|---|---|
| `AngularVelocityTooHigh` | robot yaw rate > `maxAngularVelocityRadPerSec` (6.0) |
| `NoTags` | `tagCount == 0` |
| `InvalidTimestamp` | timestamp ≤ 0 |
| `HighAmbiguity` | single-tag and ambiguity > `maxAmbiguity` (0.35) |
| `BelowFloor` | pose Z < −`floorError` |
| `ZTooHigh` | pose Z > `maxZError` (2 m) |
| `TagsTooFar` | average tag distance > `maxDistanceMeters` (8.0) |
| `SingleTagTooFar` | single-tag and distance > `maxSingleTagDistanceMeters` (4.0) |
| `PitchRollTooLarge` | pitch or roll > 25° |
| `OutsideField` | pose outside the AprilTag layout bounds |

**Standard deviations.** Base × distance² / tagCount, then ×2 for single-tag, then the
per-camera factor. MegaTag2 observations halve linear std dev and set angular to infinity.

**Why the single-tag distance limit exists.** Every catastrophically wrong accepted pose in
the 2026 State and Worlds logs (2.5–9.6 m off) was a single-tag solve at ≥ 3.8 m, some with
near-zero ambiguity. The ambiguity filter alone could not catch them. Multi-tag solves at
4–6 m were never catastrophically wrong, so they keep the looser limit.

**Known gaps.** Pose-estimator divergence is detected (`Odometry/OffField`, an Alert) but not
guarded. Camera transforms are identity
placeholders. A camera died mid-event in 2026 (`RobotLeft`, after q13) and nothing alerted
beyond the disconnect `Alert`.

---

## 12. RobotState — API

Current surface (game-agnostic):

| Method | Returns |
|---|---|
| `getEstimatedPose()` | `Pose2d` from Drive's estimator |
| `getRotation()` | current heading |
| `getFieldRelativeVelocity()` | `ChassisSpeeds`, field frame |
| `getRobotRelativeVelocity()` | `ChassisSpeeds`, robot frame |
| `getDistanceToPoint(Translation2d)` | `Distance` |
| `getAngleToTarget(Translation2d)` | bearing for the robot **front** |
| `setPoseSupplier(Supplier<Pose2d>)` | wiring, called once by `Drive` |
| `updateModuleStates(SwerveModuleState[])` | called each loop by `Drive` |

**2026 game state (this branch):**

| Method | Returns |
|---|---|
| `getAllianceHubTarget()` | our hub's top center, flipped for red |
| `getAngleToAllianceHub()` | heading that points the **rear shooter** at our hub (`@AutoLogOutput`) |
| `getShooterAngleToTarget(Translation2d)` | heading that points the rear shooter at a point — 2026's `getAngleToTarget` |
| `getDistanceToAllianceHub()` | `@AutoLogOutput` |
| `getHubRelativeVelocity()` | for shoot-on-the-move (`@AutoLogOutput`) |
| `getPassTarget()` | (4.5, 2.3 or 6.1) blue / (12.0, …) red, by field half |
| `isAlignedToHub()` / `…Loose()` | within 1.5° / 6° |
| `isAlignedToPass()` / `…Loose()` | within 7° / 7° |
| `getBroadZone()` | alliance zone / alliance trench / neutral / opposing trench / opposing zone |
| `getSpecificZone`, `getApproachingZoneX/Y`, `getApproachingZone` | finer zones; only `ApproachingZoneX` is read (sim) |

Note `getAngleToTarget` returns the true bearing. The 2026 version silently added 180°
because that shooter faced backward; on this branch that offset lives in
`getShooterAngleToTarget`, and every 2026 call site uses it.

---

## 13. Triggers.java

Singleton; owns the controllers privately — flight stick, Xbox, and the sim keyboard, ports in
`Constants.Controllers`. Exposes robot-convention axis suppliers (`driveXSupplier`,
`driveYSupplier`, `driveRotSupplier` — sign flips live here, no deadband) and one named accessor
per robot function. State triggers go here as `LoggedTrigger` fields.
`.claude/rules/01-architecture.md` explains why.

**This branch carries the 2026 layout,** including the dashboard drive-controller swap:
`latchDriveController()` is called once from `teleopInit`, and `sourced()` routes each function
to one device. The full button map is [driver-controls-card.md](driver-controls-card.md).

| Accessor | Flight stick drives | Xbox drives |
|---|---|---|
| `shootButton` | FS 1 | RT |
| `trenchAlignButton` | FS 2 | X |
| `intakeInButton` / `intakeOutButton` | FS 3 / 4 | LB / RB |
| `intakeRollerButton` | FS 5 | LT |
| `intakeCompressButton` | FS 6 | FS 6 |
| `demoDistanceShot` | FS 7 | FS 7 |
| `wvroxOdometryReset` | FS 8 | D-pad down |
| `passButton` | FS 9 | D-pad up |
| `shootFromTowerButton` | FS 10 | Y |
| `autoXOverride` | FS 12 | FS 12 |
| `allianceWinFlipper` | Xbox A | FS 3 or 4 |
| `allianceWinDisabler` | Xbox Y | FS 2 |
| `doubleCompressOverride` | Xbox B | — |

**State triggers** (2026 log keys): `isShootSafeZone`, `isShootSafeTime`, `isShootClear`,
`isAlignedForCurrentShot` and `isAlignedLooser` (both debounced 0.3 s rising),
`isFlywheelSpunUp`; new on this branch, `isHubInactiveInZone`, `isShotTuningMode`,
`isDemoMode`.

---

## 14. Command Implementations

See §9 for drive commands and §10 for each mechanism's. The composition layer:

| Factory | What it does |
|---|---|
| `ShootSequences.autoShootToHub` | The auto "Shoot": aim flywheel and hood at the hub, feed once spun up and loosely aligned (≤ 1 s), agitate and compress |
| `ShootSequences.stopAll` | Stop every shooter mechanism and stow the hood; auto "stopAll" and teleopInit |
| `ShootSequences.shootEndBehavior` | Unbound in 2026 |
| `SpitSequences.*` | Clear the robot; none bound in 2026 |

Patterns that must survive the season:

- Static factories only, every one `.withName()`'d
- Every `waitUntil` has a `withTimeout`
- Ready → Align → Act, where phase 2 gets the *remaining* budget, not a fresh one
- `ContinuousConditionalCommand` when a mode can change mid-execution

---

## 15. RobotContainer — Binding Logic

Wiring layer. Constructs subsystems per `Constants.currentMode` (REAL / SIM / REPLAY),
registers PathPlanner named commands and event triggers **before**
`AutoBuilder.buildAutoChooser()`, builds the auto chooser, then configures bindings.

This branch reproduces the two 2026 binding sets, which diverge — `configureRealBindings()`
on a roboRIO, `configureSimBindings()` in simulation. The real set, by mechanism:

- **Drive:** shoot in zone → `alignOrXForShoot` at half speed; shoot out of zone (not demo) → aim
  at the pass target at half speed; trench button → trench align; odometry reset → a `run`
  command that holds the drivetrain (a 2026 issue, kept).
- **Flywheel + prestage:** shoot while clear → hub-distance speed; shoot out of zone → pass
  speed; pass / tower / tuning / demo buttons → fixed-speed shots with timed feeding.
- **Feed:** shoot, not in-zone-with-hub-inactive, loosely aligned → wait for spin-up (≤ 1 s),
  then feed. The intake roller agitates at 3 V under the same condition.
- **Pivot:** in / out / manual compress, plus automatic compress while feeding and on tower or
  pass shots. Manual pivot input cancels the automatic compress until the trigger is released.
- **Hood:** hub map while clear, 2.5° tower, 28° pass, tuning and demo angles.

**Named commands:** `DeployIntake`, `RetractIntake`, `RunIntake`, `Shoot`, `stopAll`,
`HoodDownNamed`. **Event markers:** `DeployIntake`, `RetractIntake`, `RunIntake` (no
requirements), `HoodDown` (requires the hood — a 2026 issue, kept). The chooser defaults to
`2.5-Left-Comp`; the "Auto Delay" dashboard value delays the start. SysId routines stay in the
chooser for shop testing (commented out in 2026).

**Choosers:** "Auto Choices", "Driver Preset" (Parker 1.35 / Christian 2.0 rotation exponent),
"Drive controller" (Thrustmaster / Xbox).

No game logic here. Ever.

---

## 16. Utility Classes

| Class | Purpose |
|---|---|
| `AllianceFlipUtil` | Alliance mirroring; `refresh()` once per loop, `shouldFlip()` reads the cache |
| `ContinuousConditionalCommand` | Conditional that re-evaluates while running |
| `FieldConstants` | Field dimensions + AprilTag layout; gains game geometry each season |
| `BatteryLogger` | Per-subsystem current accounting for brownout analysis |
| `PhoenixUtil` | `tryUntilOk` — retries CTRE config until it sticks |
| `LocalADStarAK` | Replay-safe PathPlanner pathfinder |
| `LoggedTrigger` | Trigger wrapper that logs its own state |
| `Elastic` | Dashboard notifications and tab switching |
| `ThrowingRunnable` | Functional interface for throwing config calls |
| `LoggedTunableNumber` / `LoggedTunableBoolean` / `LoggedTunableProfiledPID` | Dashboard-adjustable values, gated on `tuningMode` — see [tunables.md](tunables.md) |
| `MotorSpecs` | Motor free speeds and sim gearboxes from WPILib's `DCMotor` |
| `CommandLogger`, `LoopTimeMonitor`, `CANBusMonitor`, `FaultMonitor`, `MatchMetadataLogger`, `PhoenixSignalLogger` | Instrumentation — see the README table |

`frc/lib/mechanism/` is the motor abstraction and the three mechanism kinds; see
`template/GUIDE.md`.

---

## 17. Known Issues Carried From 2026

| Issue | Impact |
|---|---|
| Loop overruns — ~30 Hz actual vs 50 Hz budget, Drive + Vision dominant | Stale control, missed odometry samples |
| 491 brownouts across 23 matches; battery to ~6.3 V at 390 A p95 | Motor cutouts mid-match |
| Pose divergence detected but not guarded | A bad estimate persists until good vision arrives |
| `maxPoseJumpMeters` filter written but never tuned or enabled | One less defense against bad poses |
| Jam detection exists (`RollerMechanism`) but has no measured thresholds | Jams stay silent until the 2027 intake's thresholds are logged and set |
| Autos overran the auto period; last path truncated every match | Lost auto points — the autos on this branch are the same files |
| `Measure`-typed log fields record in SI base units | Temperatures read as Kelvin |

---

## 18. Robot Physical Specifications

See `docs/hardware-layout.md`: 63.503 kg, MOI 5.162 kg·m², wheel COF 2.225, 4.0 m/s
configured (voltage-saturated near 3.8 m/s in 2026 logs), 22 in square wheelbase, 2 in wheels.
