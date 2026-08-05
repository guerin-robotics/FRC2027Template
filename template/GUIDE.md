# Season Template Guide

*Guerin Robotics — how this codebase is structured and why.*

This directory holds a working example of the team's subsystem style: five files that show
the exact shape every mechanism should take. Copy them, rename `Example` to your mechanism,
fill in the TODOs.

The rest of this document explains what carried into this template, what you write fresh
each season, and what the 2026 season taught us.

---

## The Five Files

Every mechanism is the same five files. No exceptions, no shortcuts.

```
subsystems/myMechanism/
├── io/
│   ├── MyMechanismIO.java       ← interface + @AutoLog inputs class
│   ├── MyMechanismIOReal.java   ← TalonFX code lives HERE and only here
│   └── MyMechanismIOSim.java    ← physics sim (or stubs), same interface
└── MyMechanism.java             ← logic only, zero hardware imports

commands/MyMechanismCommands.java ← static factories, all .withName()'d
```

The one rule that makes it work: **`MyMechanism.java` must behave identically whether the
IO is real, sim, or replay.** That is the entire point of the abstraction. The moment a
`TalonFX` import appears in the subsystem class, log replay stops working and you lose the
ability to debug a match after the fact.

`MyMechanism.periodic()` always starts with:

```java
io.updateInputs(inputs);
Logger.processInputs("MyMechanism", inputs);   // NEVER remove this line
```

---

## A — What Already Carried Over

These are in `src/` right now. They are season-agnostic and have been through two
competition seasons. Don't rewrite them; extend them.

| Piece | Why it's here |
|---|---|
| AdvantageKit IO layer | Enables full log replay; diagnosing a match afterward requires it |
| `subsystems/drive/` | Swerve is mechanically stable across seasons — only constants change |
| `subsystems/vision/` | PhotonVision filtering structure is stable; thresholds were earned from real logs |
| `RobotState` singleton | Decouples subsystems from Drive; prevents cross-subsystem references |
| Static command factories | Composable, no hidden state, traceable in logs |
| `AllianceFlipUtil` + per-loop caching | Prevents thousands of `Optional` allocations per match |
| `BatteryLogger` | Per-subsystem current tracking; the only way to diagnose a brownout |
| `LoggedTrigger` | Makes trigger state visible in AdvantageKit logs |
| `ContinuousConditionalCommand` | Correct answer when a command's mode changes while running |
| `PhoenixUtil.tryUntilOk` | CTRE silently ignores configs on a busy bus; this retries until they stick |
| `LocalADStarAK` | Required for PathPlanner pathfinding to work under replay |
| `LoggedDashboardChooser` | Logs which auto was selected — essential for post-match review |

---

## B — What You Write Fresh Each Season

| Piece | Why it changes |
|---|---|
| All mechanism subsystems | New mechanisms every year |
| `HardwareConstants` | New CAN IDs, new setpoints, new timeouts — doesn't exist yet, create it |
| `Triggers.java` | Button and state triggers — doesn't exist yet, create it once bindings outgrow `RobotContainer` |
| Game geometry in `FieldConstants` | Field elements change completely |
| Scoring targets and zone logic in `RobotState` | Field coordinates change |
| A sequences file (2026 had `ShootSequences` / `SpitSequences`) | The scoring pipeline is the game |
| PathPlanner `.auto` files and paths | Field-specific |
| Named commands in `RobotContainer` | Follow the game |
| Interpolation tables (distance → setpoint) | Measured against this year's field element |
| Swerve encoder offsets and gains in `TunerConstants` | Re-run Tuner X on the new robot |
| Vision camera transforms | New robot, new camera positions |

---

## C — Debt That Was Fixed On The Way In

Recorded so nobody reintroduces these. All of these were real problems in 2026 and are
already resolved in this template:

| 2026 problem | Resolution |
|---|---|
| `DriveConstants` duplicated the module layout from `TunerConstants` | Deleted; `TunerConstants` is the sole source of truth |
| `COMP_`/`ALPHA_TunerConstants` selected by editing `Constants.java` | Collapsed to one `TunerConstants`; `RobotType` enum removed |
| `Drive.alignForDefenseShot()` — game-specific name on a generic helper | Renamed `pathfindToPose()` |
| `Drive.aligningDefensively` flag with no remaining reader | Deleted |
| `RobotState.getAngleToTarget()` silently added 180° for the rear-facing shooter | Returns the true bearing; mechanism offset is now a call-site concern |
| Dead `@AutoLogOutput` methods running every loop with no consumer | Removed during the port; watch for this reappearing |

---

## D — Debt Still Open

These were **not** fixed, because they need the 2027 robot to exist first. Decide on them
deliberately early in the season rather than discovering them at competition.

### 1. No pose-estimator divergence guard

If the pose estimate goes bad, nothing pulls it back except good vision observations. The
`maxPoseJumpMeters` filter was written in 2026 and left disabled because it was never tuned
against real logs. Either tune and enable it, or add an explicit sanity check that rejects
estimates outside the field.

### 2. Loop timing

The 2026 robot ran nearer 30 Hz than the 20 ms budget, with Drive and Vision dominating.
This template is lighter simply because there is less code — that will not stay true. Watch
`robotPeriodic` timing from the first day the robot drives.

### 3. No jam or stall detection pattern

2026 ran open-loop rollers and belts with no feedback, so jams were silent. Whatever the
2027 intake is, build supply-current monitoring into its `periodic()` from the start.

### 4. Implicit readiness instead of a state machine

2026 determined "ready to score" by chaining `waitUntil(isSpunUp)` and
`waitUntil(isAligned)` with timeouts. It worked, but you could not ask "what state is the
scoring mechanism in right now?" without reconstructing it from the command tree. Consider
an explicit enum state machine in a coordinator class if the 2027 scoring flow is
comparably complex.

### 5. `RobotContainer` growth

It is small and readable today. In 2026 it reached 1300 lines. Split bindings into
per-subsystem `configureXxxBindings()` methods before it gets there, and move triggers into
`Triggers.java` as soon as there are more than a handful.

---

## Where Things Go

```
src/main/java/frc/robot/
├── Robot.java                ← lifecycle; keep it thin
├── RobotContainer.java       ← wiring and bindings ONLY, no game logic
├── RobotState.java           ← shared state; add game geometry here
├── Constants.java            ← runtime mode
├── HardwareConstants.java    ← CREATE THIS: CAN IDs, setpoints, timeouts
├── Triggers.java             ← CREATE THIS once bindings outgrow RobotContainer
├── generated/
│   └── TunerConstants.java   ← regenerate with Tuner X; never hand-edit
├── subsystems/
│   ├── drive/                ← carried over; update constants only
│   ├── vision/               ← carried over; update camera config
│   └── [new mechanisms]/     ← the five-file pattern above
├── commands/
│   ├── DriveCommands.java    ← carried over; add game alignment commands here
│   └── [new commands]/       ← static factories, one file per subsystem
└── util/                     ← carried over; add to it, don't rewrite it

frc/lib/                      ← FieldConstants, AllianceFlipUtil, shared helpers
```

---

## Note on This Directory

`template/` is **not** part of the Gradle build — the source set is `src/main/java` only, so
nothing here is compiled or deployed. The example files intentionally reference classes that
don't exist yet (`HardwareConstants`) and call methods you're expected to add
(`isAtVelocity()`). They are a reading reference and a copy source, not working code.

Spotless *does* format files here, so keep them syntactically valid Java.
