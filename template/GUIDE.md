# Season Template Guide

*Guerin Robotics — how this codebase is structured and why.*

This directory holds a working example of the team's subsystem style: six files that show
the exact shape every mechanism should take. Copy them, rename `Example` to your mechanism,
fill in the TODOs.

The rest of this document explains what carried into this template, what you write fresh
each season, and what the 2026 season taught us.

---

## The Six Files

Every mechanism is the same six files. No exceptions, no shortcuts.

```
subsystems/myMechanism/
├── io/
│   ├── MyMechanismIO.java          ← interface + @AutoLog inputs class
│   ├── MyMechanismIOReal.java      ← TalonFX code lives HERE and only here
│   └── MyMechanismIOSim.java       ← physics sim (or stubs), same interface
├── MyMechanism.java                ← logic only, zero hardware imports
└── MyMechanismConstants.java       ← every number, nothing inlined elsewhere

commands/MyMechanismCommands.java   ← static factories, all .withName()'d
```

The constants file is flat except for a nested `Sim` block, because sim needs its own gains —
the model has no backlash, no belt stretch and a guessed inertia, so gains that behave against
it are routinely wrong on hardware. Mode-aware `getKP()`-style accessors pick the right set so
no caller has to know which mode it is in.

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
| `Constants.CanIds` / `.Setpoints` / `.Waits` / `.Thresholds` | New IDs, setpoints and timeouts every year — sections exist, fill them in |
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

### 1. Pose-estimator divergence is detected, not guarded

`Drive` now logs `Odometry/OffField`, `Odometry/LargestVisionJumpMeters` and
`Odometry/PoseJumpCount`, and raises an Alert when the estimate leaves the field. That is
**detection only** — nothing rejects or corrects a bad estimate.

That is deliberate. The 2026 codebase had a `maxPoseJumpMeters` rejection filter that was
written and then left disabled because it was never tuned, and an untuned rejection filter
that discards good vision is worse than none. Collect real numbers from
`LargestVisionJumpMeters` across practice matches first, then turn it into rejection with
evidence behind the threshold.

### 2. Loop timing is now measured — watch it

`LoopTimeMonitor` logs `LoopTiming/*` and raises an Alert when the rolling average goes over
budget, so the 2026 failure (nearly 30 Hz all season, discovered post-season) cannot repeat
silently. The template is lighter simply because there is less code, and that will not stay
true as mechanisms are added. Check the number the first day the robot drives, and again
after each subsystem lands.

### 3. No jam or stall detection pattern

2026 ran open-loop rollers and belts with no feedback, so jams were silent. Whatever the
2027 intake is, build supply-current monitoring into its `periodic()` from the start — and
register the fault with `FaultMonitor` so it reaches the pit rather than only the log.

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
├── Constants.java            ← THE constants file: mode, CAN IDs, setpoints,
│                                waits, thresholds. Anything you'd change in the pit.
├── Triggers.java             ← CREATE THIS once bindings outgrow RobotContainer
├── generated/
│   └── TunerConstants.java   ← regenerate with Tuner X; never hand-edit
├── subsystems/
│   ├── drive/                ← carried over; update constants only
│   ├── vision/               ← carried over; update camera config
│   └── [new mechanisms]/     ← the six-file pattern above
├── commands/
│   ├── DriveCommands.java    ← carried over; add game alignment commands here
│   └── [new commands]/       ← static factories, one file per subsystem

frc/lib/                      ← ALL shared utilities: field/alliance, hardware helpers,
                                 health monitors, tuning. Carried over; add to it,
                                 don't rewrite it. There is no frc/robot/util.
```

---

## Note on This Directory

`template/` is **not** part of the Gradle build — the source set is `src/main/java` only, so
nothing here is compiled or deployed. Spotless *does* format these files, so keep them valid
Java.

**It deliberately does not compile.** Six values cannot be guessed, so their declarations are
commented out while the config still assigns them. Copy the scaffold and the compiler names
exactly what you owe it, one error per value:

```
EXAMPLE_MOTOR                 CAN ID
EXAMPLE_ENCODER               CAN ID (position mechanisms only)
ROTOR_TO_SENSOR_RATIO         motor rotations per encoder rotation
SENSOR_TO_MECHANISM_RATIO     encoder rotations per mechanism rotation
FORWARD_SOFT_LIMIT_ROTATIONS  travel bound
REVERSE_SOFT_LIMIT_ROTATIONS  travel bound
```

Commenting out the *assignments* instead would compile, and would be worse. Phoenix defaults
`SensorToMechanismRatio` to 1.0, so a config that quietly skips it reports motor rotations while
every setpoint and gain assumes mechanism rotations — a confidently wrong robot, which is far
harder to notice than one that refuses to build.

Everything else has a defensible default: 40 A supply, 80 A stator, ±80 A torque clamp, ±12 V.

**To check the rest of the scaffold**, temporarily put it in the source set:

```groovy
// build.gradle, temporarily
sourceSets { main { java { srcDir 'template/src/main/java' } } }
```

Expect exactly those six errors and no others. Anything else is rot. That check is what found
the scaffold still using the `TalonFX(int, String)` constructor, which Phoenix 6 deprecated for
removal — a scaffold is the worst place for a deprecated call, since being copied is its whole
purpose.
