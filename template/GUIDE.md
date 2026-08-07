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
└── MyMechanismConstants.java       ← gains, ratios, limits — how it is built

commands/MyMechanismCommands.java   ← static factories, all .withName()'d
```

### Units

Fixed across every mechanism, so nobody has to check which one a given file uses:

| Quantity | Unit |
|---|---|
| Velocity | RPM |
| Acceleration | RPM per second |
| Rotating position | degrees |
| Linear position | inches |
| Gains | amps — every closed loop is `TorqueCurrentFOC` |

Phoenix works in rotations and rotations per second. The conversion happens **once**, in
`getFXConfig()`, through the helpers at the top of the constants file. A bare `/ 60.0` in an IO
class is how a factor-of-sixty error gets in — it compiles, deploys, and moves the mechanism,
just not the way anyone expected.

Top speed is computed, never typed:

```java
public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;
public static final double GEAR_RATIO = 15.0;                 // TOTAL, motor -> mechanism
public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(GEAR_RATIO);
```

`GEAR_RATIO` is the total reduction and the only ratio anyone quotes. Phoenix needs it split at
the encoder, and `ROTOR_TO_SENSOR_RATIO * SENSOR_TO_MECHANISM_RATIO` must multiply back to it.
The split is decided by **where the encoder is**:

| Where the encoder is | `ROTOR_TO_SENSOR_RATIO` | `SENSOR_TO_MECHANISM_RATIO` |
|---|---|---|
| Motor encoder only | `1.0` | `GEAR_RATIO` |
| CANcoder on the mechanism itself | `GEAR_RATIO` | `1.0` |
| CANcoder on an intermediate shaft | ratio above it | ratio below it |

**Before fitting an absolute encoder, check that full travel stays under one sensor rotation.**
An absolute reading repeats every turn, so a mechanism that moves further cannot say which turn
it is on. The intake pivot travels 0.26 rotations and is fine; a 24 in elevator on a 2 in drum
travels 3.82, so a reading of 0.5 could be 3.1, 9.4, 15.7 or 22.0 inches. Over one turn, either
gear the sensor down, or use the motor encoder and add a zeroing routine against a hard stop.
The ambiguity never shows up in sim, because sim always starts at zero.

The first two rows are what we build almost every time — a hex-bore CANcoder on the mechanism
shaft, or the motor encoder alone. The third is the 2026 hood, where a 12T→122T pair still sat between
the encoder and the hood. That file calls the encoder's shaft the "output shaft", meaning the
gearbox output rather than the mechanism, which is exactly how the case gets missed. The question
that resolves it: **does the encoder turn 1:1 with the thing you are measuring?**

Compute top speed from `GEAR_RATIO`, never from a split half — with a CANcoder on the mechanism
shaft the other half is `1.0`, so you would be quoting the motor's raw free speed.

`MotorSpecs` reads free speeds from WPILib's `DCMotor`, so the number driving the speed math and
the number driving the sim model are the same one. It also builds the sim gearbox, so those
cannot drift apart either.

Motion Magic defaults — tunable once set, because a profile is what you most want to adjust with
the mechanism in front of you, and it is far safer to change than a gain:

| Mechanism | Cruise velocity | Acceleration |
|---|---|---|
| Linear position | `MAX_SPEED_RPM / 2` | 9000 RPM/s |
| Rotation position | 60 RPM | 300 RPM/s |
| Velocity | n/a — the setpoint is the cruise | 9000 RPM/s |

The scaffold will not compile until you pick one — `CRUISE_VELOCITY_RPM_DEFAULT` and
`ACCELERATION_RPM_PER_SEC_DEFAULT` are commented out for the same reason the gear ratio is.
These are the only defaults where the wrong answer would still build, and an arm that
inherits the lift's row gets 30x the acceleration it should have, aimed at a hard stop.

Both end up as `LoggedTunableNumber`s, so the chosen value is only the compiled-in default
and the starting dashboard value — the profile stays adjustable at runtime.

Gear ratio and `MAX_SPEED_RPM` stay static. They describe how the machine is built.

---

**Setpoints are the exception — they do not live here.** `MyMechanismConstants.java` describes
how the mechanism is *built*: gains, gear ratios, current limits, soft limits, tolerances, the
sim model. What the mechanism is *commanded to* — velocities, heights, angles, voltages — goes
in `Constants.Setpoints`, alongside every other mechanism's. The test is whether a driver might
ask you to change it between matches. When they do, you open one file, not seven.

Command factories take setpoints as parameters and `RobotContainer` supplies them from
`Constants.Setpoints`. A finished `RobotContainer` contains no bare numbers at all — if a unit
import is still needed in it, a setpoint got left behind. See `.claude/rules/03-commands.md`.

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
| The test suite in `src/test/` | Config validation, the ArchUnit rules that enforce `.claude/rules/`, wiring, and sim convergence. Covers a new mechanism the moment it exists — see `docs/testing.md` |

---

## B — What You Write Fresh Each Season

| Piece | Why it changes |
|---|---|
| All mechanism subsystems | New mechanisms every year |
| `Constants.CanIds` / `.Setpoints` / `.Waits` / `.Thresholds` | New IDs, setpoints and timeouts every year — sections exist, fill them in |
| `Triggers.java` | Button and state triggers. Exists with the drive bindings; add an accessor per robot function |
| Game geometry in `FieldConstants` | Field elements change completely — see `frc/lib/ExampleFieldConstants.java` in this directory for the tag-pose-derived pattern to copy, and `docs/target-alignment.md` for how it plugs into `driveToPose` |
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
| `DriveCommands` called `DriverStation.getAlliance()` twice per loop in three factories, including the drivetrain's default command | Replaced with `AllianceFlipUtil.shouldFlip()`; `ArchitectureRulesTest` now fails the build if it comes back |

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
