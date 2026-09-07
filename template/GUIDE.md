# Season Template Guide

*Guerin Robotics — how this codebase is structured and why.*

This directory holds three working scaffolds of the team's subsystem style — one per kind of
mechanism. Copy the one that matches what you are building, rename it, and fill in the TODOs
the compiler names for you.

The rest of this document explains what carried into this template, what you write fresh
each season, and what the 2026 season taught us.

---

## Three Scaffolds — Pick One

There is no generic mechanism. Everything this team builds is one of three kinds, and they differ
in ways that matter enough that mixing them up damages hardware. So there are three scaffolds, each
complete and self-consistent, with nothing to delete and no commented-out fork to choose between.

| Copy this | For | Control | Position sensor |
|---|---|---|---|
| `exampleRoller/` | roller, flywheel, feeder, transport, intake | velocity, RPM | motor rotor only |
| `exampleArm/` | arm, pivot, hood, wrist, turret | rotary position, degrees | fused CANcoder |
| `exampleLift/` | elevator, lift, extension | linear position, inches | motor rotor + zeroing |

**This used to be a single `ExampleSubsystem` with the three variants commented out.** That version
asked you to delete the blocks that did not apply, and the deletions it asked for were exactly the
ones with the worst failure modes when you got them wrong. An arm that inherited the lift's motion
profile row gets roughly 30x the acceleration it should have, aimed at a hard stop — and it
compiles, deploys and runs. Three files that are each right beat one file with instructions.

### What actually differs between them

Worth reading before you pick, because the differences are not cosmetic:

| | `exampleRoller` | `exampleArm` | `exampleLift` |
|---|---|---|---|
| Control request | `MotionMagicVelocityTorqueCurrentFOC` | `MotionMagicTorqueCurrentFOC` | `MotionMagicTorqueCurrentFOC` |
| Neutral mode | Coast — nothing falls | **Brake** — a coasting arm falls | **Brake** — a coasting lift falls |
| Soft limits | Off; a roller has no travel | On, in degrees | On, in inches |
| Gravity | none — no `kG` at all | `Arm_Cosine` + horizontal offset | `Elevator_Static`, constant |
| Cruise velocity | n/a — the setpoint is the cruise | 60 RPM | `MAX_SPEED_RPM / 2` |
| Acceleration | 9000 RPM/s | 300 RPM/s | 9000 RPM/s |
| Knows position at boot | does not care | yes, absolute | **no — must be zeroed** |
| Sim model | `FlywheelSim` | `SingleJointedArmSim` | `ElevatorSim` |
| Visualizer | none, deliberately | yes | yes |
| Extra block | jam detection | — | zeroing routine |

Two of those get copied wrong often enough to call out:

- **`kG` is absent from `exampleRoller` entirely.** A roller's mass is balanced about its axis, so
  gravity does no net work on it and a gravity feedforward has nothing to compensate.
- **`Arm_Cosine` and `Elevator_Static` are not interchangeable.** Gravity torque on a pivot varies
  with the cosine of its angle, peaking horizontal and vanishing vertical. A lift fights the same
  weight everywhere. Use the arm's on a lift and the compensation fades toward the top of travel,
  exactly where the mechanism is most extended.

### Absolute encoder, or zeroing? — the question that picks arm vs lift

An absolute reading repeats every sensor turn, so a mechanism whose sensor moves further than one
rotation cannot say which turn it is on.

An arm travelling 90° is fine, which is why `exampleArm` fits a fused CANcoder and has no zeroing
code at all. A 24 in elevator on a 2 in drum travels 3.82 rotations, so a reading of 0.5 could be
3.1, 9.4, 15.7 or 22.0 inches — four plausible heights and nothing to choose between them. That is
why `exampleLift` uses the motor encoder and ships a **working** zeroing routine rather than a
commented-out one.

**The ambiguity never shows up in sim, because sim always starts at zero.** It shows up as a
mechanism that boots to a confidently wrong height, once, on the practice field.

If your mechanism does not match its scaffold's assumption — a short extension whose travel fits in
one turn, or a continuously-rotating turret — take the encoder block from the other scaffold. Check
the arithmetic before deciding; do not assume.

### The two files

Every mechanism is the same two, whichever scaffold you started from.

```
subsystems/myMechanism/
├── MyMechanism.java            ← extends RollerSubsystem / RotarySubsystem / LinearSubsystem
└── MyMechanismConstants.java   ← ratio, limits, gains, travel bounds, tolerance, the sim model
```

**It used to be six**, and the other four are now in `frc/lib/mechanism/`. They were the same code
in every subsystem with different names on it, and the differences between the copies were
accidents at least as often as decisions — one logged stator current and called it supply, another
never read a sticky fault, a third left its follower's signals at the optimize floor by omission.

| What used to be a file | Where it lives now |
|---|---|
| `io/MyMechanismIO.java` | `MotorIO` — one interface, one `@AutoLog` schema for every mechanism |
| `io/MyMechanismIOReal.java` | `MotorIOTalonFX` — signal rates, followers, CANcoder, `connected` |
| `io/MyMechanismIOSim.java` | `MotorIOTalonFXSim` plus the per-kind `*MechanismSim` |
| `MyMechanismVisualizer.java` | `RotaryVisualizer` / `LinearVisualizer`, geometry taken from the mechanism |
| `commands/MyMechanismCommands.java` | `RollerCommands` / `RotaryCommands` / `LinearCommands` |

You still write a `MyMechanismCommands.java` when the mechanism has verbs of its own. The library
covers the verbs every mechanism of that kind shares; it does not know what your game does.

### The library underneath

```
frc/lib/mechanism/
├── MotorIO.java                ← the shared @AutoLog schema: motor, follower, encoder
├── MotorConfig.java            ← the builder; refuses to build without the unguessable values
├── MotorIOTalonFX.java         ← real hardware
├── MotorIOTalonFXSim.java      ← the same config against a simulated device
├── Mechanism.java              ← periodic, logging, battery reporting, tunable gains, faults
├── Gains.java  MotionProfile.java  MechanismVisualization.java
├── roller/    ← RollerMechanism, RollerSubsystem, RollerCommands, RollerSettings, sim
├── rotary/    ← RotaryMechanism, RotarySubsystem, RotaryCommands, RotarySettings, sim, visualizer
└── linear/    ← LinearMechanism, LinearSubsystem, LinearCommands, LinearGeometry, sim, visualizer
```

Unlike `template/`, this **is** in the build, and it has tests. That is the point of moving it: the
scaffolds could only ever be proof-read, and the library can be proved to work.

Two things it gives you that the old scaffolds could not:

- **The builder refuses to build.** A mechanism with no current limit, no gear ratio, no gains or —
  on a position mechanism — no travel bounds throws at startup, naming the mechanism and every
  missing value at once. A limit nobody chose is how 2026 logged hundreds of brownouts across one
  event.
- **Simulation runs the real gains.** `MotorIOTalonFXSim` applies the same `TalonFXConfiguration` to
  a simulated Talon, so the closed loop, the Motion Magic profile and the current limits in
  simulation are the ones that will run on the robot. The 2026 sim IOs ran their own WPILib
  `PIDController` against a second set of gains, which meant a tuning session in sim taught you very
  little.

### The visualizer — and why the roller does not have one

`RotaryVisualizer` and `LinearVisualizer` draw the mechanism as a `LoggedMechanism2d`. They are
**diagnostic only** — nothing in them affects robot behaviour, and deleting them would change only
what you can see.

There is deliberately none for the roller. A spinning drum has no position worth drawing, and a
ligament turning at 6000 RPM sampled at 50 Hz aliases into a bar that appears to drift slowly
backward. The velocity plot beside `closedLoopReference` already answers every question a picture
would, and answers it better.

What they catch that a plot does not — all of these were real, and all of them look like perfectly
plausible numbers in a log:

| Mistake | How it appears |
|---|---|
| Inverted sense | Commanded up, drawn down. The plot rises either way |
| Wrong gear ratio | The arm sweeps three times the travel it physically has |
| Wrong zero | Stowed reads 90°, so every setpoint is offset by a constant nobody wrote down |
| Goal never reached | Measured and goal drawn together — a saturated profile is obvious |
| Goal outside the bounds | The bound markers show a setpoint that can never be reached |
| Linear: wrong `STAGE_COUNT` | The carriage travels an exact integer multiple of its real travel |

Three things about them are deliberate:

- **`MechanismVisualization.ENABLED` is one switch for all of them.** They publish to NetworkTables
  every loop, and the 2026 robot ran closer to 30 Hz than 50 Hz all season. Leave it on through
  bring-up, where it is the whole point; check `LoopTiming/` with it enabled before competition and
  turn it off if the budget is tight. One flag rather than one per subsystem, because a switch you
  have to find in six files is a switch nobody throws.
- **`SmartDashboard.putData` is called once, in the constructor.** The 2026 `IntakePivotVisualizer`
  called it inside its update method, republishing the Sendable fifty times a second. The dashboard
  holds a reference and reads through it — once is all it ever needed.
- **The geometry comes from the mechanism**, not from a block of display constants. The bounds
  drawn are the soft limits actually enforced, and the elevator's travel is computed through the
  same `LinearGeometry` the device was configured with. A picture built from a second copy of the
  numbers draws a reassuring picture of a lie.

**What did not carry over:** 2026's `RobotModelVisualizer` published a single `Pose3d[]` at
`RobotModel/ComponentPoses` for the articulated 3D model, which is the right shape once there are
two or more moving components — AdvantageScope wants one field dragged onto the robot object, not
four. It is not here because it described the 2026 robot's four components exactly and none of
those numbers transfer. Rebuild it in `frc/lib` when the second articulated mechanism lands, taking
a `Supplier<Angle>` per component rather than subsystem references.

2026's `FlywheelVisualizer` was not this pattern at all — it projected a shot trajectory, which is
game logic and belongs with the sequences.

### Units

Fixed across every mechanism, so nobody has to check which one a given file uses:

| Quantity | Unit |
|---|---|
| Velocity | RPM |
| Acceleration | RPM per second |
| Rotating position | degrees |
| Linear position | inches |
| Gains | amps — every closed loop is `TorqueCurrentFOC` |

Phoenix works in rotations and rotations per second. The conversion happens **once**, at the
library boundary: `MotorConfig` takes typed `Angle` soft limits, `MotionProfile.of` takes typed
velocity and acceleration, and `LinearGeometry` is the single place a drum rotation becomes a
height. A bare `/ 60.0` written by hand is how a factor-of-sixty error gets in — it compiles,
deploys, and moves the mechanism, just not the way anyone expected.

`exampleLift` is the sharpest case: its IO layer speaks **drum rotations**, because that is what the
hardware reports and what a log should record, and the subsystem is the boundary that converts to
inches. Logging inches would bake a possibly-wrong drum diameter into the log, and a replay could
never be re-interpreted after that number was corrected.

Top speed is computed, never typed:

```java
public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;
public static final double GEAR_RATIO = 15.0;                 // TOTAL, motor -> mechanism
public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(GEAR_RATIO);
```

`GEAR_RATIO` is the total reduction and the only ratio anyone quotes. Phoenix needs it split at the
encoder, and `ROTOR_TO_SENSOR_RATIO * SENSOR_TO_MECHANISM_RATIO` must multiply back to it. The
split is decided by **where the encoder is**:

| Where the encoder is | `ROTOR_TO_SENSOR_RATIO` | `SENSOR_TO_MECHANISM_RATIO` |
|---|---|---|
| Motor encoder only | `1.0` | `GEAR_RATIO` |
| CANcoder on the mechanism itself | `GEAR_RATIO` | `1.0` |
| CANcoder on an intermediate shaft | ratio above it | ratio below it |

`exampleRoller` and `exampleLift` are the first row, and both derive
`SENSOR_TO_MECHANISM_RATIO = GEAR_RATIO` for you — there is no split to get wrong. Only
`exampleArm` asks for both numbers, because only it has a remote sensor.

The third row is the 2026 hood, where a 12T→122T pair still sat between the encoder and the hood.
That file calls the encoder's shaft the "output shaft", meaning the gearbox output rather than the
mechanism, which is exactly how the case gets missed. The question that resolves it: **does the
encoder turn 1:1 with the thing you are measuring?**

Compute top speed from `GEAR_RATIO`, never from a split half — with a CANcoder on the mechanism
shaft the other half is `1.0`, so you would be quoting the motor's raw free speed.

`MotorSpecs` reads free speeds from WPILib's `DCMotor`, so the number driving the speed math and the
number driving the sim model are the same one. It also builds the sim gearbox, so those cannot drift
apart either.

Motion Magic defaults are **already correct in each scaffold** — they are the row in the differences
table above, and they are the single most important reason the three are separate files. All of them
are `LoggedTunableNumber`s, so the compiled-in value is only the starting dashboard value and the
profile stays adjustable at runtime.

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
| The test suite in `src/test/` | CAN ID validation, `RobotContainer` wiring, vision filtering and sim convergence. The first two cover a new mechanism the moment it exists — see `docs/testing.md` |

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
| `DriveCommands` called `DriverStation.getAlliance()` twice per loop in three factories, including the drivetrain's default command | Replaced with `AllianceFlipUtil.shouldFlip()`. Nothing stops it coming back — watch for it in review |

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

### 3. Jam detection has a pattern now, but no numbers

2026 ran open-loop rollers and belts with no feedback, so jams were silent.

`RollerMechanism` now implements the detector — the conjunction of commanded-motion AND
velocity-far-below-commanded AND high stator current, held for a dwell — and it is covered by
tests. What is still open is that **none of its four thresholds can be guessed**. They come from
logging stator current during a real jam *and* during a normal pickup, since telling those two
apart is the entire job.

So the structure is solved and the numbers are not. Collect them on the 2027 intake early, put them
in that mechanism's `RollerSettings`, and bind `RollerCommands.unjam` or a `FaultMonitor` condition
to `isJammed()` so a mechanism jamming repeatedly reaches the pit rather than only the log.

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
│   └── [new mechanisms]/     ← copy a scaffold from template/src/
├── commands/
│   ├── DriveCommands.java    ← carried over; add game alignment commands here
│   └── [new commands]/       ← only for verbs the mechanism library does not already have

frc/lib/                      ← ALL shared utilities: field/alliance, hardware helpers,
│                                health monitors, tuning. Carried over; add to it,
│                                don't rewrite it. There is no frc/robot/util.
└── mechanism/                ← the motor abstraction and the three mechanism kinds.
                                 In the build, and tested. Add a kind here, not a copy
                                 of one in a subsystem package.
```

---

## Note on This Directory

`template/` is **not** part of the Gradle build — the source set is `src/main/java` only, so
nothing here is compiled or deployed. Spotless *does* format these files, so keep them valid
Java.

**Each scaffold deliberately does not compile.** The values that cannot be guessed have their
declarations commented out while the code still uses them, so copying a scaffold makes the compiler
name exactly what you owe it — and no way to skip one by accident.

| Scaffold | Owes | Symbols |
|---|---|---|
| `exampleRoller` | 2 | `EXAMPLE_ROLLER_MOTOR`, `GEAR_RATIO` |
| `exampleArm` | 7 | `EXAMPLE_ARM_MOTOR`, `EXAMPLE_ARM_ENCODER`, `ROTOR_TO_SENSOR_RATIO`, `SENSOR_TO_MECHANISM_RATIO`, `MAGNET_OFFSET_ROTATIONS`, `FORWARD_SOFT_LIMIT`, `REVERSE_SOFT_LIMIT` |
| `exampleLift` | 5 | `EXAMPLE_LIFT_MOTOR`, `GEAR_RATIO`, `DRUM_PITCH_DIAMETER`, `STAGE_COUNT`, `MAX_TRAVEL` |

The arm owed eight before the library landed. `GEAR_RATIO` came off the list because
`MotorConfig.rotorToMechanismRatio()` derives it from the two ratios, so there is no longer a third
number that has to agree with the other two — and no longer an invariant check needed to enforce
that it does.

Note what is *not* on these lists:

- **Motion-profile values** are set per scaffold, not asked for. Splitting the file into three is
  what let each have a right answer; the old single scaffold had to leave them blank because the
  wrong one still compiles.
- **`SENSOR_TO_MECHANISM_RATIO`** is just `GEAR_RATIO` in the roller and the lift, because the
  sensor is the rotor and there is no split. Only the arm asks for both.
- **`SENSOR_DISCONTINUITY_POINT`** defaults to `0.5`. `1.0` is the obvious-looking answer and is
  rarely right — it puts the wrap at 0, which is almost always the stow position.
- **`ENCODER_DIRECTION`** defaults to `CounterClockwise_Positive`, but there is no usual answer —
  the 2026 intake pivot needed `Clockwise_Positive`. Pass `.encoderDirection(...)` to change it.
- **Current limits** default to 40 A supply everywhere, with a stator ceiling that depends on the
  scaffold: 80 A on `exampleRoller`, 40 A on the two position scaffolds.

That last split is about what a wrong setpoint does. A position mechanism sent somewhere it cannot
reach drives into its own hard stop with everything the limit allows and holds there, so the two
position scaffolds start deliberately weak and are raised once a log shows what the mechanism
actually draws. A roller sent to a speed it cannot reach just spins slower, so it starts at a
working value; a roller that *can* stall is covered by the jam detector rather than by a low limit.

**The torque-current clamp now follows the stator limit automatically.** `MotorConfig` derives
±clamp from the stator limit unless you override it, so the 2026 trap of raising one without the
other — a clamp above the stator limit is not a clamp, and a stator limit raised without the clamp
leaves the mechanism exactly as weak as before — cannot happen by omission.

**To check the scaffolds**, temporarily put them in the source set:

```groovy
// build.gradle, temporarily
sourceSets { main { java { srcDir 'template/src/main/java' } } }
```

Expect exactly the symbols in the table above and no others — every error is `cannot find symbol`.
`REVERSE_SOFT_LIMIT` is named twice in the arm, because the simulation also starts the arm there;
everything else is named once per scaffold. Anything else in the list is rot.

Run this check after editing a scaffold. It is what caught the old one still using the
`TalonFX(int, String)` constructor, which Phoenix 6 deprecated for removal — a scaffold is the
worst place for a deprecated call, since being copied is its whole purpose.
