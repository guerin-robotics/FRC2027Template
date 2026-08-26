---
name: add-subsystem
description: Scaffold a new mechanism subsystem end to end — the six files, CAN IDs, setpoints, RobotContainer wiring in all three modes, and operator triggers. Use when adding any new mechanism (intake, elevator, arm, shooter, climber, hood, feeder). Produces code that compiles and runs in sim, with everything unmeasured clearly marked.
---

# Add a Subsystem

Scaffold a mechanism from the template's six-file pattern, wired into the robot and
building, with every value that has to be measured on hardware marked as unmeasured
rather than quietly guessed.

**The goal is to get the boring, error-prone structure right in one pass so the team can
spend its time on logic.** Nothing here invents robot behavior.

---

## Step 1 — Gather the hardware facts

Ask for anything not supplied. Do not guess these; every one changes the generated code,
and a wrong guess produces code that compiles and misbehaves.

| Need | Why it matters |
|---|---|
| Mechanism name | Package name, log key, command names |
| **Motor model — Kraken X60 or X44** | Sets free speed and the sim model, via `MotorSpecs` |
| Motor count, and which is the leader | Determines whether a `Follower` is generated. Torque scales with count; free speed does not |
| CAN IDs and bus for each device | A wrong ID silently commands the wrong motor |
| Follower orientation — same way or opposite | Wrong means the motors fight; it cooks a gearbox |
| **Gear ratio**, motor rotations per mechanism rotation | Everything downstream is in the wrong units without it, and it sets the computed top speed |
| **Encoder, and exactly where it sits** | Decides the ratio split and whether `FusedCANcoder` is used. Ask for the shaft, not just yes/no — see below |
| **Which of the three kinds is it — spins, pivots, or travels in a line?** | Picks the scaffold, and with it the control request, neutral mode, gravity type, profile defaults, sim model and units. This is the first question; everything else follows from it |
| **Linear only: drum/sprocket PITCH diameter** | The only way to turn rotations into inches. Pitch diameter, not the outer diameter of the flange |
| **Linear only: rigging stage count** | A 2-stage cascade travels twice per rotation. Miss it and every height is off by an exact integer factor |
| Gravity-affected? Arm or elevator? | Decides `kG` and `GravityTypeValue` |
| **Arm only: what angle reads as horizontal?** | `Arm_Cosine` measures from horizontal. If zero is the stow position, `GravityArmPositionOffset` has to carry the difference |
| **How far does it travel, in sensor rotations?** | Over one turn, an absolute encoder cannot say which turn it is on — see below |
| Travel limits, if position-controlled | Soft limits. Without them a position goal can drive a mechanism into itself |
| Setpoints it gets commanded to | These go in `Constants.Setpoints` |
| Operator controls | Which Xbox buttons, and what each does |

If the user gives a mechanism type but not the details ("add an elevator"), ask once with
the specifics batched, rather than asking one at a time or inventing answers.

**Never guess the drum diameter or the stage count.** Both silently scale every height the
mechanism reports, so wrong values produce a subsystem that compiles, runs, logs plausible
numbers, and is wrong everywhere. Ask.

---

## Step 1b — Units, and the profile defaults

These are fixed conventions. Do not invent alternatives per mechanism.

| Quantity | Unit |
|---|---|
| Velocity | **RPM** |
| Acceleration | **RPM per second** |
| Rotating mechanism position | **degrees** |
| Linear mechanism position | **inches** |
| Gains | amps (every closed loop is `TorqueCurrentFOC`) |

Phoenix works in rotations and rotations per second. Convert **once**, at the config
boundary, using the helpers in the constants file — never with a bare `/ 60.0` sprinkled
through the IO.

Compute the top speed rather than guessing it:

```java
public static final MotorSpecs MOTOR = MotorSpecs.KRAKEN_X60_FOC;
public static final double GEAR_RATIO = 15.0;                 // TOTAL, motor -> mechanism
public static final double MAX_SPEED_RPM = MOTOR.maxMechanismRpm(GEAR_RATIO);
```

`GEAR_RATIO` is the total reduction. Phoenix needs it split at the encoder, and the split is
decided by one question: **where is the encoder?** Ask it explicitly. "Does it have a CANcoder"
is not enough — you need the shaft.

| Where the encoder is | `ROTOR_TO_SENSOR_RATIO` | `SENSOR_TO_MECHANISM_RATIO` |
|---|---|---|
| Motor encoder only, no CANcoder | `1.0` | `GEAR_RATIO` |
| CANcoder on the mechanism itself | `GEAR_RATIO` | `1.0` |
| CANcoder on an intermediate shaft | ratio above it | ratio below it |

The first two cover nearly everything this team builds — a hex-bore CANcoder on the mechanism
shaft, or the motor encoder alone. Confirm which, then move on.

### Before fitting an absolute encoder: does travel stay under one turn?

An absolute reading spans **one sensor rotation**. If the mechanism moves further than that, the
reading repeats, and the encoder cannot say which turn it is on. `FusedCANcoder` seeds position
from it at boot, so the mechanism boots believing a height or angle that is right only by luck.

Compute it before choosing:

```
rotations across full travel = full travel / travel per sensor rotation
```

| Mechanism | Travel | Per sensor rotation | Turns | Absolute encoder? |
|---|---|---|---|---|
| Intake pivot | 95° | 360° | **0.26** | Yes — unambiguous |
| Elevator, drum-mounted | 24 in | 6.28 in (2 in drum) | **3.82** | No — 0.5 could be 3.1, 9.4, 15.7 or 22.0 in |

Under one turn, fit the encoder. Over one turn, you have three options: gear the sensor down so
its full range covers the travel, accept a relative encoder and establish zero another way, or
add a limit switch. **A relative encoder needs a zeroing routine**, and `exampleLift` ships one working — `ExampleLiftCommands.zero()`, the zeroing state in `ExampleLift.java`, and `zeroPosition()` in
`ExampleLiftIO`. Copy it rather than writing one. The shape: drive into a hard stop under current
AND velocity sensing (not current alone — it spikes on static-friction breakaway well before the
real stop), declare that position zero, and **refuse to zero if the stop was never found within a
timeout**. A zero taken at an unknown position is worse than no zero.

This is also the question that picks `exampleArm` vs `exampleLift`: the arm assumes a fused
CANcoder and has no zeroing at all, the lift assumes the motor encoder and needs it.

This is the question people skip, because the encoder works perfectly in sim — where the
mechanism always starts at zero and the ambiguity never appears.


**The third case is the one to catch.** The 2026 hood had a CANcoder on a shaft that still drove
a 12T→122T gear pair after it, so the encoder turned about ten times per hood rotation.
Compounding the trap, that code calls the encoder's shaft the "output shaft" — meaning the
*gearbox* output, not the mechanism. The question that actually resolves it: **does the encoder
turn 1:1 with the thing you are measuring?** If anything geared sits after it, it is case three
and both numbers have to be worked out.

Always compute `MAX_SPEED_RPM` from `GEAR_RATIO`, never from a split half. On a mechanism with a
CANcoder on its own shaft, `SENSOR_TO_MECHANISM_RATIO` is `1.0`, so using it would report the
motor's raw free speed as the mechanism's top speed.

Motion Magic defaults are **already correct in each scaffold**, which is the main reason there are
three of them:

| Scaffold | Cruise velocity | Acceleration |
|---|---|---|
| `exampleLift` | `MAX_SPEED_RPM / 2` | 9000 RPM/s |
| `exampleArm` | 60 RPM | 300 RPM/s |
| `exampleRoller` | n/a — the setpoint is the cruise | 9000 RPM/s |

Do not carry a value across scaffolds. An arm that inherits the lift's row gets 30x the
acceleration it should have, aimed at a hard stop — and it compiles. That failure is exactly what
splitting one scaffold into three was meant to make impossible, so do not reintroduce it by hand.

Both end up as `LoggedTunableNumber`s, so the compiled-in value is only the starting dashboard
value — the profile stays adjustable at runtime.

A cruise velocity above `MAX_SPEED_RPM` does not fail loudly. The motor saturates and the
profile stops being followed, which looks like bad tuning rather than an impossible request
— so state the computed top speed in your report.

### Two settings that are silently wrong at their defaults

**Gravity reference on a pivot.** `GravityTypeValue.Arm_Cosine` scales kG by
`cos(position + offset)` and assumes the peak — arm horizontal — lands at a cosine argument of
zero. Mechanism zero is usually the *stow* position instead, so set
`Slot0.GravityArmPositionOffset` to the negative of the angle at which the arm is level. Left at
zero, the pivot gets too little hold current where gravity is strongest and too much where there
is none. Ask for the horizontal angle; do not assume zero.

**CANcoder discontinuity point.** `1.0` looks like the natural default and is usually wrong: it
puts the reading's wrap at 0, which is where the mechanism sits most of the match, so the value
flips between ~0.0 and ~1.0 at rest and a position loop chases a full rotation of phantom error.
Use `0.5` for arms and pivots unless the travel actually crosses 180°. `exampleArm` already
defaults to `0.5`; leave it there unless you can say where this mechanism travels.

---

## Step 2 — Generate the six files

Copy **the scaffold that matches the mechanism** — not a generic one:

| Mechanism spins | `template/src/main/java/frc/robot/subsystems/exampleRoller/` |
| Mechanism pivots to an angle | `template/src/main/java/frc/robot/subsystems/exampleArm/` |
| Mechanism travels in a line | `template/src/main/java/frc/robot/subsystems/exampleLift/` |

plus the matching `template/src/main/java/frc/robot/commands/Example*Commands.java`. Read the
files first — they carry the current conventions, and they are kept up to date.

Each one is complete and internally consistent: there is nothing to delete and no commented-out
branch to choose. If you find yourself deleting a block to make a scaffold fit, you probably
picked the wrong scaffold.

```
subsystems/<name>/
├── io/<Name>IO.java          interface + @AutoLog inputs
├── io/<Name>IOReal.java      every Phoenix call, and only here
├── io/<Name>IOSim.java       physics sim against the same interface
├── <Name>.java               logic; zero hardware imports
├── <Name>Constants.java      gains, ratios, limits, getFXConfig()
└── <Name>Visualizer.java     OPTIONAL — position mechanisms only

commands/<Name>Commands.java  static factories, all .withName()'d
```

**Generate the visualizer for position mechanisms; skip it for velocity ones.** It draws the
mechanism as a `LoggedMechanism2d` and publishes a `Pose3d` for the AdvantageScope 3D model,
which is how an inverted sense, a wrong gear ratio or a wrong zero become visible — each of
those produces perfectly plausible numbers in a position plot. A spinning roller has no
position worth drawing, so the file is dead weight there.

Copy `ExampleArmVisualizer` or `ExampleLiftVisualizer` — each already has just the one `update()`
for its kind, and the soft-limit bound markers already live rather than commented. Copy the
`Visualization` block from the matching constants file with it, and tell the user which values need
CAD numbers: `PIVOT_OFFSET` / `PIVOT_ROTATION` on the arm, `CARRIAGE_OFFSET` on the lift.
Everything else in that block is cosmetic and can be nudged by eye while watching AdvantageScope.

Non-negotiables, each of which has burned this team or is load-bearing for replay:

- `Logger.processInputs()` in `periodic()`. Removing it breaks log replay.
- Hardware imports appear in `<Name>IOReal` only. A `TalonFX` import in the subsystem
  class ends replay.
- `PhoenixUtil.tryUntilOk(5, ...)` around every config apply. CTRE silently ignores config
  when the bus is busy at startup, and the motor then boots with no current limits.
- Log **both** motors of a follower pair. A follower that has quietly died looks exactly
  like a leader that is underpowered.
- Register status signals for both motors at 50 Hz **before** `optimizeBusUtilization()`,
  or the unregistered ones drop to 4 Hz and read stale.
- Torque current is logged whenever the control mode is any `*TorqueCurrentFOC` — it is the
  control signal, and stator current is not a substitute.
- **Closed-loop reference and error are logged on every closed-loop mechanism.** Reference at
  50 Hz next to the measured value, error at 10 Hz. Without them a log cannot distinguish weak
  gains from a saturated profile from a wrong setpoint.
- **Signal rates follow the standard**: 50 Hz for velocity, position, stator, supply, torque,
  voltage and closed-loop reference; 10 Hz for device temperature and closed-loop error; 4 Hz
  for sticky faults; 50 Hz for a CANcoder's absolute position. See `.claude/rules/02-hardware.md`.
- **`connected` comes from the 50 Hz group alone**, debounced with `Debouncer(0.5, kFalling)`.
  Mixing rate groups into one status means the slowest signal governs and every alert fires for
  the first quarter second after boot. Followers use `TalonFX.isConnected()`; a separate device
  like a CANcoder gets its own status, because it can fail while the motor is fine.
- **`optimizeBusUtilization()` runs last, on every device** — followers and CANcoders included.
  It slows unregistered signals to 4 Hz rather than disabling them, so anything forgotten is
  *stale* rather than missing — plausible numbers a quarter second old. Followers at 4 Hz are
  fine; just arrive there on purpose.
- `Follower(int, MotorAlignmentValue)`. The old boolean overload does not exist in Phoenix 6
  2026.
- Supply current, not stator, goes to `BatteryLogger.reportCurrentUsage()`.
- **If the mechanism can stall against a game piece, it gets jam detection.** Rollers, feeders,
  intakes, transports. 2026 ran these open-loop and jams were silent — the mechanism stopped
  working and nothing in the log said why. Copy the JAM DETECTION block from
  `ExampleRoller` and `ExampleRollerConstants`: the conjunction of commanded-motion AND
  not-turning AND high **stator** current, debounced for longer than spin-up takes. Detect in
  the subsystem, respond in a command, register it with `FaultMonitor`. Ask for the thresholds
  or mark them unmeasured — telling a jam from a normal pickup is the whole job, and it needs
  real current data.

---

## Step 3 — Constants go in the right file

This split is the one most often gotten wrong. See `.claude/rules/03-commands.md`.

| Value | Goes in |
|---|---|
| Gains, gear ratios, current limits, soft limits, tolerances, sim model | `<Name>Constants.java` |
| CAN IDs | `Constants.CanIds`, with `// RIO CAN` or `// CANivore` on each line |
| Setpoints — what it is commanded to | `Constants.Setpoints` |
| Command timeouts | `Constants.Waits` |

Real-robot gains and the Motion Magic profile are `LoggedTunableNumber`s so they can be tuned
without a redeploy; sim gains stay plain doubles. Gear ratio and `MAX_SPEED_RPM` are static —
they describe the machine, not a knob.

If the mechanism is closed-loop, include the `updateTunedConfig()` block from the scaffold's
`*IOReal`. It must apply **both** `config.Slot0` and `config.MotionMagic`:
the gains are in the first, cruise and acceleration in the second, and applying only Slot0
leaves the profile knobs moving on the dashboard while the mechanism ignores them. See
`docs/tunables.md`.

**`RobotContainer` must end up with no bare numbers in it.** If a unit import is still
needed there, a setpoint was left behind.

---

## Step 4 — Wire it up

**`RobotContainer`** — construct in all three branches of the mode switch. The replay
branch needs an anonymous `<Name>IO() {}`; without it the logged inputs are not replayed
and the whole point of AdvantageKit is lost for that mechanism.

**`Triggers.java`** — every controller button accessor. `RobotContainer` never touches a
controller object, and never constructs a `Trigger`. Name accessors for the action, not the
button: `intake()`, not `leftBumper()`. Analog triggers use
`Constants.Controllers.TRIGGER_THRESHOLD`.

**Default command** — every subsystem needs one, and it must not end. A safe idle: stopped,
stowed, or holding position.

**`FaultMonitor`** — register a disconnect condition per motor. A mechanism that silently
stops working mid-match is worth an alert.

---

## Step 5 — Verify, then report honestly

```bash
./gradlew build           # compiles, checks formatting, and runs the test suite
./gradlew simulateJava    # must reach "Robot program startup complete" and stay up
```

`@AutoLog` generates `<Name>IOInputsAutoLogged` at build time. If the compiler says it is
missing a field you just added, run `./gradlew clean generateSources`.

**Run the full `build`, not just `compileJava`.** Two existing tests judge this scaffold with
nothing new written, because both read the real sources rather than a registry:

| If it fails | What the scaffold got wrong |
|---|---|
| `CanIdUniquenessTest` | The new CAN ID collides, is out of range, or reuses the swerve block |
| `RobotContainerSmokeTest` | Wiring throws, the chooser breaks, or a default command is unnamed |

**Everything else is on you.** Nothing automated checks the architecture rules — a `TalonFX` in
the subsystem instead of the IO impl, a `periodic()` missing `Logger.processInputs`, a subsystem
field on the new subsystem, or an abstract method on `<Name>IO` that breaks the replay branch
will all compile and pass. Re-read `.claude/rules/01-architecture.md` against what you wrote.
See [docs/testing.md](../../../docs/testing.md).

**Tests you should write**, per `docs/testing.md`:

- Pure logic — interpolation tables, readiness bands, zone math. Use
  `.claude/prompts/write-test.md`; known-correct cases come from measurement, not from the
  code.
- A sim convergence test for any closed-loop mechanism, shaped like `DriveToPoseSimTest`.
  This requires a physics `IOSim` — if the scaffold left it as stubs, say so rather than
  writing a test that asserts against fiction.

Then state plainly:

1. **What was measured versus what was guessed.** Every placeholder gain, every estimated
   moment of inertia, every soft limit picked from nothing. Do not bury these — a guessed
   drum diameter or gear ratio silently scales every conversion downstream.
2. **What still has to happen on hardware.** Inversion checked at low output, follower
   orientation confirmed, soft limits found by driving to the stops, characterization run.
3. **Anything that looked wrong in the existing code** while wiring it in.

A scaffold that compiles is not a working mechanism, and reporting it as one is the failure
mode of this skill.

---

## Scope

Do not, without being asked:

- Change existing subsystem code
- Add the mechanism to an auto routine or a PathPlanner named command
- Change CAN IDs of existing devices
- Invent setpoints for game actions nobody described

If the request needs one of those, say so and ask.

---

## Related

- `template/GUIDE.md` — the six-file pattern and what goes where
- `.claude/prompts/add-subsystem.md` — the fill-in-the-blanks request form
- `.claude/rules/01-architecture.md` — IO layer, `RobotState`, `Triggers`
- `.claude/rules/02-hardware.md` — CAN, config, inversion, signal frequency
- `.claude/rules/03-commands.md` — factories, timeouts, where setpoints live
- `docs/testing.md` — what the existing suite already covers, and what to write for a new
  mechanism
- `docs/new-mechanism-bringup.md` — **what the user does next**: the ordered path from a
  compiling subsystem to a tuned one. Point them at it in your report
- `docs/tunables.md` — tunable gains and the Phoenix Tuner X ownership trap
- `docs/characterization-and-tuning.md` — measuring what the scaffold left as placeholders
