# Characterization & Gain Extraction

How to run the built-in characterization routines and turn their output into real gains.

This template's swerve runs **`ClosedLoopOutputType.TorqueCurrentFOC`** on both drive and
steer. That one setting changes the meaning of every number in this document, so start
here.

---

## Read This First: The Units

> **Under `TorqueCurrentFOC`, every gain and every characterization output is in AMPS, not
> volts.** Not "roughly like volts." Amps.

`ModuleIOTalonFX.setDriveOpenLoop(double output)` switches on the configured output type:

```java
driveTalon.setControl(
    switch (constants.DriveMotorClosedLoopOutput) {
      case Voltage          -> voltageRequest.withOutput(output);        // output = volts
      case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);  // output = AMPS
    });
```

`TunerConstants` sets both to `TorqueCurrentFOC`, so the `output` that
`DriveCommands.feedforwardCharacterization` ramps is a **current request in amps**, and the
`kS` / `kV` it prints are already in the units `driveGains` needs. No conversion.

Two consequences that bite people:

1. **`FF_RAMP_RATE = 0.1` is 0.1 A/s, not 0.1 V/s.** The trailing comment in
   `DriveCommands.java` says `Volts/Sec` because it came from the upstream AdvantageKit
   template, which defaults to voltage output. It is wrong for this configuration.
2. **SysId's "voltage" is amps too.** `Drive.sysId` wires
   `(voltage) -> runCharacterization(voltage.in(Volts))`, so a 7 "volt" step is a 7 **amp**
   request. WPILib's default SysId config (1 V/s ramp, 7 V step) is far gentler than
   intended when those numbers land as amps — expect to pass an explicit
   `SysIdRoutine.Config` if you use SysId at all.

### Gain units reference

| Gain | `Voltage` output | `TorqueCurrentFOC` output |
|---|---|---|
| `kS` | V to break static friction | **A** to break static friction |
| `kV` | V per mechanism rot/s | **A** per mechanism rot/s |
| `kA` | V per mechanism rot/s² | **A** per mechanism rot/s² |
| `kG` | V to hold against gravity | **A** to hold against gravity |
| `kP` (velocity loop) | V per rot/s of error | **A** per rot/s of error |
| `kP` (position loop) | V per rotation of error | **A** per rotation of error |
| `kD` (position loop) | V per rot/s of error | **A** per rot/s of error |

"Mechanism" means whatever the sensor reports after `SensorToMechanismRatio` /
`RotorToSensorRatio` — **not** motor shaft rotations. For this template:

- **Drive:** `driveConfig.Feedback.SensorToMechanismRatio = DriveMotorGearRatio`, so
  velocity is **wheel** rotations/sec. `getFFCharacterizationVelocity()` returns the same
  unit, so the fit output drops straight into `driveGains`.
- **Steer:** `FusedCANcoder` with `RotorToSensorRatio = SteerMotorGearRatio`, so position is
  **azimuth** rotations. One "rotation" of error is a full module spin.

---

## Tuning Without Redeploying

Every loop below is "change a gain, run, look, change it again". Doing that through
edit-build-deploy-enable is roughly two minutes per iteration, and a session is dozens of
iterations — most of a tuning session is spent waiting rather than tuning.

`LoggedTunableNumber` removes that. Set `Constants.tuningMode = true`, deploy once, and the
tunables appear on the dashboard under **Tuning/** where they can be adjusted while enabled.

```java
private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 4.0);

// Cheap reads: just call get()
controller.setP(kP.get());

// Expensive rebuilds: only when something actually changed
LoggedTunableNumber.ifChanged(
    hashCode(),
    () -> controller.setPID(kP.get(), 0.0, kD.get()),
    kP, kD);
```

The heading-hold gains in `DriveCommands` are wired this way as the worked example.

Two rules, both learned the hard way by teams every year:

1. **Write the values back into the code and commit them.** Dashboard values live in
   NetworkTables, so the robot reverts to its compiled-in defaults at the next reboot. They are
   recoverable from the log under `NetworkInputs/Tuning/...` if nobody wrote them down, but that
   is a recovery path, not a workflow — see `docs/tunables.md`. A session that ends without a commit
   accomplished nothing.
2. **`tuningMode` must be false for competition.** With it on, every tunable does NT traffic
   inside the 20 ms loop, and the robot is one stray dashboard edit away from a different gain
   set. Turning it off is on the pre-competition checklist.

---

## Safety

Changing gains is a **Level 3** change (`docs/change-classification.md`), and doing it
outside a deliberate tuning session is a **Hard Stop** (`CLAUDE.md`). Before any session:

- Robot on blocks with wheels free, or a clear run of carpet with someone on the e-stop
- One person on the driver station, hand on disable, for every run
- Record the *before* values so you can revert — commit them, don't rely on memory
- Change **one gain at a time**. Two at once and you learn nothing from the result
- Current limits stay where they are. They are not tuning knobs
  (`.claude/rules/00-safety.md`)

Everything below assumes you have already regenerated `TunerConstants` with Tuner X for the
real robot. Characterizing against the carried-over 2026 gear ratios produces confidently
wrong numbers.

---

## What The Template Gives You

Six entries are registered on the auto chooser in `RobotContainer`:

| Chooser entry | What it does |
|---|---|
| `Drive Wheel Radius Characterization` | Spins in place, compares gyro rotation to wheel rotation → true wheel radius |
| `Drive Simple FF Characterization` | Ramps drive current, fits `kS` and `kV` |
| `Drive SysId (Quasistatic Forward/Reverse)` | WPILib SysId slow ramp |
| `Drive SysId (Dynamic Forward/Reverse)` | WPILib SysId step |

Select one, enable in **autonomous**, let it run, then **disable to end it**. The FF and
wheel-radius routines print their results to the driver station console via
`System.out.println` in a `finallyDo` — they only print when the command ends, so you must
disable to see output.

Note what is **not** here: there is no built-in steer characterization.
`Module.runCharacterization()` holds steer at zero and ramps only the drive motor. See
[Part 2](#part-2--steer-motors).

---

## The Order — Do Not Shuffle This

Each step depends on the ones above it. Skipping ahead produces numbers that look
plausible and are wrong.

| # | Step | Sets | Why it must come after the previous |
|---|---|---|---|
| 1 | Tuner X swerve generator | CAN IDs, gear ratios, module positions | Everything reads these |
| 2 | Encoder offsets (Tuner X wizard) | `k*EncoderOffset` | Modules must point where the code thinks they point |
| 3 | Direction / inversion check | `k*Inverted` | Tuning on top of a wrong inversion wastes the session |
| 4 | **Pigeon mount calibration** | `pigeonConfigs` | Wheel radius is derived from gyro rotation |
| 5 | **Wheel radius characterization** | `kWheelRadius` | Needs a trustworthy heading |
| 5b | **Odometry accuracy check** | *(verification, sets nothing)* | Catches gear ratio / radius / module position errors before they compound into everything downstream |
| 6 | **Drive FF characterization** | `kS`, `kV` | Needs correct gear ratios |
| 7 | Drive `kA` *(if path following)* | `kA` | Feedforward must already be right |
| 8 | Drive `kP` | `kP` | Only corrects what feedforward misses |
| 9 | Steer tuning (manual) | steer `kP`/`kD`/`kS` | Independent of drive, but do it before driving hard |
| 10 | **Slip current** | `kSlipCurrent` | Needs the drivetrain running properly to push against a wall |
| 11 | **Top speed** | `kSpeedAt12Volts` | Needs correct wheel radius *and* working feedforward |
| 12 | Mass / MOI / wheel COF | `PP_CONFIG` in `Drive.java` | Path following inputs |
| 13 | PathPlanner PID | `AutoBuilder.configure()` | Needs all of the above |
| 14 | Heading-hold gains | `ANGLE_KP` / `ANGLE_KD` | Needs a drivetrain that tracks velocity |

The three easiest mistakes, all of which produce confident nonsense:

- **Measuring top speed before characterizing.** You measure how badly the drivetrain is
  tuned, then write that number down as the robot's capability.
- **Wheel radius before Pigeon calibration.** The routine divides by gyro rotation.
- **Any of it before regenerating `TunerConstants`.** The gear ratios in this template are
  the 2026 robot's.

---

## Part 0 — Before Any Characterization

### Pigeon 2 mount calibration

Everything that reads a heading depends on this: odometry, wheel radius characterization,
`joystickDriveAtAngle`, and every vision rejection filter that uses yaw rate. If the Pigeon
is not mounted perfectly flat and square — and it never is — its raw yaw axis is tilted
relative to the robot's, and the error grows the further you drive.

**Do this before the wheel radius routine, not after.**

1. Robot on a **flat, level** surface. Not the shop floor if the shop floor slopes
2. In Tuner X, select the Pigeon 2 → **Mount Calibration** → run the wizard. It measures the
   device's orientation and computes yaw/pitch/roll mount offsets
3. Record the three numbers it reports

Then decide where they live. `TunerConstants` ships with:

```java
// Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
private static final Pigeon2Configuration pigeonConfigs = null;
```

| Option | Behavior | Tradeoff |
|---|---|---|
| Leave `null` | Tuner X writes the calibration to the device; code never overwrites it | Calibration lives only on that physical device. Swap the Pigeon, lose it silently |
| Set in code | Applied on every boot | Version controlled, survives device swap and firmware reflash. **Preferred** |

To set it in code:

```java
private static final Pigeon2Configuration pigeonConfigs =
    new Pigeon2Configuration()
        .withMountPose(
            new MountPoseConfigs()
                .withMountPoseYaw(0.0)     // degrees, from Tuner X mount calibration
                .withMountPosePitch(0.0)
                .withMountPoseRoll(0.0));
```

Import `com.ctre.phoenix6.configs.MountPoseConfigs`. Recent Phoenix 6 also accepts `Angle`
measures (`Degrees.of(...)`) on these setters — either compiles; use whichever matches the
surrounding style.

**Verify it worked:** with the robot level and stationary, `Drive/Gyro/…` pitch and roll
should read ≈ 0. Then push the robot in a straight line by hand across the field — heading
should stay put. Then spin it 360° by hand and confirm it returns to its starting heading.

### Motor direction and inversion check

Before any closed-loop control, confirm at low output that each module drives and steers the
direction you expect. Getting this wrong and then tuning on top of it wastes a whole
session. Inversion changes are a Hard Stop (`.claude/rules/02-hardware.md`) — verify, don't
guess.

---

## Part 1 — Drive Motors

### Step 0: Wheel radius

**Prerequisite: the Pigeon must be mount-calibrated first** — this routine derives radius
from the ratio of gyro rotation to wheel rotation, so a wrong yaw axis produces a
confidently wrong radius. See [Pigeon 2 mount calibration](#pigeon-2-mount-calibration).

Wheel radius sets the conversion between wheel rotations and meters. A 3% radius error is a
3% error in every odometry distance, every commanded velocity (`Module` converts a m/s
setpoint to wheel rot/s by dividing by the radius), and your measured top speed. Nominal
2 in wheels measure closer to 1.9 in once the tread wears.

> It does **not** corrupt `kS`/`kV`. Those are measured in amps per *wheel rotation/sec*,
> and `SensorToMechanismRatio = DriveMotorGearRatio` means the encoder already reports wheel
> rotations regardless of radius. Do wheel radius first anyway — top speed and
> `kSpeedAt12Volts` both depend on it, and it is cheap to run.

1. Robot on carpet with room to spin, all wheels loaded normally (not on blocks — tread
   compression matters)
2. Select **Drive Wheel Radius Characterization**, enable autonomous
3. It spins in place, accelerating to 0.25 rad/s. Let it complete **at least two full
   rotations** — more is better
4. Disable. Read the console:

```
********** Wheel Radius Characterization Results **********
  Wheel Delta: 12.345 radians
  Gyro Delta: 3.210 radians
  Wheel Radius: 0.0498 meters, 1.961 inches
```

5. Put the result in `TunerConstants.kWheelRadius`

If the number is more than ~5% off nominal, something else is wrong — usually
`kDriveGearRatio`. Fix that before continuing; a bad gear ratio hides in the wheel radius
result.

### Step 0b: Verify odometry before trusting anything downstream

Wheel radius, gear ratio, and module positions each look fine in isolation and only reveal
themselves as a compounding position error. Nothing in the code checks them. Spend ten
minutes here — it is the cheapest bug you will ever find.

**Straight-line test.** Tape a start line and a mark exactly 5.00 m away.

1. Reset pose (`B` on the controller, or `drive.setPose()`)
2. Drive straight to the mark, stop, read `Odometry/Robot` X in AdvantageScope
3. It should read 5.00 m ± 5 cm (1%)

| Result | Points at |
|---|---|
| Consistently long or short by a fixed % | Wheel radius, or `kDriveGearRatio` |
| Error grows with distance | Same — confirm on a 10 m run to separate it from noise |
| Distance right but heading drifts | Pigeon mount calibration |

**Box test.** Drive a 2 m square back to the start *without rotating* (pure translation,
field-relative).

1. Reset pose, drive the square, stop where you started
2. `Odometry/Robot` should return to within ~10 cm of the origin
3. Physically measure the robot's real offset from the start mark and compare

| Result | Points at |
|---|---|
| Closes in odometry but the robot is physically elsewhere | Module positions in `TunerConstants`, or an inversion |
| Consistent rotational drift around the loop | Pigeon mount calibration, or `kSteerGearRatio` |
| Large error in one direction only | A single module — check each `k*XPos` / `k*YPos` sign |

**Spin test.** Rotate in place exactly 10 full turns by joystick, then compare
`Odometry/Robot` rotation against 3600°. More than ~2% off means gyro or steer ratio.

Redo all three after any drivetrain rebuild, tread change, or `TunerConstants`
regeneration. Record the numbers in `docs/robot-spec.md` so you can tell normal drift from a
new problem later.

### Step 1: kS and kV

`kS` is the current needed to overcome static friction. `kV` is the current per unit of
steady-state velocity — it fights viscous drag and gearbox losses.

1. Robot on carpet, straight clear path of at least 6–8 m, or on blocks if you only need
   the fit and can't get the space
2. Select **Drive Simple FF Characterization**, enable autonomous
3. It holds 0 A for 2 s to let the modules align, then ramps at `FF_RAMP_RATE` (0.1 A/s)
   while logging velocity and commanded current
4. Let it run until the robot is moving at a good fraction of free speed, then disable
5. Read the console:

```
********** Drive FF Characterization Results **********
  kS: 7.26299
  kV: 0.41537
```

6. Put them in `TunerConstants.driveGains`: `.withKS(...).withKV(...)`
   *(the numbers above show the output format — they are not values to copy)*

**Caveats worth knowing:**

- The routine fits a straight line `current = kS + kV · velocity` across **all** samples,
  including the ones collected before the robot breaks loose. Those early points sit at
  velocity ≈ 0 with a rising current and drag the intercept down, so `kS` tends to come out
  a little low. If you want a cleaner number, discard samples below ~0.5 rot/s before
  fitting, or read the breakaway current directly off an AdvantageScope plot of
  `Drive/Module0/DriveTorqueCurrentAmps` against velocity.
- At 0.1 A/s the ramp takes ~30 s to reach 3 A. That is deliberate — a slow ramp keeps the
  robot quasi-static so the fit isn't polluted by acceleration. Don't raise it to save time.
- The drive torque current request is clamped to `kSlipCurrent` (80 A) by
  `driveConfig.TorqueCurrent.PeakForwardTorqueCurrent`. The ramp will never exceed it.

### Step 2: kA (optional, but do it if you run path-following)

`feedforwardCharacterization` does **not** measure `kA` — there is no acceleration term in
its fit. PathPlanner benefits from a real `kA`, so measure it separately:

1. Command a step change in velocity (e.g. 0 → 2 m/s) with `kP = 0` and your measured
   `kS`/`kV` applied
2. In AdvantageScope, plot commanded torque current against measured acceleration
   (differentiate `SwerveStates/Measured` velocity, or use the math helpers in
   `tools/wpilib-agent-tools`)
3. `kA ≈ (excess current above the kS + kV prediction) / (acceleration in rot/s²)`

**Analytical sanity check.** For a rotating mechanism with known inertia:

```
kA [A per rot/s²] = 2π · J / (G · kT)

  J  = mechanism moment of inertia [kg·m²]
  G  = gear ratio (motor rotations per mechanism rotation)
  kT = motor torque constant [N·m/A]
```

Get `kT` from WPILib rather than a datasheet you half-remember:

```java
DCMotor m = DCMotor.getKrakenX60Foc(1);
double kT = m.stallTorqueNewtonMeters / m.stallCurrentAmps;  // ≈ 0.019 N·m/A
```

For the *drive* axis, `J` has to include the robot's translational mass reflected through
the wheel, which makes the analytical route messy. Measure it empirically and use the
formula only as an order-of-magnitude check.

### Step 3: kP

With feedforward carrying the steady state, `kP` only has to correct residual error.

1. Start at `kP = 0` and confirm the robot roughly tracks velocity setpoints on
   feedforward alone. If it doesn't, `kS`/`kV` are wrong — fix them before adding `kP`
2. Raise `kP` until velocity error closes promptly, then back off ~20%
3. Watch `SwerveStates/Measured` against `SwerveStates/SetpointsOptimized` in AdvantageScope

Leave `kI = 0`. Integral on a drive velocity loop winds up during traction loss and
produces a lurch when grip returns. `kD` on a velocity loop amplifies encoder noise —
leave it at 0 unless you have a specific reason.

**Sanity check your numbers.** Predict steady-state current at full speed and compare it
against `kSlipCurrent`:

```
top wheel speed [rot/s] = kSpeedAt12Volts / (2π · kWheelRadius)
steady-state current    ≈ kS + kV · (top wheel speed)
```

With a 2 in wheel and 4.0 m/s that is `4.0 / (2π · 0.0508) ≈ 12.5` wheel rot/s. Whatever
`kS + kV · 12.5` comes to should sit **well under** `kSlipCurrent`, leaving the rest as
headroom for acceleration. If it lands near the slip limit, something is wrong — most often
a gear ratio or a wheel radius, not the gains.

> **A note on `kV = 0`.** The gains shipped in `TunerConstants` today have `kV = 0`, which
> would be broken under voltage control but is a legitimate starting point under
> `TorqueCurrentFOC`: at constant velocity a torque-current loop only has to supply friction
> torque, so `kS` alone can carry a surprising amount and `kP` handles the rest. Measure `kV`
> anyway — if it comes out meaningfully non-zero, your drivetrain has real viscous drag and
> feedforward should carry it rather than making `kP` fight for it every loop.

### Step 4: Slip current

`kSlipCurrent` is the torque current at which the wheels break traction. It is not a
comfort setting — it is a measurement, and it feeds three things at once in
`ModuleIOTalonFX`:

```java
driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
driveConfig.CurrentLimits.StatorCurrentLimit       = constants.SlipCurrent;
```

Set it too high and the closed loop commands torque the carpet cannot deliver — the wheels
spin, odometry drifts, and acceleration actually gets *worse*. Too low and you leave
acceleration on the table.

**Measure it after `kS`/`kV`,** so the drivetrain is running properly when you push it.

1. Robot on **competition carpet** — a different surface gives a different number, and this
   is a friction measurement
2. Push the robot squarely against a solid wall or have several people brace it. It must not
   be able to move
3. Slowly ramp drive torque current — a scratch command stepping
   `runCharacterization(amps)` upward works, or use Tuner X control on one motor
4. Watch `Drive/Module0/DriveTorqueCurrentAmps` against
   `SwerveStates/Measured` velocity in AdvantageScope
5. **Slip is the current at which measured velocity starts rising while the robot is not
   moving.** Record it
6. Put it in `TunerConstants.kSlipCurrent`

Take the number from the module that slips *first* — the drivetrain is limited by its worst
module, not its average. Re-measure whenever tread is replaced or the robot's weight changes
significantly.

> Safety: a robot that suddenly gains traction against a wall lunges. Keep hands and feet
> clear, and have someone on disable.

### Step 5: Top speed

`kSpeedAt12Volts` is used by `SwerveDriveKinematics.desaturateWheelSpeeds()` and by
`Drive.getMaxLinearSpeedMetersPerSec()`, so it bounds every joystick command and every
path. **Measure it last** — you need correct wheel radius for the conversion and correct
`kS`/`kV` for the drivetrain to actually reach its ceiling.

1. Longest clear straight run you have, on carpet, with a **fully charged** battery
2. Drive flat out in a straight line long enough for velocity to plateau
3. Read the plateau from `SwerveChassisSpeeds/Measured` `vx` in AdvantageScope, or from
   `SwerveStates/Measured` speeds
4. Take the **sustained plateau**, not the peak sample
5. Set `TunerConstants.kSpeedAt12Volts` to the measured value

Set it to what the robot actually does, not the theoretical free speed. The 2026 robot was
configured for 4.0 m/s and logs showed it topping out near 3.8 m/s, **voltage-saturated
rather than current-limited** — meaning more current limit would not have helped. An
overstated value makes the kinematics believe in headroom that does not exist, and
desaturation then scales module speeds wrongly.

Re-check on a low battery. If top speed falls off sharply, that is a battery or brownout
problem, not a tuning problem.

---

## Part 2 — Steer Motors

**There is no built-in steer characterization routine.** `Module.runCharacterization()`
pins steer at zero and ramps only drive, and `Drive.sysId` does the same. Steer is a
position loop against a fused CANcoder, and it is tuned rather than fitted.

This is a real gap. If you want a repeatable routine, add one — but the manual procedure
below is usually enough, because an azimuth is a light, low-inertia, gravity-free load.

### What the gains mean here

Steer position is in **azimuth rotations** (`FusedCANcoder` + `RotorToSensorRatio`). So:

- `kP` is **amps per azimuth rotation of error**
- `kD` is **amps per azimuth rot/s of error**
- `kS` is amps to break static friction in the azimuth

That is why a steer `kP` in the **thousands** is not a typo. One full rotation of error is
enormous; the realistic operating range is a degree or two, so divide by 360 to get a feel
for it:

```
amps per degree of error = kP / 360
degrees to saturation    = 360 · (steer stator limit) / kP
```

For the `kP = 2000` shipped today, against the 40 A steer stator limit:

```
kP = 2000 A/rot  →  1° error  ≈ 5.6 A
                 →  5° error  ≈ 28 A
                 →  7.2° error → 40 A  ← saturates the steer clamp
```

The steer request is clamped to the stator limit by
`turnConfig.TorqueCurrent.PeakForwardTorqueCurrent`, so past that error the loop saturates
and behaves like bang-bang until it gets close. That is fine and intended for an azimuth.

Recompute both numbers whenever you change `kP` or the steer stator limit — they move
together, and "kP looks big" is never the right reason to lower one.

### Procedure

1. **Robot on blocks, wheels free.** A steer loop that oscillates on carpet will hop the
   robot around
2. Start from `kP` = current value, `kD = 0`, `kS = 0`, `kV = 0`, `kA = 0`
3. Command 90° steps (drive a joystick direction, or write a scratch command that calls
   `setTurnPosition`)
4. Raise `kP` until the module snaps to the setpoint without visibly overshooting. Too high
   and it buzzes audibly at rest — back off until the buzz stops
5. Add `kD` only if there is overshoot at the end of a step. Start around `kP / 100`
6. Add `kS` last, and only if the module consistently stalls a fraction of a degree short of
   the setpoint. Raise until it settles cleanly; too much and it hunts around the target
7. `kV` compensates viscous drag while slewing. It ships at 0 — add it only if the module
   visibly lags on fast slews
8. Leave `kA = 0` — an azimuth has very little inertia and there is no acceleration
   feedforward on a plain `PositionTorqueCurrentFOC` request

### StaticFeedforwardSign

```java
.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
```

Keep this on any position loop. The default (`UseVelocitySign`) picks the direction of `kS`
from measured velocity, which is ~0 and noisy when sitting at a setpoint — the sign flips
back and forth and the module chatters. `UseClosedLoopSign` takes the direction from the
error instead, which is stable at rest.

### Verifying

Plot `Drive/Module0/TurnTorqueCurrentAmps` against `SwerveStates/Measured` angle during a
90° step:

- Current spikes to the clamp, then tapers as error closes → healthy
- Current stays pinned at the clamp after arrival → `kP` too high, or something is binding
- Current near zero with residual error → `kP` too low, or `kS` needed

---

## Part 3 — Generalized: Any New Mechanism

> Gains are step 2 of 5. `docs/new-mechanism-bringup.md` has the full order — the gearing,
> inversion, soft limits and encoder calibration that must be right first, and the motion
> profile and tolerances that come after. A gain measured against a wrong gear ratio is a
> number you will throw away.

Everything above is drive-specific. Here is the general procedure for a 2027 mechanism.

### First: pick the control mode

| The mechanism… | Control mode | Request |
|---|---|---|
| spins, and you care about *speed* (shooter wheel, roller, feeder) | **Velocity** | `VelocityTorqueCurrentFOC` |
| moves to a place and holds (arm, elevator, pivot, turret, hood) | **Position** | `PositionTorqueCurrentFOC` or `MotionMagicTorqueCurrentFOC` |
| just needs to run (simple roller, belt) | **Open loop** | `VoltageOut` or `TorqueCurrentFOC` — no gains to tune |

Set `SensorToMechanismRatio` so the sensor reads in the units you want to command. Do this
*before* characterizing, or every gain you measure will be off by the gear ratio.

### Velocity control

**Order: kS → kV → kA → kP. Never start with kP.**

1. **kS** — command increasing current with the mechanism free. Record the current at which
   it first moves. That is `kS` in amps
2. **kV** — command a series of steady currents, let velocity settle at each, record
   `(velocity, current)` pairs. Fit a line; the slope is `kV` in A per mechanism rot/s and
   the intercept should land near your `kS`. Four or five points across the operating range
   beats a single measurement
3. **kA** — step the current up and measure the resulting acceleration.
   `kA = Δcurrent / acceleration [rot/s²]`. Cross-check against
   `kA = 2π·J / (G·kT)` if you know the inertia. Skip `kA` if the mechanism only ever runs
   at steady state — a shooter wheel that spins up once per shot doesn't need it
4. **kP** — with feedforward doing the work, add just enough to close residual error.
   Raise until it responds promptly, then back off ~20%
5. **kI = 0.** If there is steady-state error, `kS`/`kV` are wrong. Integral on a velocity
   loop winds up during a jam and dumps the accumulated output the instant the jam clears
6. **kD = 0** on velocity loops. It differentiates a signal that is already a derivative

**Readiness check.** Give the subsystem an `isAtVelocity()` with an explicit tolerance and
log it. Commands gate on it (`.claude/rules/03-commands.md`), and you cannot debug a
sequence from a log without it.

### Position control

**Order: kG (if gravity) → kS → kP → kD. `kV`/`kA` only matter with MotionMagic.**

1. **kG** — only for mechanisms that fight gravity (arm, elevator). With the mechanism held
   at rest, find the current that exactly holds it against gravity. Set the matching
   `GravityType`:
   - `Elevator_Static` — constant force regardless of position
   - `Arm_Cosine` — force varies with `cos(angle)`, so **the cosine reference must be
     horizontal**. When mechanism zero is the stow position instead, pass the angle at which
     the arm is level to `MotorConfig.Builder.gravityOffset(...)` rather than redefining zero;
     it negates it into Phoenix's `Slot0.GravityArmPositionOffset`, which is added to the
     position before the cosine is taken. Get the
     sensor offset right or `kG` fights you through the range of motion
2. **kS** — additional current to break static friction, beyond `kG`
3. **kP** — amps per **mechanism rotation** of error. Remember the scale: if your mechanism
   only ever moves a few degrees, a "reasonable-looking" `kP` of 10 gives you 0.17 A at 1°
   of error, which is nothing. Compute what your typical error should produce before
   deciding a value looks wrong
4. **kD** — add only to damp overshoot. Start around `kP/100` and adjust
5. **kI = 0.** On a position loop with gravity, integral windup during a stall is how
   mechanisms get bent
6. **`StaticFeedforwardSign.UseClosedLoopSign`** on every position loop, for the reason in
   Part 2

**Use MotionMagic for anything with real inertia or travel.** A raw
`PositionTorqueCurrentFOC` step commands maximum effort instantly. `MotionMagic` generates a
trapezoidal profile:

```java
config.MotionMagic.MotionMagicCruiseVelocity = ...;  // mechanism rot/s
config.MotionMagic.MotionMagicAcceleration  = ...;   // mechanism rot/s²
```

With MotionMagic, `kV` and `kA` feed forward the *profile's* velocity and acceleration, so
they become worth measuring — same methods as the velocity section. Cruise and acceleration
are Hard Stop items (`.claude/rules/00-safety.md`): too aggressive damages mechanisms.

### Sanity checks before you trust any gain set

- **Does the magnitude make sense?** Compute what your typical error produces in amps and
  compare against the current limit. If typical operation saturates the limit, the gain is
  wrong
- **Does it hold under load?** Gains tuned on blocks routinely fall apart under game-piece
  load or defense
- **Does it survive a brownout?** Voltage sag reduces available torque current. Watch
  behavior at 9 V, not just 12.5 V
- **Is it logged?** Log the setpoint, the measurement, and the torque current. A gain set
  you can't see in a match log is a gain set you can't debug

---

## Recording Results

Characterization output is measurement data. Losing it means re-running on robot time.

- Commit gains in the same commit as a message saying what was measured and how
  (`.claude/rules/05-git.md` — the *why*, not the *what*)
- Record the date, the robot, battery voltage, and surface (carpet vs blocks) — these
  numbers are not portable between them
- Put final values in `docs/robot-spec.md` alongside the subsystem they belong to
- Save the log. `tools/log-sync` gets it off the robot; a characterization run is worth
  keeping

Template for a tuning commit message:

```
Set drive kS/kV from 2027-01-15 characterization run

Measured on carpet, full battery, after regenerating TunerConstants for the
new gearbox. kS 2.90 A, kV 1.05 A per wheel rot/s (was 7.26299 / 0.0 placeholder).

Behavioral consequence: feedforward now carries steady-state velocity without
kP contribution; path following should show less cross-track error.
Failure mode if wrong: under-driven feedforward makes kP do the work, which
shows up as sluggish response and overshoot on direction changes.
```

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Gains "look insanely large" | You are reading amps as volts. A steer `kP` in the thousands is normal — it is A per *azimuth rotation* |
| Characterization prints nothing | You never disabled. Results print in `finallyDo`, on command end |
| kV is way off from last season | Gear ratio or `SensorToMechanismRatio` changed, or wheel radius is stale — redo Step 0 |
| Wheel radius result is absurd | `kDriveGearRatio` is wrong, or the gyro isn't reporting (check `Drive/Gyro/connected`) |
| Mechanism buzzes at rest | `kP` too high, or `kS` with `UseVelocitySign` on a position loop |
| Settles just short of target every time | Needs `kS`; on an arm, needs `kG` |
| Fine on blocks, bad on the field | Tuned unloaded. Re-tune under real load |
| Fine at match start, bad at the end | Battery sag. Torque current is limited by available voltage |
| SysId barely moves the robot | Its "volts" are amps here. Pass an explicit `SysIdRoutine.Config` with larger numbers |

---

## Related

- `.claude/skills/pid-tune` — sim-first tuning loop; proposes gains, you confirm
- `.claude/prompts/tune-constants.md` — template for handing measured values to an agent
- `.claude/rules/00-safety.md` — why gains are a Hard Stop
- `.claude/rules/02-hardware.md` — what every motor must log, including torque current
- `docs/change-classification.md` — risk tiers
