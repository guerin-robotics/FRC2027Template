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

## Part 1 — Drive Motors

### Step 0: Wheel radius (do this first)

Everything downstream is scaled by wheel radius. A 3% radius error is a 3% error in every
velocity setpoint, every odometry distance, and every kV you measure. Nominal 2 in wheels
measure closer to 1.9 in once the tread wears.

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
  kS: 3.18000
  kV: 1.20000
```

6. Put them in `TunerConstants.driveGains`: `.withKS(3.18).withKV(1.2)`

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

**Sanity check with the carried-over numbers.** Drive `kV = 1.2 A` per wheel rot/s. Top
speed 4.0 m/s with a 2 in wheel is `4.0 / (2π · 0.0508) ≈ 12.5` wheel rot/s, so kV
contributes ~15 A, plus `kS` 3.18 A ≈ **18 A steady state at full speed**. Against an 80 A
slip limit that leaves plenty of headroom for acceleration. If your measured numbers imply
a steady-state draw near the slip limit, something is wrong.

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

That is why the carried-over `kP = 3750` is not a typo. One full rotation of error is
enormous; the realistic operating range is a degree or two:

```
kP = 3750 A/rot  →  1° error   = 3750 / 360  ≈ 10.4 A
                 →  5° error   ≈ 52 A
                 →  6° error   ≈ 60 A  ← saturates the 60 A steer clamp
```

The steer request is clamped to the stator limit by
`turnConfig.TorqueCurrent.PeakForwardTorqueCurrent`, so the loop saturates past ~6° of
error and behaves like bang-bang until it gets close. That is fine and intended for an
azimuth.

### Procedure

1. **Robot on blocks, wheels free.** A steer loop that oscillates on carpet will hop the
   robot around
2. Start from `kP` = current value, `kD = 0`, `kS = 0`, `kV = 0`, `kA = 0`
3. Command 90° steps (drive a joystick direction, or write a scratch command that calls
   `setTurnPosition`)
4. Raise `kP` until the module snaps to the setpoint without visibly overshooting. Too high
   and it buzzes audibly at rest — back off until the buzz stops
5. Add `kD` only if there is overshoot at the end of a step. `kD = 50` in the carried-over
   gains is doing exactly this
6. Add `kS` last, and only if the module consistently stalls a fraction of a degree short of
   the setpoint. Raise until it settles cleanly; too much and it hunts around the target
7. `kV` compensates viscous drag while slewing. The carried-over `1.94` is small relative to
   `kP`; leave it unless the module lags on fast slews
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
   - `Arm_Cosine` — force varies with `cos(angle)`, so **zero must be horizontal**. Get the
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
new gearbox. kS 2.90 A, kV 1.05 A per wheel rot/s (was 3.18 / 1.2 from 2026).

Behavioral consequence: feedforward now carries steady-state velocity without
kP contribution; path following should show less cross-track error.
Failure mode if wrong: under-driven feedforward makes kP do the work, which
shows up as sluggish response and overshoot on direction changes.
```

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Gains "look insanely large" | You are reading amps as volts. `kP = 3750` is normal for an azimuth in A/rotation |
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
