# Tunables — Changing Values Without Redeploying

Edit, build, deploy, enable, test is about two minutes. A tuning session is dozens of
iterations, so most of the session is spent waiting rather than tuning. Tunables remove that
loop.

This document covers what a tunable is, the two different kinds of gain and why they need
different code, and where Phoenix Tuner X fits.

**Prerequisite:** `docs/characterization-and-tuning.md` covers what values to look for and in
what order. This covers the mechanics of changing them live.

---

## The one-minute version

```java
// 1. Declare. The default is what ships in competition.
private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 4.0);

// 2. Read. Cheap — just call get().
controller.setP(kP.get());

// 3. Or, when reacting is expensive, react only to changes.
LoggedTunableNumber.ifChanged(hashCode(), () -> reapplyGains(), kP, kD);
```

Set `Constants.tuningMode = true`, deploy once, and every tunable appears on the dashboard
under **Tuning/**. Adjust while enabled, watch the response, repeat.

With `tuningMode = false` the dashboard entry is never created and `get()` returns the
compiled-in default. Tunables cost nothing in competition.

---

## Two kinds of gain, two different problems

This is the distinction that matters, and it is why one pattern is not enough.

| | Lives in | Changing it costs | Example |
|---|---|---|---|
| **RAM-side** | A Java object on the roboRIO | A field write | `ProfiledPIDController` in `DriveCommands` |
| **Device-side** | Flash on the TalonFX | A CAN transaction | `Slot0` gains for any closed loop the Talon runs |

A `PIDController` gain is a `double` in memory. Setting it is free, so you can call
`setP(kP.get())` every loop and never think about it.

A TalonFX Slot0 gain is not on the roboRIO at all. The motor controller runs its own 1 kHz
closed loop against gains stored on the device, which is exactly why it performs better than
one running at 50 Hz over CAN. The cost is that changing a gain means sending it over the
bus.

---

## Pattern A — RAM-side (WPILib controllers)

Read directly. No ceremony needed.

```java
private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 4.0);
private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0.1);

// Inside the command, per loop:
LoggedTunableNumber.ifChanged(
    hashCode(), () -> controller.setPID(kP.get(), 0.0, kD.get()), kP, kD);
```

`ifChanged` is used here not because `setPID` is expensive but because a `ProfiledPIDController`
rebuild resets internal state. For a plain `setP()` on a controller you own, calling it
unconditionally is fine.

For a *profiled* controller, do not hand-roll this — use `LoggedTunableProfiledPID` below.

### `LoggedTunableProfiledPID`

For a profiled controller, `frc.lib.LoggedTunableProfiledPID` bundles the five tunables — kP, kI,
kD, max velocity, max acceleration — so the declarations and the `ifChanged` wiring are not
rewritten per mechanism:

```java
private final LoggedTunableProfiledPID heading =
    new LoggedTunableProfiledPID("Drive/Heading", 8.5, 0.0, 0.3, 12.0, 20.0);

// Every loop, before calculate():
heading.updateGains();
double omega = heading.calculate(currentHeading, targetHeading);
```

Gains and constraints are checked separately inside it, because `setConstraints` rebuilds the
motion profile and doing that on every kP nudge would discard profile state mid-motion.

`DriveCommands` heading-hold is wired this way as the worked example — read it before writing
your own. Note it declares the controller `static`: two instances would publish duplicate
dashboard keys for the same gain, and sharing is safe because every command using it requires
the drive subsystem, so only one can run at a time.

This is a RAM-side helper. It has nothing to do with TalonFX gains.

---

## Pattern B — Device-side (TalonFX Slot0)

Declare the gains as tunables in the subsystem's constants file, then have the **IO
implementation** push them when they move.

**For a mechanism built on `frc/lib/mechanism`, none of this is code you write.** State the
gains and the profile in the `MotorConfig`, and `Mechanism` does the rest: it builds a
`LoggedTunableNumber` for every gain and every profile value, watches them all together, and pushes
`Slot0` and `MotionMagic` to the device when any of them moves.

```java
// MyMechanismConstants.java — the whole of it
.gains(new Gains(kP, kI, kD, kS, kV, kA, kG))
.motionProfile(MotionProfile.of(cruiseVelocity, acceleration))
```

They appear on the dashboard under `Tuning/MyMechanism/Gains/` and `Tuning/MyMechanism/Profile/`.

The rest of this section is why that machinery is shaped the way it is — worth reading before
changing it, and worth reading if you are tuning something that is not a mechanism, such as a
WPILib `PIDController` in a command. Here is what it does, once per loop:

```java
// Mechanism.pushTunables(), simplified
LoggedTunableNumber.ifChanged(
    hashCode(),
    () -> {
      io.setGains(Gains.fromTunables(gainTunables));
      io.setMotionProfile(MotionProfile.fromTunables(profileTunables));
    },
    allTunables);
```

Five things in that are deliberate.

**`apply(config.Slot0)`, not `apply(config)`.** The Slot0 overload writes only the gain block.
Applying a whole `TalonFXConfiguration` would also rewrite current limits, soft limits, inversion
and feedback ratios — so a full apply in a tuning loop silently reverts any other change made on
the device, including anything typed into Tuner X.

**The Slot0 block comes from the config the device already has, not a hand-built one.** This is
easy to get wrong and the failure is silent. A hand-built `new Slot0Configs().withKS(...).withKP(...)`
carries only the gains you list, leaving `GravityType` and `StaticFeedforwardSign` at their
defaults. Tuning kP on an arm would then quietly switch its gravity compensation from `Arm_Cosine`
to `Elevator_Static`, and the arm would start sagging at angles where it used to hold — while the
log shows only that you changed kP. `MotorIOTalonFX` holds the `TalonFXConfiguration` it applied at
construction and mutates the gain numbers in place, so the gains and their modifiers cannot
disagree.

**Gains and profile are pushed together.** A mechanism that will not reach its setpoint might have
weak gains or a profile too slow to ask for the motion, and telling those apart requires moving
both. Pushing only the gains leaves the profile knobs moving on the dashboard while the mechanism
ignores them, which reads as a broken tunable rather than as a gain that does nothing.

**Gated on `tuningMode`.** In competition the block returns immediately and does zero CAN work.
The gains the robot runs are the compiled-in defaults, applied once at construction.

**`ifChanged`, not every loop.** `apply()` is a blocking CAN transaction. Calling it per loop
would blow the 20 ms budget continuously. Even gated to changes, expect the loop it fires on to
overrun — acceptable in a tuning session, which is the only time it can happen.

**`hasChanged` returns true on its first call.** The block fires once on the first loop after
enabling. That is harmless — it re-applies the same values the constructor already wrote — but
it means the first "change" you see in a log is not a change.

### It belongs in the IO layer

Gains are hardware configuration, so the `apply()` call lives in `MotorIOTalonFX`, next to every
other Phoenix call. The subsystem and its constants stay free of hardware imports and log replay
keeps working.

**Simulation gets the same gains, not a second set.** `MotorIOTalonFXSim` extends the real IO and
applies the same configuration to a simulated device, so tuning in simulation moves the numbers
that will run on the robot. The 2026 sim IOs ran a separate roboRIO-side `PIDController` against
their own gains, which is why a tuning session in sim taught you very little.

### Units

Under `TorqueCurrentFOC` control, every gain is in **amps**, not volts. `kV` is amps per
rotation/sec, `kS` is amps to overcome static friction. A tunable relabels nothing — if you type
a voltage-era value into an amps-era gain the robot will surprise you. See the units section at
the top of `docs/characterization-and-tuning.md`.

---

## Phoenix Tuner X

Tuner X talks to the devices directly, with robot code stopped or running. It is the right tool
for a specific set of jobs and the wrong tool for gains.

**Use it for:**

- **Self-test snapshot** — the fastest way to see what a device actually believes: its config,
  its faults, its firmware version, its sensor readings
- **Plotting** — real signals at device rate, better resolution than anything logged at 50 Hz
- **Control** — drive one motor with no robot code, which is how you check inversion and
  mechanical binding before any closed loop exists
- **CAN bus health** — utilization and error counts, when devices drop out intermittently
- **Firmware** and **device ID assignment**
- **The swerve generator and encoder-offset wizard** — these own `TunerConstants` and the
  offsets, and code must never write them

### The trap

**Gains typed into Tuner X are silently overwritten the next time robot code starts.**

`MotorIOTalonFX`'s constructor does this, once per mechanism:

```java
PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
```

That runs on every boot, every redeploy, every code restart. It writes the compiled-in
configuration over whatever is on the device. So the sequence "tune in Tuner X, like the result,
redeploy code" loses the tuning, and the failure presents as "the gains didn't take" rather than
as anything traceable.

This is not a bug to fix. Code owning the device configuration is what makes the robot
reproducible: any robot, freshly flashed, behaves the same as the one in the repo. A device
holding hand-typed state that exists nowhere in version control is how a mechanism ends up
behaving differently after a motor swap with nobody able to say why.

**The rule: `LoggedTunableNumber` owns gains. Tuner X is a diagnostic tool.**

If you do use Tuner X to explore a gain — which is reasonable, its plotting is better — treat
the result as a number to type into the constants file, not as a change you have made. It is not
saved until it is committed.

---

## Discipline

**Write the values back into the code and commit them.** This is manual, and deliberately so —
see below. A tuning session that ends without a commit leaves the robot reverting to its
compiled-in defaults at the next reboot.

The numbers are not lost, though. `LoggedTunableNumber` wraps AdvantageKit's
`LoggedNetworkNumber`, which logs its value every loop, so every tunable is recorded in the
`.wpilog` under:

```
NetworkInputs/Tuning/Elevator/kP
NetworkInputs/Tuning/Elevator/CruiseVelocityRpm
```

If a session ends before anyone writes them down, open the log in AdvantageScope and read them
off the end of the run. Treat that as a recovery path, not a workflow — a value that exists
only in a log is not a value the robot will use.

**`tuningMode` must be false for competition.** With it on, every tunable does NetworkTables
traffic inside the 20 ms loop, and the robot is one stray dashboard edit away from a different
gain set. It is on the pre-match checklist for this reason.

**Change one gain at a time.** Two at once and the result teaches you nothing.

**Current limits are not tuning knobs.** They were set against motor heating data. Raising one
to make a mechanism feel stronger is how windings get destroyed — see
`.claude/rules/00-safety.md`.

**Tunables are for gains and thresholds, not setpoints.** Setpoints live in
`Constants.Setpoints` where the drive team can find them. A setpoint that only exists on a
dashboard is a setpoint nobody can review.

---

## What should be tunable

Not everything benefits from a dashboard knob, and some things are actively worse for having
one.

| Value | Tunable? | Why |
|---|---|---|
| kS, kV, kA, kG, kP, kI, kD | **Yes** | The whole point — dozens of iterations per session |
| Motion Magic cruise velocity and acceleration | **Yes** | What you most want to adjust with the mechanism in front of you, and far safer to change than a gain |
| Alignment and readiness thresholds | **Yes** | Earned from watching real behaviour |
| Gear ratio, `MAX_SPEED_RPM` | No | Describes how the machine is built. If it is wrong, the fix is in CAD or the constants, not a knob |
| Current limits | No | A protection boundary, not a performance knob. A dashboard-adjustable current limit is one someone raises at 11 pm to make a mechanism feel stronger |
| Soft limits | No | A dashboard-adjustable soft limit is a mechanism you can drive into its own hard stop from a laptop |
| Magnet offset, drum geometry, inversion | No | Calibration and physical fact. Changing them at runtime makes every logged position mean something different mid-session |
| Setpoints | No — `Constants.Setpoints` | The drive team asks for these by name; they need to be reviewable in a diff |

The dividing line: **tunable if you would change it while watching the mechanism, static if
changing it would invalidate what you already measured.**

### The second test: is it read live?

A value only qualifies if something reads it *again* after the dashboard changes it. Two shapes
fail that, and both fail silently — the dashboard knob moves, the logged number moves, and the
robot does not:

| Shape | Why the knob does nothing |
|---|---|
| Captured in a constructor | `new Debouncer(DWELL, ...)` reads the dwell once, when the subsystem is built |
| Captured at command-build time | `.withTimeout(BUDGET)` bakes the number into the command object |

Device-side gains look like this too and are the exception that proves the rule: they are read
once per `apply()`, which is why Pattern B exists to re-apply them. There is no equivalent for a
`Debouncer` — short of rebuilding it every loop, which throws away the debounce state that is
the entire point.

So either make it read live, or leave it a plain `double` and say in the Javadoc that it is not
tunable and why. `ExampleLiftConstants.ZEROING_STALL_DEBOUNCE_SECONDS` and
`ZEROING_TIMEOUT_SECONDS` are the worked examples. A knob that appears to work and does not is
worse than no knob, because it costs a tuning session before anyone suspects the knob.

---

## Why writing back is manual

Nothing in this codebase writes a tuned value into a source file, and nothing persists one to
the roboRIO. That is a choice, not a gap.

A value persisted on the robot is a value that exists nowhere in version control — the same
problem described above for Tuner X, arriving by a different route. The guarantee worth having
is that **any robot, freshly flashed from this repo, behaves the way the repo says it does.**
A file on the RIO holding last Tuesday's gains breaks that quietly, and the symptom is one
robot behaving differently from another with no diff to explain it.

So the transcription step stays. It is the moment the value enters version control, and it is
cheap next to the session that produced it.

---

## Naming

The dashboard key is what someone reads at 11 pm with the robot on blocks. Use
`Mechanism/Gain`:

```
Elevator/kP          Elevator/kG          Drive/Heading/kP
```

Everything lands under a `Tuning/` table automatically. Do not repeat "Tuning" in the key, and
do not use bare `kP` — with four mechanisms tuned in one session, an unqualified key is
ambiguous exactly when it matters.

---

## Related

- `docs/characterization-and-tuning.md` — what to measure, in what order, and the amps-vs-volts
  units trap
- `.claude/skills/pid-tune/SKILL.md` — the sim-based tuning loop
- `.claude/rules/00-safety.md` — gains are a Hard Stop outside a deliberate tuning session
- `docs/pre-match-checklist.md` — where `tuningMode` gets turned back off
