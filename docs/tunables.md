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
private static final LoggedTunableNumber angleKp = new LoggedTunableNumber("Drive/HeadingKp", 8.5);
private static final LoggedTunableNumber angleKd = new LoggedTunableNumber("Drive/HeadingKd", 0.3);

// Inside the command, per loop:
LoggedTunableNumber.ifChanged(
    hashCode(), () -> controller.setPID(angleKp.get(), 0.0, angleKd.get()), angleKp, angleKd);
```

`DriveCommands` heading-hold is wired this way as the worked example — read it before writing
your own.

`ifChanged` is used here not because `setPID` is expensive but because a `ProfiledPIDController`
rebuild resets internal state. For a plain `setP()` on a controller you own, calling it
unconditionally is fine.

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

This is a RAM-side helper. It has nothing to do with TalonFX gains.

---

## Pattern B — Device-side (TalonFX Slot0)

Declare the gains as tunables in the subsystem's constants file, then have the **IO
implementation** push them when they move.

```java
// MyMechanismConstants.java
public static final LoggedTunableNumber KP = new LoggedTunableNumber("MyMechanism/kP", 0.0);
public static final LoggedTunableNumber KS = new LoggedTunableNumber("MyMechanism/kS", 0.0);
public static final LoggedTunableNumber KV = new LoggedTunableNumber("MyMechanism/kV", 0.0);

/** The watch list for ifChanged. Keep it next to the declarations — a gain missing from
 *  here still tunes on the dashboard but never reaches the motor, which looks like a dead
 *  gain rather than a missing array entry. */
public static final LoggedTunableNumber[] TUNABLE_GAINS = {KS, KV, KP};
```

```java
// MyMechanismIOReal.java — called from updateInputs()
private void updateTunedGains() {
  if (!Constants.tuningMode) {
    return;
  }
  LoggedTunableNumber.ifChanged(
      hashCode(),
      () -> {
        // Rebuilding the config re-reads the gain accessors, which read the tunables.
        Slot0Configs gains = MyMechanismConstants.getFXConfig().Slot0;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(gains));
      },
      MyMechanismConstants.TUNABLE_GAINS);
}
```

Five things in that block are deliberate.

**`apply(config.Slot0)`, not `apply(config)`.** The Slot0 overload writes only the gain block.
Applying a whole `TalonFXConfiguration` would also rewrite current limits, soft limits, inversion
and feedback ratios — so a full apply in a tuning loop silently reverts any other change made on
the device, including anything typed into Tuner X.

**The Slot0 block comes from `getFXConfig()`, not hand-built.** This one is easy to get wrong and
the failure is silent. A hand-built `new Slot0Configs().withKS(...).withKP(...)` carries only the
gains you list, leaving `GravityType` and `StaticFeedforwardSign` at their defaults. Tuning kP on
an arm would then quietly switch its gravity compensation from `Arm_Cosine` to `Elevator_Static`,
and the arm would start sagging at angles where it used to hold — while the log shows only that
you changed kP. Taking the whole block from the canonical config means the gains and their
modifiers cannot disagree.

**Gated on `tuningMode`.** In competition the block returns immediately and does zero CAN work.
The gains the robot runs are the compiled-in defaults, applied once at construction.

**`ifChanged`, not every loop.** `apply()` is a blocking CAN transaction. Calling it per loop
would blow the 20 ms budget continuously. Even gated to changes, expect the loop it fires on to
overrun — acceptable in a tuning session, which is the only time it can happen.

**`hasChanged` returns true on its first call.** The block fires once on the first loop after
enabling. That is harmless — it re-applies the same values the constructor already wrote — but
it means the first "change" you see in a log is not a change.

### It belongs in the IO layer

Gains are hardware configuration, so the `apply()` call goes in `MyMechanismIOReal`, next to
every other Phoenix call. `MyMechanism.java` stays free of hardware imports and log replay keeps
working. `MyMechanismIOSim` simply does not implement it — sim uses the nested `Sim` gains,
which are a different set for good reason.

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

Every `IOReal` constructor does this:

```java
PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(MyMechanismConstants.getFXConfig()));
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

**Write the values back into the code and commit them.** Dashboard values live in NetworkTables
and are gone at the next reboot. A tuning session that ends without a commit accomplished
nothing. This is the single most common way teams lose an afternoon.

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

## Naming

The dashboard key is what someone reads at 11 pm with the robot on blocks. Use
`Mechanism/Gain`:

```
Elevator/kP          Elevator/kG          Drive/HeadingKp
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
