# Bringing Up a New Mechanism

The order to work through after a subsystem compiles and before it is trusted in a match.

`docs/characterization-and-tuning.md` covers **how to measure a gain**. This covers **what to
set, in what order, and how to know each step worked** — including the values that must be
right before a gain means anything at all.

**Do not skip ahead to gains.** Every step here feeds the next. A kP measured against a wrong
gear ratio is a number you will throw away.

---

## Phase 0 — Values that must be right before it moves

None of these are tunable and none are guessable. Get them from CAD, from the mechanism, or
by measuring. The scaffold refuses to compile without most of them for exactly this reason.

### Gearing

`GEAR_RATIO` is the **total** reduction, motor rotations per mechanism rotation. Phoenix needs
it split at the encoder, and the split depends on where the encoder physically sits:

| Where the encoder is | `ROTOR_TO_SENSOR_RATIO` | `SENSOR_TO_MECHANISM_RATIO` |
|---|---|---|
| Motor encoder only | `1.0` | `GEAR_RATIO` |
| CANcoder on the mechanism itself | `GEAR_RATIO` | `1.0` |
| CANcoder on an intermediate shaft | ratio above it | ratio below it |

The two must multiply back to `GEAR_RATIO`. The question that resolves the third case: **does
the encoder turn 1:1 with the thing you are measuring?** If anything geared sits after it, it
does not.

### Does full travel stay under one sensor rotation?

Ask this before fitting an absolute encoder at all. An absolute reading spans one rotation and
then repeats, so a mechanism that travels further cannot say which turn it is on.

| Mechanism | Travel | Per sensor rotation | Turns | Verdict |
|---|---|---|---|---|
| Intake pivot | 95° | 360° | 0.26 | Absolute encoder works |
| Elevator, drum-mounted | 24 in | 6.28 in | 3.82 | Ambiguous — 0.5 could be 3.1, 9.4, 15.7 or 22.0 in |

Over one turn: gear the sensor down until its range covers the travel, or drop the absolute
encoder and use the motor's own. A relative encoder then needs zero established some other way —
see Phase 1.

**This never appears in simulation**, because the sim mechanism always starts at zero and the
ambiguity is invisible from there. It appears the first time the robot is powered on with the
mechanism somewhere other than the bottom.

### The first sanity check you get for free

`MAX_SPEED_RPM` is computed, not typed. **Read it before going further.**

```
Kraken X60 FOC, 15:1  →  5800 / 15  ≈ 387 RPM at the mechanism
Kraken X44 FOC, 45:1  →  7368 / 45  ≈ 164 RPM at the mechanism
```

If that number is nowhere near what the mechanism needs to do, the gear ratio is wrong and
everything downstream is built on it. Catching it here costs a minute; catching it on the
practice field costs an afternoon.

### Linear mechanisms only

Two numbers that cannot be derived from the gear ratio, and both scale every reported height:

- **Drum pitch diameter** — where the rope or belt actually rides, not the outer flange
- **Stage count** — the rigging multiplier. A 2-stage cascade travels twice per drum rotation

A wrong stage count is off by an exact integer factor, which is at least a recognisable tell.
A wrong drum diameter reads a few percent high forever and looks like a tuning problem.

### Limits and mode

- **Current limits** are a protection boundary, not a performance knob. Leave them where the
  scaffold puts them until you have a measured reason.
- **Neutral mode `Brake`** for anything gravity acts on. A coasting elevator falls when
  disabled.

---

## Phase 1 — First motion, open loop

Use the `runAtVoltage` factory. **No closed loop yet** — a wrong inversion under a position
loop drives the mechanism into its stop at full torque instead of drifting there slowly.

Work at low output, 1–2 V, with a hand on disable.

1. **Direction.** Does positive output do what the constant says it does? If not, flip
   `INVERTED` — do not compensate by negating setpoints, which leaves the next person with a
   mechanism whose sign conventions disagree with its config.

2. **Follower alignment.** Watch both motors' stator current. Similar current and the
   mechanism moves: correct. High current on both and nothing moves: they are fighting, so
   flip `FOLLOWER_ALIGNMENT` between `Aligned` and `Opposed`. Do not leave this running.

3. **Encoder direction and reading.** Move the mechanism by hand or at low output and confirm
   the logged position increases in the direction you call positive.

4. **Soft limits.** Drive slowly to each hard stop, read the position, back off a margin, and
   write those in. Until these are set the mechanism has no protection but your reflexes.

5. **CANcoder calibration**, if there is one:
   - **Magnet offset** — move to a known position, read the raw sensor, set the offset so the
     reading matches reality.
   - **Discontinuity point** — `0.5` for arms and pivots, which puts the wrap half a turn from
     anywhere they travel. `1.0` puts the wrap at zero, which is usually the stow position, so
     the reading flips between ~0.0 and ~1.0 at rest and a position loop chases a full rotation
     of phantom error.

6. **Zeroing, for any mechanism without an absolute encoder.** Position is relative, so it
   means nothing until it is established. Power-up assumes the mechanism is at zero, which is
   wrong the moment someone powers on with it raised or moves it by hand while disabled.

   Build a routine that drives into a hard stop and declares that position zero — the commented
   ZEROING blocks in `template/src/.../example/ExampleCommands.java`,
   `ExampleSubsystem.java`, `ExampleSubsystemIO.java` and `ExampleSubsystemIOReal.java` are the
   worked example. Three things make it safe:

   - It is a **command**, not a subsystem method, so it requires the subsystem and the scheduler
     interrupts it if the operator commands a position mid-run. The subsystem holds only the
     primitive: "declare the current position to be zero."
   - It detects the stop with **current high AND velocity near zero**, debounced, so it does not
     trigger on the inrush current at the instant the motor starts.
   - On timeout it **gives up without zeroing**. A zero taken at an unknown position is worse
     than no zero, because the mechanism believes it for the rest of the match — including in
     its soft limits.

   Log whether zeroing has happened and register it with `FaultMonitor`, so nobody silently
   trusts the power-up assumption.

7. **Gravity reference**, for rotating mechanisms only. `Arm_Cosine` scales kG by
   `cos(position + offset)` and expects the peak at horizontal. If mechanism zero is the stow
   position — it usually is — find the angle at which the arm is level and set
   `GRAVITY_HORIZONTAL_OFFSET_DEGREES` to its negative. Left at zero, kG peaks where gravity is
   weakest.

**Nothing past this point works if Phase 1 is wrong.** Finish it.

---

## Phase 2 — Gains

Turn on `Constants.tuningMode`, deploy once, and work from the dashboard.

Full procedure per control mode is in
[characterization-and-tuning.md](characterization-and-tuning.md) Part 3. The order, briefly:

| Control mode | Order |
|---|---|
| Velocity | kS → kV → kA → kP. kI and kD stay 0 |
| Position | kG → kS → kP → kD. kI stays 0 |

Two things that catch people every year:

- **Gains are in AMPS**, not volts — every loop here is `TorqueCurrentFOC`. A voltage-era
  number typed into an amps-era gain will surprise you.
- **Feedforward does the work; kP cleans up what is left.** If kP is carrying the motion, kS
  and kV are wrong and you are tuning around them.

---

## Phase 3 — Motion profile

Only after gains. A profile tuned against bad gains is tuned against the wrong plant.

The template starts you at:

| Mechanism | Cruise velocity | Acceleration |
|---|---|---|
| Linear position | `MAX_SPEED_RPM / 2` | 9000 RPM/s |
| Rotation position | 60 RPM | 300 RPM/s |
| Velocity | n/a — the setpoint is the cruise | 9000 RPM/s |

Both are `LoggedTunableNumber`s, so raise them from the dashboard rather than redeploying.

### Raising cruise velocity

Increase until either the motion is fast enough or the mechanism stops keeping up. **The
ceiling is not `MAX_SPEED_RPM`** — that is free speed with nothing attached, and a loaded
mechanism saturates well below it.

You have gone too far when the commanded and measured velocities diverge while torque current
sits pinned at the limit. Watch:

```
[Mechanism]/VelocityRpm             — what it is actually doing
[Mechanism]/TorqueCurrentAmps       — pinned means saturated
[Mechanism]/AtPosition              — is it still arriving
```

**Saturation does not raise an error.** The profile simply stops being followed, which reads as
sloppy tuning rather than an impossible request. This is why `MAX_SPEED_RPM` is worth knowing.

### Raising acceleration

9000 RPM/s is deliberately close to unlimited — most mechanisms reach cruise in tens of
milliseconds, so motion ends up bounded by cruise velocity and the current limit. That is the
intent for rollers and lifts.

Rotation mechanisms start at 300 for a reason: an arm that accelerates hard arrives at its hard
stop hard, and gravity torque changes with angle in a way an aggressive profile will not
respect. Raise it deliberately, in steps, watching for shock load at the ends of travel.

---

## Phase 4 — Tolerance and readiness

`POSITION_TOLERANCE_DEGREES` / `_INCHES` / `VELOCITY_TOLERANCE_RPM` decide when
`isAtPosition()` and `isAtVelocity()` go true.

**Too tight is the failure that hides.** A mechanism that never reports ready makes every
sequence gated on it run to its full timeout instead of proceeding — the robot still works, it
is just mysteriously slow, and nothing logs an error. Too loose and it acts before arriving.

Set it from what you measured in Phase 3: watch the steady-state error the mechanism actually
settles to, and set the tolerance somewhat above it.

These live in the subsystem constants, not `Constants` — how close counts as "there" is a
property of the mechanism, not a knob the drive team turns.

---

## Phase 5 — Write it back

**Dashboard values are not saved.** Every number found in Phases 2–4 has to be typed into the
constants file and committed, or the mechanism reverts at the next reboot.

If a session ends before anyone writes them down they are still recoverable — AdvantageKit logs
every tunable under `NetworkInputs/Tuning/...` on every loop, so open the log in AdvantageScope
and read them off the end of the run. That is a recovery path, not a workflow.

Then:

- Set `Constants.tuningMode = false`. It must be false for competition.
- Move any value the drive team will ask about into `Constants.Setpoints`.
- Run `./gradlew build` and commit.

---

## Quick reference

| Phase | What you set | Where it lives |
|---|---|---|
| 0 | Gear ratio, encoder split, drum geometry, current limits | Subsystem constants, static |
| 1 | Inversion, follower alignment, soft limits, magnet offset, gravity offset | Subsystem constants, static |
| 2 | kS, kV, kA, kG, kP, kI, kD | Subsystem constants, tunable |
| 3 | Cruise velocity, acceleration | Subsystem constants, tunable |
| 4 | Tolerances | Subsystem constants, static |
| 5 | Setpoints the drive team touches | `Constants.Setpoints` |

---

## Related

- [tunables.md](tunables.md) — how tunables work, and where Phoenix Tuner X fits
- [characterization-and-tuning.md](characterization-and-tuning.md) — measuring each gain
- [../.claude/skills/add-subsystem/SKILL.md](../.claude/skills/add-subsystem/SKILL.md) — scaffolding the subsystem in the first place
- [../.claude/rules/00-safety.md](../.claude/rules/00-safety.md) — gains outside a deliberate tuning session are a Hard Stop
