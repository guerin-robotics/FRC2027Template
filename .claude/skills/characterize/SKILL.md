---
name: characterize
description: Run and interpret characterization (wheel radius, kS/kV/kA feedforward, SysId, mechanism FF) and turn the output into gains — in AMPS, because drive and steer run TorqueCurrentFOC. Use when characterizing the drivetrain or any new mechanism, or when a fitted gain looks absurd. Proposes values with evidence; the user confirms before any gain is written.
---

# Characterize

Turn a characterization run into gains the code can use. Read
`docs/characterization-and-tuning.md` first — this skill is the loop around it.

## Units — read this before anything else

> **Drive and steer run `ClosedLoopOutputType.TorqueCurrentFOC`. Every gain and every
> characterization output is in AMPS, not volts.**

| Gain | Unit under TorqueCurrentFOC |
|---|---|
| `kS` | **A** to break static friction |
| `kV` | **A** per mechanism rot/s |
| `kA` | **A** per mechanism rot/s² |
| `kG` | **A** to hold against gravity |
| `kP` (velocity) | **A** per rot/s of error |
| `kP` / `kD` (position) | **A** per rotation / per rot/s of error |

Consequences to state to the user every time:

- `DriveCommands.FF_RAMP_RATE = 0.1` is **0.1 A/s**. Its trailing comment now says Amps/Sec; if you
  see "Volts" anywhere describing it, that is stale.
- SysId's "voltage" is **amps** here: `Drive.sysId` passes `voltage.in(Volts)` straight into a
  current request. SysId's default 7 V step and 1 V/s ramp therefore land as 7 A and 1 A/s. Pass an
  explicit `SysIdRoutine.Config`; do not trust the defaults.
- A steer `kP` in the thousands is normal — it is amps per **azimuth rotation**. It looks absurd
  only if read as volts.
- "Mechanism" means the unit after `SensorToMechanismRatio` / `RotorToSensorRatio`, **not** motor
  shaft rotations. Drive is wheel rot/s; steer is azimuth rotations.
- A mechanism that uses `Voltage` or `MotionMagicVoltage` instead is in volts. Check the actual
  control request in its `*IOReal` before labeling any number. If it differs from the drive, say so
  explicitly.

Label every number you report with its unit. Never write a bare gain.

## Hard rules

- Changing any gain is a **Hard Stop** outside a deliberate session. Every write needs the user's
  explicit confirmation; the final output of this skill is a *proposed* set with evidence.
- Do not change inversion, encoder offsets, or current limits to make a fit look better. Current
  limits are not tuning knobs; check which one (supply or stator) is binding before blaming either.
- Gains in `generated/TunerConstants.java` come from Tuner X. Propose; do not hand-edit it without
  confirmation.

## Step 1 — Which characterization

| Target | Routine | Where it lives |
|---|---|---|
| Wheel radius | `Drive Wheel Radius Characterization` auto | `DriveCommands.wheelRadiusCharacterization` |
| Drive kS / kV | `Drive Simple FF Characterization` auto | `DriveCommands.feedforwardCharacterization` |
| Drive kA, path following | SysId quasistatic + dynamic autos | `Drive.sysIdQuasistatic` / `sysIdDynamic` |
| Steer | Manual, per `docs/characterization-and-tuning.md` Part 2 | Tuner X |
| New mechanism | Part 3 of the same doc — velocity vs position | Mechanism `*IOReal` / `*Constants` |

Follow the doc's order. Do not shuffle it: wheel radius, odometry check, then kS/kV, kA, kP, slip
current, top speed. A gain fitted on a wrong wheel radius is wrong.

## Step 2 — Gather the data

In sim, only the *procedure* and the fitting code can be checked; sim motor models do not match a
real Kraken, so sim-fitted gains are not usable on the robot. Say so.

On the robot, the human runs the routine (robot on blocks for the first run, then on the floor for
wheel radius and kA). Ask for the log. Read the raw characterization data, not just the printed
result: the fit needs a clean linear region.

## Step 3 — Fit and sanity check

Fit kS/kV (and kA) from the log. Report: the value **with its unit (A, A·s/rot ...)**, the number of
samples, R², and the residual pattern. Reject and re-run if:

- the ramp never left static friction, or the data is dominated by one jump
- kS is negative or kV is zero
- the fit R² is poor (curvature means the model, or a saturated current limit, is wrong)
- the result differs from TunerConstants' existing value by an order of magnitude — a unit mix-up
  (volts read as amps) is the usual cause

Compare against the sanity checks in `docs/characterization-and-tuning.md` ("before you trust any
gain set") and against the previous values.

## Step 4 — Propose

Output a table: gain, old value, proposed value, unit, evidence. Include the behavioral consequence
("robot will accelerate X% harder at the same command") and the failure mode if wrong ("oscillation,
or an undershooting auto"). Wait for confirmation. After confirmation, write the values, record
them in the commit message using the template at the end of the doc, and re-run the routine to
check the result.

## Related

- `docs/characterization-and-tuning.md` — the full procedure and the TorqueCurrentFOC trap
- `/pid-tune` — sim-based tuning of the roboRIO-side controllers once the FF is right
- `docs/skills.md` — how to run this skill in sim and on the robot
