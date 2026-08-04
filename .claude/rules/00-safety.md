# Safety Rules

These rules exist because this code runs on a ~120 lb machine with fast-spinning wheels,
high-current motors, and a live match environment. Incorrect code changes can cause:

- Robot mechanism failure mid-match
- Unexpected motion that injures people
- Brownouts from overcurrent
- Incorrect field positioning that violates game rules

---

## Absolute Hard Stops

The following require explicit user confirmation with the word "confirm" or equivalent
before any code is written. If unsure whether a change falls here, it does.

### CAN Configuration

- **CAN IDs** — wrong ID silently controls the wrong motor
- **CAN bus assignment** (CANivore vs rio) — wrong bus means the device is invisible
- **Motor inversion flags** — an inverted drive motor causes uncontrolled swerve rotation
- **Steer encoder offsets** — incorrect offset = all swerve modules point the wrong direction

### Control Gains

- **PID Kp/Ki/Kd** — changing these on a live subsystem can cause oscillation or runaway
- **Feedforward Ks/Kv/Ka** — wrong FF causes undershoot/overshoot at velocity targets
- **MotionMagic cruise velocity/acceleration** — too aggressive causes mechanism damage

### Current Limits

- **Supply current limit** — too high risks brownout or wire fire
- **Stator current limit** — too high destroys motor windings
- **Never raise a current limit** without understanding why it was set where it is

### Safety Interlocks

- **Timeouts** on any `waitUntil` in a scoring or handoff sequence
- **Any trigger that gates whether a mechanism is allowed to fire or extend**
- **Demo/tuning mode flags** — must be false for competition

### AdvantageKit Integrity

- **`Logger.processInputs()`** — removing this breaks replay; every `periodic()` must call it
- **`@AutoLog` fields** — adding/removing fields changes the log schema; real + sim must match
- **`BatteryLogger.reportCurrentUsage()`** — removing this hides brownout data

---

## High-Risk Changes Requiring Explicit Summary

Make these changes only after stating: "This changes [X] from [old] to [new].
The behavioral consequence is [Y]. The failure mode if wrong is [Z]."

- Any change to drive or vision subsystem code
- Any change to alignment tolerances or the state that feeds them
- Any change to composite state triggers
- Any change to a scoring or handoff command sequence
- Any change to PathPlanner auto configuration (`AutoBuilder.configure()`)
- Any change to odometry or vision fusion (`addVisionMeasurement()`)
- Any change to `PhoenixOdometryThread`

---

## Failure Mode Catalog

Know these before modifying related code:

| Change | Failure mode |
|---|---|
| Wrong CAN ID | Controls wrong motor; mechanism moves unexpectedly |
| Wrong inversion | Swerve module fights itself; robot spins uncontrollably |
| Wrong encoder offset | All modules point wrong; robot drives sideways |
| Missing `processInputs()` | Replay broken; cannot diagnose post-match |
| Missing timeout on `waitUntil` | Robot hangs mid-sequence forever |
| Wrong current limit | Brownout (too high) or mechanism can't move (too low) |
| Modified vision filter threshold | Bad poses accepted; robot teleports during auto |
| Removed interlock check | Mechanism fires or extends when unsafe |
| Wrong target coordinates | Scoring or passing shots miss entirely |

---

## Carried Forward From 2026

Two failure modes that cost real match time and are worth re-checking early in 2027:

- **Loop overruns.** The 2026 robot ran closer to 30 Hz than the 50 Hz budget, with Drive
  and Vision dominating `robotPeriodic`. Watch loop timing from the first day the robot
  drives, not at the first competition. Unused `@AutoLogOutput` methods were a measurable
  part of this — AdvantageKit calls them every loop whether or not anything reads them.
- **Brownouts.** The 2026 robot logged hundreds of brownouts across a single event, with
  the battery sagging under peak draw. Current limits in `TunerConstants` were tuned
  against that data. Treat raising them as a High-risk change with a stated justification.
