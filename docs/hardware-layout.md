# Hardware Layout Reference

Source of truth for physical robot hardware. Keep this updated as hardware changes —
in 2026 this table was the fastest way to answer "what is on CAN ID 14?" mid-debug.

> **NEEDS UPDATING FOR THE 2027 ROBOT.**
>
> The swerve section below reflects the values currently in
> `generated/TunerConstants.java`, which were carried over from the **2026** robot.
> They are wrong for any other drivetrain. Regenerate `TunerConstants.java` with CTRE
> Tuner X against the real robot, then rewrite this section to match.
>
> The mechanism, camera, and physical-spec sections are empty on purpose — fill them
> in as devices are assigned. Do not defer this until competition.

---

## CAN Bus: CANivore (`"Canivore"`)

Swerve devices belong here. The bus is low-latency and deterministic, which odometry
needs. Anything else should have a documented reason for being on it.

*Values below are 2026 — replace after running Tuner X.*

| CAN ID | Device | Type | Subsystem | Notes |
|---|---|---|---|---|
| 0 | Pigeon 2 | IMU | Drive | Yaw consumed by `PhoenixOdometryThread` |
| 1 | Back Left Drive | TalonFX (Kraken X60) | Drive | Inverted: no |
| 2 | Back Left Steer | TalonFX (Kraken X60) | Drive | Inverted: no |
| 3 | Back Left Encoder | CANcoder | Drive | Offset: −0.373046875 rot |
| 4 | Front Left Drive | TalonFX (Kraken X60) | Drive | Inverted: no |
| 5 | Front Left Steer | TalonFX (Kraken X60) | Drive | Inverted: no |
| 6 | Front Left Encoder | CANcoder | Drive | Offset: +0.139404296875 rot |
| 7 | Front Right Drive | TalonFX (Kraken X60) | Drive | Inverted: yes |
| 8 | Front Right Steer | TalonFX (Kraken X60) | Drive | Inverted: no |
| 9 | Front Right Encoder | CANcoder | Drive | Offset: −0.123779296875 rot |
| 10 | Back Right Drive | TalonFX (Kraken X60) | Drive | Inverted: yes |
| 11 | Back Right Steer | TalonFX (Kraken X60) | Drive | Inverted: no |
| 12 | Back Right Encoder | CANcoder | Drive | Offset: −0.212646484375 rot |

**Encoder offsets are calibration data.** They were produced by physically zeroing each
module and cannot be guessed or derived. Only the Tuner X wizard should change them.

---

## CAN Bus: RIO (`"rio"`)

Everything that is not odometry-critical.

| CAN ID | Device | Type | Subsystem | Notes |
|---|---|---|---|---|
| | | | | |

Add a row per device as it is wired. Match each entry to a constant in
`HardwareConstants.CanIds` with a `// CANivore` or `// RIO CAN` comment on the line.

---

## Vision Cameras (PhotonVision over NetworkTables)

Names must match the coprocessor config exactly, and transforms are measured from robot
center at floor level to the camera lens.

| Name | Position (X fwd / Y left / Z up) | Facing | Purpose |
|---|---|---|---|
| `camera0` | TODO | TODO | TODO |
| `camera1` | TODO | TODO | TODO |
| `camera2` | TODO | TODO | TODO |
| `camera3` | TODO | TODO | TODO |

Coprocessor: TODO (2026 used an Orange Pi running PhotonVision)

See `docs/VISION_DEBUG_CHECKLIST.md` §5 for the measurement procedure and the sign
conventions — a camera tilted *upward* takes a *negative* pitch.

---

## Swerve Module Geometry

*2026 values — regenerate with Tuner X.*

| Module | X | Y |
|---|---|---|
| Front Left | +11 in | +11 in |
| Front Right | +11 in | −11 in |
| Back Left | −11 in | +11 in |
| Back Right | −11 in | −11 in |

Drive gear ratio: 7.03125:1
Steer gear ratio: 26.09:1
Wheel radius: 2 in (0.0508 m)
Coupling ratio: 4.5 (drive motor rotations per steer rotation)

---

## Robot Physical Specs (PathPlanner)

These feed `PP_CONFIG` in `Drive.java`. Wrong values make path following inaccurate in a
way that is hard to distinguish from bad gains.

| Property | 2026 value | 2027 value |
|---|---|---|
| Mass | 63.503 kg | TODO — weigh the robot |
| Moment of inertia | 5.162 kg·m² | TODO |
| Wheel COF | 2.225 | TODO |
| Max drive speed at 12 V | 4.0 m/s configured | TODO |

Note from 2026 logs: the robot topped out near 3.8 m/s and was **voltage-saturated, not
current-limited**. Measure the real number rather than trusting the configured one.

---

## Current Limits

The limits in `TunerConstants` were tuned against real 2026 motor heating and brownout
data. Carry them forward as starting points.

| Limit | 2026 value | Note |
|---|---|---|
| Drive supply | 60 A | Raised from 40 A mid-season: +12% peak accel, but 4× the brownouts |
| Steer stator | 60 A | Caps per-module peaks at 60 A (was hitting 95 A before the cap) |
| Steer supply | 40 A | |
| Slip current | 80 A | |

**Never raise a current limit without stating why the old one was insufficient.**

---

## Electrical

| Item | Notes |
|---|---|
| Log storage | USB drive at `/U/` — must be present for match logs |
| PDH | Powers all subsystems; voltage logged via `BatteryLogger` |
| RoboRIO | Main control; `"rio"` CAN bus terminates here |
| CANivore | Separate USB-CAN adapter; `"Canivore"` bus |

2026 logged 491 brownouts across 23 matches, with the battery sagging to ~6.3 V under a
390 A p95 draw. Track this from the first practice match, not from the first event.

---

## Vendor Library Versions

*All 2026 — update every one at the start of the season.*

| Library | Version | Config file |
|---|---|---|
| WPILib | 2026.2.1 | `build.gradle`, `settings.gradle` (`frcYear`) |
| CTRE Phoenix 6 | 26.3.0 | `vendordeps/Phoenix6-26.3.0.json` |
| PhotonVision | v2026.3.4 | `vendordeps/photonlib.json` |
| AdvantageKit | see vendordep | `vendordeps/AdvantageKit.json` |
| PathPlannerLib | see vendordep | `vendordeps/PathplannerLib.json` |
| Studica (NavX) | see vendordep | `vendordeps/Studica.json` |
| GrappleRobotics | see vendordep | `vendordeps/libgrapplefrc2026.json` |

Update vendor libraries at the start of each season. Do not update mid-season unless
fixing a known critical bug.
