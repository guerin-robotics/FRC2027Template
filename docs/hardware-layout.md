# Hardware Layout Reference

Source of truth for physical robot hardware. Keep this updated as hardware changes —
in 2026 this table was the fastest way to answer "what is on CAN ID 14?" mid-debug.

> **BRANCH `feature/rebuilt2026-port`: THIS DESCRIBES THE 2026 COMPETITION ROBOT.**
>
> Every table here is the 2026 robot as the Rebuilt2026 `main` code configures it. This
> branch is a port of that robot onto the 2027 template and is never merged to `main`;
> on `main` this file is a 2027 skeleton.
>
> Where the Rebuilt2026 `docs/hardware-layout.md` disagreed with its own code, the
> code won — see "Corrections to the 2026 document" at the bottom.

---

## CAN Bus: CANivore (`"Canivore"`)

Swerve devices belong here. The bus is low-latency and deterministic, which odometry
needs. Anything else should have a documented reason for being on it.

*2026 COMP values, as in `generated/TunerConstants.java`.*

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
| 30 | Flywheel Leader | TalonFX (Kraken X60) | Flywheel | Inverted (CW+) |
| 31 | Flywheel Follower 1 | TalonFX (Kraken X60) | Flywheel | Aligned with leader |
| 32 | Flywheel Follower 2 | TalonFX (Kraken X60) | Flywheel | Aligned with leader |
| 33 | Flywheel Follower 3 | TalonFX (Kraken X60) | Flywheel | Opposes leader |
| 34 | Flywheel Follower 4 | TalonFX (Kraken X60) | Flywheel | Opposes leader |
| 35 | Hood Motor | TalonFX (Kraken X60) | Hood | Fused with CANcoder 50 |
| 36 | Upper Feeder | TalonFX (Kraken X60) | Upper Feeder | |
| 37 | Prestage Right | TalonFX (Kraken X60) | Prestage | Follower — opposes leader (38) |
| 38 | Prestage Left | TalonFX (Kraken X60) | Prestage | Leader, inverted (CW+) |
| 39 | Lower Feeder | TalonFX (Kraken X60) | Lower Feeder | Brake, unlike every other roller |
| 40 | Transport | TalonFX (Kraken X60) | Transport | |
| 41 | Intake Pivot Motor | TalonFX (Kraken X60) | Intake Pivot | Remote CANcoder 44 |
| 42 | Intake Roller Leader | TalonFX (Kraken X60) | Intake Roller | |
| 43 | Intake Roller Follower | TalonFX (Kraken X60) | Intake Roller | Opposes leader |
| 44 | Intake Pivot Encoder | CANcoder | Intake Pivot | Offset −0.58, CW+, discontinuity 0.625 |
| 50 | Hood Encoder | CANcoder | Hood | Offset −0.16, CW+, discontinuity 0.625 |

29 devices in Phoenix Tuner: 13 on the CANivore, 16 here. Every entry matches a constant in
`Constants.CanIds` with a `// RIO CAN` comment on the line.

### Mechanism configuration

All TalonFX closed loops run `*TorqueCurrentFOC`, so every gain is in **amps**.

| Mechanism | Ratio | Neutral | Supply (lower after 1 s) | Stator | Gains | Profile |
|---|---|---|---|---|---|---|
| Flywheel | 36:24 | Coast | 40 A (35 A) | 45 A | kS 8, kV 0.12, kP 15 | 100 rps² ramp |
| Prestage | 24:11 | Coast | 40 A (35 A) | 45 A | kS 8, kP 8 | 100 rps² ramp |
| Upper Feeder | 24:11 | Coast | 40 A (35 A) | 60 A | kS 2, kP 13 | 120 rps² ramp |
| Lower Feeder | 24:11 | Brake | 40 A (35 A) | 40 A | kS 2, kP 16 | 120 rps² ramp |
| Transport | 33:11 | Coast | 40 A (35 A) | 40 A | kS 5, kP 4.2 | 120 rps² ramp |
| Intake Roller | 24:11 | Coast | 40 A (35 A) | 100 A | kS 1.5 | 100 rps² ramp |
| Hood | 5.33 × 122:12 | Brake | 40 A (35 A) | 20 A | kS 9, kP 4000 | 1 rps / 10 rps² |
| Intake Pivot | 45:1 | Brake | 50 A (45 A) | 80 A | kG 8 (cosine), kP 750, kD 5 | 1 rps / 2 rps² |

Soft limits: hood 0–62°. The intake pivot's are set to the encoder's wrap bounds (−0.375 to
0.625 rot), which never engage — 2026 ran it with soft limits disabled. Its designed
0–0.4 rot window is `IntakePivotConstants.DESIGN_*_LIMIT`.

---

## Vision Cameras (PhotonVision over NetworkTables)

Names must match the coprocessor config exactly, and transforms are measured from robot
center at floor level to the camera lens.

| Name | Position (X fwd / Y left / Z up) | Facing | Purpose |
|---|---|---|---|
| `RobotRight` | X=+1.0", Y=−12.171", Z=+6.438" | Yaw 270° (right) | Side AprilTags |
| `RobotLeft` | X=+1.0", Y=+12.421", Z=+6.438" | Yaw 90° (left) | Side AprilTags |
| `ShooterRight` | X=−12.572", Y=−6.125", Z=+12.509" | Yaw 180° (rear) | Shooter-side tags |
| `ShooterLeft` | X=−12.572", Y=+5.375", Z=+12.509" | Yaw 180° (rear) | Shooter-side tags |

All cameras: pitch −15° (tilted up), roll 0°.

Coprocessor: Orange Pi running PhotonVision

See `docs/VISION_DEBUG_CHECKLIST.md` §5 for the measurement procedure and the sign
conventions — a camera tilted *upward* takes a *negative* pitch.

---

## Swerve Module Geometry

*2026 values.*

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

| Property | 2026 value |
|---|---|
| Mass | 63.503 kg |
| Moment of inertia | 5.162 kg·m² |
| Wheel COF | 2.225 |
| Max drive speed at 12 V | 4.0 m/s configured (4.2 was tried and reverted) |

Note from 2026 logs: the robot topped out near 3.8 m/s and was **voltage-saturated, not
current-limited**. Measure the real number rather than trusting the configured one.

---

## Current Limits

The limits in `TunerConstants` were tuned against real 2026 motor heating and brownout
data. Carry them forward as starting points.

| Limit | Shipped value | Note |
|---|---|---|
| Drive supply | 60 A | 2026 raised this from 40 A mid-season: +12% peak accel, but 4× the brownouts |
| Steer stator | 60 A | 2026 COMP value (the template ships 40 A). Also clamps the steer torque-current request in `ModuleIOTalonFX` |
| Steer supply | 40 A | 2026 ran a 60 A stator cap here after seeing 95 A per-module peaks |
| Drive stator / slip | 80 A | `kSlipCurrent` — **must be measured**, see characterization doc § Step 4 |

Values above are what `TunerConstants` ships with today, not measurements for any particular
robot. Verify them against the file rather than trusting this table — if the two disagree,
the file is right and this table is stale.

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

Update vendor libraries at the start of each season. Do not update mid-season unless
fixing a known critical bug.

---

## Corrections to the 2026 document

The Rebuilt2026 `docs/hardware-layout.md` disagreed with the Rebuilt2026 code in two places.
This file follows the code:

| 2026 document said | 2026 code does |
|---|---|
| Intake pivot motor 41 and encoder 44 are on the CANivore | Both are constructed on `"rio"` (`IntakePivotIOReal`) |
| Flywheel followers 31–33 oppose the leader, 34 is aligned | 31 and 32 are `Aligned`, 33 and 34 are `Opposed` (`FlywheelIOPhoenix6`) |
| Wheel COF 1.2 | `Drive.WHEEL_COF = 2.225` |
