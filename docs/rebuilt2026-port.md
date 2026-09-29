# Rebuilt2026 on the 2027 Template — Port Notes

> **Branch `feature/rebuilt2026-port` only. Never merged to `main`.**

This branch rebuilds the 2026 competition robot (Rebuilt2026 `main`, commit `5329ee5`,
13 Aug 2026) on the 2027 template. It is a measuring stick. It shows:

- what the template carries;
- what had to be written back on top of it;
- where the template's patterns forced a change in behavior.

It may run on the robot in the shop. It is not for events.

Everything below is a **difference from how Rebuilt2026 behaved**. If something is not
listed, it was ported to behave the same. The code carries a comment at each place where a
choice was made, so `git grep "KNOWN 2026"` and `git grep "2026 behavior"` find the in-code
side of this list.

---

## 1. Decisions for you

Each needs a yes/no. The current choice is the one that preserves 2026 behavior.

| # | Question | Current choice | The alternative |
|---|---|---|---|
| 1 | **Intake pivot soft limits.** 2026 wrote 0–0.4 rot and **disabled** them. The library will not build a rotary mechanism without soft limits. | Set to the encoder's wrap bounds (−0.375 / 0.625), which never engage. Same as 2026. | Enforce 0–0.4. Safer against over-travel, but it puts the reverse limit exactly on the deployed position. The pivot then cannot push down there, a behavior 2026 never ran with. Test it before adopting. |
| 2 | **AdvantageScope 3D models** (`Robot_Omega`, `Robot_Alpha_2026`, ~195 MB of GLB). | Not committed. `RobotModelVisualizer` still publishes `RobotModel/ComponentPoses`. | Commit them. Every clone of the template repo then downloads them, even from a side branch. Or keep them only in Rebuilt2026 and point AdvantageScope there. |
| 3 | **2026 bugs carried over** (§4). | All kept and marked in place. | Fix any of them on this branch. Each is small. |

---

## 2. Behavior that changed because of the template

### Mechanism library (`frc/lib/mechanism`)

| Area | Rebuilt2026 | This branch | Effect |
|---|---|---|---|
| Config apply | `apply()` two or three times per motor, no retry | One complete config per motor, `tryUntilOk(5)` | Configs stick on a busy bus. Nothing a correct 2026 boot did differs. |
| Intake pivot config | A trailing bare `FeedbackConfigs` reset `RotorToSensorRatio` to 1.0 | 45.0, the intended value | Not in the `RemoteCANcoder` position path, so control is unchanged. The sim gearbox is now right. |
| Peak torque current | Phoenix default, ±800 A | ±stator limit on every mechanism | The library's position is that a clamp above the stator limit is not a clamp. **Confirm on the robot:** torque-current logs should match 2026 logs for the same shot. |
| Hood `kS` sign | Phoenix default `UseVelocitySign` | Kept, stated explicitly | The library's rotary default is `UseClosedLoopSign`; with 9 A of `kS` that would change the hold at rest. |
| Hood gravity type | `Elevator_Static`, kG 0 | `Arm_Cosine`, kG 0 | None — kG is zero. |
| Hood setpoints | Raw angle to the device; the device's soft limits stop it | Clamped to 0–62° in software first | Same motion. `Hood/GoalWasClamped` is logged when it bites. |
| Logging | Per-mechanism `@AutoLog` classes | The shared `MotorIO` schema | **Log keys changed** (below). Adds sticky faults, closed-loop reference and error everywhere, a `connected` flag, and a group per follower and encoder. |
| Follower signal rates | Flywheel and prestage followers at 50 Hz; intake roller follower at the 4 Hz floor | The same, now explicit (`followerSignalHz`) | None. |
| Health | None | `FaultMonitor` alerts per mechanism: disconnect, reboot, over-temperature, hardware fault | New pit-visible alerts. |
| `isSpunUp()` after a voltage command | Compared against the last velocity target | The goal resets to 0 on voltage or stop | Only matters if something checks spin-up after a voltage command; nothing does. |
| Simulation | Separate sim gains per mechanism; hood sim had no physics; pivot was a gravity-free `DCMotorSim` | The real gains run in sim. Physics come from the library (`FlywheelSim`, `SingleJointedArmSim`). | Sim looks different. The hood sim mass and arm length are placeholders, and pivot gravity is off to match 2026. |
| Jam detection | None | Available but not configured | None. |

**Log keys** (a 2026 AdvantageScope layout will not find these without edits):

| Rebuilt2026 | This branch |
|---|---|
| `Feeder/Upper`, `Feeder/Lower` | `UpperFeeder`, `LowerFeeder` |
| `Intake Roller`, `Intake Pivot` | `IntakeRoller`, `IntakePivot` |
| `Flywheel/LeaderVelocity`, `Flywheel/Follower1…4*` | `Flywheel/Velocity`, `Flywheel/Follower0…3/*` |
| `Hood/HoodPosition` (degrees) | `Hood/Position` (rotations; `Hood/PositionDegrees` is also logged) |
| `Flywheel/targetRPM` | `Flywheel/GoalRpm` |
| IntakePivot `Mechanism2d` visualizer | Library `RotaryVisualizer` (both hood and pivot now have one) |
| Battery logger: `Flywheel/Leader`, `Prestage/Left`, `Intake/Roller-Leader`… | One entry per mechanism name |

The trigger keys (`isShootSafeZone`, `isAlignedLooser`, …) and every `RobotState/*`, `Auto/*`
and dashboard key are unchanged.

### Robot and wiring

| Area | Rebuilt2026 | This branch |
|---|---|---|
| Drive coast | Brake always | Template: coasts 3 s after disable, brakes on enable |
| Hoot logging | None | Template: `PhoenixSignalLogger` records while enabled |
| Instrumentation | None | Template: loop-time, CAN bus, match metadata, command logging |
| `AllianceFlipUtil.refresh()` | After the scheduler | Before the scheduler, so commands read this loop's alliance |
| `HubShiftUtil` alliance reads | `DriverStation.getAlliance()`, several times per loop | The per-loop cache; same answer |
| Heading-hold PID | A new controller per command | One shared, dashboard-tunable controller (template); same gains |
| Swerve gains | — | Restored to 2026 COMP (the template ships placeholders) |
| SysId in the auto chooser | Commented out | Present (template), for shop testing |
| Template drive buttons | — | Gyro reset / lock heading / X on flight stick 2–4 **removed**. They collided with 2026's trench align / intake in / intake out. |
| Flight stick port | 2 | 2 (template default is 0) |
| Inline setpoints (pass 2050 RPM, pass hood 28°, tower hood 2.5°, tower reset pose) | In `RobotContainer` | `Constants.Setpoints` |
| Auto preview / start-pose check | In `RobotContainer` | `AutoPreview` class; same keys |
| `flywheel.isFlywheelSpunUp` trigger | A field on the subsystem | `Triggers.isFlywheelSpunUp`, fed a supplier |
| Elastic layout | 2026 tabs | 2026 tabs + the template's Health tab |
| Sequences read setpoints | `HardwareConstants` directly | `Constants.Setpoints` directly — a deliberate deviation from the template's "setpoints are parameters" rule, explained in `ShootSequences` |

---

## 3. Not ported

| Item | Why |
|---|---|
| `IntakePivot.setPivotVelocity` | Unbound in 2026; the rotary library has no velocity mode |
| `IntakePivot.zeroPivotEncoder` | Unbound in 2026 |
| `HardwareConstants.TestConstants` | Nothing referenced it |
| `ALPHA_TunerConstants`, `RobotType` | Per instruction — COMP only |
| `CANUpdateThread`, `libgrapplefrc2026` | Nothing called `CANUpdateThread`; grapple was only its import |
| `deploy/apriltags/*.json`, `AprilTagLayoutType` | The template's built-in welded layout has identical tag poses (all 32 compared) |
| Sim Xbox controller on port 5 | Constructed in 2026, never read |
| AdvantageScope 3D models | See §1, decision 2 |

---

## 4. 2026 bugs carried over, unchanged

Each is marked at the code site. None would have been visible in a normal match, which is
presumably why they shipped.

| Where | What happens |
|---|---|
| Odometry reset (FS 8 / D-pad down) | A `run` command that requires the drive and never ends. After one press **the sticks stop driving** until a shoot or trench-align press takes the drivetrain. |
| `HoodDown` event marker | Requires the hood, against the event-trigger rule. Inside a group that also requires the hood it would interrupt the auto. No current auto uses it. |
| `Zones` offsets | Static finals computed through `AllianceFlipUtil` at class load. If the class loads while red, the "approaching" offsets become ~16 m. Only sim bindings read them. |
| `HubShiftUtil` resync | Offset subtracted in one place, added in the other. Fixed on Rebuilt2026 `fix/drive-loop-cost-and-hub-shift-resync` (4804579), never merged. |
| `Flywheel.shootDynamic` | Aims at the unflipped hub, uses RPM as m/s, and treats degrees as radians. The sim binding reads the hood angle once, at startup. Its command also declares no requirement, so it fights the idle default. Sim only. |
| `xCancelled` | Set on the real robot, read only by the sim bindings. |
| Sim cancellation flags | Cleared on the *real* shoot button's release. |
| `SpitSequences.spitAfterShoot` | Its final stop group contains `run` commands, so it never finishes. Unbound. |
| `FlywheelCommands.setPassVelocity` | Reads the pass target when built, not when run. Unbound. |

**2026 docs that had drifted from the 2026 code** (the port follows the code):
`hardware-layout.md` put the intake pivot on the CANivore (it is on `rio`), had the flywheel
follower directions wrong, and listed wheel COF 1.2 (code: 2.225). `driver-controls-card.md`
predated WVROX: flight stick 8 is odometry reset, not bump align; pass is 9, not 11.

---

## 5. What the template gave for free

For the "is anything missing" question, the other direction. None of this existed in
Rebuilt2026, and all of it now covers the 2026 mechanisms:

- sticky-fault logging and pit alerts on all 29 devices
- closed-loop reference and error on every closed loop
- retrying configuration
- simulation that runs the real gains
- `CANBusMonitor`, `LoopTimeMonitor`, `CommandLogger` (including per-subsystem current command)
- hoot logging bounded to enabled periods
- auto duration and overrun logging
- off-field and pose-jump detection
- a CAN ID uniqueness test and a `RobotContainer` smoke test that checks every auto's named
  commands are registered

---

## 6. Verification done

- `./gradlew build` passes: Spotless, compile, and the whole template test suite, including
  `RobotContainerSmokeTest` against all 21 autos.
- The simulation boots and runs for 75 s with no exceptions. It was **not** enabled, so no
  command or binding was exercised.
- **Not done:** any run on the robot, and any enabled simulation of the bindings.
