# New Season Checklist

Work through this top to bottom. Many items have dependencies.

Items marked **[done]** were completed when this template was created from the 2026 robot —
they're listed so you know they're handled, not so you redo them.

---

## Before Kickoff — Infrastructure

- [x] **[done]** Repo created from this template (not forked from the 2026 robot)
- [x] **[done]** `util/` and `frc/lib/` carried over
- [x] **[done]** `subsystems/drive/` carried over
- [x] **[done]** `subsystems/vision/` carried over with tuned filter thresholds
- [x] **[done]** `Robot` / `Constants` / `RobotContainer` in AdvantageKit template shape
- [x] **[done]** `RobotState` reduced to pose / velocity / geometry helpers
- [ ] Verify team number in `.wpilib/wpilib_preferences.json`
- [ ] Update WPILib to the 2027 release (`build.gradle` plugin version, `settings.gradle` `frcYear`)
- [ ] Update AdvantageKit to the matching 2027 version
- [ ] Update CTRE Phoenix 6, PathPlannerLib, PhotonVision, Studica vendordeps
- [ ] Confirm `./gradlew build` still passes after the vendordep bumps
- [ ] Update `FieldConstants.aprilTagLayout` to the 2027 field as soon as WPILib ships it

## Kickoff Week — Game Modeling

- [ ] Add 2027 field geometry to `frc/lib/FieldConstants.java` (scoring elements, zones, lines)
- [ ] Add scoring targets and any zone classification to `RobotState`
- [ ] Create `HardwareConstants.java` — start with an empty `CanIds` class
- [ ] Sketch the mechanism list and assign each a subsystem name before writing any code

## Robot Build — Hardware Setup

- [ ] Assign CAN IDs for all new mechanism motors; comment the bus (`// CANivore` / `// RIO CAN`)
- [ ] Zero all CANcoder magnet offsets for pivoting mechanisms
- [ ] Physically measure every camera position → update `VisionConstants.robotToCameraN`
      (they are identity placeholders right now — vision is meaningless until this is done)
- [ ] Set camera names in `VisionConstants` to match the coprocessor config
- [ ] Delete unused cameras from **all three** `RobotContainer` branches (REAL, SIM, REPLAY)
- [ ] Write `docs/hardware-layout.md` as devices are assigned — do not defer this

---

## Drivetrain Bring-Up — DO THESE IN ORDER

**Read [docs/characterization-and-tuning.md](../docs/characterization-and-tuning.md) before
starting.** Gains are in **amps**, not volts — the drivetrain runs `TorqueCurrentFOC`.

Every step depends on the ones above it. Out of order, you get numbers that look plausible
and are wrong. If you only remember one thing: **measure top speed and slip current LAST.**

### Stage 1 — Configure (nothing moves yet)

- [ ] **1.** Run CTRE Tuner X Swerve Project Generator → **replace**
      `generated/TunerConstants.java` wholesale
- [ ] **2.** Verify every swerve CAN ID against the physical robot
- [ ] **3.** Run the Tuner X encoder-offset wizard; physically zero each module
      → `k*EncoderOffset`
- [ ] **4.** Check drive and steer direction at low output → `k*Inverted`
      *(inversion changes are a Hard Stop — verify, don't guess)*

### Stage 2 — Sensors (before anything is measured from motion)

- [ ] **5. Perform Pigeon 2 mount calibration** in Tuner X on a flat, level surface
- [ ] **6.** Set the result in `TunerConstants.pigeonConfigs`:
      `new Pigeon2Configuration().withMountPose(new MountPoseConfigs()
      .withMountPoseYaw(..).withMountPosePitch(..).withMountPoseRoll(..))`
      *(it is `null` today, which leaves the calibration living only on that physical device —
      set it in code so it survives a Pigeon swap)*
- [ ] **7.** Verify: level robot reads ≈0 pitch/roll; pushing it straight by hand holds
      heading; spinning it 360° by hand returns to the start

### Stage 3 — Geometry

- [ ] **8. Run wheel radius characterization** (auto chooser → enable auto → let it spin at
      least two full rotations → **disable to print results**) → `kWheelRadius`
      *(must come after Pigeon calibration — it divides by gyro rotation)*

### Stage 4 — Feedforward and feedback

- [ ] **9. Run drive FF characterization** → `kS` / `kV` into `TunerConstants.driveGains`
      *(output is amps; no conversion needed)*
- [ ] **10.** Measure drive `kA` separately if you rely on path following — the FF routine
      has no acceleration term and does not produce it
- [ ] **11.** Tune drive `kP` — only enough to correct what feedforward misses. Leave
      `kI = 0`
- [ ] **12.** Tune steer `kP` / `kD` / `kS` manually — there is **no** built-in steer
      characterization routine

### Stage 5 — Envelope (AFTER characterization, not before)

- [ ] **13. Measure slip current** on competition carpet, robot braced against a wall, ramping
      torque current until wheels break traction → `kSlipCurrent`
      *(this also sets the stator limit and the torque-current clamp)*
- [ ] **14. Measure top speed** on a long straight with a full battery; take the sustained
      plateau, not the peak sample → `kSpeedAt12Volts`
      *(measure what the robot actually does — 2026 was configured 4.0 m/s and did ~3.8)*

### Stage 6 — Path following

- [ ] **15.** Weigh the robot, estimate MOI, measure wheel COF → `ROBOT_MASS_KG` /
      `ROBOT_MOI` / `WHEEL_COF` in `Drive.java`'s `PP_CONFIG`
- [ ] **16.** Tune PathPlanner translation/rotation PID in `AutoBuilder.configure()`
- [ ] **17.** Re-tune `ANGLE_KP` / `ANGLE_KD` in `DriveCommands` for the new chassis

- [ ] Record every measured value, the date, the battery voltage and the surface in
      `docs/robot-spec.md`. These numbers are not portable between surfaces
- [ ] Commit the gains with a message saying what was measured and how

## Mechanism Subsystems

For each mechanism, using `template/src/` as the pattern:

- [ ] `subsystems/myMechanism/io/MyMechanismIO.java`
- [ ] `subsystems/myMechanism/io/MyMechanismIOReal.java`
- [ ] `subsystems/myMechanism/io/MyMechanismIOSim.java`
- [ ] `subsystems/myMechanism/MyMechanism.java`
- [ ] `commands/MyMechanismCommands.java`
- [ ] Wire real/sim/replay in `RobotContainer` — all three branches
- [ ] Confirm `Logger.processInputs()` is called in `periodic()`
- [ ] Confirm `Robot.batteryLogger.reportCurrentUsage()` is called in `periodic()`
- [ ] Add supply-current jam detection if the mechanism can stall (2026 had none — jams were silent)
- [ ] Verify the logs appear in AdvantageScope

## Tuning — Mechanisms and Vision

Drivetrain tuning is the ordered sequence above. This section is everything else.

> Same units rule applies: check each mechanism's `ClosedLoopOutputType`. Under
> `TorqueCurrentFOC` the gains are amps. See
> [docs/characterization-and-tuning.md](../docs/characterization-and-tuning.md) Part 3 for
> the velocity-vs-position recipes.

- [ ] For each closed-loop mechanism, decide **velocity or position control** before tuning
- [ ] Velocity mechanisms: `kS` → `kV` → `kA` → `kP`, in that order. Never start with `kP`
- [ ] Position mechanisms: `kG` (if gravity) → `kS` → `kP` → `kD`. Set `GravityType`
      correctly — `Arm_Cosine` requires zero to be horizontal
- [ ] Use MotionMagic for anything with real inertia or travel; set cruise velocity and
      acceleration deliberately (they are Hard Stop items)
- [ ] `kI = 0` everywhere unless you can explain the windup behavior during a stall
- [ ] Build any distance → setpoint interpolation tables against the real field element
- [ ] Validate vision std dev scaling against real AprilTag observations
- [ ] Re-validate the vision rejection thresholds against 2027 logs (they're 2026-derived)
- [ ] Decide on the pose-estimator divergence guard (still open — see GUIDE.md section D)
- [ ] Put every timeout in a constants class; none inline

## Auto

- [ ] Register all `NamedCommands` **before** `AutoBuilder.buildAutoChooser()`
- [ ] Register `EventTrigger` objects **without** subsystem requirements
- [ ] Create PathPlanner paths and autos
- [ ] **Time every auto against the auto period** — 2026 overran and truncated the last path
      in every single match
- [ ] Consider rebuilding the auto preview / start-pose check (see `.claude/prompts/new-auto.md`)

## Driver Controls

- [ ] Update button bindings for the new mechanism layout
- [ ] Move triggers out of `RobotContainer` into `Triggers.java` once they outgrow it
- [ ] Write `docs/driver-controls-card.md` and print it for the drive team
- [ ] Drive practice before the first event, not at it

## Pre-Competition

- [ ] `./gradlew build` passes clean
- [ ] `Constants.currentMode` resolves to `REAL` on the robot
- [ ] Tuning and demo mode flags are OFF
- [ ] USB drive present so `WPILOGWriter` can write to `/U/logs`
- [ ] Battery logger reporting for every subsystem
- [ ] Check `robotPeriodic` loop timing against the 20 ms budget
- [ ] Smoke test every subsystem on carpet
- [ ] Confirm the AprilTag layout on the coprocessor matches `FieldConstants` (welded vs AndyMark)
- [ ] Set up `tools/log-sync` so match logs upload from the pit between matches
