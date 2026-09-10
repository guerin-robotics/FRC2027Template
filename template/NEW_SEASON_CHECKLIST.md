# New Season Checklist

Work through this top to bottom. Many items have dependencies.

Items marked **[done]** were completed when this template was created from the 2026 robot —
they're listed so you know they're handled, not so you redo them.

---

## Before Kickoff — Infrastructure

- [x] **[done]** Repo created from this template (not forked from the 2026 robot)
- [x] **[done]** `frc/lib/` (all shared utilities) carried over
- [x] **[done]** `subsystems/drive/` carried over
- [x] **[done]** `subsystems/vision/` carried over with tuned filter thresholds
- [x] **[done]** `Robot` / `Constants` / `RobotContainer` in AdvantageKit template shape
- [x] **[done]** `RobotState` reduced to pose / velocity / geometry helpers
- [ ] Verify team number in `.wpilib/wpilib_preferences.json`
- [ ] Configure `tools/log-sync` with the 2027 team-number IP and **verify one upload
      actually works** — do this at season start, not the night before an event
- [ ] Update WPILib to the 2027 release (`build.gradle` plugin version, `settings.gradle` `frcYear`)
- [ ] Update AdvantageKit to the matching 2027 version
- [ ] Update CTRE Phoenix 6, PathPlannerLib, PhotonVision, Studica vendordeps
- [ ] Confirm `./gradlew build` still passes after the vendordep bumps
- [ ] Update `FieldConstants.aprilTagLayout` to the 2027 field as soon as WPILib ships it

## Kickoff Week — Game Modeling

- [ ] Add 2027 field geometry to `frc/lib/util/FieldConstants.java` (scoring elements, zones, lines)
- [ ] Add scoring targets and any zone classification to `RobotState`
- [ ] Fill in `Constants.CanIds` as devices are assigned — one constants file, no `HardwareConstants`
- [ ] **Delete `CanIds.EXAMPLE_MOTOR` / `EXAMPLE_FOLLOWER` / `EXAMPLE_ENCODER`** once the first
      real mechanism exists. They are scaffold placeholders and will otherwise ship to a
      competition robot as fictional hardware IDs
- [ ] Replace `Waits.MECHANISM_READY_SECONDS` / `TOTAL_TIMEOUT_SECONDS` with measured values and
      rename them for what they gate
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

### Stage 3b — Verify before building on it

- [ ] **8b. Odometry accuracy check.** Drive a measured 5 m straight, a 2 m box without
      rotating, and 10 spins in place. Compare `Odometry/Robot` to reality each time.
      Nothing in the code checks gear ratio, wheel radius or module positions — this is the
      only thing that catches them, and they compound into every auto and every vision fusion
      downstream. See the characterization doc § Step 0b for what each failure mode points at
- [ ] Record the three results in `docs/robot-spec.md` so later drift is distinguishable
      from a new problem

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

For each mechanism, **first pick the scaffold** in `template/src/` — the choice sets the control
request, neutral mode, gravity type, motion profile, sim model and units, and carrying a value
across scaffolds is how an arm ends up with 30x the acceleration it should have:

- [ ] It spins, and only speed matters → `exampleRoller`
- [ ] It pivots to an angle → `exampleArm`
- [ ] It travels in a line → `exampleLift`

Then, for each mechanism:

- [ ] `subsystems/myMechanism/MyMechanism.java` — extends the library subsystem for its kind
- [ ] `subsystems/myMechanism/MyMechanismConstants.java`
- [ ] `commands/MyMechanismCommands.java` — **only** if the mechanism has verbs the library's
      `RollerCommands` / `RotaryCommands` / `LinearCommands` do not already cover
- [ ] Wire real/sim/replay in `RobotContainer` — all three branches
- [ ] Call `registerFaultMonitors()` on it in `RobotContainer`, so a device that drops off the
      bus, reboots mid-match or overheats reaches the pit rather than only the log
- [ ] Add jam detection if the mechanism can stall — `RollerSettings.withJamDetection(...)`, and
      measure its four thresholds (2026 had none, so jams were silent)
- [ ] If it has no absolute encoder, run `LinearCommands.zeroAtHardStop(...)` before trusting any
      reported position
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
- [ ] **Copy the sim gain-sweep harness** for the first mechanism you tune. `GainSweepTest`
      already exists and is what `/pid-tune` drives — it sweeps gains against the physics sim
      and reports rise time, overshoot and steady-state error, the same way the 2026 season
      found its PathPlanner gains without touching the robot. Point it at the new mechanism's
      library's `*Mechanism.sim(...)` rather than building a harness from scratch
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

## First Deploy Of The Season

Run the first-deploy gate in
[.claude/rules/04-build.md](../.claude/rules/04-build.md#first-deploy-of-the-season--extra-gate)
before the robot is enabled for the first time. Short version:

- [ ] `./gradlew build` passes (not just `compileJava`)
- [ ] `TunerConstants` has been regenerated with Tuner X **for this robot**
- [ ] Phoenix Tuner device count matches what the code expects
- [ ] `Constants.currentMode` resolves to `REAL`; `simMode` not left on `REPLAY`
- [ ] `Constants.tuningMode` is FALSE — tunables must not be dashboard-adjustable at an event
- [ ] Any gain found during a tuning session has been written back into the constants and
      committed; dashboard values do not survive a reboot
- [ ] USB drive present for logging
- [ ] Working tree clean — `GitDirty` reads "All changes committed"
- [ ] Robot on blocks, hand on disable, for the first enable

## Pre-Competition

- [ ] Fill in `docs/pre-match-checklist.md` with the 2027 mechanism checks and the expected
      Phoenix device count, then **print it for the pit**
- [ ] Update the `Pre-Match Checklist` dashboard string in `Robot.java` to match it
- [ ] Battery logger reporting for every subsystem
- [ ] Check `robotPeriodic` loop timing against the 20 ms budget
- [ ] Smoke test every subsystem on carpet
- [ ] Confirm the AprilTag layout on the coprocessor matches `FieldConstants` (welded vs AndyMark)
- [ ] Confirm `tools/log-sync` still uploads from the pit network at the venue
- [ ] Start a competition change log — every change between matches, with why (see
      `docs/pre-match-checklist.md`)
