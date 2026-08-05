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

- [ ] Run CTRE Tuner X Swerve Project Generator → **replace** `generated/TunerConstants.java`
- [ ] Verify every swerve CAN ID against the physical robot
- [ ] Zero all CANcoder magnet offsets for pivoting mechanisms
- [ ] Assign CAN IDs for all new mechanism motors; comment the bus (`// CANivore` / `// RIO CAN`)
- [ ] Physically measure every camera position → update `VisionConstants.robotToCameraN`
      (they are identity placeholders right now — vision is meaningless until this is done)
- [ ] Set camera names in `VisionConstants` to match the coprocessor config
- [ ] Delete unused cameras from **all three** `RobotContainer` branches (REAL, SIM, REPLAY)
- [ ] Measure robot mass and estimate MOI → update `ROBOT_MASS_KG` / `ROBOT_MOI` / `WHEEL_COF`
      in `Drive.java`'s `PP_CONFIG`
- [ ] Write `docs/hardware-layout.md` as devices are assigned — do not defer this

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

## Tuning

> Read [docs/characterization-and-tuning.md](../docs/characterization-and-tuning.md) first.
> Gains here are in **amps**, not volts — the drivetrain runs `TorqueCurrentFOC`.

- [ ] Run wheel radius characterization **before** anything else — every downstream number
      is scaled by it
- [ ] Run drive FF characterization; put measured `kS`/`kV` in `TunerConstants.driveGains`
- [ ] Measure drive `kA` separately if you rely on path following (the FF routine does not
      produce it)
- [ ] Tune swerve steer and drive gains in `TunerConstants`
- [ ] Tune PathPlanner translation/rotation PID in `Drive.java`'s `AutoBuilder.configure()`
- [ ] Re-tune `ANGLE_KP` / `ANGLE_KD` in `DriveCommands` for the new chassis
- [ ] Run `wheelRadiusCharacterization` and `feedforwardCharacterization` from the auto chooser
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
