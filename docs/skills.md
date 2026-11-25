# Team Skills — How Humans Use Them

Skills are slash commands in `.claude/skills/`. This page says, for each one, **what a human does in
simulation, what a human does on the real robot, and what to double-check** before trusting the
result. A skill proposes; a human confirms. Nothing here replaces the Hard Stops in `CLAUDE.md`.

## Ground rules for every skill

- **Sim proves logic, not hardware.** Sim motor, battery, camera and CAN models are not the real
  ones. A number fitted in sim is a *starting point*, never a value to deploy.
- **Gains, current limits, inversion, encoder offsets and CAN IDs are never written without
  explicit confirmation.** If a skill proposes one, read the evidence before saying yes.
- **Units.** Drive and steer run `TorqueCurrentFOC`, so gains are in **amps**. Always check the unit
  next to a number before comparing it with another one.
- **There are no tuning or demo flags.** Tuning commands run only in enabled Test mode behind the
  held dead-man button (`Triggers.tuningEnabled`). In 2027 each one becomes a `@Utility` OpMode.
  Pre-match, check that the Driver Station is not in Test mode and no tuning OpMode is selected.
- Skills that read logs are read-only. Skills that edit code work on a feature branch and finish with
  `./gradlew compileJava`, `spotlessCheck` and `build`.

## At a glance

| Skill | Use it for | Changes code? |
|---|---|---|
| `/add-subsystem` | A new mechanism | Yes |
| `/add-command` | A new command or binding on an existing mechanism | Yes |
| `/pid-tune` | Tuning roboRIO-side controllers in sim | Proposes |
| `/characterize` | kS/kV/kA, wheel radius, SysId → gains, in amps | Proposes |
| `/camera-calibrate` | `robotToCameraN` transforms | Proposes |
| `/bline-auto` | BLine autos and follower gains | Yes / proposes |
| `/debug-match-log` | Root cause of a field problem | No |
| `/loop-timing` | Why the loop is slower than 50 Hz | No |
| `/brownout-analysis` | What browned the robot out | No |
| `/port-commands-v3` | The Commands V3 port, later | Yes |

---

## `/add-command`

**Use when:** adding a command, binding, or sequence to an existing mechanism or the drive.

**In sim:** run `./gradlew simulateJava`, trigger the command, and watch AdvantageScope. Confirm
it starts and ends, shows its name in the command log, stops the mechanism on release, and that
its timeout fires when the awaited condition never happens (force the condition false).

**On the robot:** first enable with the robot on blocks and a hand on disable. Press the new button
and check direction, speed and stopping. Test the interrupted case — release mid-motion.

**Double-check:**
- Every `waitUntil` has a timeout from `Constants.Waits`.
- Setpoints came from `Constants.Setpoints`, not a number at the binding.
- The binding is named for the action and lives in `Triggers.java`.
- The command does not require the drive if it is bound to a BLine event.
- `docs/driver-controls-card.md` shows the new binding.

## `/characterize`

**Use when:** characterizing the drivetrain or a new mechanism, or when a fitted gain looks wrong.

**In sim:** exercises the procedure and the fitting code only. Sim-fitted gains are not usable on
the robot — do not deploy them.

**On the robot:** robot on blocks for the first kS/kV run, on the floor for wheel radius and kA.
Run the characterization autos from the chooser. Fix the wheel radius first; everything after
depends on it. Provide the log; the skill fits and proposes.

**Double-check:**
- **Every number has a unit and it is amps.** `FF_RAMP_RATE` is 0.1 A/s. SysId's "volts" are amps
  here, so pass an explicit `SysIdRoutine.Config`; the 7 V default is a 7 A step.
- A steer kP in the thousands is normal (amps per azimuth rotation).
- A mechanism not on `TorqueCurrentFOC` is in volts — check its control request.
- kS positive, kV nonzero, R² reasonable, and within an order of magnitude of the old value.
- Nothing is written until you confirm, and the commit message records the evidence.

## `/camera-calibrate`

**Use when:** a camera is added, moved or remounted, vision poses are offset, or at season start
while the transforms are still identity placeholders.

**In sim:** checks that the camera count and transforms are consumed in all three modes. It cannot
validate a transform — the sim camera uses the same constants and so agrees with itself.

**On the robot:** measure each camera from the robot origin with a tape measure and angle finder,
place the robot **stationary** at a taped, measured pose in view of tags, and record. Drive-by data
is not usable. Provide the log; the skill proposes transforms.

**Double-check:**
- Camera names match the coprocessor, and the `VisionIO` count matches in REAL, SIM and REPLAY.
- A camera tilted up has **negative** pitch.
- The Pigeon 2 mount calibration is right — a tilted gyro looks like a camera yaw error.
- Verified at two or more distances; error that grows with distance is rotation, constant error is
  translation.
- The filter thresholds in `VisionConstants` were not touched.
- Re-check after any impact that could have bent a mount.

## `/loop-timing`

**Use when:** the loop-overrun alert fires, `LoopTiming/OverrunFraction` is nonzero, or before the
first drive session of the season.

**In sim:** finds relative hot spots and allocation. A desktop is far faster than a roboRIO, so do
not judge the 20 ms budget from sim.

**On the robot:** drive normally, including enabling vision and running an auto, then provide the
log. Overruns that only show with targets in view point at Vision; at enable, at command
construction.

**Double-check:**
- Judge the budget from `LoopTiming/AverageMs`, `PeakMs` and `OverrunFraction` in a real log.
- The proposed fix moved the number after it was applied. If it did not, the diagnosis was wrong.
- No JVM flags changed. No `Logger.processInputs()` or `BatteryLogger.reportCurrentUsage()`
  removed to save time.

## `/brownout-analysis`

**Use when:** after any brownout or low-voltage warning, or when the battery sags under load.

**In sim:** the sim battery model is a rough placeholder. Use it only to check the analysis runs;
real attribution needs a real log.

**On the robot:** note which battery, its age and charge, and what the robot was doing. Provide the
log. Keep the battery data from a fresh pack for comparison.

**Double-check:**
- It used **supply** current, not stator, for battery math.
- The battery was not simply tired — late-match sag with low `Energy` headroom is not a mechanism.
- It reports any consumer missing from `reportCurrentUsage`.
- **A current limit is never raised.** Any limit change is High risk: check which limit (supply or
  stator) is binding and weigh brownout against a slower mechanism before confirming.

## `/port-commands-v3`

**Use when:** the team has decided to port, and the 2027 platform bump already runs on V2
(`docs/2027-migration.md`). Not before.

**In sim:** this is where most verification happens. Run each ported command, check activation,
release, timeout, and the log. Replay an old log against the ported code.

**On the robot:** treat the first enable as a first deploy — blocks, hand on disable, one
mechanism at a time.

**Double-check:**
- `Logger.processInputs()` still runs for every subsystem, including `Vision` — V3 has no
  `periodic()`. The smoke test must prove it.
- No untimed wait of either kind. Every `WaitResult` is checked.
- Every command is named, and bound ones are not left on automatic names.
- BLine event rule kept until BLine-Lib ships V3 support.
- Signatures re-checked against the current release, not the alpha-7 text in `commands-v3.md`.

## Existing skills

`/add-subsystem`, `/pid-tune`, `/bline-auto` and `/debug-match-log` are described in their own
`SKILL.md` and in `docs/ai-development-handbook.md`. They follow the same rules above.

---

## What to check on the robot, every time

1. Working tree clean and committed (`BuildConstants.GIT_DIRTY` reads "All changes committed").
2. `Constants.currentMode` resolves to `REAL`, not `REPLAY`.
3. Not in Test mode and no tuning OpMode selected.
4. USB drive in. No USB means no log, and the log is what these skills read.
5. Robot on blocks and someone on disable for the first enable of any new behavior.
6. Gains, limits and calibration changed only with a confirmation you can point to.

## Related

- `CLAUDE.md` — hard stops and risk tiers
- `docs/pre-match-checklist.md`, `docs/characterization-and-tuning.md`, `docs/tunables.md`
