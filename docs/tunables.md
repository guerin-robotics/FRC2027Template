# Tunable Setpoints

This template uses logged dashboard values only to find mechanism setpoints during a deliberate
Test-mode session. Match code reads compiled values from `Constants.Setpoints`; changing a dashboard
value cannot change match behavior.

The `LoggedTunableNumber` wrapper is backed by AdvantageKit's `LoggedNetworkNumber`, so the input is
recorded and replayed. Keep that wrapper until the 2027 AdvantageKit path provides equivalent replay
logging for WPILib's Tunables API.

## Workflow

Each mechanism scaffold provides a `tune*` command for one setpoint:

- `ExampleRollerCommands.tuneVelocity` — mechanism RPM
- `ExampleArmCommands.tunePosition` — mechanism degrees
- `ExampleLiftCommands.tuneHeight` — inches; it refuses to run until the lift is zeroed

Bind each one with `Triggers.getInstance().tuningEnabled.whileTrue(...)`. Two gates have to be open
at once, and each covers a different failure:

- **Test mode.** The field never enables it, so tuning cannot happen in a match. This is also
  enforced inside `LoggedTunableNumber.get()`, which returns the compiled default in every other
  mode — a tunable read by match code by mistake is inert, not live.
- **The operator holding Start.** Test mode alone is not consent to move: it gets enabled in the
  pit to check one sensor. Without a held button, every bound mechanism would jump to whatever
  value was left on the dashboard the moment anyone enabled test. Let go and the command ends.

When a command ends, a lift holds its last height, an arm holds its last angle, and a roller stops.
The 2027 port can replace the binding with a `@Utility` OpMode; keep the held button inside it.

When the value is right, copy it into `Constants.Setpoints` and commit it. The dashboard value is
not persistent configuration and disappears at reboot. Match commands must use the compiled
setpoint, never call `LoggedTunableNumber.get()` — outside test mode it only returns its
default anyway.

## Gains and limits

Gains, motion profiles, tolerances, zeroing thresholds, current limits, and soft limits are plain
compiled constants. Do not make them dashboard tunables: those values affect control or safety
throughout the match.

Use Phoenix Tuner X to inspect signals, check device configuration, plot device-rate data, test
motor direction, and explore Phoenix device gains. The robot reapplies its compiled configuration
at startup, so a Tuner X edit is exploratory. Copy the selected value into the mechanism's constants
file and commit it; the next boot then reproduces the reviewed value.

WPILib controllers on the roboRIO, such as the drive heading and pose controllers, are not tuned in
Phoenix Tuner X. Tune those in simulation with `/pid-tune`, then commit the selected constants.

Never adjust current limits, soft limits, encoder calibration, inversion, or other protection values
as part of ordinary setpoint tuning. See `.claude/rules/00-safety.md` and
`.claude/rules/02-hardware.md` before changing those values.

## Replay and naming

`LoggedTunableNumber` stores its input under `NetworkInputs/Tuning/<key>`. Use mechanism-scoped keys
such as `ExampleArm/Tune/PositionDeg`; do not use an unqualified key like `kP`.

## Related

- `docs/new-mechanism-bringup.md` — measured bring-up sequence and setpoint transcription
- `docs/characterization-and-tuning.md` — characterization order and TorqueCurrentFOC units
- `.claude/skills/pid-tune/SKILL.md` — sim-based tuning for roboRIO-side controllers
- `.claude/rules/00-safety.md` — hard stops and safety classification
