# AdvantageScope Layouts

Saved AdvantageScope window layouts. Open one with **File → Import Layout** in
AdvantageScope.

## AdvantageScope Swerve Calibration.json

Swerve and odometry only — module drive position, turn position, drive velocity and
current, measured vs optimized setpoint states, chassis speeds, and the odometry pose and
trajectory. Every key it references exists in this template unchanged, so it works on day
one without editing.

Use it for:

- Swerve calibration and steer/drive gain tuning
- Checking measured module states against setpoints
- Watching path-following error (`Odometry/Trajectory` vs `Odometry/Robot`)
- Per-module current draw during brownout investigation

## Not carried over

The 2026 repo had four more layouts (`AdvantageScope 7-26-2026`, `PreStatesLayout`,
`Week 3 Comp Layout`, `mid-comp-week3`). All four were built around that season's
mechanism keys — flywheel, hood, prestage, feeders, transport, intake — so every graph in
them would open empty here. Build 2027 equivalents once the mechanisms exist, and commit
the ones that earn their keep between matches.

Also not carried: `Advantage Scope Assets/`, which held 195 MB of `.glb` 3D models for the
2026 competition and alpha robots. Export new models for the 2027 robot if you want the 3D
view; the `RobotModelVisualizer` that fed articulated component poses to those models was
game-specific and did not carry over either.
