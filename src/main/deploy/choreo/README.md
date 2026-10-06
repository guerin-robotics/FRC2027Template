# Choreo trajectories

Create the Choreo project here, with the Choreo GUI: **File → New Project**, saved into this
directory. Choreo writes one `.chor` project file plus one `.traj` file per trajectory, and
ChoreoLib loads them from `/home/lvuser/deploy/choreo/` on the robot.

## Robot config — set this before drawing anything

The GUI's robot config is what the trajectories are optimised against. It replaces the
PathPlanner `RobotConfig` that used to live in `Drive.java`. These were the **2026** values, kept
as a starting point only; measure the 2027 robot:

| Field | 2026 value | Source |
|---|---|---|
| Mass | 63.503 kg | Weighed with battery and bumpers |
| Moment of inertia | 5.162 kg·m² | CAD, or SysId angular characterization |
| Wheel coefficient of friction | 2.225 | Measured against carpet |
| Wheel radius, gearing, max speed | — | Take from `generated/TunerConstants.java` |
| Bumper size, module locations | — | Measure / `TunerConstants` |

A wrong mass or MOI produces trajectories the robot cannot actually follow, and the follower in
`Drive.followTrajectory` spends the whole auto catching up.

## Event markers

A marker named `Intake` fires whatever `RobotContainer` bound to `"Intake"` with
`ChoreoAutos.bind(...)`. `RobotContainerSmokeTest` fails the build if a `.traj` here uses a marker
name nothing is bound to.

See [docs/autos.md](../../../../docs/autos.md).
