# maple-sim drivetrain — deferred

maple-sim has no WPILib 2027 build yet (latest is `v0.4.0-beta`, WPILib 2026). These files were
moved out of `src/` so the 2027 build compiles. They are not compiled here.

**To restore when maple-sim ships a 2027 vendordep:** add its vendordep, move the four files back to
`frc.robot.subsystems.drive` (the test to `src/test`), re-add `DriveConstants` sim constants
(`USE_MAPLE_SIM`, `SIM_ROBOT_MASS_KG` 52.0, `SIM_BUMPER_*` 0.90 m, `SIM_WHEEL_COEFFICIENT_OF_FRICTION`
1.2, all placeholders), restore the `SIM` branch in `RobotContainer`, `DriveSimulation.periodic()` in
`Robot.simulationPeriodic()` and `DriveSimulation.resetPose(pose)` in `Drive.setPose()`. See git history
for the 2026 wiring.
