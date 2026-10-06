# Autos

Autos come in **three layers**, all in one chooser and all built on one
`DriveCommands.driveToPose`:

| Layer | Authored in | How the robot moves | Use it when |
|---|---|---|---|
| **1. Choreo trajectory** | Choreo GUI → `src/main/deploy/choreo/*.traj`; routine in `robot/autos/ChoreoAutos.java` | `Drive.followTrajectory`: the sample's velocity as feedforward + PID on x, y, heading | The route itself matters — curves, speed through a gap, mechanism actions mid-path |
| **2. Choreo + PID finish** | Same, plus `ChoreoAutos.thenAlign` on `traj.done()` | Trajectory for the route, then `driveToPoseWithin` to the trajectory's end pose | The end pose must be exact — scoring, a pickup — and the follower's last few cm are not good enough |
| **3. Point-to-point** | Java: `robot/autos/PointToPointAutos.java` (`WaypointAuto`) | `driveToWaypoint` from stop to stop, straight lines | A few fixed stops with nothing in between to avoid; fast to write, no GUI |

The team has decided on Choreo for 2027 and has not decided how much PID-to-pose to use. This
layout keeps that open: a routine can use any mix of the layers, and leaning harder on one later
means writing routines differently, not re-plumbing anything.

---

## How top teams do it

Researched from public repos, October 2026:

| Team | Autos authored in | Route | PID-to-pose | PID-to-pose finishes on |
|---|---|---|---|---|
| **1114** (2026, Simbot Tim) | PathPlanner GUI | PathPlanner's own PID path follower (`PPHolonomicDriveController`) | `DriveToPose` class: profiled distance PID **seeded with current speed**, optional "approach strength" lane-curving, profiled heading PID. Present but **not used by any 2026 auto** | `atGoal()` on both PIDs (5 cm, 2.75°) |
| **6328** (2025) | Code + Choreo | Choreo trajectory, then `DriveToPose` for the approach | **Velocity-seeded** profile, **setpoint feedforward faded by distance**, driver-override blend, auto/teleop constraint sets, all tunable | Never self-finishes; callers gate on `withinTolerance` |
| **254** (2025) | Code | `AutoAlignToPoseCommand` | Profiled PID + **setpoint feedforward × ffScaler** | 4 cm / 2° |
| **1678** (2025) | Choreo | Choreo trajectory, then `PIDToPoseCommand` | Profiled PID | Tolerance |
| **3467** W8-Library | Code | Chain of `DriveToPose` per waypoint | Profiled PID on distance, angular PID, output clamps | Optional distance/angle tolerance |
| **Us** | Choreo GUI + code | Layers 1–3 above | **Velocity-seeded** profiles + **setpoint feedforward** faded inside `Drive/ToPose/FFMinRadius`–`FFMaxRadius` | `driveToPoseWithin`: tolerance **or mandatory timeout** |

**What 1114 actually does.** "PID with the PathPlanner GUI" means PathPlanner's path follower,
which is PID on the path — the same thing layer 1 does with Choreo's follower. Their PID-to-pose is
a separate tool. Layer 2 is the 6328/1678 hybrid: draw the route, PID the finish.

---

## Layer 1 — Choreo

Draw trajectories in the Choreo GUI with the project saved to `src/main/deploy/choreo/` (set the
robot config first — see that folder's README). Then, in `ChoreoAutos`:

```java
public static AutoRoutine twoPieceLeft(AutoFactory factory, Drive drive) {
  AutoRoutine routine = factory.newRoutine("Two Piece Left");
  AutoTrajectory toFirst = routine.trajectory("LeftToFirst");
  AutoTrajectory toScore = routine.trajectory("FirstToScore");

  routine.active().onTrue(Commands.sequence(toFirst.resetOdometry(), toFirst.cmd()));
  toFirst.done().onTrue(toScore.cmd());
  return routine;
}
```

and list it in `ChoreoAutos.all(...)`:

```java
routines.put("Two Piece Left", () -> twoPieceLeft(factory, drive).cmd());
```

Trajectories are flipped for red by ChoreoLib. `Autos/Trajectory` logs the running trajectory and
`Odometry/TrajectorySetpoint` the sample the follower is chasing.

### Event markers

A marker placed in the GUI fires whatever is bound to its name:

```java
// RobotContainer, BEFORE any routine is built
ChoreoAutos.bind(choreo, "Intake", IntakeCommands.deploy(intake));
```

- A routine only sees bindings that existed when it was created.
- A bound command must **not** require the drive — it runs beside the trajectory, which does.
- `RobotContainerSmokeTest` fails if a `.traj` uses a marker nothing is bound to.

`traj.atTime("Intake")` / `traj.atTime(1.2)` / `traj.done()` triggers work too, under the same
rule.

## Layer 2 — Choreo + PID finish

```java
toScore.done().onTrue(ChoreoAutos.thenAlign(drive, toScore));
```

`thenAlign` runs `driveToPoseWithin` to the trajectory's (alliance-flipped) final pose with the
default waypoint tolerances and timeout. It may take the drive because the trajectory has already
ended. Chain the scoring action after it, or on another trigger.

## Layer 3 — Point-to-point

An auto is data: a `WaypointAuto` (name + `Waypoint`s). Each waypoint has a blue-origin pose,
position and heading tolerances, and a **timeout** — if it is not reached in time the auto moves on,
which is what guarantees it ends.

```java
public static WaypointAuto leftTwoPiece() {
  return new WaypointAuto(
      "Left Two Piece",
      List.of(
          waypoint(1.5, 6.5, 0.0),   // starting pose — the robot is placed here
          waypoint(4.0, 6.5, 0.0),
          waypoint(2.0, 5.5, 180.0)));
}
```

List it in `PointToPointAutos.all()`. Defaults come from
`Constants.Thresholds.WAYPOINT_*_TOLERANCE_*` and `Constants.Waits.WAYPOINT_TIMEOUT_SECONDS`.
`Autos/<name>/Waypoints` logs the route for an AdvantageScope preview.

**In the pit:** `PointToPointAutos.stepThrough(drive, auto, forward, back)` runs one waypoint per
press. Bind it to two trigger accessors in `Triggers.java` (a commented example is there). Robot on
the carpet, someone on disable.

---

## The PID-to-pose underneath layers 2 and 3

`driveToPose` seeds its profiles with the robot's current velocity and feeds the profile's setpoint
velocity forward — so a robot already moving keeps moving instead of braking at each waypoint, and
tracks its profile rather than trailing it. Details in [target-alignment.md](target-alignment.md).

**Before trusting it on carpet:** `driveController` is an **untuned placeholder**
(`TODO(2027)` in `DriveCommands`), and the Choreo follower gains in `Drive` are carried over from
the 2026 PathPlanner controller. Tune both with `/pid-tune` on the real chassis.

---

## The chooser

`RobotContainer` keeps AdvantageKit's `LoggedDashboardChooser<Supplier<Command>>`, not Choreo's
`AutoChooser`. The selection is a logged input, so replay knows which auto ran; each option is a
supplier, so a routine — and its trajectories — is built only when it is selected and auto starts.

---

## Time budget — every layer

In 2026 every auto overran the auto period and its last path was truncated. Time each auto in sim
before an event. For layers 2 and 3 the worst case adds every PID step's timeout; keep the total
inside the auto period, or accept on purpose that the last steps may be skipped.
