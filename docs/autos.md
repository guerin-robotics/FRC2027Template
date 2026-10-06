# Autos

Two kinds, side by side in one chooser:

| | PathPlanner | Point-to-point |
|---|---|---|
| Authored in | PathPlanner GUI (`src/main/deploy/pathplanner/`) | Java (`frc/robot/autos/PointToPointAutos.java`) |
| Motion | Follows a path: curves, speed through a gap, event markers | Drives straight to each waypoint with `DriveCommands.driveToWaypoint` |
| Mechanism actions | Named commands, registered before `buildAutoChooser()` | Any command in the sequence |
| Use when | The route itself matters | An auto is a few fixed stops with nothing to avoid in between |

The point-to-point style comes from FRC 3467's W8-Library. It was rebuilt here as data plus
static factories (see [library.md](library.md)).

---

## Point-to-point

An auto is data: a `WaypointAuto` with a name and a list of `Waypoint`s. Each waypoint has:

- a pose, **blue-origin** (flipped for red at run time, never in the data);
- a position tolerance and a heading tolerance that count as "arrived";
- a **timeout**. A waypoint that is not reached in time is abandoned, and the auto continues from
  wherever the robot is. This is what guarantees an auto ends; `.claude/rules/03-commands.md`
  forbids untimed waits.

The defaults come from `Constants.Thresholds.WAYPOINT_*_TOLERANCE_*` and
`Constants.Waits.WAYPOINT_TIMEOUT_SECONDS`.

### Adding one

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

List it in `PointToPointAutos.all()` and it appears in the chooser. The first waypoint is also
where odometry is reset, so place the robot there.

To act at a waypoint, build the sequence yourself instead of calling `build()`. Interleave
`DriveCommands.driveToWaypoint(drive, w)` with mechanism commands from their `*Commands`
factories, and give every `waitUntil` its timeout.

### Preview and debugging

- **Preview:** each auto logs `Autos/<name>/Waypoints` when it is built. Show it as a pose array
  in AdvantageScope to see the route before running it.
- **While running:** `AutoAim/DriveToPose/*` logs the target, the error and the current pose
  every loop.
- **In the pit:** `PointToPointAutos.stepThrough(drive, auto, forward, back)` runs one waypoint
  per press of `forward`. Bind it to two trigger accessors in `Triggers.java`; a commented
  example is already there. Use it with the robot on the carpet and someone on disable.

### Before trusting one on carpet

`DriveCommands.driveController`, the translation PID every waypoint uses, is an **untuned
placeholder** (see its `TODO(2027)`). Tune it with `/pid-tune` on the real chassis first.
Without that, a point-to-point auto shows that the waypoints are in the right order, and nothing
about how well the robot drives them.

---

## Time budget — both kinds

In 2026 every auto overran the auto period and its last path was truncated. Time each auto in
sim before an event. For point-to-point, the worst case is the sum of the waypoint timeouts. Keep
that inside the auto period, or accept on purpose that the last stops may be skipped.
