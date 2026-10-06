# Architecture Rules

These rules encode decisions made during the 2025/2026 seasons that must not be
reversed without deliberate team discussion. They exist because the alternatives
were tried and caused problems.

---

## AdvantageKit IO Layer

**Rule:** Every subsystem must wrap hardware behind an `XxxIO` interface.

```
CORRECT                          WRONG
──────────────────────           ──────────────────────
class Mechanism {                class Mechanism {
  MechanismIO io;                  TalonFX motor;
  void periodic() {                void periodic() {
    io.updateInputs(inputs);         motor.get...   ← HARDWARE IN SUBSYSTEM
    Logger.processInputs(...)      }
  }                              }
}
```

**Why:** This enables AdvantageKit log replay. If hardware calls are in the subsystem,
you cannot replay a match log to reproduce a bug. We caught real match bugs this way.

**Corollary:** every subsystem must have a simulation implementation. It can be stubs.
Without one, the robot cannot run in simulation.

`Drive` and `Vision` own their IO layers directly and are the reference implementations for a
subsystem whose hardware is not a motor.

**Motor-driven mechanisms do not write an IO layer at all.** `frc/lib/mechanism` provides it —
one `MotorIO` with one shared `@AutoLog` schema, `MotorIOTalonFX` for hardware, and
`MotorIOTalonFXSim` for simulation, which applies the same config to a simulated device so the
gains that run in sim are the gains that run on the robot. Adding a per-mechanism copy of any of
those is the thing that library exists to prevent.

---

## RobotState Singleton

**Rule:** No subsystem may hold a reference to another subsystem.

All shared state (pose, distances, zone classification, alignment booleans) must
go through `RobotState.getInstance()`.

```
WRONG:   mechanism.setSpeedForPose(drive.getPose())   ← subsystem ↔ subsystem
CORRECT: mechanism.setSpeedForTarget()                ← reads RobotState internally
CORRECT: supplier callback in constructor             ← RobotContainer passes drive::getPose
```

**Why:** Subsystem cross-references create initialization order dependencies,
circular logic, and make unit testing impossible.

**Exception:** `RobotContainer` is the wiring layer — it may hold references to all
subsystems for the purpose of passing them to commands and composing bindings.

**Performance note:** anything in `RobotState` annotated `@AutoLogOutput` is called by
AdvantageKit every loop, whether or not your code reads it. Do not annotate expensive
methods, and delete the annotation when a method loses its last real caller.

---

## Static Command Factories

**Rule:** Commands are static factory methods, not classes.

```java
// CORRECT
public class MechanismCommands {
    public static Command runAtVelocity(Mechanism mechanism, AngularVelocity velocity) {
        return Commands.run(() -> mechanism.setVelocity(velocity), mechanism)
            .withName("Mechanism_Velocity");
    }
}

// WRONG
public class RunMechanismAtVelocityCommand extends Command {
    private final Mechanism mechanism;
    ...
}
```

**Why:** Static factories are composable, testable, and traceable in AdvantageKit logs.
Named command classes create unnecessary files and hide composition structure.

**Under Commands V3 this rule stays.** V3 makes `mechanism.run(...)` the builder entry point; the
factories still live in `*Commands` classes, not on the subsystem. V3 also has **no
`Subsystem.periodic()`** — nothing calls a mechanism's `periodic()` for you, so the port must wire
every one explicitly or `Logger.processInputs()` silently stops running. See
[docs/commands-v3.md](../../docs/commands-v3.md).

---

## Triggers

**Rule:** All `Trigger` and `LoggedTrigger` objects live in `Triggers.java`.
`RobotContainer` reads from `Triggers.getInstance()` and never creates a trigger itself.

**Rule:** Controller objects are `private` inside `Triggers.java`. Nothing else in the
codebase may touch one. Axis reads go through `driveXSupplier()` / `driveYSupplier()` /
`driveRotSupplier()`; buttons go through named accessors.

**Rule:** Trigger accessors are named for the robot action, not the button —
`resetGyro()`, not `bButton()`. Moving a function to a different button then touches one
line, and reviewing a binding does not require a controller diagram.

`Triggers.java` exists and the drive bindings already use it. The layout is flight stick
drives, Xbox operates; ports are in `Constants.Controllers`.

**Why:** Button logic and state triggers are easier to audit in one place — we caught
timing bugs in 2026 by seeing every condition in one file. Keeping the controllers private
is what makes the dashboard controller swap addable by editing a single file, and it
prevents the 2026 bug where one command read a stick directly and followed an input nobody
was holding while ordinary driving looked fine (`docs/drive-controller-mode.md`).

---

## AllianceFlipUtil

**Rule:** Never call `DriverStation.getAlliance()` in a hot path (any method called
more than once per match).

Use `AllianceFlipUtil.shouldFlip()`, which caches the result once per loop.
`Robot.robotPeriodic()` calls `AllianceFlipUtil.refresh()` before the scheduler runs —
do not remove that call.

**Why:** `DriverStation.getAlliance()` returns `Optional<Alliance>` and allocates on
every call. At 50 Hz this creates GC pressure and unpredictable `periodic()` timing.

---

## Choreo Wiring

Autos come in three layers: Choreo trajectories (followed by `Drive.followTrajectory`), a Choreo
trajectory with a PID-to-pose finish (`ChoreoAutos.thenAlign`), and code-only point-to-point
waypoints (`PointToPointAutos`). The last two share `DriveCommands.driveToPose`. See [docs/autos.md](../../docs/autos.md).

**Rule:** The trajectory follower lives in `Drive.java` — `Drive.followTrajectory(SwerveSample)`
— not in `RobotContainer` or `ChoreoAutos`. The follower gains and the output path belong with
the drivetrain.

**Rule:** There is exactly one Choreo `AutoFactory`, created by `ChoreoAutos.createFactory(drive)`
in `RobotContainer`. Routines are static factories in `robot/autos/ChoreoAutos.java` that take it.

**Rule:** Event markers are bound with `ChoreoAutos.bind(factory, name, command)` in
`RobotContainer`, **before any routine is built**. A routine only sees bindings that existed when
it was created, and an unbound marker does nothing on the field — silently.
`RobotContainerSmokeTest` fails if a `.traj` uses a marker name nothing is bound to.

**Rule:** A command bound to an event marker or a trajectory trigger (`atTime`, `done`) must not
require the drive. It is scheduled alongside the trajectory command, which requires the drive, so
sharing that requirement interrupts the trajectory mid-path.

**Rule:** The auto chooser is AdvantageKit's `LoggedDashboardChooser<Supplier<Command>>`, never
Choreo's `AutoChooser`. The selection is then a logged input and replays; the supplier builds the
routine at auto start, so only the selected auto's trajectories load.

**Why the robot config is not in code any more:** Choreo optimises trajectories against the mass,
MOI, wheel COF and bumpers set in the GUI project (`src/main/deploy/choreo/*.chor`). The robot only
follows what the GUI produced. A wrong mass there makes trajectories the robot cannot follow, and no
code change fixes it — re-export from the GUI.

---

## Pose Estimator

**Rule:** There is exactly one `SwerveDrivePoseEstimator` in the codebase — inside `Drive.java`.
`RobotState` delegates to it via `poseSupplier`.

**Why:** Two independent estimators diverge when vision is lost and cause pose jumps
when they resync. This was a real bug in an earlier version of this codebase.

**Known gap carried over from 2026:** the pose estimator has no off-field divergence
guard. Vision rejection filters in `VisionConstants` are the only defense. If the estimate
is ever badly wrong, nothing pulls it back except good vision observations.

---

## Single Source of Truth for Drivetrain Geometry

**Rule:** Module positions, gear ratios, wheel radius, CAN IDs and swerve gains live in
`generated/TunerConstants.java` only. `Drive.getModuleTranslations()` reads from it.

Do not redeclare drivetrain geometry in `DriveConstants` or anywhere else. The 2026
codebase carried a duplicate hardcoded module layout that had to be kept in sync by hand.

---

## No Opportunistic Cleanup

**Rule:** Do not change code outside the scope of the current task.

If you see a magic number, a style issue, or commented-out code while working on
something else — note it, do not fix it. Unrequested changes hide in reviews and
introduce silent regressions.
