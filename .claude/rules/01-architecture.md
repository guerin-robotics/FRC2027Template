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

**Under Commands V3 nothing calls `periodic()` for you.** A V3 `Mechanism` has no
`Subsystem.periodic()`, so `RobotContainer.periodic()` calls every subsystem's `periodic()` (which
calls `Logger.processInputs()`) explicitly, in a fixed order, before the scheduler runs. A new
subsystem must be added there; `RobotContainerSmokeTest` fails when one is missing.

**Why:** This enables AdvantageKit log replay. If hardware calls are in the subsystem,
you cannot replay a match log to reproduce a bug. We caught real match bugs this way.

**Corollary:** every subsystem must have a simulation implementation. It can be stubs.
Without one, the robot cannot run in simulation.

Every mechanism owns its own `XxxIO` (interface + `@AutoLog` inputs), `XxxIOReal` (every Phoenix
call, and only there) and `XxxIOSim`. Copy them from the matching scaffold in `template/src/` —
`exampleRoller`, `exampleArm` or `exampleLift` — which carry the signal-rate, `connected`,
follower and sticky-fault patterns the hardware rules require. `Drive` and `Vision` are the same
pattern in production form.

**Review a new IO against its scaffold, not against another mechanism.** Each mechanism carries its
own copy, so a difference between two copies is an accident at least as often as a decision.

Sensors and lights are the exception: `frc/lib/device` provides their IOs (absolute encoder,
distance sensor, beam break, lights), and the owning subsystem calls each device's `periodic()`.
See [docs/devices.md](../../docs/devices.md).

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
        return mechanism
            .run(co -> {
              mechanism.setVelocity(velocity);
              co.park();
            })
            .whenExited(mechanism::stop)
            .named("Mechanism_Velocity");
    }
}

// WRONG
public class RunMechanismAtVelocityCommand implements Command {
    private final Mechanism mechanism;
    ...
}
```

**Why:** Static factories are composable, testable, and traceable in AdvantageKit logs.
Named command classes create unnecessary files and hide composition structure.

`mechanism.run(...)` is the V3 builder entry point and invites putting factories on the mechanism.
Keep them in `*Commands` classes. V3 makes `.named(...)` a compile-time requirement, so "every
command is named" is no longer a review item. Full rules: `03-commands.md` and
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

**Rule:** Never call `MatchState.getAlliance()` (2027's `DriverStation.getAlliance()`) in a hot
path (any method called more than once per match).

Use `AllianceFlipUtil.shouldFlip()`, which caches the result once per loop.
`Robot.robotPeriodic()` calls `AllianceFlipUtil.refresh()` before the scheduler runs —
do not remove that call.

**Why:** `MatchState.getAlliance()` returns `Optional<Alliance>` and allocates on
every call. At 50 Hz this creates GC pressure and unpredictable `periodic()` timing.

---

## BLine Wiring

Autos are BLine paths, followed by BLine-Lib's V3 `FollowPath` (`bline.commands.FollowPath`,
`2027.0.0-beta.3`). Point-to-point autos and the PID-to-pose finish were removed for now. See
[docs/autos.md](../../docs/autos.md).

**Rule:** The path follower is BLine-Lib's `FollowPath`, configured in `BLineAutos.createBuilder`
and fed by exactly these `Drive` methods: `getPose`, `setPose` (the pose-reset callback),
`getChassisSpeeds` (robot-relative, measured) and `runVelocity` (robot-relative, commanded). `Drive`
is the `Mechanism` the path requires. `Drive` holds no path-following code and no BLine import. The
follower gains are named constants at the top of `BLineAutos`.

**Rule:** There is exactly one `FollowPath.Builder`, created by `BLineAutos.createBuilder(drive,
mirror, preview)` in `RobotContainer`. `mirror` is the supplier for the Left/Right side, and
`preview` is the one `AutoPreview`; `createBuilder` is also the only place BLine's four logging
consumers are set. Routines are static factories in `robot/autos/BLineAutos.java` that take it. The
builder reuses its PID objects, so two path commands from it must never run at once.

**Rule:** Build path commands with `BLineAutos.follow(builder, path)` — never `builder.build(path)`
directly, or the command is not mirrored. In 2027 BLine mirroring and pose reset are **per
command**, not builder state.

**Rule:** Event triggers are bound with `BLineAutos.bind(key, command)` in `RobotContainer`, before
any routine is built. An unregistered `lib_key` only logs a warning on the field, and the auto
follows its path without the action. `RobotContainerSmokeTest` fails if a path in
`deploy/autos/paths` uses a `lib_key` nothing is bound to.

**Rule:** A command bound to an event trigger must not require the drive. BLine schedules it on the
default `Scheduler` independently of the path command, which requires the drive; an equal-priority
command sharing that requirement interrupts the path mid-run. When an event command shares a
requirement with a later step of the same routine, structure the routine so the event command is
not canceled with a group (see `03-commands.md`).

**Rule:** Autos are authored blue-left only. The Driver Station alliance flips them
(`withDefaultShouldFlip`) and the Right OpMode mirrors them (`withShouldMirror`); neither is
reimplemented in code. Never also call `Path.flip()` or `mirror()` on a path you hand to the
builder, or it transforms twice.

**Rule:** Every path command in an auto has a timeout from `Constants.Waits`. BLine follows
physical progress, not a clock, so a blocked robot never finishes a path on its own.

**Rule:** Reset odometry from the first path of a routine only, with
`BLineAutos.followFirst(builder, path)`, and use `follow` for every later path. Resetting from a
later path teleports the pose estimate mid-auto. Do not call `drive.setPose(path.getStartPose())`
yourself: it skips the alliance flip BLine applies.

**Rule:** Each routine is registered as an **Autonomous OpMode** (per side) by `frc.robot.opmodes
.OpModes`, from the map `BLineAutos.all(...)` returns. The supplier builds the routine when the
OpMode starts, so only the selected auto's paths load. AdvantageKit logs the selected OpMode, so the
selection replays. There is no dashboard auto chooser.

**Why the robot config is not in code:** BLine paths carry no robot model. Speed and acceleration
ceilings come from `src/main/deploy/autos/config.json` and the per-path constraints exported by
BLine Web, and a wrong ceiling there makes paths the robot cannot follow. No code change fixes it:
measure the limits and re-export from BLine Web. The handoff mode (`default_handoff_mode`:
`PROGRESS`) is also project config in 2027, where 2026 set it on the builder.

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
