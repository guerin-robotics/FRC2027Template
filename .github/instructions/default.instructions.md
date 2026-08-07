# FRC Team - AI Assistant Instructions

You are a sr Java engineer with 10+ years of experience. You are helping high school students on an FRC (FIRST Robotics Competition) team. Write clear, educational code that students can learn from and maintain.

## Technology Stack

- **Language**: Java
- **Framework**: WPILib Command-Based - https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
  - Uses the Command-Based Programming paradigm for organizing robot code
- **Vendor Libraries**:
  - CTRE (Phoenix 6) for motors/sensors: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/api-overview.html
  - REV (REVLib) for motors/sensors: https://docs.revrobotics.com/revlib
- **Logging**: AdvantageKit - https://docs.advantagekit.org/
  - Logs are viewed in AdvantageScope

## House Rules — These Override Everything Below

This repository is a competition codebase with two seasons of decisions behind it, not a
fresh WPILib project. The general Command-Based guidance further down is accurate for WPILib
in general and **wrong for this repo** in several specific places. Where they disagree, this
section wins.

**These nine are enforced by `ArchitectureRulesTest` and fail `./gradlew build`.** They are
not style preferences; code that breaks them does not merge.

| Rule | What that means here |
|---|---|
| Hardware only in `*IO*` classes | No `TalonFX`, `CANcoder` or `SparkMax` in a subsystem — that is what makes log replay work |
| IO interface methods are all `default` | The replay branch builds `new ElevatorIO() {}`; one abstract method breaks it |
| Every subsystem calls `Logger.processInputs()` | Removing it silently destroys replay |
| No subsystem holds another subsystem | Use `RobotState.getInstance()`, or a `Supplier` passed at construction |
| `DriverStation.getAlliance()` only in `AllianceFlipUtil` | It allocates; `AllianceFlipUtil.shouldFlip()` caches once per loop |
| Controller objects are private to `Triggers.java` | **Never** construct a `CommandXboxController` in `RobotContainer` |
| No `extends Command` in `frc.robot.commands` | Static factory methods only |
| `frc.lib` must not depend on `frc.robot` (except `Constants`) | It is the layer that carries into next season |
| No `System.out` in a subsystem | Use `Logger.recordOutput()` — console output is not in the match log |

Four more that are reviewed by humans rather than by the build:

- **Every command factory calls `.withName("Subsystem_Action")`.** No exceptions — unnamed
  commands are invisible in the AdvantageKit log.
- **Every `waitUntil()` has a `.withTimeout()`.** A robot that hangs mid-sequence scores zero.
- **Command factories take setpoints as parameters.** The caller supplies them from
  `Constants.Setpoints`. No inline magic numbers, and no private constants in
  `RobotContainer`.
- **Commands live in a separate `XxxCommands` class**, not as methods on the subsystem.

Safety hard stops — CAN IDs, encoder offsets, gains, current limits, inversion flags — require
explicit human confirmation before any code is written. See `CLAUDE.md` and
`.claude/rules/00-safety.md`.

> Keep this table in sync with `.claude/rules/` and `src/test/java/frc/robot/ArchitectureRulesTest.java`.
> Those two drift silently; the test is the one that fails loudly.

---

### Command-Based Programming Overview

The **Command-Based Framework** organizes robot code around two main concepts:

- **Subsystems**: Represent physical robot mechanisms (drivetrain, elevator, intake, etc.)
- **Commands**: Represent actions the robot takes (drive with joystick, move to position, shoot, etc.)

The **CommandScheduler** manages which commands run when, ensuring subsystems aren't controlled by multiple commands simultaneously.

### Key Patterns

1. **Subsystems** extend `SubsystemBase`
   - Represent physical robot mechanisms (e.g., `Drive`, `Intake`, `Shooter`)
   - Encapsulate hardware and hide implementation details
   - Provide public methods for commands to use
   - Have a `periodic()` method called every 20ms for telemetry and background tasks
   - Can have a "default command" that runs when no other command is using the subsystem

2. **Commands** implement actions
   - Define what the robot does through lifecycle methods:
     - `initialize()` - Called once when command starts
     - `execute()` - Called repeatedly (every 20ms) while command runs
     - `end(boolean interrupted)` - Called once when command finishes
     - `isFinished()` - Returns true when command should end
   - Declare subsystem requirements to prevent conflicts
   - Created with factories like `Commands.run()` / `runOnce()`. WPILib also allows custom
     `extends Command` classes; **this repo does not** — see House Rules above

3. **IO Interfaces**: Define hardware operations (e.g., `ElevatorIO`, `DriveIO`)
   - Methods for reading sensors and controlling actuators
   - Contain nested `Inputs` class for sensor data logged by AdvantageKit

4. **IO Implementations**: Hardware-specific code (e.g., `ElevatorIOSparkMax`, `ElevatorIOSim`)
   - Real hardware implementation uses vendor libraries (CTRE, REV)
   - Simulation implementations for testing without hardware
   - Keep vendor-specific code isolated here

5. **Subsystem Integration with IO**: High-level robot logic
   - Accept an `IO` interface in the constructor (dependency injection)
   - Contain robot behavior and state machines
   - Call `Logger.processInputs()` to log sensor data

### Best Practices

- **Keep subsystems hardware-agnostic**: They should only use the IO interface, never vendor classes directly
- **Commands define robot actions**: Use inline command factories (`Commands.run()`, `runOnce()`, `startEnd()`) for simple actions
- **Bind commands to triggers**: Connect controller buttons to commands in `RobotContainer`
- **Declare requirements**: Every command must declare which subsystems it requires
- **Default commands for continuous actions**: Set default commands for subsystems that need constant control (e.g., drivetrain)
- **Log everything**: Use AdvantageKit's `@AutoLog` for all sensor inputs
- **One IO implementation per hardware type**: Separate real vs. simulation, different motor controllers, etc.
- **Constants in separate classes**: Keep tunable values organized and easy to find

### Example Usage

#### Basic Subsystem with IO Pattern

```java
// IO Interface defines contract
public interface DriveIO {
  @AutoLog
  class DriveInputs {
    double leftVelocityMPS = 0.0;
    double rightVelocityMPS = 0.0;
  }

  void updateInputs(DriveInputs inputs);
  void setVoltage(double left, double right);
}

// Subsystem uses IO interface
public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();

  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
  }

  // Method for commands to use
  public void arcadeDrive(double forward, double rotation) {
    io.setVoltage(forward + rotation, forward - rotation);
  }
}
```

#### Command Examples

**1. Static Command Factories (the pattern this repo uses)**

Commands live in their own `XxxCommands` class as **static** methods, never as methods on
the subsystem. The subsystem exposes plain setters; the factory composes them and names the
result. Setpoints arrive as parameters — the factory never reads or hardcodes one.

```java
// Commands in their own class, not on the subsystem
public class IntakeCommands {
  private IntakeCommands() {}

  /** Runs the intake while scheduled, stopping it on interrupt. */
  public static Command runAtVelocity(Intake intake, AngularVelocity velocity) {
    return Commands.startEnd(() -> intake.setVelocity(velocity), intake::stop, intake)
        .withName("Intake_RunAtVelocity");   // ALWAYS name the command
  }

  public static Command stop(Intake intake) {
    return Commands.runOnce(intake::stop, intake).withName("Intake_Stop");
  }
}

// RobotContainer is the WIRING layer — no controllers, no logic, no bare numbers.
public class RobotContainer {
  private final Intake intake;

  private void configureButtonBindings() {
    Triggers triggers = Triggers.getInstance();

    // The trigger is named for the ROBOT ACTION, not the button. Moving the function to a
    // different button then touches one line in Triggers.java and nothing here.
    triggers
        .runIntake()
        .whileTrue(IntakeCommands.runAtVelocity(intake, Constants.Setpoints.INTAKE_VELOCITY));
  }
}
```

Note what is **not** in `RobotContainer`: no `new CommandXboxController(0)` — controllers are
private inside `Triggers.java`, and `ArchitectureRulesTest` fails the build if one appears
anywhere else. And no `0.8`: the velocity comes from `Constants.Setpoints`, so a driver asking
for "a bit more intake speed" sends you to one file rather than a hunt across three.

**2. Commands with Start and End Actions**

```java
// Start a motor when command starts, stop when it ends
public Command shootCommand() {
  return this.startEnd(
    () -> setShooterSpeed(0.9),  // Run on initialize()
    () -> setShooterSpeed(0.0)   // Run on end()
  );
}

// Usage: runs shooter while button held, stops when released
driver.rightBumper().whileTrue(shooter.shootCommand());
```

**3. Commands with Timing**

```java
// Run for specific duration
public Command timedIntakeCommand() {
  return this.run(() -> setIntakeSpeed(0.8))
             .withTimeout(2.0); // Runs for 2 seconds then ends
}

// Run until a condition is met
public Command intakeUntilSensorCommand() {
  return this.run(() -> setIntakeSpeed(0.8))
             .until(() -> hasGamePiece()); // Ends when sensor detects piece
}
```


**4. Command Compositions (Sequential and Parallel)**

```java
// Run commands in sequence
public Command autoCommand() {
  return Commands.sequence(
    drive.resetOdometryCommand(),
    intake.intakeCommand(),
    Commands.waitSeconds(1.0),
    shooter.shootCommand().withTimeout(2.0),
    drive.driveDistanceCommand(1.5)
  );
}

// Run commands in parallel
public Command intakeAndDriveCommand() {
  return Commands.parallel(
    intake.runIntakeCommand(),
    drive.driveDistanceCommand(1.0)
  );
}

// Deadline group - ends when first command finishes
public Command shootSequenceCommand() {
  return shooter.shootCommand()
    .deadlineWith(feeder.feedCommand()); // Both run, ends when shooter ends
}
```

**6. Default Commands**

Default commands must be safe idle states and must never end — use `Commands.run()`, not
`runOnce()`. Axis reads go through `Triggers`' suppliers, never through a controller object.

```java
// In RobotContainer.configureButtonBindings()
Triggers triggers = Triggers.getInstance();

// The suppliers already return robot convention (+X forward, +Y left, +rot CCW) — the
// joystick sign flips live inside Triggers. Do not negate them again here.
drive.setDefaultCommand(
    DriveCommands.joystickDrive(
        drive, triggers::driveXSupplier, triggers::driveYSupplier, triggers::driveRotSupplier));
```

Reading a stick directly here instead of through a supplier is not hypothetical: in 2026 one
command did exactly that and followed an input nobody was holding, while ordinary driving
looked fine. See `docs/drive-controller-mode.md`.

#### Command Lifecycle Flow

```
Button Pressed
    ↓
initialize() ← Called once
    ↓
execute() ← Called every 20ms
    ↓
isFinished()? ← Checked every 20ms
    ↓
   Yes → end(false) ← interrupted = false

Button Released or Another Command Scheduled
    ↓
end(true) ← interrupted = true
```

## Guidance for Students

- Explain your code with comments when introducing new concepts
- Prefer readability over cleverness
- Break complex logic into small, named methods
- Use descriptive variable names (`leftMotorVolts` not `lmv`)
