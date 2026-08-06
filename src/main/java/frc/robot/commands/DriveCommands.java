// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.LoggedTunableProfiledPID;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Game-agnostic drive command factories.
 *
 * <p>Add game-specific alignment commands here as the 2027 game is understood. The 2026 season had
 * seven of them (align-to-goal, snap-to-trench, snap-to-tower, sweep headings, and so on); they
 * were all built by passing a heading supplier to {@link #joystickDriveAtAngle}, which is the
 * pattern to follow:
 *
 * <pre>
 * public static Command alignForScore(Drive drive, DoubleSupplier x, DoubleSupplier y) {
 *   return joystickDriveAtAngle(drive, x, y,
 *       () -&gt; RobotState.getInstance().getAngleToTarget(SOME_TARGET));
 * }
 * </pre>
 *
 * <p>{@link #driveToPose} is the same idea one level up: instead of holding a heading while the
 * driver steers, it drives translation AND heading to a fixed field pose via PID, with no
 * pathfinding. Reach for it for pick/place-style games where the destination is a specific spot (a
 * station, a fixed scoring position) and the path there never has anything to avoid — {@link
 * Drive#pathfindToPose} is the pathfinding alternative when that is not true.
 */
public class DriveCommands {
  private static final double DEADBAND = 0.1;

  // Heading-hold controller. Gains were tuned on the 2026 competition robot — re-tune for
  // 2027, since they depend on robot mass and moment of inertia, not on the game.
  //
  // TODO(2027): re-tune via /pid-tune (or in sim) once the 2027 chassis exists. These numbers
  // describe the 2026 robot's mass and MOI; treat them as a starting point, not a given.
  //
  // This is the worked example of LoggedTunableProfiledPID. With Constants.tuningMode on, kP,
  // kI, kD, maxVelocity and maxAcceleration appear under "Tuning/Drive/Heading/..." and can be
  // adjusted while the robot is enabled; with it off they are exactly these numbers and cost
  // nothing. Whatever you land on during a session, write it back here and commit it —
  // dashboard values live only in NetworkTables and are gone at the next reboot.
  //
  // Static, so every command built by this class shares one controller and one set of
  // dashboard keys. Two instances would publish duplicate keys for the same gain. Sharing is
  // safe because every command that uses it requires the drive subsystem, so only one can run
  // at a time.
  private static final LoggedTunableProfiledPID angleController =
      new LoggedTunableProfiledPID("Drive/Heading", 8.5, 0.0, 0.3, 12.0, 20.0);

  static {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Translation controller for driveToPose. UNTUNED PLACEHOLDER — this is a new capability with
  // no tuning data behind it yet, unlike angleController above. driveToPose reuses angleController
  // for heading rather than declaring a second one, for the same reason angleController is
  // shared and static: every command in this class requires the drive subsystem, so only one
  // can run at a time, and sharing means one set of dashboard keys instead of two.
  //
  // TODO(2027): run a /pid-tune (or in-sim) session on the real chassis and commit the result
  // here before binding driveToPose to a button. DriveToPoseSimTest only proves the command
  // converges against a generic sim model — it is not evidence these gains are good on hardware.
  private static final LoggedTunableProfiledPID driveController =
      new LoggedTunableProfiledPID("Drive/ToPose", 3.0, 0.0, 0.0, 3.0, 3.0);

  // How close counts as "there" for driveToPose. Tight enough that DistanceErrorMeters settles
  // near zero at rest; loose enough that direction-vector noise right at the goal (the atan2 of
  // a near-zero vector) doesn't turn into commanded chatter.
  private static final double DRIVE_TO_POSE_TOLERANCE_METERS = 0.02;

  private static final double FF_START_DELAY = 2.0; // Secs

  // UNITS: this is the ramp rate of the value handed to Module.runCharacterization(), which
  // ModuleIOTalonFX.setDriveOpenLoop() forwards according to the configured
  // ClosedLoopOutputType. TunerConstants selects TorqueCurrentFOC, so this is AMPS/sec and
  // the kS/kV printed by feedforwardCharacterization come out in amps — exactly what
  // driveGains needs. It is only Volts/sec if you switch the drive output type to Voltage.
  // See docs/characterization-and-tuning.md.
  private static final double FF_RAMP_RATE = 0.1; // Amps/Sec (see note above)

  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control at low speed
    linearMagnitude = Math.pow(linearMagnitude, 2.0);

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
            () -> {
              drive.areWheelsXed = false;
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Raise rotation value to the configured exponent for more precise control.
              // DriveConstants.rotationExponent is settable per driver preset.
              omega =
                  Math.copySign(Math.pow(Math.abs(omega), DriveConstants.rotationExponent), omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .withName("Drive_Joystick");
  }

  /**
   * Field relative drive command with linear speed clamped to {@link DriveConstants#limitedVelo}.
   *
   * <p>Useful as a "precision mode" binding, or gated behind a mechanism-safety condition (in 2026
   * this ran whenever the intake was extended).
   */
  public static Command joystickDriveLimited(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
            () -> {
              drive.areWheelsXed = false;
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * DriveConstants.limitedVelo,
                      linearVelocity.getY() * DriveConstants.limitedVelo,
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .withName("Drive_JoystickLimited");
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   *
   * <p>This is the building block for every "align to something" command: pass a supplier that
   * returns the heading you want to hold. Possible use cases include snapping to an angle, aiming
   * at a vision target, or controlling absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Construct command
    return Commands.run(
            () -> {
              drive.areWheelsXed = false;

              // Pick up live gain edits while tuning. No-op when tuningMode is off, and even
              // when on this only rebuilds when a value actually changed rather than every loop.
              angleController.updateGains();

              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Log target and current angles every loop for debugging
              Logger.recordOutput("AutoAim/TargetAngle", rotationSupplier.get());
              Logger.recordOutput("AutoAim/CurrentAngle", drive.getRotation());
              Logger.recordOutput("AutoAim/AngleErrorRad", angleController.getPositionError());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .finallyDo(
            () -> {
              Logger.recordOutput("AutoAim/TargetAngle", 0.0);
              Logger.recordOutput("AutoAim/CurrentAngle", 0.0);
              Logger.recordOutput("AutoAim/AngleErrorRad", 0.0);
            })
        .withName("Drive_JoystickAtAngle");
  }

  /**
   * Drives straight to a fixed field pose using PID control on both translation and heading — no
   * pathfinding, no obstacle avoidance. See {@link Drive#pathfindToPose} for the pathfinding
   * alternative when the route there is not always clear.
   *
   * <p>Runs until interrupted, the same as {@link #joystickDriveAtAngle} — it does not finish on
   * its own. To gate a follow-up action on arrival, build an "is aligned" check at the call site
   * from {@code drive.getPose()} against the same target pose (see the Ready → Align → Act pattern
   * in {@code .claude/rules/03-commands.md}); {@code AutoAim/DriveToPose/*} is logged every loop to
   * make that easy to verify against a match log.
   *
   * @param drive Drive subsystem
   * @param targetPoseSupplier Field-relative goal pose, blue-alliance origin. Read every loop, so a
   *     moving target is fine. Not alliance-flipped here — flip it with {@code AllianceFlipUtil} at
   *     the call site if the target should mirror by alliance.
   */
  public static Command driveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    return Commands.run(
            () -> {
              drive.areWheelsXed = false;

              // Pick up live gain edits while tuning, same as joystickDriveAtAngle.
              angleController.updateGains();
              driveController.updateGains();

              Pose2d current = drive.getPose();
              Pose2d target = targetPoseSupplier.get();

              Translation2d toTarget = target.getTranslation().minus(current.getTranslation());
              double distanceMeters = toTarget.getNorm();

              // Profiled PID drives the scalar distance to zero, so its output is the rate the
              // distance should be SHRINKING at — negative while far away (measurement=distance
              // is above goal=0). Pointing that negative scalar along the target-minus-current
              // vector would aim it away from the target, not at it; pointing it along the
              // reverse (current-minus-target) vector cancels the sign and aims correctly.
              double driveVelocity = driveController.calculate(distanceMeters, 0.0);
              Translation2d velocity =
                  distanceMeters < DRIVE_TO_POSE_TOLERANCE_METERS
                      ? Translation2d.kZero
                      : new Translation2d(
                          driveVelocity,
                          current.getTranslation().minus(target.getTranslation()).getAngle());

              double omega =
                  angleController.calculate(
                      current.getRotation().getRadians(), target.getRotation().getRadians());

              Logger.recordOutput("AutoAim/DriveToPose/TargetPose", target);
              Logger.recordOutput("AutoAim/DriveToPose/CurrentPose", current);
              Logger.recordOutput("AutoAim/DriveToPose/DistanceErrorMeters", distanceMeters);
              Logger.recordOutput(
                  "AutoAim/DriveToPose/AngleErrorRad", angleController.getPositionError());

              // toTarget is already expressed in the absolute field frame (blue-alliance
              // origin), so this only needs the actual robot heading to become robot-relative —
              // unlike joystickDrive, there is no driver-perspective input here to alliance-flip.
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      velocity.getX(), velocity.getY(), omega, current.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              Pose2d current = drive.getPose();
              Pose2d target = targetPoseSupplier.get();
              driveController.reset(current.getTranslation().getDistance(target.getTranslation()));
              angleController.reset(current.getRotation().getRadians());
            })
        .finallyDo(
            () -> {
              Logger.recordOutput("AutoAim/DriveToPose/TargetPose", Pose2d.kZero);
              Logger.recordOutput("AutoAim/DriveToPose/CurrentPose", Pose2d.kZero);
              Logger.recordOutput("AutoAim/DriveToPose/DistanceErrorMeters", 0.0);
              Logger.recordOutput("AutoAim/DriveToPose/AngleErrorRad", 0.0);
            })
        .withName("Drive_ToPose");
  }

  /**
   * Stops the drive and points the modules in an X pattern to resist being pushed.
   *
   * <p>The modules return to normal orientation the next time a nonzero velocity is requested.
   */
  public static Command stopWithX(Drive drive) {
    return Commands.run(
            () -> {
              drive.areWheelsXed = true;
              drive.stopWithX();
            },
            drive)
        .withName("Drive_StopWithX");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
