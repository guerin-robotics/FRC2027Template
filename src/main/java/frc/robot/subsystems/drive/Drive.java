// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.FieldConstants;
import frc.lib.LocalADStarAK;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  /** True while the modules are held in the X pattern by {@code DriveCommands.stopWithX}. */
  public boolean areWheelsXed = false;

  // ---- Pose sanity detection ----
  //
  // This DETECTS a diverged pose estimate. It deliberately does not reject or correct one.
  //
  // The 2026 codebase had a maxPoseJumpMeters rejection filter that was written and then left
  // disabled because it was never tuned against real logs — an untuned rejection filter that
  // throws away good vision is worse than no filter at all. So: surface it loudly, gather real
  // data, and turn it into rejection later with numbers behind it.
  //
  // How far outside the field boundary the estimate must land before it is considered bad.
  private static final double OFF_FIELD_MARGIN_METERS = 1.0;

  // A single vision update moving the estimate further than this is worth knowing about.
  private static final double POSE_JUMP_THRESHOLD_METERS = 1.0;

  private final Alert offFieldAlert =
      new Alert("Pose estimate is off the field — odometry or vision is wrong.", AlertType.kError);

  private double largestJumpThisLoop = 0.0;
  private int poseJumpCount = 0;

  /**
   * Kraken X60s begin thermally limiting output well before they fault. A robot that gets slower
   * late in a match with no other explanation is usually this.
   */
  private static final double MOTOR_HOT_CELSIUS = 70.0;

  private final Alert motorHotAlert =
      new Alert(
          "A swerve motor is running hot — output will be thermally limited.", AlertType.kWarning);

  private final Alert stickyFaultAlert =
      new Alert(
          "A swerve motor latched a sticky fault (reboot, undervoltage, over-temp or hardware). "
              + "Check Drive/Module*/Sticky* in the log.",
          AlertType.kWarning);

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 63.503;
  private static final double ROBOT_MOI = 5.162;
  private static final double WHEEL_COF = 2.225;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // PathPlanner wiring is NOT done here — see configureAutoBuilder(), called by
    // RobotContainer. It mutates PathPlanner's global state, which a constructor should not.

    // Configure SysId
    //
    // UNITS WARNING: SysId's Measure<Voltage> is forwarded to runCharacterization(), which
    // ModuleIOTalonFX sends as a TORQUE CURRENT request because TunerConstants selects
    // ClosedLoopOutputType.TorqueCurrentFOC. So SysId's "volts" are amps here, and the
    // null config below means WPILib's defaults (1 V/s ramp, 7 V step) land as 1 A/s and
    // 7 A — far gentler than intended. Pass an explicit SysIdRoutine.Config with
    // amp-appropriate values before relying on these routines.
    // See docs/characterization-and-tuning.md.
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Wire RobotState to use Drive's single pose estimator.
    // This ensures there is only ONE SwerveDrivePoseEstimator — eliminating the dual-estimator
    // divergence bug where two independent estimators would drift apart.
    RobotState.getInstance().setPoseSupplier(this::getPose);
  }

  /**
   * Wires this drivetrain into PathPlanner. Call once, from {@code RobotContainer}, and <b>before
   * {@code AutoBuilder.buildAutoChooser()}</b>.
   *
   * <p>The configuration itself stays here rather than in {@code RobotContainer} — the gains, the
   * robot config and the output consumer all belong with the drivetrain, and {@code
   * .claude/rules/01-architecture.md} requires it. What moved out of the constructor is only
   * <i>when</i> it runs.
   *
   * <p><b>Why it is not in the constructor.</b> Every call below mutates PathPlanner's global
   * static state: one `AutoBuilder`, one pathfinder, one pair of logging callbacks, shared by the
   * whole JVM. Constructing a `Drive` therefore reached outside itself and reconfigured a
   * singleton, which meant a second `Drive` — four sim tests build one — silently stole the binding
   * from the first and made PathPlanner report an error on every test run.
   *
   * <p><b>If you forget to call this</b>, {@code AutoBuilder.buildAutoChooser()} throws {@code
   * AutoBuilderException: AutoBuilder was not configured}, taking out the whole auto chooser rather
   * than one routine. That is loud by design, and {@code RobotContainerSmokeTest} catches it before
   * it reaches a robot.
   */
  public void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        // Gains tuned via PathFollowingGainSweepTest (kP 50 saturated the modules and amplified
        // vision pose corrections into velocity steps, causing chop)
        // Sim-derived values (10.0 / 7.5); last real-robot tested values from drive
        // practice were 40.0 / 35.0 (PR #91)
        new PPHolonomicDriveController(
            new PIDConstants(10.0, 0.0, 0.0), new PIDConstants(7.1, 0.0, 0.1)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  @Override
  public void periodic() {
    // try/finally so the odometry thread is never left blocked on a lock this method failed to
    // release. In practice an exception here takes the robot program down with it, so this is
    // hygiene rather than a live hazard — but an unreleased lock turns a crash into a hang.
    odometryLock.lock(); // Prevents odometry updates while reading data
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    RobotState.getInstance().updateModuleStates(getModuleStates());

    // Report current usage to the battery logger for each swerve module
    double hottestMotorCelsius = 0.0;
    boolean anyStickyFault = false;
    for (int i = 0; i < 4; i++) {
      // SUPPLY current, not stator. BatteryLogger converts this to power against battery
      // voltage, and stator current would overstate the draw — badly, at low speed under load.
      Robot.batteryLogger.reportCurrentUsage(
          "Drive/Module" + i + "-Drive", true, modules[i].getDriveSupplyCurrentAmps());
      Robot.batteryLogger.reportCurrentUsage(
          "Drive/Module" + i + "-Turn", true, modules[i].getTurnSupplyCurrentAmps());

      hottestMotorCelsius =
          Math.max(
              hottestMotorCelsius,
              Math.max(modules[i].getDriveTempCelsius(), modules[i].getTurnTempCelsius()));
      anyStickyFault |= modules[i].hasStickyFault();
    }

    Logger.recordOutput("Drive/HottestMotorCelsius", hottestMotorCelsius);
    motorHotAlert.set(hottestMotorCelsius > MOTOR_HOT_CELSIUS);

    Logger.recordOutput("Drive/AnyStickyFault", anyStickyFault);
    stickyFaultAlert.set(anyStickyFault);

    // ---- Pose sanity ----
    Pose2d estimate = getPose();
    boolean offField =
        estimate.getX() < -OFF_FIELD_MARGIN_METERS
            || estimate.getX() > FieldConstants.fieldLength + OFF_FIELD_MARGIN_METERS
            || estimate.getY() < -OFF_FIELD_MARGIN_METERS
            || estimate.getY() > FieldConstants.fieldWidth + OFF_FIELD_MARGIN_METERS;

    Logger.recordOutput("Odometry/OffField", offField);
    Logger.recordOutput("Odometry/LargestVisionJumpMeters", largestJumpThisLoop);
    Logger.recordOutput("Odometry/PoseJumpCount", poseJumpCount);
    offFieldAlert.set(offField);
    largestJumpThisLoop = 0.0;

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /**
   * Sets the drive motors to brake or coast.
   *
   * <p>Coast while disabled makes the robot pushable in the pit; brake while enabled keeps it where
   * the driver put it. {@code Robot} handles the transitions — see the coast delay there.
   *
   * @param brake true for brake, false for coast
   */
  public void setDriveBrakeMode(boolean brake) {
    for (var module : modules) {
      module.setDriveBrakeMode(brake);
    }
    Logger.recordOutput("Drive/BrakeMode", brake);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Schedules a PathPlanner pathfinding command that drives the robot to {@code targetPose}.
   *
   * <p>In 2026 this backed the "drive to the defensive shooting spot" button. It is left here as a
   * general capability — give it any field pose and PathPlanner will navigate to it around the
   * configured obstacles.
   *
   * <p>Note this schedules the command itself rather than returning it, so the caller does not need
   * to compose it. That also means it will interrupt whatever else currently requires this
   * subsystem.
   *
   * @param targetPose Field-relative pose to drive to
   */
  public void pathfindToPose(Pose2d targetPose) {
    Command followCommand =
        AutoBuilder.pathfindToPose(targetPose, PathConstraints.unlimitedConstraints(12));
    Logger.recordOutput("Drive/PathfindTargetPose", targetPose);
    CommandScheduler.getInstance().schedule(followCommand);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** True when the gyro is reporting. Registered with {@code FaultMonitor}. */
  public boolean isGyroConnected() {
    return gyroInputs.connected;
  }

  /**
   * True if any swerve motor has latched a sticky fault this power cycle. Registered with {@code
   * FaultMonitor}.
   */
  public boolean hasStickyFault() {
    for (var module : modules) {
      if (module.hasStickyFault()) {
        return true;
      }
    }
    return false;
  }

  /**
   * True when the pose estimate has left the field boundary. Registered with {@code FaultMonitor}.
   */
  public boolean isPoseOffField() {
    return offFieldAlert.get();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    Pose2d before = poseEstimator.getEstimatedPosition();
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    Pose2d after = poseEstimator.getEstimatedPosition();

    // Vision can be applied several times per loop (one per camera per observation), so keep
    // the largest displacement and report it once in periodic().
    double jump = after.getTranslation().getDistance(before.getTranslation());
    if (jump > largestJumpThisLoop) {
      largestJumpThisLoop = jump;
    }
    if (jump > POSE_JUMP_THRESHOLD_METERS) {
      poseJumpCount++;
    }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
