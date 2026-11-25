package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Volts;

import frc.lib.util.FieldConstants;
import frc.robot.generated.TunerConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;

/**
 * The one maple-sim physics world for the drivetrain: a rigid body with four swerve modules inside
 * a field boundary. Simulation only — real and replay never touch this class.
 *
 * <p>Geometry, gear ratios, wheel radius and friction come from {@code TunerConstants}. The motor
 * types do not: they are assumed here to be Kraken X60 drive and X44 steer, so change them in
 * {@link #create()} if the robot uses others. Simulated current limits are not the real ones
 * either, so do not take brownout or current numbers from the sim. Mass, bumper size and wheel
 * friction are robot properties {@code TunerConstants} does not carry; they are the {@code SIM_*}
 * placeholders in {@link DriveConstants}.
 *
 * <p>The arena is a bare field boundary built from {@link FieldConstants}, not a maple-sim season
 * arena, so it follows whatever field the AprilTag layout describes. When maple-sim ships a 2027
 * arena, swap it in {@link #create()} and place game pieces there.
 *
 * <p>{@code Robot.simulationPeriodic()} must call {@link #periodic()} once per loop.
 */
public final class DriveSimulation {
  private static volatile SwerveDriveSimulation instance;

  private DriveSimulation() {}

  /** Builds the physics world on first use. The robot starts at field center. */
  public static synchronized SwerveDriveSimulation get() {
    if (instance == null) {
      instance = create();
    }
    return instance;
  }

  private static SwerveDriveSimulation create() {
    var fl = TunerConstants.FrontLeft;
    var moduleConfig =
        new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX44Foc(1),
            fl.DriveMotorGearRatio,
            fl.SteerMotorGearRatio,
            Volts.of(fl.DriveFrictionVoltage),
            Volts.of(fl.SteerFrictionVoltage),
            Meters.of(fl.WheelRadius),
            KilogramSquareMeters.of(fl.SteerInertia),
            DriveConstants.SIM_WHEEL_COEFFICIENT_OF_FRICTION);

    // Order must match Drive's modules: FL, FR, BL, BR. maple-sim indexes modules the same way.
    DriveTrainSimulationConfig config =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(DriveConstants.SIM_ROBOT_MASS_KG))
            .withBumperSize(
                Meters.of(DriveConstants.SIM_BUMPER_LENGTH_METERS),
                Meters.of(DriveConstants.SIM_BUMPER_WIDTH_METERS))
            .withCustomModuleTranslations(Drive.getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(moduleConfig);

    SimulatedArena.overrideInstance(new FieldBoundaryArena());
    var simulation =
        new SwerveDriveSimulation(
            config,
            new Pose2d(
                FieldConstants.fieldLength / 2.0,
                FieldConstants.fieldWidth / 2.0,
                Rotation2d.ZERO));
    SimulatedArena.getInstance().addDriveTrainSimulation(simulation);
    return simulation;
  }

  /** Steps the physics world one robot loop. Call from {@code Robot.simulationPeriodic()}. */
  public static void periodic() {
    if (instance == null) {
      return;
    }
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPose", instance.getSimulatedDriveTrainPose());
  }

  /**
   * Ground-truth pose. Feed this to the simulated cameras: they see the real robot, not odometry.
   */
  public static Pose2d getTruePose() {
    return get().getSimulatedDriveTrainPose();
  }

  /**
   * Moves the simulated robot. A no-op until the world exists, so real and replay are unaffected.
   */
  public static synchronized void resetPose(Pose2d pose) {
    if (instance != null) {
      instance.setSimulationWorldPose(pose);
      // Teleporting must not leave the old momentum behind, or sim drifts away from the reset.
      instance.setRobotSpeeds(new ChassisVelocities());
    }
  }

  /** Four walls at the edges of the field, and nothing else. */
  private static final class FieldBoundaryArena extends SimulatedArena {
    FieldBoundaryArena() {
      super(
          new FieldMap() {
            {
              double x = FieldConstants.fieldLength;
              double y = FieldConstants.fieldWidth;
              addBorderLine(new Translation2d(0, 0), new Translation2d(x, 0));
              addBorderLine(new Translation2d(x, 0), new Translation2d(x, y));
              addBorderLine(new Translation2d(x, y), new Translation2d(0, y));
              addBorderLine(new Translation2d(0, y), new Translation2d(0, 0));
            }
          });
    }

    @Override
    public void placeGamePiecesOnField() {}
  }
}
