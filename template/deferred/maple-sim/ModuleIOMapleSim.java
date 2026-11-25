package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;

/**
 * Module IO backed by one maple-sim swerve module. Like {@link ModuleIOSim} it is voltage-driven
 * with simple PID loops in code, but the wheels push a simulated chassis instead of spinning free,
 * so traction limits, collisions and wheel slip show up in the logs.
 *
 * <p>Torque-current channels stay at zero for the same reason as {@link ModuleIOSim}: the sim
 * models voltage and current draw, not the torque-producing component a FOC controller commands.
 */
public class ModuleIOMapleSim implements ModuleIO {
  // Same loop gains as ModuleIOSim. They tune the sim's stand-in for the Talon's closed loop and
  // have nothing to do with the real robot's gains.
  private static final double DRIVE_KP = 0.05;
  private static final double TURN_KP = 8.0;

  /**
   * Steer current limit for the simulated motor: an invented number, not the real limit. The drive
   * limit passed in is the slip current, also not the configured stator limit.
   */
  private static final double STEER_CURRENT_LIMIT_AMPS = 20.0;

  private final SwerveModuleSimulation module;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;
  private final double driveKvVoltsPerRadPerSec;

  private final PIDController driveController = new PIDController(DRIVE_KP, 0, 0);
  private final PIDController turnController = new PIDController(TURN_KP, 0, 0);
  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOMapleSim(SwerveModuleSimulation module, double driveCurrentLimitAmps) {
    this.module = module;
    this.driveMotor =
        module.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(driveCurrentLimitAmps));
    this.turnMotor =
        module.useGenericControllerForSteer().withCurrentLimit(Amps.of(STEER_CURRENT_LIMIT_AMPS));

    // Volts per wheel rad/s at steady state: the motor's back-EMF constant, through the gearing.
    driveKvVoltsPerRadPerSec =
        module.config.DRIVE_GEAR_RATIO / module.config.driveMotorConfigs.motor.KvRadPerSecPerVolt;
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(module.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(module.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }
    driveAppliedVolts = Math.clamp(driveAppliedVolts, -12.0, 12.0);
    turnAppliedVolts = Math.clamp(turnAppliedVolts, -12.0, 12.0);
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    inputs.driveConnected = true;
    inputs.drivePositionRad = module.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = module.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(module.getDriveMotorStatorCurrent().in(Amps));
    inputs.driveSupplyCurrentAmps = Math.abs(module.getDriveMotorSupplyCurrent().in(Amps));
    inputs.driveTempCelsius = 25.0; // not modeled

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = module.getSteerAbsoluteFacing();
    inputs.turnPosition = module.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec = module.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(module.getSteerMotorStatorCurrent().in(Amps));
    inputs.turnSupplyCurrentAmps = Math.abs(module.getSteerMotorSupplyCurrent().in(Amps));
    inputs.turnTempCelsius = 25.0; // not modeled

    // One sample per loop, matching ModuleIOSim.
    inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveKvVoltsPerRadPerSec * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
