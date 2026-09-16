// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.lib.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Temperature> driveTemp;
  private final StatusSignal<Boolean> driveStickyBootDuringEnable;
  private final StatusSignal<Boolean> driveStickyUndervoltage;
  private final StatusSignal<Boolean> driveStickyOverTemp;
  private final StatusSignal<Boolean> driveStickyHardware;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;
  private final StatusSignal<Current> turnTorqueCurrent;
  private final StatusSignal<Current> turnSupplyCurrent;
  private final StatusSignal<Temperature> turnTemp;
  private final StatusSignal<Boolean> turnStickyBootDuringEnable;
  private final StatusSignal<Boolean> turnStickyUndervoltage;
  private final StatusSignal<Boolean> turnStickyOverTemp;
  private final StatusSignal<Boolean> turnStickyHardware;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
    turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
    cancoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

    // Configure drive motor
    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor.
    // Start from the constants' initial configs, the same way the drive motor above does. Building
    // a bare TalonFXConfiguration here silently dropped the steer current limits, leaving the
    // azimuth on the Phoenix defaults of 120 A stator / 70 A supply instead of the 40 A / 40 A that
    // TunerConstants asks for. Match logs showed steer peaks of 114 A as a result.
    // Note: this object is shared across all four modules, so each constructor mutates it and
    // applies before the next one runs. That is the existing pattern the drive motor uses.
    var turnConfig = constants.SteerMotorInitialConfigs;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // The steer closed loop outputs torque current, so the stator limit alone does not bound what
    // the controller asks for — clamp the request too, mirroring the drive motor above
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = turnConfig.CurrentLimits.StatorCurrentLimit;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -turnConfig.CurrentLimits.StatorCurrentLimit;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new RuntimeException(
              "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTemp = driveTalon.getDeviceTemp();
    driveStickyBootDuringEnable = driveTalon.getStickyFault_BootDuringEnable();
    driveStickyUndervoltage = driveTalon.getStickyFault_Undervoltage();
    driveStickyOverTemp = driveTalon.getStickyFault_DeviceTemp();
    driveStickyHardware = driveTalon.getStickyFault_Hardware();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnPosition.clone());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();
    turnTorqueCurrent = turnTalon.getTorqueCurrent();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnTemp = turnTalon.getDeviceTemp();
    turnStickyBootDuringEnable = turnTalon.getStickyFault_BootDuringEnable();
    turnStickyUndervoltage = turnTalon.getStickyFault_Undervoltage();
    turnStickyOverTemp = turnTalon.getStickyFault_DeviceTemp();
    turnStickyHardware = turnTalon.getStickyFault_Hardware();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTorqueCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTorqueCurrent,
        driveSupplyCurrent,
        turnSupplyCurrent);
    // Temperature changes on the order of minutes, so it does not belong in the 50 Hz group.
    // Registered explicitly anyway: optimizeBusUtilization() below slows everything that is
    // not registered to 4 Hz, and being at 4 Hz by intent is different from being at 4 Hz by
    // omission. Temperature and faults both change slowly, and sticky faults latch until
    // cleared, so 4 Hz loses nothing here.
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0,
        driveTemp,
        turnTemp,
        driveStickyBootDuringEnable,
        driveStickyUndervoltage,
        driveStickyOverTemp,
        driveStickyHardware,
        turnStickyBootDuringEnable,
        turnStickyUndervoltage,
        turnStickyOverTemp,
        turnStickyHardware);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            driveTorqueCurrent,
            driveSupplyCurrent,
            driveTemp,
            driveStickyBootDuringEnable,
            driveStickyUndervoltage,
            driveStickyOverTemp,
            driveStickyHardware);
    var turnStatus =
        BaseStatusSignal.refreshAll(
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent,
            turnTorqueCurrent,
            turnSupplyCurrent,
            turnTemp,
            turnStickyBootDuringEnable,
            turnStickyUndervoltage,
            turnStickyOverTemp,
            turnStickyHardware);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTempCelsius = driveTemp.getValueAsDouble();
    inputs.driveStickyBootDuringEnable = driveStickyBootDuringEnable.getValue();
    inputs.driveStickyUndervoltage = driveStickyUndervoltage.getValue();
    inputs.driveStickyOverTemp = driveStickyOverTemp.getValue();
    inputs.driveStickyHardwareFault = driveStickyHardware.getValue();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
    inputs.turnTempCelsius = turnTemp.getValueAsDouble();
    inputs.turnStickyBootDuringEnable = turnStickyBootDuringEnable.getValue();
    inputs.turnStickyUndervoltage = turnStickyUndervoltage.getValue();
    inputs.turnStickyOverTemp = turnStickyOverTemp.getValue();
    inputs.turnStickyHardwareFault = turnStickyHardware.getValue();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setDriveBrakeMode(boolean brake) {
    // setNeutralMode() is the fast path — it does not re-apply the whole configuration, so it
    // is safe to call on a mode transition without risking a slow CAN operation mid-match.
    driveTalon.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }
}
