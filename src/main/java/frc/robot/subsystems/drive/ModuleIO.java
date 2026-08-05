// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    // STATOR current — current in the windings. Use for stall and heating.
    public double driveCurrentAmps = 0.0;
    // SUPPLY current — current drawn from the battery. Use for power and brownout analysis.
    // These are NOT interchangeable: at low speed and high torque, supply is a fraction of
    // stator. Feeding stator into a battery-power calculation overstates the draw, which is
    // exactly the bug this pair of fields exists to prevent.
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;
    // Torque-producing current. With a *TorqueCurrentFOC request this IS the control
    // signal, which stator current is not — a stalled module and a free-spinning one can
    // draw similar stator current but very different torque current.
    public double driveTorqueCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTempCelsius = 0.0;
    public double turnTorqueCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}

  /**
   * Sets the drive motor neutral mode. Steer stays in brake always — a coasting azimuth flops
   * around and makes the modules hard to align by hand.
   *
   * @param brake true for brake, false for coast
   */
  public default void setDriveBrakeMode(boolean brake) {}
}
