package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;

/** Gyro read from the maple-sim chassis, so heading follows collisions and wheel slip. */
public class GyroIOMapleSim implements GyroIO {
  private final GyroSimulation gyro;

  public GyroIOMapleSim(GyroSimulation gyro) {
    this.gyro = gyro;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyro.getGyroReading();
    inputs.yawVelocityRadPerSec = gyro.getMeasuredAngularVelocity().in(RadiansPerSecond);

    // One sample per loop, matching ModuleIOSim: high-rate odometry buys nothing in simulation.
    inputs.odometryYawTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
  }
}
