package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.drivesims.GyroSimulation;

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
    inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
  }
}
