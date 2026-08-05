package frc.robot.subsystems.example.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of ExampleSubsystemIO.
 *
 * <p>TEMPLATE INSTRUCTIONS: 1. Choose the right WPILib sim model: - FlywheelSim for spinning
 * mechanisms (wheels, rollers) - SingleJointedArmSim for pivoting mechanisms - ElevatorSim for
 * linear mechanisms 2. Update motor type and gear ratio 3. Call sim.update(0.02) in updateInputs()
 * 4. For mechanisms where sim physics don't matter, stub methods are fine
 *
 * <p>NOTE: An empty stub (all defaults from the interface) is acceptable for sim if the mechanism
 * is not critical to simulate. Don't over-engineer sim models.
 */
public class ExampleSubsystemIOSim implements ExampleSubsystemIO {

  // TODO: choose appropriate sim class and configure motor + gear ratio
  // TODO: choose the sim class that matches the mechanism, and pull the model parameters from
  // ExampleSubsystemConstants.Sim rather than inlining them. The MOI in particular sets how fast
  // the simulated mechanism accelerates, so gains tuned against a guessed value are meaningless.
  //
  //   new FlywheelSim(
  //       LinearSystemId.createFlywheelSystem(
  //           ExampleSubsystemConstants.Sim.MOTOR,
  //           ExampleSubsystemConstants.Sim.MOI,
  //           ExampleSubsystemConstants.GEAR_RATIO),
  //       ExampleSubsystemConstants.Sim.MOTOR);
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.001, 1.0),
          DCMotor.getKrakenX60(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ExampleSubsystemIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.motorVoltage = Volts.of(appliedVolts);
    inputs.motorVelocity = RotationsPerSecond.of(sim.getAngularVelocityRPM() / 60.0);
    // FlywheelSim has no position. Swap to SingleJointedArmSim or ElevatorSim for a position
    // mechanism, and report its angle here.
    inputs.motorPosition = Rotations.of(0);
    inputs.motorStatorAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.motorSupplyAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.motorTemperature = Celsius.of(25.0);

    // Leave motorTorqueCurrentAmps at 0. WPILib sim models total current draw, not the
    // torque-producing component a FOC controller commands, so anything written here
    // would be fiction that looks real in a log. Don't "helpfully" populate it. The field
    // still exists in the shared AutoLog schema, so the real and sim schemas match.
  }

  @Override
  public void setVoltage(Voltage volts) {
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    // For sim: approximate by applying voltage proportional to target velocity
    // TODO: replace with better model if needed
    appliedVolts = velocity.in(RotationsPerSecond) * 0.1;
  }
}
