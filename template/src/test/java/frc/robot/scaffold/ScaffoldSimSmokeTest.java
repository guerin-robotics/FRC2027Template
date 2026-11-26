package frc.robot.scaffold;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import frc.robot.subsystems.exampleArm.ExampleArmConstants;
import frc.robot.subsystems.exampleArm.io.ExampleArmIO.ExampleArmIOInputs;
import frc.robot.subsystems.exampleArm.io.ExampleArmIOSim;
import frc.robot.subsystems.exampleLift.ExampleLiftConstants;
import frc.robot.subsystems.exampleLift.io.ExampleLiftIO.ExampleLiftIOInputs;
import frc.robot.subsystems.exampleLift.io.ExampleLiftIOSim;
import frc.robot.subsystems.exampleRoller.io.ExampleRollerIO.ExampleRollerIOInputs;
import frc.robot.subsystems.exampleRoller.io.ExampleRollerIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.controller.PIDController;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

class ScaffoldSimSmokeTest {
  private static final int STEPS = 300; // Six simulated seconds at 20 ms per update.
  private static final double EPSILON = 1e-6;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    RoboRioSim.setVInVoltage(12.0);
  }

  @Test
  void rollerConvergesToVelocity() {
    // Controller gains are specific to the synthetic fixture.
    var sim = new ExampleRollerIOSim(new PIDController(0.18, 4.0, 0.0));
    var inputs = new ExampleRollerIOInputs();
    sim.setVelocity(RotationsPerSecond.of(3.0));
    step(
        () -> {
          sim.updateInputs(inputs);
          checkCurrent(
              inputs.motorVoltage,
              inputs.motorStatorAmps,
              inputs.motorSupplyAmps,
              inputs.motorTorqueCurrentAmps);
        });
    assertEquals(3.0, inputs.motorVelocity.in(RotationsPerSecond), 0.05);
  }

  @Test
  void armConvergesAndRespectsBothTravelLimits() {
    var sim = new ExampleArmIOSim(new PIDController(12.0, 4.0, 0.0));
    var inputs = new ExampleArmIOInputs();
    double lower = ExampleArmConstants.REVERSE_SOFT_LIMIT_DEGREES;
    double upper = ExampleArmConstants.FORWARD_SOFT_LIMIT_DEGREES;
    Runnable tick =
        () -> {
          sim.updateInputs(inputs);
          checkCurrent(
              inputs.motorVoltage,
              inputs.motorStatorAmps,
              inputs.motorSupplyAmps,
              inputs.motorTorqueCurrentAmps);
          checkTravel(inputs.motorPosition.in(Degrees), lower, upper);
        };

    sim.setPosition(Degrees.of(45.0));
    step(tick);
    assertEquals(45.0, inputs.motorPosition.in(Degrees), 0.5);

    sim.setPosition(Degrees.of(upper + 30.0));
    step(tick);
    assertEquals(upper, inputs.motorPosition.in(Degrees), EPSILON);

    sim.setPosition(Degrees.of(lower - 30.0));
    step(tick);
    assertEquals(lower, inputs.motorPosition.in(Degrees), EPSILON);
  }

  @Test
  void liftConvergesRespectsBothTravelLimitsAndZeros() {
    var sim = new ExampleLiftIOSim(new PIDController(10.0, 5.0, 0.0));
    var inputs = new ExampleLiftIOInputs();
    double lower = ExampleLiftConstants.MIN_TRAVEL_INCHES;
    double upper = ExampleLiftConstants.MAX_TRAVEL_INCHES;
    Runnable tick =
        () -> {
          sim.updateInputs(inputs);
          checkCurrent(
              inputs.motorVoltage,
              inputs.motorStatorAmps,
              inputs.motorSupplyAmps,
              inputs.motorTorqueCurrentAmps);
        };
    Runnable boundedTick =
        () -> {
          tick.run();
          checkTravel(liftHeightInches(inputs), lower, upper);
        };

    sim.setPosition(liftPosition(18.0));
    step(boundedTick);
    assertEquals(18.0, liftHeightInches(inputs), 0.1);

    sim.setPosition(liftPosition(lower - 12.0));
    step(boundedTick);
    assertEquals(lower, liftHeightInches(inputs), EPSILON);

    sim.setPosition(liftPosition(upper + 12.0));
    step(boundedTick);
    assertEquals(upper, liftHeightInches(inputs), EPSILON);

    sim.zeroPosition();
    // The previous command still holds the carriage at its physical upper stop.
    tick.run();
    assertEquals(0.0, liftHeightInches(inputs), EPSILON);

    sim.setPosition(liftPosition(-6.0));
    step(
        () -> {
          tick.run();
          checkTravel(liftHeightInches(inputs), lower - upper, 0.0);
        });
    assertEquals(-6.0, liftHeightInches(inputs), 0.1);
  }

  private static Angle liftPosition(double inches) {
    return Rotations.of(ExampleLiftConstants.inchesToRotations(inches));
  }

  private static double liftHeightInches(ExampleLiftIOInputs inputs) {
    return ExampleLiftConstants.rotationsToInches(inputs.motorPosition.in(Rotations));
  }

  private static void step(Runnable tick) {
    for (int i = 0; i < STEPS; i++) {
      tick.run();
    }
  }

  private static void checkTravel(double position, double lower, double upper) {
    assertTrue(
        position >= lower - EPSILON && position <= upper + EPSILON,
        () -> "Position outside simulated travel: " + position);
  }

  private static void checkCurrent(
      Voltage voltage, Current stator, Current supply, Current torque) {
    double scaledStator = Math.abs(stator.in(Amps)) * Math.abs(voltage.in(Volts)) / 12.0;
    assertEquals(
        scaledStator, supply.in(Amps), EPSILON, "Supply current must be duty-cycle scaled");
    assertEquals(0.0, torque.in(Amps), "FOC torque current must stay unmodeled");
  }
}
