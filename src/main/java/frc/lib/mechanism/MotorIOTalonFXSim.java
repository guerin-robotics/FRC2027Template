package frc.lib.mechanism;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The simulated hardware implementation, backed by Phoenix's own device simulation.
 *
 * <p>It extends {@link MotorIOTalonFX} rather than reimplementing it, which is the whole point: the
 * real {@link MotorConfig} is applied to a simulated Talon, so the gains, the Motion Magic profile,
 * the current limits and the soft limits that run in simulation are the ones that will run on the
 * robot. A gain found in a simulated tuning session means something when it is deployed.
 *
 * <p>The alternative — a sim IO running its own WPILib {@code PIDController} against a physics
 * model — is simpler and was what the 2026 scaffolds did, but the numbers it produces are not the
 * robot's numbers, so a tuning session in it tells you very little.
 *
 * <h2>How a loop fits together</h2>
 *
 * <p>The mechanism owns the physics; this class owns the device. Each loop, before inputs are read:
 *
 * <ol>
 *   <li>{@link #updateSupplyVoltage()} — tell the device what the battery is doing
 *   <li>{@link #getSimVoltage()} — read what the closed loop decided to apply
 *   <li>step the physics model with that voltage
 *   <li>{@link #setMechanismState} — hand the resulting motion back to the device
 * </ol>
 *
 * <p>The mechanism sim subclasses do this for you. It is written out here because getting the order
 * wrong produces a loop that is off by one cycle, which looks like a mechanism with mysteriously
 * sluggish gains.
 *
 * <h2>Torque current in simulation</h2>
 *
 * <p>This class populates torque current, and {@code .claude/rules/02-hardware.md} names it as the
 * exception to the rule against doing so. That rule is about values taken from WPILib's simulation
 * classes, which model <i>total current draw</i> rather than the torque-producing component.
 * Phoenix's device simulation computes stator, supply and torque current from its own motor model
 * given the rotor state we hand it, so the number here is the same quantity it is on the robot —
 * modelled rather than measured, but not invented.
 *
 * <p>Treat the magnitudes as indicative. It says the loop is asking for roughly the right effort;
 * it does not say what the mechanism will draw with a game piece in it.
 */
public class MotorIOTalonFXSim extends MotorIOTalonFX {

  private final TalonFXSimState motorSim;
  private final TalonFXSimState[] followerSims;
  private final CANcoderSimState encoderSim;

  private final double rotorToMechanism;
  private final double sensorToMechanism;

  public MotorIOTalonFXSim(MotorConfig motorConfig) {
    super(motorConfig);

    motorSim = motor.getSimState();
    // Inversion is a property of how the motor is mounted, so the simulated device has to be told
    // about it separately from the config — otherwise a mechanism that is inverted on the robot
    // runs backwards in simulation and every sign in the physics model gets "fixed" to compensate.
    motorSim.Orientation =
        motorConfig.inverted()
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    followerSims = new TalonFXSimState[followers.length];
    for (int i = 0; i < followers.length; i++) {
      TalonFX follower = followers[i];
      followerSims[i] = follower.getSimState();
      followerSims[i].Orientation =
          motorConfig.followers().get(i).opposed() != motorConfig.inverted()
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
    }

    encoderSim = hasAbsoluteEncoder() ? encoder.getSimState() : null;
    if (encoderSim != null) {
      // Same reasoning as the motor's Orientation above, and the same caveat: CTRE's javadoc says
      // this field tracks the mechanical linkage rather than the invert config. This library feeds
      // its devices MECHANISM-frame positions out of the physics model, and a configured invert is
      // precisely what stands between a device's own frame and the mechanism's — so here the two
      // amount to the same thing, and compensating is what makes a simulated device report the
      // position the model says the mechanism is at. Leaving it default while the motor compensates
      // would make the two disagree in sign under FusedCANcoder the moment an encoder is configured
      // Clockwise_Positive, and the fusion would fight itself.
      encoderSim.Orientation =
          motorConfig.encoderDirection() == SensorDirectionValue.Clockwise_Positive
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;

      // CTRE's own hook for exactly this, documented as "allowing for a non-zero magnet offset
      // config to behave correctly in simulation": setRawPosition subtracts it before applying
      // Orientation. The simulated CANcoder carries the same calibrated MagnetOffset as the real
      // one, because MotorIOTalonFX applies the whole CANcoder config here too — so without this
      // the device adds the offset to every seeded position and the mechanism runs shifted by a
      // constant, with setpoints landing wrong and soft limits tripping early or late.
      encoderSim.SensorOffset = motorConfig.magnetOffsetRotations();
    }

    rotorToMechanism = motorConfig.rotorToMechanismRatio();
    sensorToMechanism = motorConfig.sensorToMechanismRatio();
  }

  /**
   * Tells every simulated device what the battery is doing.
   *
   * <p>Call this before reading {@link #getSimVoltage()}. A device with no supply voltage set
   * reports zero applied output regardless of what was commanded, which presents as a mechanism
   * that ignores every command.
   */
  public void updateSupplyVoltage() {
    double volts = RobotController.getBatteryVoltage();
    motorSim.setSupplyVoltage(volts);
    for (TalonFXSimState followerSim : followerSims) {
      followerSim.setSupplyVoltage(volts);
    }
    if (encoderSim != null) {
      encoderSim.setSupplyVoltage(volts);
    }
  }

  /**
   * What the closed loop is currently applying to the motor.
   *
   * <p>This is the input to the physics model — the whole reason the simulation is worth anything
   * is that this number came out of the real gains running on a real Motion Magic profile.
   */
  public Voltage getSimVoltage() {
    return motorSim.getMotorVoltageMeasure();
  }

  /**
   * Hands the physics model's output back to the simulated devices.
   *
   * <p>Both arguments are in <b>mechanism</b> units; the ratios from the config convert them to the
   * rotor and sensor units the devices expect. That conversion existing here rather than in each
   * mechanism is deliberate: it is the one piece of arithmetic that has to agree exactly with what
   * the device was configured with, and there is exactly one place it can disagree.
   *
   * @param position Mechanism position from the physics model
   * @param velocity Mechanism velocity from the physics model
   */
  public void setMechanismState(Angle position, AngularVelocity velocity) {
    motorSim.setRawRotorPosition(position.times(rotorToMechanism));
    motorSim.setRotorVelocity(velocity.times(rotorToMechanism));

    for (TalonFXSimState followerSim : followerSims) {
      followerSim.setRawRotorPosition(position.times(rotorToMechanism));
      followerSim.setRotorVelocity(velocity.times(rotorToMechanism));
    }

    if (encoderSim != null) {
      // The CANcoder sits at the sensor location, one reduction short of the mechanism. The magnet
      // offset and the sensor direction are handled by SensorOffset and Orientation, set once in
      // the constructor, so what goes in here is the physics model's position and nothing else.
      encoderSim.setRawPosition(position.times(sensorToMechanism));
      encoderSim.setVelocity(velocity.times(sensorToMechanism));
    }
  }
}
