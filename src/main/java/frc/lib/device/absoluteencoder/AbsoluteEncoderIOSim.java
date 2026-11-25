package frc.lib.device.absoluteencoder;

import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.function.Supplier;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Reports whatever the simulation says the measured joint is doing. Pass a supplier from the
 * physics model that owns the joint; with none, it reads zero.
 */
public class AbsoluteEncoderIOSim implements AbsoluteEncoderIO {

  private final Supplier<Angle> position;
  private final Supplier<AngularVelocity> velocity;

  public AbsoluteEncoderIOSim(Supplier<Angle> position, Supplier<AngularVelocity> velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  public AbsoluteEncoderIOSim(Supplier<Angle> position) {
    this(position, () -> RotationsPerSecond.of(0));
  }

  @Override
  public void updateInputs(AbsoluteEncoderInputs inputs) {
    inputs.connected = true;
    inputs.absolutePosition = position.get();
    inputs.velocity = velocity.get();
    inputs.magnetHealth = 3; // green
    inputs.stickyBootDuringEnable = false;
  }
}
