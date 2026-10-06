package frc.lib.device.absoluteencoder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/**
 * A standalone absolute encoder: one that is not the feedback sensor of a motor.
 *
 * <p><b>If the encoder belongs to a motor, use {@code MotorConfig.Builder.encoder(...)}
 * instead.</b> Fused into the TalonFX, the motor reads it off the bus and closes its loop on it;
 * read through this class, it is only a number the code can look at. Reach for this one for a
 * turret's second-gear encoder, a passive hinge, or anything with no motor of its own.
 *
 * <p>Three implementations, like every device here: {@code AbsoluteEncoderIOCANcoder} on the robot,
 * {@code AbsoluteEncoderIOSim} in simulation, and {@code new AbsoluteEncoderIO() {}} in replay.
 */
public interface AbsoluteEncoderIO {

  @AutoLog
  class AbsoluteEncoderInputs {
    public boolean connected = false;

    /** Absolute position after the magnet offset, in the range the config's discontinuity sets. */
    public Angle absolutePosition = Rotations.of(0);

    public AngularVelocity velocity = RotationsPerSecond.of(0);

    /**
     * {@code MagnetHealthValue.value}: 1 red, 2 orange, 3 green. A mounting property — a red magnet
     * reads plausible numbers that are wrong.
     */
    public int magnetHealth = 0;

    /** The encoder rebooted while the robot was enabled. Latches until cleared. */
    public boolean stickyBootDuringEnable = false;
  }

  default void updateInputs(AbsoluteEncoderInputs inputs) {}
}
