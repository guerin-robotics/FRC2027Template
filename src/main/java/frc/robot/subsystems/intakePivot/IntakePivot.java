package frc.robot.subsystems.intakePivot;

import frc.lib.mechanism.rotary.RotaryMechanism;
import frc.lib.mechanism.rotary.RotarySubsystem;

/**
 * The 2026 intake pivot, ported from Rebuilt2026 {@code IntakePivot}.
 *
 * <p>Two 2026 methods did not come across, both unbound anywhere in 2026: {@code setPivotVelocity}
 * (the rotary library has no velocity mode) and {@code zeroPivotEncoder}, which reset the
 * CANcoder's relative position while 2026 logged its absolute one, so it never changed what the log
 * showed.
 */
public class IntakePivot extends RotarySubsystem {

  public IntakePivot(RotaryMechanism mechanism) {
    super(mechanism);
  }
}
