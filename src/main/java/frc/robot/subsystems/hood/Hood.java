package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import frc.lib.mechanism.rotary.RotaryMechanism;
import frc.lib.mechanism.rotary.RotarySubsystem;

/**
 * The 2026 shooter hood, ported from Rebuilt2026 {@code Hood}.
 *
 * <p>Every commanded angle goes through {@code RotaryMechanism.setPosition}, which clamps it to the
 * 0–62° soft limits and logs {@code Hood/GoalWasClamped} when it had to. 2026 sent the raw angle to
 * the device and let the device's own soft limits stop it; the motion is the same, the log is new.
 */
public class Hood extends RotarySubsystem {

  public Hood(RotaryMechanism mechanism) {
    super(mechanism);
  }

  /** Five degrees above wherever the hood is now. Unbound in 2026. */
  public void incrementHoodPos() {
    setPosition(getPosition().plus(Degrees.of(5)));
  }

  public void setHoodPosForHub() {
    setPosition(HoodPosCalculator.getInstance().getHoodPosForHub());
  }

  public void setHoodPosForPass() {
    setPosition(HoodPosCalculator.getInstance().getHoodPosForPassing());
  }

  public void stowHood() {
    setPosition(Degrees.of(0));
  }
}
