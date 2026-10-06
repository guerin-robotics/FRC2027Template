package frc.lib.device.absoluteencoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.device.CanIdRegistry;
import frc.lib.util.PhoenixUtil;

/**
 * A CANcoder read on its own. Rates follow {@code .claude/rules/02-hardware.md}: position and
 * velocity at 50 Hz, magnet health at 10 Hz, the sticky fault at 4 Hz, then {@code
 * optimizeBusUtilization()} last.
 */
public class AbsoluteEncoderIOCANcoder implements AbsoluteEncoderIO {

  private final CANcoder encoder;
  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<MagnetHealthValue> magnetHealth;
  private final StatusSignal<Boolean> stickyBootDuringEnable;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * @param name Owner name for the CAN ID registry, e.g. "Turret encoder"
   * @param canId From {@code Constants.CanIds}
   * @param bus Usually {@code Constants.CanIds.RIO_BUS}
   * @param config The complete configuration — magnet offset, sensor direction, discontinuity point
   *     — applied once. Never apply a hand-built sub-config afterwards.
   */
  public AbsoluteEncoderIOCANcoder(
      String name, int canId, CANBus bus, CANcoderConfiguration config) {
    CanIdRegistry.claim(canId, bus, name);
    encoder = new CANcoder(canId, bus);
    PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(config));

    absolutePosition = encoder.getAbsolutePosition();
    velocity = encoder.getVelocity();
    magnetHealth = encoder.getMagnetHealth();
    stickyBootDuringEnable = encoder.getStickyFault_BootDuringEnable();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePosition, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, magnetHealth);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, stickyBootDuringEnable);
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AbsoluteEncoderInputs inputs) {
    BaseStatusSignal.refreshAll(magnetHealth, stickyBootDuringEnable);
    // Connection from the 50 Hz group alone; the slow signals would read stale for the first
    // quarter second and report a healthy encoder as missing.
    var status = BaseStatusSignal.refreshAll(absolutePosition, velocity);
    inputs.connected = connectedDebounce.calculate(status.isOK());

    inputs.absolutePosition = absolutePosition.getValue();
    inputs.velocity = velocity.getValue();
    inputs.magnetHealth = magnetHealth.getValue().value;
    inputs.stickyBootDuringEnable = stickyBootDuringEnable.getValue();
  }
}
