package frc.lib.device.distance;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import frc.lib.device.CanIdRegistry;
import frc.lib.util.PhoenixUtil;

/**
 * A CTRE CANrange. Distance and detection at 50 Hz, ambient at 10 Hz, the sticky fault at 4 Hz,
 * then {@code optimizeBusUtilization()} last — per {@code .claude/rules/02-hardware.md}.
 *
 * <p>{@code valid} is the CANrange's own detection flag, which applies the proximity threshold and
 * hysteresis in {@code config.ProximityParams}. Set those in the config, not in code that reads the
 * distance.
 */
public class DistanceSensorIOCANrange implements DistanceSensorIO {

  private final CANrange sensor;
  private final StatusSignal<Distance> distance;
  private final StatusSignal<Boolean> detected;
  private final StatusSignal<Double> ambient;
  private final StatusSignal<Boolean> stickyBootDuringEnable;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * @param name Owner name for the CAN ID registry, e.g. "Indexer sensor"
   * @param canId From {@code Constants.CanIds}
   * @param bus Usually {@code Constants.CanIds.RIO_BUS}
   * @param config The complete configuration, applied once
   */
  public DistanceSensorIOCANrange(
      String name, int canId, CANBus bus, CANrangeConfiguration config) {
    CanIdRegistry.claim(canId, bus, name);
    sensor = new CANrange(canId, bus);
    PhoenixUtil.tryUntilOk(5, () -> sensor.getConfigurator().apply(config));

    distance = sensor.getDistance();
    detected = sensor.getIsDetected();
    ambient = sensor.getAmbientSignal();
    stickyBootDuringEnable = sensor.getStickyFault_BootDuringEnable();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, distance, detected);
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, ambient);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, stickyBootDuringEnable);
    sensor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DistanceSensorInputs inputs) {
    BaseStatusSignal.refreshAll(ambient, stickyBootDuringEnable);
    var status = BaseStatusSignal.refreshAll(distance, detected);
    inputs.connected = connectedDebounce.calculate(status.isOK());

    inputs.distance = distance.getValue();
    inputs.valid = inputs.connected && detected.getValue();
    inputs.ambientSignal = ambient.getValue();
    inputs.stickyBootDuringEnable = stickyBootDuringEnable.getValue();
  }
}
