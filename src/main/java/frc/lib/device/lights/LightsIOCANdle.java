package frc.lib.device.lights;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.device.CanIdRegistry;
import frc.lib.util.PhoenixUtil;

/**
 * A CTRE CANdle. Animations run in slot 0; switching to a solid colour clears that slot first, or
 * the animation keeps drawing over it.
 *
 * <p>Control requests are built once and mutated rather than rebuilt per call; {@link Lights} only
 * calls in here when the request changes. Connection comes from {@code isConnected()} — the CANdle
 * publishes nothing this code needs at a fast rate, so there is no signal group to derive it from.
 */
public class LightsIOCANdle implements LightsIO {

  private static final int ANIMATION_SLOT = 0;

  private final CANdle candle;
  private final SolidColor solid;
  private final StrobeAnimation strobe;
  private final RainbowAnimation rainbow;
  private final EmptyAnimation clearAnimation = new EmptyAnimation(ANIMATION_SLOT);

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * @param name Owner name for the CAN ID registry, e.g. "Lights"
   * @param canId From {@code Constants.CanIds}
   * @param bus Usually {@code Constants.CanIds.RIO_BUS}
   * @param config The complete configuration (strip type, brightness), applied once
   * @param ledCount Total LEDs including the CANdle's 8 onboard ones — so a 30-LED strip is 38
   */
  public LightsIOCANdle(
      String name, int canId, CANBus bus, CANdleConfiguration config, int ledCount) {
    CanIdRegistry.claim(canId, bus, name);
    candle = new CANdle(canId, bus);
    PhoenixUtil.tryUntilOk(5, () -> candle.getConfigurator().apply(config));

    int last = ledCount - 1;
    solid = new SolidColor(0, last);
    strobe = new StrobeAnimation(0, last).withSlot(ANIMATION_SLOT);
    rainbow = new RainbowAnimation(0, last).withSlot(ANIMATION_SLOT);
  }

  @Override
  public void updateInputs(LightsInputs inputs) {
    inputs.connected = connectedDebounce.calculate(candle.isConnected());
  }

  @Override
  public void off() {
    candle.setControl(clearAnimation);
    candle.setControl(solid.withColor(new RGBWColor(0, 0, 0, 0)));
  }

  @Override
  public void setSolid(Color color) {
    candle.setControl(clearAnimation);
    candle.setControl(solid.withColor(toRgbw(color)));
  }

  @Override
  public void setStrobe(Color color) {
    candle.setControl(strobe.withColor(toRgbw(color)));
  }

  @Override
  public void setRainbow() {
    candle.setControl(rainbow);
  }

  private static RGBWColor toRgbw(Color color) {
    return new RGBWColor(
        (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0);
  }
}
