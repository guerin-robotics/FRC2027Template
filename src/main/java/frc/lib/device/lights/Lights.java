package frc.lib.device.lights;

import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.FaultMonitor;
import org.littletonrobotics.junction.Logger;

/**
 * An LED strip, owned by a subsystem.
 *
 * <p>Requests are forwarded only when they change, so a command may set the same colour every loop
 * without flooding the CAN bus, and resent when the controller reconnects. What was asked for is
 * logged as {@code <name>/Mode} and {@code <name>/Color} — an output, so replay shows what the code
 * requested even though the strip itself was never observable.
 */
public class Lights {

  private enum Mode {
    OFF,
    SOLID,
    STROBE,
    RAINBOW
  }

  private final String name;
  private final LightsIO io;
  private final LightsInputsAutoLogged inputs = new LightsInputsAutoLogged();

  private Mode mode = null;
  private Color color = Color.kBlack;
  private boolean wasConnected = false;

  /**
   * @param name Log key and fault prefix, e.g. "Lights"
   */
  public Lights(String name, LightsIO io) {
    this.name = name;
    this.io = io;
  }

  /** Call once per loop from the owning subsystem's {@code periodic()}. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs); // NEVER remove — it is what makes replay work

    // A CANdle that rebooted or came up late has forgotten what it was showing. Requests are only
    // sent on change, so resend the current one when the device (re)appears.
    if (inputs.connected && !wasConnected && mode != null) {
      send();
    }
    wasConnected = inputs.connected;

    Logger.recordOutput(name + "/Mode", mode == null ? "NONE" : mode.name());
    Logger.recordOutput(name + "/Color", color.toHexString());
  }

  /** Call once from {@code RobotContainer} at wiring time. */
  public void registerFaultMonitors() {
    FaultMonitor.getInstance().register(name + " disconnected", () -> !inputs.connected);
  }

  public void off() {
    request(Mode.OFF, Color.kBlack);
  }

  public void setSolid(Color requested) {
    request(Mode.SOLID, requested);
  }

  public void setStrobe(Color requested) {
    request(Mode.STROBE, requested);
  }

  public void setRainbow() {
    request(Mode.RAINBOW, Color.kBlack);
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public String getName() {
    return name;
  }

  private void request(Mode newMode, Color newColor) {
    if (newMode == mode && newColor.equals(color)) {
      return;
    }
    mode = newMode;
    color = newColor;
    send();
  }

  private void send() {
    switch (mode) {
      case OFF -> io.off();
      case SOLID -> io.setSolid(color);
      case STROBE -> io.setStrobe(color);
      case RAINBOW -> io.setRainbow();
    }
  }
}
