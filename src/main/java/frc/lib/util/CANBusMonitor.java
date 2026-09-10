package frc.lib.util;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/**
 * Logs CAN bus health so bandwidth problems are visible in a match log instead of only in Tuner X
 * while someone happens to be looking.
 *
 * <p><b>Why this exists.</b> Every status-signal frequency decision — the 250 Hz odometry signals,
 * the 50 Hz current signals, adding torque current — spends bus bandwidth, and the 2026 codebase
 * had no way to see the result after the fact. Advice like "confirm against bus utilization in
 * Tuner X" only works if someone remembers to do it on the right day. This records it continuously.
 *
 * <p>A saturated bus shows up as dropped or stale signals, which look like sensor faults or
 * mysterious control lag rather than a bandwidth problem.
 *
 * <p>Publishes under {@code CANBus/<name>/}: utilization fraction, receive and transmit error
 * counts, bus-off and TX-full counts, and a connected flag.
 *
 * <p>{@code getStatus()} costs a real query, so this samples at a low rate rather than every loop.
 */
public class CANBusMonitor {

  /** Sample every N loops. 25 loops ≈ 0.5 s at 20 ms. */
  private static final int SAMPLE_PERIOD_LOOPS = 25;

  /** Utilization above this is worth investigating before it becomes dropped frames. */
  private static final double HIGH_UTILIZATION = 0.80;

  private final CANBus bus;
  private final String name;
  private final Alert highUtilizationAlert;
  private final Alert busErrorAlert;

  private int loopCounter = 0;

  /**
   * @param bus The bus to watch, e.g. {@code TunerConstants.kCANBus}
   * @param name Label used in the log key, e.g. {@code "Canivore"}
   */
  public CANBusMonitor(CANBus bus, String name) {
    this.bus = bus;
    this.name = name;
    this.highUtilizationAlert =
        new Alert(
            "CAN bus \"" + name + "\" utilization is high. Reduce signal frequencies.",
            AlertType.kWarning);
    this.busErrorAlert =
        new Alert(
            "CAN bus \"" + name + "\" is reporting errors. Check wiring and termination.",
            AlertType.kError);
  }

  /** Call once per loop from {@code robotPeriodic()}. Samples internally at a reduced rate. */
  public void periodic() {
    if (loopCounter++ % SAMPLE_PERIOD_LOOPS != 0) {
      return;
    }

    var status = bus.getStatus();
    String key = "CANBus/" + name + "/";

    boolean ok = status.Status.isOK();
    Logger.recordOutput(key + "Connected", ok);
    Logger.recordOutput(key + "Utilization", status.BusUtilization);
    Logger.recordOutput(key + "ReceiveErrorCount", status.REC);
    Logger.recordOutput(key + "TransmitErrorCount", status.TEC);
    Logger.recordOutput(key + "BusOffCount", status.BusOffCount);
    Logger.recordOutput(key + "TxFullCount", status.TxFullCount);

    highUtilizationAlert.set(ok && status.BusUtilization > HIGH_UTILIZATION);
    busErrorAlert.set(!ok || status.BusOffCount > 0);
  }
}
