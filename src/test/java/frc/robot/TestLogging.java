package frc.robot;

import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public final class TestLogging {
  private static final Set<String> keys = new CopyOnWriteArraySet<>();
  private static boolean started;

  private TestLogging() {}

  public static synchronized void start() {
    if (started) {
      return;
    }
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.addDataReceiver(
        new LogDataReceiver() {
          @Override
          public void putTable(LogTable table) {
            keys.addAll(table.getAll(false).keySet());
          }
        });
    Logger.start();
    started = true;
  }

  static Set<String> keys() {
    return keys;
  }
}
