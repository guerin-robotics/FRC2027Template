// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class PhoenixUtil {
  /**
   * Attempts to run the command until no error is produced, at most {@code maxAttempts} times.
   *
   * <p>Returns whether it succeeded, and reports to the Driver Station when it did not. It used to
   * give up silently, which let a caller carry on as though a write had landed — a zero that never
   * reached the Talon, or a config that left the motor at its defaults. A caller whose correctness
   * depends on the write (zeroing, anything that sets state the code then trusts) must check the
   * result; NASA/JPL Power of 10 rule 7, see docs/review-checklist.md.
   *
   * @return true if the command returned OK within {@code maxAttempts}
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    StatusCode last = StatusCode.OK;
    for (int i = 0; i < maxAttempts; i++) {
      last = command.get();
      if (last.isOK()) {
        return true;
      }
    }
    DriverStation.reportError(
        "Phoenix call failed after " + maxAttempts + " attempts: " + last.getName(), true);
    return false;
  }
}
