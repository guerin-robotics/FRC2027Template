package frc.lib.device;

import com.ctre.phoenix6.CANBus;
import java.util.HashMap;
import java.util.Map;

/**
 * Every CAN device claims its ID here when its IO is constructed, so two devices on one ID stop the
 * robot at boot instead of half-working.
 *
 * <p>A duplicate CAN ID has no symptom of its own: nothing errors, one device wins arbitration and
 * the other is never heard from, and the mechanism looks merely weak. {@code CanIdUniquenessTest}
 * catches collisions among the constants at build time; this catches the ones the test cannot see —
 * an ID typed straight into a constructor, or two mechanisms built from the same constant.
 *
 * <p><b>Who claims.</b> Every IO that constructs a CTRE or Grapple device: {@code MotorIOTalonFX}
 * (and so its Phoenix sim subclass, because simulated devices on one ID collide too) and each real
 * IO under {@code frc/lib/device}. Replay and pure-Java sim IOs construct no device and claim
 * nothing. Swerve devices are not claimed here: they live in generated {@code TunerConstants} and
 * {@code CanIdUniquenessTest} already cross-checks them against {@code Constants.CanIds}.
 */
public final class CanIdRegistry {

  /** Highest ID a CTRE device accepts; the same bound {@code CanIdUniquenessTest} checks. */
  public static final int MAX_ID = 62;

  /** Bus name, then ID, then who claimed it. */
  private static final Map<String, Map<Integer, String>> claims = new HashMap<>();

  private CanIdRegistry() {}

  /**
   * Claims {@code id} on {@code bus} for {@code owner}.
   *
   * @throws IllegalArgumentException if the ID is outside 0–62
   * @throws IllegalStateException if another device already claimed the ID on this bus — the
   *     message names both, so the fix is one look at {@code Constants.CanIds}
   */
  public static synchronized void claim(int id, CANBus bus, String owner) {
    claim(id, bus.getName(), owner);
  }

  /** As {@link #claim(int, CANBus, String)}, for devices that only know a bus name. */
  public static synchronized void claim(int id, String busName, String owner) {
    if (id < 0 || id > MAX_ID) {
      throw new IllegalArgumentException(
          owner + " was given CAN ID " + id + " on bus '" + busName + "'; IDs run 0–" + MAX_ID);
    }
    Map<Integer, String> onBus = claims.computeIfAbsent(busName, b -> new HashMap<>());
    String existing = onBus.putIfAbsent(id, owner);
    if (existing != null) {
      throw new IllegalStateException(
          "CAN ID "
              + id
              + " on bus '"
              + busName
              + "' is claimed by both "
              + existing
              + " and "
              + owner
              + ". Two devices on one ID half-work silently; fix Constants.CanIds.");
    }
  }

  /**
   * Forgets every claim. Tests only: a test class that builds the same mechanism twice in one JVM
   * would otherwise collide with itself.
   */
  public static synchronized void clearForTesting() {
    claims.clear();
  }
}
