package frc.lib.mechanism;

import frc.lib.util.LoggedTunableNumber;

/**
 * One closed-loop gain set: the three PID terms and the four feedforward terms.
 *
 * <h2>These are amps</h2>
 *
 * <p>Every closed loop in this codebase runs a {@code *TorqueCurrentFOC} request, so every gain
 * here is in <b>amps</b> per unit of error, not volts. Reading them as volts makes correct values
 * look absurd — a steer {@code kP} in the thousands is normal, because it is amps per mechanism
 * rotation — and makes wrong ones look reasonable. Read {@code docs/characterization-and-tuning.md}
 * before changing one.
 *
 * <h2>What each term does</h2>
 *
 * <ul>
 *   <li>{@code kP} — amps per unit of error. The main lever.
 *   <li>{@code kI} — amps per unit of accumulated error. Usually zero; a mechanism that needs it
 *       usually has a friction or gravity problem that {@code kS} or {@code kG} should own.
 *   <li>{@code kD} — amps per unit of error rate. Damps overshoot.
 *   <li>{@code kS} — amps to overcome static friction, applied in the direction of travel.
 *   <li>{@code kV} — amps per unit of velocity. Under torque control this is small, because current
 *       commands torque rather than speed; it is not the same number a voltage loop wants.
 *   <li>{@code kA} — amps per unit of acceleration. This is the one that carries real weight under
 *       torque control, since current and torque are proportional.
 *   <li>{@code kG} — amps to hold against gravity. Constant for a lift, cosine-scaled for an arm;
 *       which of those applies is set by {@link MotorConfig.Builder#gravity}, not here, because it
 *       describes how the mechanism is built rather than how hard it is being pushed.
 * </ul>
 *
 * <p>The gravity type living in the config rather than in this record is deliberate. A gain set is
 * what gets edited live from a dashboard during a tuning session; the gravity model is not
 * something anyone should be able to flip mid-session by nudging a number.
 *
 * @param kP Proportional gain, amps per unit error
 * @param kI Integral gain, amps per unit accumulated error
 * @param kD Derivative gain, amps per unit error rate
 * @param kS Static friction feedforward, amps
 * @param kV Velocity feedforward, amps per unit velocity
 * @param kA Acceleration feedforward, amps per unit acceleration
 * @param kG Gravity feedforward, amps
 */
public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {

  /**
   * Every gain at zero.
   *
   * <p>A legitimate and honest starting point for a mechanism nobody has characterized yet. It is
   * offered as a named constant rather than left as a default so that {@link MotorConfig} can still
   * insist the call was made — "we have not tuned this" is a decision worth writing down, and it
   * reads very differently from "we forgot".
   */
  public static Gains zero() {
    return new Gains(0, 0, 0, 0, 0, 0, 0);
  }

  /** Proportional only. The usual first step of a bring-up. */
  public static Gains p(double kP) {
    return new Gains(kP, 0, 0, 0, 0, 0, 0);
  }

  /** PID with no feedforward. */
  public static Gains pid(double kP, double kI, double kD) {
    return new Gains(kP, kI, kD, 0, 0, 0, 0);
  }

  /** Reads the current value out of a bank of tunables. Order matches {@link #tunables}. */
  static Gains fromTunables(LoggedTunableNumber[] tunables) {
    return new Gains(
        tunables[0].get(),
        tunables[1].get(),
        tunables[2].get(),
        tunables[3].get(),
        tunables[4].get(),
        tunables[5].get(),
        tunables[6].get());
  }

  /**
   * Builds the dashboard-editable copy of this gain set, under {@code Tuning/<name>/}.
   *
   * <p>Inert unless {@code Constants.tuningMode} is true — {@link LoggedTunableNumber} returns the
   * compiled-in default when tuning is off, and never creates a NetworkTables entry. So a mechanism
   * always constructs these and a competition robot pays nothing for them.
   *
   * <p>Every gain is listed. A gain that is tunable but missing from the bank the mechanism watches
   * would move on the dashboard and never reach the motor, which reads to whoever is tuning as "the
   * tunable is broken" rather than "the gain does nothing".
   */
  LoggedTunableNumber[] tunables(String name) {
    return new LoggedTunableNumber[] {
      new LoggedTunableNumber(name + "/Gains/kP", kP),
      new LoggedTunableNumber(name + "/Gains/kI", kI),
      new LoggedTunableNumber(name + "/Gains/kD", kD),
      new LoggedTunableNumber(name + "/Gains/kS", kS),
      new LoggedTunableNumber(name + "/Gains/kV", kV),
      new LoggedTunableNumber(name + "/Gains/kA", kA),
      new LoggedTunableNumber(name + "/Gains/kG", kG),
    };
  }
}
