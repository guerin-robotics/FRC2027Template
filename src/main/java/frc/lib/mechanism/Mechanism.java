package frc.lib.mechanism;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.FaultMonitor;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * What every motor-driven mechanism does the same way.
 *
 * <p>Reading inputs, logging them, reporting supply current for brownout analysis, pushing
 * dashboard-edited gains to the device, and answering "is the hardware there" — none of that varies
 * between a flywheel and an elevator, and through 2026 every subsystem carried its own slightly
 * different copy of all of it.
 *
 * <p>The three subclasses add only what genuinely differs: what a "goal" means, what counts as
 * being there, and what the mechanism looks like when drawn.
 *
 * <h2>This is not a Subsystem</h2>
 *
 * <p>Deliberately. A {@code SubsystemBase} is a scheduling unit — the thing a command requires —
 * and a mechanism is a piece of hardware. Most of the time those are one-to-one, and {@code
 * RollerSubsystem} and friends exist so that the common case costs nothing. But a subsystem that
 * owns two mechanisms is a real shape, and if this class were a {@code Subsystem} that shape would
 * have to be expressed as two subsystems that must never be required separately.
 *
 * <h2>Ownership</h2>
 *
 * <p>A mechanism never holds a reference to another subsystem or mechanism. Shared state goes
 * through {@code RobotState}, or through a supplier passed in at construction. See {@code
 * .claude/rules/01-architecture.md}.
 */
public abstract class Mechanism {

  /**
   * Fraction of the stator limit at which the mechanism counts as saturated.
   *
   * <p>Not 1.0: current limiting is a control loop of its own and rides just under the ceiling
   * rather than pinning to it, so an exact comparison reports saturation far less often than it
   * happens.
   */
  private static final double STATOR_SATURATION_FRACTION = 0.95;

  protected final String name;
  protected final MotorConfig config;
  protected final MotorIO io;

  protected final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();
  private final FollowerInputsAutoLogged[] followerInputs;
  private final EncoderInputsAutoLogged encoderInputs;

  private final LoggedTunableNumber[] gainTunables;
  private final LoggedTunableNumber[] profileTunables;
  private final LoggedTunableNumber[] allTunables;

  /**
   * Scratch buffer for the per-loop battery report.
   *
   * <p>Allocated once. {@code reportCurrentUsage} takes a varargs, and calling it with a fresh
   * array every loop is a per-cycle allocation inside the 20 ms budget — which is exactly the kind
   * of garbage the GC notes in {@code .claude/rules/04-build.md} say to avoid, since pause
   * behaviour here comes from not allocating rather than from any collector flag.
   */
  private final double[] supplyAmpsBuffer;

  protected Mechanism(MotorConfig config, MotorIO io) {
    this.config = config;
    this.io = io;
    this.name = config.name();

    this.followerInputs = new FollowerInputsAutoLogged[io.followerCount()];
    for (int i = 0; i < followerInputs.length; i++) {
      followerInputs[i] = new FollowerInputsAutoLogged();
    }
    this.encoderInputs = io.hasAbsoluteEncoder() ? new EncoderInputsAutoLogged() : null;

    this.supplyAmpsBuffer = new double[1 + followerInputs.length];

    // Always constructed, never conditional. LoggedTunableNumber returns the compiled-in default
    // and creates no NetworkTables entry unless Constants.tuningMode is true, so a competition
    // robot pays nothing for these and there is no second code path to get wrong.
    this.gainTunables = config.gains().tunables(name);
    this.profileTunables = config.profile().tunables(name);
    this.allTunables = new LoggedTunableNumber[gainTunables.length + profileTunables.length];
    System.arraycopy(gainTunables, 0, allTunables, 0, gainTunables.length);
    System.arraycopy(profileTunables, 0, allTunables, gainTunables.length, profileTunables.length);
  }

  // ============================================================================================
  // Loop
  // ============================================================================================

  /**
   * Call once per loop, from the owning subsystem's {@code periodic()}.
   *
   * <p>Simulation runs first, then inputs, then logging. That order is not arbitrary: the physics
   * model has to have advanced before the device is asked what it sees, or every simulated
   * mechanism runs a cycle behind and looks like it has sluggish gains.
   */
  public void periodic() {
    simulationPeriodic();

    io.updateInputs(inputs);
    Logger.processInputs(name, inputs); // NEVER remove this line — it is what makes replay work

    for (int i = 0; i < followerInputs.length; i++) {
      io.updateFollowerInputs(i, followerInputs[i]);
      Logger.processInputs(name + "/Follower" + i, followerInputs[i]);
    }

    if (encoderInputs != null) {
      io.updateEncoderInputs(encoderInputs);
      Logger.processInputs(name + "/Encoder", encoderInputs);
    }

    reportBatteryUsage();
    pushTunables();

    Logger.recordOutput(name + "/AtStatorLimit", isAtStatorLimit());
  }

  /**
   * Overridden by the simulated variants to advance their physics model. No-op on real hardware.
   */
  protected void simulationPeriodic() {}

  /**
   * Reports what this mechanism drew from the battery.
   *
   * <p><b>Supply current, not stator.</b> This feeds a power calculation, and at low speed under
   * load supply is a fraction of stator because the controller is chopping. The 2026 drivetrain
   * passed stator here and every power and energy figure from that season is inflated.
   *
   * <p>Followers are included. A two-motor mechanism draws two motors' worth.
   */
  private void reportBatteryUsage() {
    supplyAmpsBuffer[0] = inputs.supplyAmps.in(Amps);
    for (int i = 0; i < followerInputs.length; i++) {
      supplyAmpsBuffer[i + 1] = followerInputs[i].supplyAmps.in(Amps);
    }
    Robot.batteryLogger.reportCurrentUsage(name, false, supplyAmpsBuffer);
  }

  /**
   * Pushes dashboard-edited gains and profile values to the device, but only when one moved.
   *
   * <p>Both live in flash on the motor controller, not in RAM on the roboRIO — that is why the
   * Talon closes its loop at 1 kHz instead of at our 50 Hz. The cost is that changing one is a
   * blocking CAN transaction rather than a field write, so it cannot be done unconditionally every
   * loop the way a WPILib {@code PIDController} gain can. Expect the one loop this fires on to
   * overrun the 20 ms budget; that is acceptable in a tuning session, which is the only time it
   * happens.
   *
   * <p>Gains and profile are pushed together because they are watched together. A mechanism that
   * will not reach its setpoint might have weak gains or a profile too slow to ask for the motion,
   * and telling those apart requires moving both.
   *
   * <p>{@code hasChanged} reports true on its first call, so this fires once on the first loop and
   * re-applies what the constructor already wrote. Harmless, but it means the first change visible
   * in a log is not a change.
   */
  private void pushTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          io.setGains(Gains.fromTunables(gainTunables));
          io.setMotionProfile(MotionProfile.fromTunables(profileTunables));
        },
        allTunables);
  }

  /**
   * Registers this mechanism's hardware with {@link FaultMonitor}, so a device that drops off the
   * bus reaches the pit between matches instead of only the log.
   *
   * <p>Call it from {@code RobotContainer} once, at wiring time. Each device is registered
   * separately, deliberately: a CANcoder can fail while its motor stays healthy, and a follower can
   * die while its leader looks merely underpowered. Merging them into one boolean hides exactly the
   * failures worth catching.
   */
  public void registerFaultMonitors() {
    FaultMonitor monitor = FaultMonitor.getInstance();
    monitor.register(name + " disconnected", () -> !inputs.connected);
    monitor.register(name + " rebooted while enabled", () -> inputs.stickyBootDuringEnable);
    monitor.register(name + " over temperature", () -> inputs.stickyOverTemp);
    monitor.register(name + " hardware fault", () -> inputs.stickyHardwareFault);

    for (int i = 0; i < followerInputs.length; i++) {
      final int index = i;
      monitor.register(
          name + " follower " + i + " disconnected", () -> !followerInputs[index].connected);
    }

    if (encoderInputs != null) {
      monitor.register(name + " encoder disconnected", () -> !encoderInputs.connected);
    }
  }

  // ============================================================================================
  // Control passthroughs
  // ============================================================================================

  /** Open-loop voltage. Bring-up and characterization; match logic should close a loop. */
  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  /** Open-loop torque current. The honest open-loop mode under FOC. */
  public void setTorqueCurrent(Current amps) {
    io.setTorqueCurrent(amps);
  }

  /** Stop commanding output. Neutral behaviour follows the configured neutral mode. */
  public void stop() {
    io.stop();
  }

  /** Coast for pit handling, brake for everything else. */
  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }

  /** Tells the motor to read its current physical position as {@code position}. */
  public void setEncoderPosition(Angle position) {
    io.setEncoderPosition(position);
  }

  // ============================================================================================
  // State
  // ============================================================================================

  public String getName() {
    return name;
  }

  /** Mechanism position, in mechanism rotations. Not rotor position. */
  public Angle getPosition() {
    return inputs.position;
  }

  /** Mechanism velocity. */
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  /** Current through the windings. Heating and stall. */
  public Current getStatorCurrent() {
    return inputs.statorAmps;
  }

  /** Current drawn from the battery. Brownout math. */
  public Current getSupplyCurrent() {
    return inputs.supplyAmps;
  }

  /** Torque-producing current — the control signal under a {@code *TorqueCurrentFOC} request. */
  public Current getTorqueCurrent() {
    return inputs.torqueAmps;
  }

  public Voltage getAppliedVolts() {
    return inputs.appliedVolts;
  }

  /** What the closed loop asked for this instant, in mechanism rotations or rotations/sec. */
  public double getClosedLoopReference() {
    return inputs.closedLoopReference;
  }

  /** Is the leader answering on the bus? */
  public boolean isLeaderConnected() {
    return inputs.connected;
  }

  /** Is follower {@code index} answering? */
  public boolean isFollowerConnected(int index) {
    return followerInputs[index].connected;
  }

  /** Is the absolute encoder answering? False when there is no encoder to answer. */
  public boolean isEncoderConnected() {
    return encoderInputs != null && encoderInputs.connected;
  }

  /**
   * Is every device on this mechanism present?
   *
   * <p>The convenience roll-up. Prefer the individual checks when deciding what to do about a
   * failure — they say which device is missing, and that is the difference between a pit crew
   * checking one connector and checking all of them.
   */
  public boolean isConnected() {
    if (!inputs.connected) {
      return false;
    }
    for (FollowerInputsAutoLogged follower : followerInputs) {
      if (!follower.connected) {
        return false;
      }
    }
    return encoderInputs == null || encoderInputs.connected;
  }

  /**
   * True while stator current is sitting at the configured ceiling.
   *
   * <p>This is how you tell "the gains are wrong" apart from "the current limit is the constraint",
   * and those need opposite fixes. Read it beside the closed-loop reference: reference tracking the
   * goal with the measurement lagging and this flag true is saturation; the same lag with this flag
   * false is a gain problem.
   */
  public boolean isAtStatorLimit() {
    return inputs.statorAmps.in(Amps)
        > config.statorCurrentLimitAmps() * STATOR_SATURATION_FRACTION;
  }
}
