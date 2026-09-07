package frc.lib.mechanism;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Everything a mechanism's motor group needs to know about itself, stated once.
 *
 * <p>This is the file a new mechanism actually writes. It replaces the hand-built {@code
 * TalonFXConfiguration} that every 2026 subsystem carried a private copy of — five hundred lines
 * per mechanism, most of it identical, and the parts that differed differing by accident as often
 * as by decision.
 *
 * <h2>What the builder defaults, and what it refuses to</h2>
 *
 * <p>Anything with a defensible house answer is defaulted: brake mode, no inversion, ±12 V output
 * caps, torque-current clamps derived from the stator limit so the two cannot disagree, the gravity
 * model and static-feedforward sign implied by the kind of mechanism.
 *
 * <p>Six things have no defensible default, and {@link Builder#build()} throws naming every one
 * that is missing rather than filling it in:
 *
 * <pre>
 * canId                    which motor this is
 * sensorToMechanismRatio   what a "rotation" means for this mechanism
 * supplyCurrentLimit       how much it may draw from the battery
 * statorCurrentLimit       how much may go through the windings
 * gains                    how it closes its loop, even if that is Gains.zero()
 * motionProfile            how fast it may move
 * </pre>
 *
 * <p>Plus soft limits, on {@link MechanismKind#ROTARY} and {@link MechanismKind#LINEAR}, because a
 * position mechanism with no travel bound does not fail — it drives into its own hard stop at
 * whatever the current limit allows and holds there.
 *
 * <p>Defaulting the current limits was considered and rejected. A limit nobody chose is how the
 * 2026 robot logged hundreds of brownouts across a single event; the point of this class is that
 * every mechanism's limits were a decision somebody made, visible on one line.
 *
 * <h2>One config, applied once</h2>
 *
 * <p>{@link #toTalonFXConfiguration()} returns a complete config. That matters more than it looks:
 * {@code apply()} on a sub-config writes the <b>entire sub-group</b>, so a second apply of a
 * hand-built block silently resets every field the first one set. The 2026 intake pivot shipped
 * three applies, the last a bare {@code FeedbackConfigs}, which reset the ratios it had just
 * configured; it survived only because that mechanism used {@code RemoteCANcoder}, where the ratio
 * is not in the position path. See {@code .claude/rules/02-hardware.md}.
 */
public final class MotorConfig {

  /**
   * What kind of mechanism this motor drives.
   *
   * <p>Three things follow from it that would otherwise be three more required calls: whether soft
   * limits are mandatory, which gravity model {@code kG} means, and whether {@code kS} is applied
   * in the direction of travel or in the direction of the error. The mechanism classes also check
   * it, so a rotary config handed to a roller is a loud throw at construction rather than a
   * mechanism that behaves oddly on the field.
   */
  public enum MechanismKind {
    /**
     * Spins. The question is how fast, never where. Roller, flywheel, feeder, transport, intake.
     */
    ROLLER,
    /** Pivots. Arm, hood, turret, wrist. Travel is bounded and gravity varies with angle. */
    ROTARY,
    /** Travels in a line. Elevator, lift, extension. Travel is bounded and gravity is constant. */
    LINEAR
  }

  /** Where the mechanism's position actually comes from. */
  public enum Feedback {
    /**
     * The motor's own rotor. No absolute reference — position reads zero at boot wherever the
     * mechanism happens to be, so anything that must know where it is needs a zeroing routine.
     */
    INTERNAL,
    /**
     * A CANcoder, fused with the rotor. Absolute across power cycles <i>and</i> continuous across
     * multiple turns, because the rotor extends the absolute reading. This is what a multi-turn
     * position mechanism wants.
     *
     * <p>{@code rotorToSensorRatio} is load-bearing here: it is how the rotor extends the encoder,
     * so getting it wrong puts the fusion off by the whole reduction while every configuration call
     * still reports success.
     */
    FUSED_CANCODER,
    /**
     * A CANcoder read directly. Absolute, but wraps to zero past one encoder rotation, so it suits
     * a mechanism whose travel fits inside a single turn of the encoder.
     */
    REMOTE_CANCODER,
    /**
     * A CANcoder that seeds the rotor once at boot and is then ignored. Survives an encoder that
     * drops off the bus mid-match, at the cost of not correcting drift.
     */
    SYNC_CANCODER
  }

  /** The gravity model {@code kG} is interpreted under. */
  public enum Gravity {
    /**
     * No gravity load worth compensating. Maps to Phoenix's {@code Elevator_Static} with a {@code
     * kG} that should be zero — Phoenix has no "none", and constant-times-zero is the honest way to
     * say it.
     */
    NONE,
    /** {@code kG} scaled by the cosine of the angle. Arms, hoods, wrists. */
    ARM_COSINE,
    /** {@code kG} applied constantly. Elevators, lifts. */
    ELEVATOR_STATIC
  }

  /** A follower motor: its CAN ID, and which way it faces relative to the leader. */
  public record Follower(int canId, boolean opposed) {}

  /** Travel bounds, in mechanism rotations. */
  public record SoftLimits(double reverseRotations, double forwardRotations) {
    public SoftLimits {
      if (reverseRotations >= forwardRotations) {
        throw new IllegalArgumentException(
            "Reverse soft limit ("
                + reverseRotations
                + ") must be less than forward ("
                + forwardRotations
                + "). A swapped pair leaves no legal travel at all.");
      }
    }
  }

  // ---- Identity ----
  private final String name;
  private final MechanismKind kind;
  private final int canId;
  private final CANBus bus;
  private final List<Follower> followers;

  // ---- Feedback ----
  private final Feedback feedback;
  private final int encoderCanId;
  private final double magnetOffsetRotations;
  private final double sensorDiscontinuityPoint;
  private final SensorDirectionValue encoderDirection;
  private final double rotorToSensorRatio;
  private final double sensorToMechanismRatio;

  // ---- Limits ----
  private final double supplyCurrentLimitAmps;
  private final double supplyCurrentLowerLimitAmps;
  private final Time supplyCurrentLowerTime;
  private final double statorCurrentLimitAmps;
  private final double peakForwardTorqueAmps;
  private final double peakReverseTorqueAmps;
  private final double peakForwardVoltage;
  private final double peakReverseVoltage;
  private final Optional<SoftLimits> softLimits;

  // ---- Output ----
  private final boolean inverted;
  private final NeutralModeValue neutralMode;
  private final boolean continuousWrap;

  // ---- Control ----
  private final Gains gains;
  private final MotionProfile profile;
  private final Gravity gravity;
  private final StaticFeedforwardSignValue staticFeedforwardSign;

  // ---- Logging ----
  private final double followerSignalHz;

  private MotorConfig(Builder b) {
    this.name = b.name;
    this.kind = b.kind;
    this.canId = b.canId;
    this.bus = b.bus;
    this.followers = List.copyOf(b.followers);
    this.feedback = b.feedback;
    this.encoderCanId = b.encoderCanId;
    this.magnetOffsetRotations = b.magnetOffsetRotations;
    this.sensorDiscontinuityPoint = b.sensorDiscontinuityPoint;
    this.encoderDirection = b.encoderDirection;
    this.rotorToSensorRatio = b.rotorToSensorRatio;
    this.sensorToMechanismRatio = b.sensorToMechanismRatio;
    this.supplyCurrentLimitAmps = b.supplyCurrentLimitAmps;
    this.supplyCurrentLowerLimitAmps =
        Double.isNaN(b.supplyCurrentLowerLimitAmps)
            ? b.supplyCurrentLimitAmps
            : b.supplyCurrentLowerLimitAmps;
    this.supplyCurrentLowerTime = b.supplyCurrentLowerTime;
    this.statorCurrentLimitAmps = b.statorCurrentLimitAmps;
    // Derived from the stator limit unless overridden, so the closed loop cannot be allowed to
    // request current the windings are not allowed to carry.
    this.peakForwardTorqueAmps =
        Double.isNaN(b.peakForwardTorqueAmps) ? b.statorCurrentLimitAmps : b.peakForwardTorqueAmps;
    this.peakReverseTorqueAmps =
        Double.isNaN(b.peakReverseTorqueAmps) ? -b.statorCurrentLimitAmps : b.peakReverseTorqueAmps;
    this.peakForwardVoltage = b.peakForwardVoltage;
    this.peakReverseVoltage = b.peakReverseVoltage;
    this.softLimits = Optional.ofNullable(b.softLimits);
    this.inverted = b.inverted;
    this.neutralMode = b.neutralMode;
    this.continuousWrap = b.continuousWrap;
    this.gains = b.gains;
    this.profile = b.profile;
    this.gravity = b.gravity == null ? defaultGravity(b.kind) : b.gravity;
    this.staticFeedforwardSign =
        b.staticFeedforwardSign == null ? defaultStaticSign(b.kind) : b.staticFeedforwardSign;
    this.followerSignalHz = b.followerSignalHz;
  }

  private static Gravity defaultGravity(MechanismKind kind) {
    return switch (kind) {
      case ROLLER -> Gravity.NONE;
      case ROTARY -> Gravity.ARM_COSINE;
      case LINEAR -> Gravity.ELEVATOR_STATIC;
    };
  }

  private static StaticFeedforwardSignValue defaultStaticSign(MechanismKind kind) {
    // A velocity loop knows which way it is going, so kS follows velocity. A position loop at rest
    // against friction has a velocity of zero and an error that is not, so kS must follow the
    // error or it vanishes exactly when it is needed.
    return kind == MechanismKind.ROLLER
        ? StaticFeedforwardSignValue.UseVelocitySign
        : StaticFeedforwardSignValue.UseClosedLoopSign;
  }

  // ============================================================================================
  // Accessors
  // ============================================================================================

  public String name() {
    return name;
  }

  public MechanismKind kind() {
    return kind;
  }

  public int canId() {
    return canId;
  }

  public CANBus bus() {
    return bus;
  }

  public List<Follower> followers() {
    return followers;
  }

  public Feedback feedback() {
    return feedback;
  }

  public int encoderCanId() {
    return encoderCanId;
  }

  public boolean hasEncoder() {
    return feedback != Feedback.INTERNAL;
  }

  public double rotorToSensorRatio() {
    return rotorToSensorRatio;
  }

  public double sensorToMechanismRatio() {
    return sensorToMechanismRatio;
  }

  /**
   * Rotor rotations per mechanism rotation.
   *
   * <p>The number a simulation needs, and the number that turns a rotor position into a mechanism
   * position. With an internal sensor {@code rotorToSensorRatio} is 1.0 and this is just the gear
   * ratio.
   */
  public double rotorToMechanismRatio() {
    return rotorToSensorRatio * sensorToMechanismRatio;
  }

  public double supplyCurrentLimitAmps() {
    return supplyCurrentLimitAmps;
  }

  public double statorCurrentLimitAmps() {
    return statorCurrentLimitAmps;
  }

  public Optional<SoftLimits> softLimits() {
    return softLimits;
  }

  public Gains gains() {
    return gains;
  }

  public MotionProfile profile() {
    return profile;
  }

  public boolean inverted() {
    return inverted;
  }

  public double followerSignalHz() {
    return followerSignalHz;
  }

  /** Leader plus followers. What a simulation gearbox should be sized for. */
  public int motorCount() {
    return 1 + followers.size();
  }

  // ============================================================================================
  // Phoenix configuration
  // ============================================================================================

  /**
   * The complete Talon configuration, built in the same block order for every mechanism.
   *
   * <p>The uniformity is the point. When every mechanism's config reads the same, a reviewer
   * comparing two of them sees only the differences that matter.
   */
  public TalonFXConfiguration toTalonFXConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimitAmps;
    config.CurrentLimits.SupplyCurrentLowerTime = supplyCurrentLowerTime.in(Seconds);
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.TorqueCurrent.PeakForwardTorqueCurrent = peakForwardTorqueAmps;
    config.TorqueCurrent.PeakReverseTorqueCurrent = peakReverseTorqueAmps;

    config.Voltage.PeakForwardVoltage = peakForwardVoltage;
    config.Voltage.PeakReverseVoltage = peakReverseVoltage;

    config.MotorOutput.NeutralMode = neutralMode;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    softLimits.ifPresent(
        limits -> {
          config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = limits.forwardRotations();
          config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = limits.reverseRotations();
        });

    config.Feedback.FeedbackSensorSource =
        switch (feedback) {
          case INTERNAL -> FeedbackSensorSourceValue.RotorSensor;
          case FUSED_CANCODER -> FeedbackSensorSourceValue.FusedCANcoder;
          case REMOTE_CANCODER -> FeedbackSensorSourceValue.RemoteCANcoder;
          case SYNC_CANCODER -> FeedbackSensorSourceValue.SyncCANcoder;
        };
    if (hasEncoder()) {
      config.Feedback.FeedbackRemoteSensorID = encoderCanId;
    }
    config.Feedback.RotorToSensorRatio = rotorToSensorRatio;
    config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

    applyGains(config, gains);
    config.Slot0.GravityType =
        gravity == Gravity.ARM_COSINE
            ? GravityTypeValue.Arm_Cosine
            : GravityTypeValue.Elevator_Static;
    config.Slot0.StaticFeedforwardSign = staticFeedforwardSign;

    applyProfile(config, profile);

    config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;

    return config;
  }

  /**
   * Writes a gain set into an existing config's Slot0, leaving every other field alone.
   *
   * <p>Used by the live-tuning path, which must apply {@code config.Slot0} taken from a fully-built
   * config rather than a {@code new Slot0Configs()} carrying only the gains. A hand-built block
   * leaves {@code GravityType} and {@code StaticFeedforwardSign} at their class defaults, which
   * would silently switch an arm's gravity compensation from cosine to constant partway through a
   * tuning session.
   */
  static void applyGains(TalonFXConfiguration config, Gains gains) {
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kA = gains.kA();
    config.Slot0.kG = gains.kG();
  }

  /** Writes a profile into an existing config's MotionMagic block. Same reasoning as above. */
  static void applyProfile(TalonFXConfiguration config, MotionProfile profile) {
    config.MotionMagic.MotionMagicCruiseVelocity = profile.cruiseVelocityRps();
    config.MotionMagic.MotionMagicAcceleration = profile.accelerationRpsSq();
    config.MotionMagic.MotionMagicJerk = profile.jerkRpsCubed();
  }

  /**
   * The CANcoder configuration, when there is one.
   *
   * @throws IllegalStateException if this mechanism has no absolute encoder
   */
  public CANcoderConfiguration toCANcoderConfiguration() {
    if (!hasEncoder()) {
      throw new IllegalStateException(
          "Mechanism '" + name + "' has no absolute encoder; feedback source is " + feedback + ".");
    }
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magnetOffsetRotations;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = sensorDiscontinuityPoint;
    config.MagnetSensor.SensorDirection = encoderDirection;
    return config;
  }

  // ============================================================================================
  // Builder
  // ============================================================================================

  /**
   * Starts a config.
   *
   * @param name The mechanism's name. Used for every log key, every alert, and every tunable, so
   *     one grep finds all of it.
   * @param kind What sort of mechanism this is
   */
  public static Builder builder(String name, MechanismKind kind) {
    return new Builder(name, kind);
  }

  /** Mutable builder. See {@link MotorConfig} for what is defaulted and what is required. */
  public static final class Builder {

    private final String name;
    private final MechanismKind kind;
    private final List<Follower> followers = new ArrayList<>();

    // Sentinels rather than zeros: 0 is a legal current limit and a legal ratio, so "unset" needs
    // a value that cannot be confused with a deliberate one.
    private int canId = -1;
    private CANBus bus;
    private double sensorToMechanismRatio = Double.NaN;
    private double supplyCurrentLimitAmps = Double.NaN;
    private double statorCurrentLimitAmps = Double.NaN;
    private Gains gains;
    private MotionProfile profile;

    private Feedback feedback = Feedback.INTERNAL;
    private int encoderCanId = -1;
    private double magnetOffsetRotations = Double.NaN;
    private double sensorDiscontinuityPoint = 0.5;
    private SensorDirectionValue encoderDirection = SensorDirectionValue.CounterClockwise_Positive;
    private double rotorToSensorRatio = 1.0;

    private double supplyCurrentLowerLimitAmps = Double.NaN;
    private Time supplyCurrentLowerTime = Seconds.of(0.1);
    private double peakForwardTorqueAmps = Double.NaN;
    private double peakReverseTorqueAmps = Double.NaN;
    private double peakForwardVoltage = 12.0;
    private double peakReverseVoltage = -12.0;
    private MotorConfig.SoftLimits softLimits;

    private boolean inverted = false;
    private NeutralModeValue neutralMode = NeutralModeValue.Brake;
    private boolean continuousWrap = false;

    private Gravity gravity;
    private StaticFeedforwardSignValue staticFeedforwardSign;

    private double followerSignalHz = 4.0;

    private Builder(String name, MechanismKind kind) {
      this.name = name;
      this.kind = kind;
    }

    /**
     * Which motor, and on which bus.
     *
     * <p>Pick the bus deliberately. Swerve owns the CANivore; anything else placed there needs a
     * documented reason. See {@code .claude/rules/02-hardware.md}.
     */
    public Builder canId(int canId, CANBus bus) {
      this.canId = canId;
      this.bus = bus;
      return this;
    }

    /**
     * Adds a follower on the leader's bus.
     *
     * <p>{@code opposed} is true when the follower is physically mounted facing the opposite way
     * from the leader. Getting it wrong makes the two motors fight: high current, no motion, and it
     * will cook a gearbox. Verify at low output before running closed-loop.
     */
    public Builder follower(int canId, boolean opposed) {
      followers.add(new Follower(canId, opposed));
      return this;
    }

    /**
     * How many rotations of the sensor make one rotation of the mechanism.
     *
     * <p>Required, always. Phoenix defaults this to 1.0, so a config that skips it reports motor
     * rotations while every setpoint, tolerance and soft limit assumes mechanism rotations — a
     * confidently wrong robot, which is much harder to notice than one that refuses to start.
     */
    public Builder sensorToMechanismRatio(double ratio) {
      this.sensorToMechanismRatio = ratio;
      return this;
    }

    /**
     * Attaches a CANcoder.
     *
     * @param source Which of the three CANcoder modes to use
     * @param encoderCanId The encoder's CAN ID, on the same bus as the motor
     * @param magnetOffsetRotations Calibration from the CTRE Tuner X wizard. Not guessable
     * @param rotorToSensorRatio Motor rotations per encoder rotation. Load-bearing under {@code
     *     FUSED_CANCODER}: it is how the rotor extends the absolute reading, so a wrong value puts
     *     the fusion off by the whole reduction while every config call still reports success
     */
    public Builder encoder(
        Feedback source,
        int encoderCanId,
        double magnetOffsetRotations,
        double rotorToSensorRatio) {
      if (source == Feedback.INTERNAL) {
        throw new IllegalArgumentException(
            "Feedback.INTERNAL means no encoder. Omit the encoder() call instead.");
      }
      this.feedback = source;
      this.encoderCanId = encoderCanId;
      this.magnetOffsetRotations = magnetOffsetRotations;
      this.rotorToSensorRatio = rotorToSensorRatio;
      return this;
    }

    /** Which way the CANcoder counts up. Defaults to counter-clockwise positive. */
    public Builder encoderDirection(SensorDirectionValue direction) {
      this.encoderDirection = direction;
      return this;
    }

    /**
     * Where the absolute reading wraps, in rotations. Defaults to 0.5, giving a ±half-rotation
     * range. Use 1.0 for a mechanism whose travel is entirely in one direction from zero.
     */
    public Builder sensorDiscontinuityPoint(double point) {
      this.sensorDiscontinuityPoint = point;
      return this;
    }

    /**
     * Steady-state current drawn from the battery, in amps. Bounds brownout risk.
     *
     * <p>Required. When a mechanism seems underpowered, check whether this or the stator limit is
     * actually binding before changing either — in 2026 the constraint turned out to be supply, and
     * raising the stator cap did nothing.
     */
    public Builder supplyCurrentLimit(double amps) {
      this.supplyCurrentLimitAmps = amps;
      return this;
    }

    /**
     * A lower supply ceiling that engages after {@code duration}, letting the mechanism draw inrush
     * without clamping. Defaults to the same value as the main limit for {@code 0.1 s}, which is
     * the same as no lower limit at all.
     */
    public Builder supplyCurrentLowerLimit(double amps, Time duration) {
      this.supplyCurrentLowerLimitAmps = amps;
      this.supplyCurrentLowerTime = duration;
      return this;
    }

    /**
     * Current through the windings, in amps. Governs heating and stall torque.
     *
     * <p>Required. Also sets the torque-current clamps to ±this unless {@link #peakTorqueCurrent}
     * overrides them, so the closed loop cannot request current the windings may not carry.
     */
    public Builder statorCurrentLimit(double amps) {
      this.statorCurrentLimitAmps = amps;
      return this;
    }

    /**
     * Overrides the peak torque current the closed loop may request, in amps.
     *
     * <p>Distinct from the stator limit and rarely needed separately. Under {@code
     * TorqueCurrentFOC} the control output is itself a current request; clamping it keeps the
     * controller operating in a range it can actually deliver, rather than discovering the ceiling
     * by saturating into it.
     */
    public Builder peakTorqueCurrent(double forwardAmps, double reverseAmps) {
      this.peakForwardTorqueAmps = forwardAmps;
      this.peakReverseTorqueAmps = reverseAmps;
      return this;
    }

    /** Overrides the ±12 V output caps. */
    public Builder peakVoltage(double forwardVolts, double reverseVolts) {
      this.peakForwardVoltage = forwardVolts;
      this.peakReverseVoltage = reverseVolts;
      return this;
    }

    /** Travel bounds, in mechanism rotations. Required for {@code ROTARY} and {@code LINEAR}. */
    public Builder softLimits(double reverseRotations, double forwardRotations) {
      this.softLimits = new MotorConfig.SoftLimits(reverseRotations, forwardRotations);
      return this;
    }

    /** Travel bounds as angles. The rotary form of {@link #softLimits(double, double)}. */
    public Builder softLimits(Angle reverse, Angle forward) {
      return softLimits(reverse.in(Rotations), forward.in(Rotations));
    }

    /** True when positive motor output produces negative mechanism motion. */
    public Builder inverted(boolean inverted) {
      this.inverted = inverted;
      return this;
    }

    /** Defaults to brake. Coast is for a mechanism that should freewheel when disabled. */
    public Builder neutralMode(NeutralModeValue mode) {
      this.neutralMode = mode;
      return this;
    }

    /**
     * Lets the closed loop take the short way around, for a mechanism that rotates continuously.
     *
     * <p>A turret, not an arm. Enabling it on a mechanism with a cable pass-through is how a cable
     * gets wound up.
     */
    public Builder continuousWrap(boolean wrap) {
      this.continuousWrap = wrap;
      return this;
    }

    /** Closed-loop gains, in amps. Required — {@link Gains#zero()} is a legitimate answer. */
    public Builder gains(Gains gains) {
      this.gains = gains;
      return this;
    }

    /** How fast the mechanism may move. Required. */
    public Builder motionProfile(MotionProfile profile) {
      this.profile = profile;
      return this;
    }

    /** Overrides the gravity model implied by the mechanism kind. */
    public Builder gravity(Gravity gravity) {
      this.gravity = gravity;
      return this;
    }

    /** Overrides the static-feedforward sign convention implied by the mechanism kind. */
    public Builder staticFeedforwardSign(StaticFeedforwardSignValue sign) {
      this.staticFeedforwardSign = sign;
      return this;
    }

    /**
     * Rate for follower signals, in Hz. Defaults to 4, which is the floor {@code
     * optimizeBusUtilization} leaves unregistered signals at.
     *
     * <p>4 Hz is a perfectly reasonable rate for a follower — enough to see it is alive, drawing
     * current and not overheating. The point of naming it here is that it is a choice. In 2026 the
     * intake roller's follower signals landed at 4 Hz by omission, and the data looked present
     * while being a quarter second old, which is far harder to spot than data that is missing.
     */
    public Builder followerSignalHz(double hz) {
      this.followerSignalHz = hz;
      return this;
    }

    /**
     * Validates and freezes the config.
     *
     * @throws IllegalStateException naming every missing required value at once, rather than one
     *     per rebuild
     */
    public MotorConfig build() {
      List<String> missing = new ArrayList<>();

      if (canId < 0 || bus == null) {
        missing.add("canId(id, bus)");
      }
      if (Double.isNaN(sensorToMechanismRatio)) {
        missing.add("sensorToMechanismRatio(ratio)");
      }
      if (Double.isNaN(supplyCurrentLimitAmps)) {
        missing.add("supplyCurrentLimit(amps)");
      }
      if (Double.isNaN(statorCurrentLimitAmps)) {
        missing.add("statorCurrentLimit(amps)");
      }
      if (gains == null) {
        missing.add("gains(...) — Gains.zero() counts");
      }
      if (profile == null) {
        missing.add("motionProfile(...)");
      }
      if (kind != MechanismKind.ROLLER && softLimits == null) {
        missing.add(
            "softLimits(...) — required on "
                + kind
                + ", because an unbounded position mechanism drives into its hard stop and holds");
      }
      if (feedback != Feedback.INTERNAL && Double.isNaN(magnetOffsetRotations)) {
        missing.add("encoder(...) magnetOffsetRotations");
      }

      if (!missing.isEmpty()) {
        throw new IllegalStateException(
            "MotorConfig for '"
                + name
                + "' is missing values that have no safe default:\n  - "
                + String.join("\n  - ", missing)
                + "\nSee frc.lib.mechanism.MotorConfig for why each one is required.");
      }

      return new MotorConfig(this);
    }
  }
}
