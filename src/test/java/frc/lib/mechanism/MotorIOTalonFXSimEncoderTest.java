package frc.lib.mechanism;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.mechanism.MotorConfig.Feedback;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * What a simulated CANcoder reports back, for a mechanism configured the way a real arm is.
 *
 * <p>This is the path nothing else covers: every other sim test runs a mechanism on its internal
 * rotor, so a whole class of sign and offset mistakes in the encoder path used to be invisible. The
 * property is simple and is the only one that matters — <b>the device reports the position the
 * physics model says the mechanism is at</b>. A magnet offset is calibration, not motion, and a
 * sensor direction is wiring, not motion; neither may show up as a shift in the reported position.
 *
 * <p>Both are handled once in the constructor, by {@code CANcoderSimState.SensorOffset} and {@code
 * .Orientation}. Get either wrong and simulation runs shifted or mirrored relative to the model,
 * which presents as setpoints landing in the wrong place and soft limits tripping early — with
 * nothing in the config looking wrong, because nothing in the config is wrong.
 *
 * <p>Like the other sim-backed tests this sleeps, because Phoenix's device simulation advances on
 * wall-clock time rather than on a clock the test controls.
 */
class MotorIOTalonFXSimEncoderTest {

  private static final CANBus BUS = new CANBus("rio");

  /** Deliberately not zero, and not a round fraction of a turn. */
  private static final double MAGNET_OFFSET_ROTATIONS = 0.30;

  private static final double ROTOR_TO_SENSOR = 25.0;
  private static final double SENSOR_TO_MECHANISM = 1.0;

  /** Far enough from zero that a sign error and an offset error look different. */
  private static final Angle MECHANISM_POSITION = Rotations.of(0.20);

  private static final long LOOP_PERIOD_MILLIS = 20;
  private static final int SEED_LOOPS = 25;

  /** Phoenix integrates seeded positions, so this is convergence tolerance, not precision. */
  private static final double TOLERANCE_ROTATIONS = 0.01;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");
    RoboRioSim.setVInVoltage(12.0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  @AfterAll
  static void shutdownHal() {
    HAL.shutdown();
  }

  private static MotorConfig config(int canId, int encoderId, SensorDirectionValue direction) {
    return MotorConfig.builder("SimEncoder" + canId, MechanismKind.ROTARY)
        .canId(canId, BUS)
        .encoder(Feedback.FUSED_CANCODER, encoderId, MAGNET_OFFSET_ROTATIONS, ROTOR_TO_SENSOR)
        .encoderDirection(direction)
        .sensorToMechanismRatio(SENSOR_TO_MECHANISM)
        .softLimits(Rotations.of(-2.0), Rotations.of(2.0))
        .supplyCurrentLimit(40.0)
        .statorCurrentLimit(60.0)
        .gains(Gains.zero())
        .motionProfile(
            MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)))
        .build();
  }

  /** Holds the mechanism at one position and reports what the encoder ends up saying. */
  private static Angle reportedPosition(int canId, int encoderId, SensorDirectionValue direction) {
    MotorIOTalonFXSim io = new MotorIOTalonFXSim(config(canId, encoderId, direction));
    for (int i = 0; i < SEED_LOOPS; i++) {
      io.updateSupplyVoltage();
      io.setMechanismState(MECHANISM_POSITION, RotationsPerSecond.of(0.0));
      try {
        Thread.sleep(LOOP_PERIOD_MILLIS);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new IllegalStateException("interrupted while stepping the simulation", e);
      }
    }

    MotorIO.EncoderInputs inputs = new MotorIO.EncoderInputs();
    io.updateEncoderInputs(inputs);
    return inputs.position;
  }

  @Test
  void aCalibratedMagnetOffsetDoesNotShiftTheSimulatedEncoder() {
    // The simulated CANcoder carries the same MagnetOffset as the real one, because the whole
    // CANcoder config is applied here too. Without SensorOffset cancelling it, every seeded
    // position comes back shifted by the offset — 0.30 rotations here, on a mechanism sitting at
    // 0.20 — and every fused-encoder arm runs in simulation somewhere it is not.
    assertEquals(
        MECHANISM_POSITION.in(Rotations),
        reportedPosition(51, 52, SensorDirectionValue.CounterClockwise_Positive).in(Rotations),
        TOLERANCE_ROTATIONS);
  }

  @Test
  void aClockwisePositiveEncoderStillReportsTheMechanismsPosition() {
    // Sensor direction is wiring. The physics model is in mechanism units either way, so flipping
    // it must not flip what comes back — and if it did, the rotor and the encoder would disagree
    // in sign under FusedCANcoder while each looked individually fine.
    assertEquals(
        MECHANISM_POSITION.in(Rotations),
        reportedPosition(53, 54, SensorDirectionValue.Clockwise_Positive).in(Rotations),
        TOLERANCE_ROTATIONS);
  }
}
