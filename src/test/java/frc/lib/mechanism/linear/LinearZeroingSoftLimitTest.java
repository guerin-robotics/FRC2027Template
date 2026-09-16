package frc.lib.mechanism.linear;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.mechanism.Gains;
import frc.lib.mechanism.MotionProfile;
import frc.lib.mechanism.MotorConfig;
import frc.lib.mechanism.MotorConfig.MechanismKind;
import frc.lib.mechanism.MotorIO;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * That the zeroing routine takes the reverse travel bound off, and always puts it back.
 *
 * <p>This is a contract test against a recording IO rather than a physics simulation, and it has to
 * be. {@code LinearMechanismSim} seeds the simulated device from the physics model every loop, so
 * the device and the carriage never disagree about where the carriage is — and the real failure is
 * exactly that disagreement. On {@code Feedback.INTERNAL} hardware the Talon reads zero rotations
 * at boot wherever the carriage physically sits, so a mechanism whose reverse bound is at zero
 * boots believing it is already against that bound and refuses to drive toward the stop. Nothing
 * moves, the stall check passes the first time it is polled, and the routine zeroes the carriage
 * wherever it was parked. No simulation built this way can reproduce that, which is why a passing
 * zeroing test sat beside the bug.
 *
 * <p>What is checkable is the contract: the bound comes off before any current is commanded, and it
 * goes back on however the routine ends.
 *
 * <p>Two cases, which between them cover both exits. {@code finallyDo} is one block on one command:
 * the path that restores the bound after an interruption is the same path that restores it after a
 * normal finish, so pinning the interrupted case pins both. The routine running start to finish
 * against real physics is covered by {@code
 * LinearMechanismSimTest.theZeroingRoutineDrivesToTheStopAndDetectsTheStall}.
 */
class LinearZeroingSoftLimitTest {

  private static final CANBus BUS = new CANBus("rio");
  private static final LinearGeometry GEOMETRY = new LinearGeometry(Inches.of(1.5), 1);

  private static final long LOOP_PERIOD_MILLIS = 20;
  private static final double SETTLE_SECONDS = 0.3;

  private RecordingIO io;
  private LinearSubsystem subsystem;

  /** Records every reverse-bound toggle, in order. */
  private static final class RecordingIO implements MotorIO {
    private final List<Boolean> softLimitCalls = new ArrayList<>();

    @Override
    public void updateInputs(MotorInputs inputs) {
      inputs.connected = true;
      // Stationary, which is what the real failure looks like from the code's side: the stall
      // check passes as soon as it is allowed to.
      inputs.velocity = RotationsPerSecond.of(0);
    }

    @Override
    public void setReverseSoftLimitEnabled(boolean enabled) {
      softLimitCalls.add(enabled);
    }
  }

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    // The scheduler will not run a command while the driver station reports disabled, and an
    // empty call list looks exactly like a routine that chose not to touch the bound.
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  @AfterAll
  static void shutdown() {
    HAL.shutdown();
  }

  @BeforeEach
  void newMechanism() {
    CommandScheduler.getInstance().cancelAll();

    MotorConfig config =
        MotorConfig.builder("ZeroingLift", MechanismKind.LINEAR)
            .canId(45, BUS)
            .sensorToMechanismRatio(5.0)
            // The shipped scaffold's shape: the reverse bound sits exactly at the bottom of
            // travel, which is the configuration the routine cannot work against.
            .softLimits(GEOMETRY.rotationsFor(Inches.of(0)), GEOMETRY.rotationsFor(Inches.of(40)))
            .supplyCurrentLimit(40.0)
            .statorCurrentLimit(60.0)
            .gains(Gains.zero())
            .motionProfile(
                MotionProfile.of(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)))
            .build();

    io = new RecordingIO();
    subsystem =
        new LinearSubsystem(
            new LinearMechanism(config, new LinearSettings(Inches.of(0.25), GEOMETRY), io));
  }

  private Command zeroing() {
    return LinearCommands.zeroAtHardStop(
        subsystem,
        Amps.of(-12),
        Seconds.of(SETTLE_SECONDS),
        InchesPerSecond.of(0.5),
        Seconds.of(4.0),
        Inches.of(0));
  }

  @Test
  void theBoundComesOffBeforeAnyCurrentIsCommanded() {
    Command zeroing = zeroing();
    CommandScheduler.getInstance().schedule(zeroing);
    CommandScheduler.getInstance().run();

    assertEquals(
        List.of(false),
        io.softLimitCalls,
        "the bound must be off before the routine drives, or the device refuses to move at all");
  }

  @Test
  void anInterruptedRoutineStillRestoresTheBound() {
    // The case that matters most. A zeroing cancelled by a driver, or interrupted by another
    // command requiring the lift, must not leave the carriage with no lower bound for the rest of
    // the match.
    Command zeroing = zeroing();
    CommandScheduler.getInstance().schedule(zeroing);
    CommandScheduler.getInstance().run();

    CommandScheduler.getInstance().cancel(zeroing);
    CommandScheduler.getInstance().run();

    assertEquals(
        List.of(false, true),
        io.softLimitCalls,
        "finallyDo runs on interruption too, so the bound is restored there as well");
  }
}
