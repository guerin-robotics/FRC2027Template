package frc.robot.subsystems.exampleLift;

import frc.lib.mechanism.linear.LinearMechanism;
import frc.lib.mechanism.linear.LinearSubsystem;

/**
 * A linear position mechanism: elevator, lift, extension.
 *
 * <p>Anything where the question is "where is it" and the answer is a distance. Gravity is constant
 * rather than angle-dependent, which is what separates this from {@code exampleArm}.
 *
 * <p>See {@code ExampleRoller} for why this file is nearly empty and what does belong in it.
 *
 * <h2>Two things an elevator gets wrong</h2>
 *
 * <p><b>Stopping is not holding.</b> {@code stop()} leaves the carriage braked but unpowered, and
 * it descends from there. {@code LinearCommands.holdAt} does not end, deliberately, and {@code
 * LinearCommands.holdCurrentPosition} is the default command you want.
 *
 * <p><b>It does not know where it is at boot.</b> On the motor's internal encoder, position reads
 * zero wherever the carriage happened to be sitting, so every setpoint is offset by however far
 * that was from the bottom. {@code LinearCommands.zeroAtHardStop} is the fix. Run it once at
 * startup or from a pit button — <b>not</b> automatically during a match, because travelling to the
 * bottom is neither always safe nor ever fast. A mechanism with a fused CANcoder needs none of
 * this.
 *
 * <h2>Wiring</h2>
 *
 * <pre>{@code
 * switch (Constants.currentMode) {
 *   case REAL -> exampleLift = new ExampleLift(
 *       LinearMechanism.real(ExampleLiftConstants.CONFIG, ExampleLiftConstants.SETTINGS));
 *   case SIM -> exampleLift = new ExampleLift(
 *       LinearMechanism.sim(
 *           ExampleLiftConstants.CONFIG, ExampleLiftConstants.SETTINGS, ExampleLiftConstants.SIM));
 *   case REPLAY -> exampleLift = new ExampleLift(
 *       LinearMechanism.replay(ExampleLiftConstants.CONFIG, ExampleLiftConstants.SETTINGS));
 * }
 * exampleLift.registerFaultMonitors();
 * exampleLift.setDefaultCommand(LinearCommands.holdCurrentPosition(exampleLift));
 * }</pre>
 *
 * <p>TEMPLATE INSTRUCTIONS: rename {@code ExampleLift} throughout this file and {@link
 * ExampleLiftConstants}, then fill in the values the compiler asks for.
 */
public class ExampleLift extends LinearSubsystem {

  public ExampleLift(LinearMechanism mechanism) {
    super(mechanism);
  }

  // ============================================================================================
  // Mechanism-specific state — delete if this mechanism has none
  // ============================================================================================
  //
  // Inherited already: isAtPosition(), nearGoal(target, tolerance), getHeight(),
  // getGoalPosition(), getLinearVelocity(), isConnected().
  //
  //   /** Low enough that the arm above it may swing without hitting the frame. */
  //   public boolean isStowed() {
  //     return nearGoal(Constants.Setpoints.LIFT_STOW, Inches.of(1.0));
  //   }
}
