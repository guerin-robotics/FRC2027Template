package frc.robot.subsystems.exampleArm;

import frc.lib.mechanism.rotary.RotaryMechanism;
import frc.lib.mechanism.rotary.RotarySubsystem;

/**
 * A rotary position mechanism: arm, pivot, hood, turret, wrist.
 *
 * <p>Anything where the question is "where is it" and the answer is an angle. Gravity varies with
 * that angle, which is what separates this from {@code exampleLift} — an arm's {@code kG} is scaled
 * by the cosine of where it is pointing, an elevator's is constant.
 *
 * <p>See {@code ExampleRoller} for why this file is nearly empty and what does belong in it. The
 * short version: everything generic lives in {@link RotaryMechanism}; what goes here is whatever is
 * true of <b>this</b> mechanism and no other.
 *
 * <h2>The one thing a rotary mechanism gets wrong more than anything else</h2>
 *
 * <p>Stopping is not holding. {@code stop()} leaves the mechanism braked but unpowered, and an arm
 * under gravity falls to its hard stop from there. A command that drives to an angle and then
 * <i>finishes</i> hands the subsystem back to its default command, which usually stops it — so the
 * arm arrives, and then sags.
 *
 * <p>{@code RotaryCommands.holdAt} does not end, deliberately. {@code
 * RotaryCommands.holdCurrentPosition} is the default command you almost certainly want, not {@code
 * stop}.
 *
 * <h2>Wiring</h2>
 *
 * <pre>{@code
 * switch (Constants.currentMode) {
 *   case REAL -> exampleArm = new ExampleArm(
 *       RotaryMechanism.real(ExampleArmConstants.CONFIG, ExampleArmConstants.SETTINGS));
 *   case SIM -> exampleArm = new ExampleArm(
 *       RotaryMechanism.sim(
 *           ExampleArmConstants.CONFIG, ExampleArmConstants.SETTINGS, ExampleArmConstants.SIM));
 *   case REPLAY -> exampleArm = new ExampleArm(
 *       RotaryMechanism.replay(ExampleArmConstants.CONFIG, ExampleArmConstants.SETTINGS));
 * }
 * exampleArm.registerFaultMonitors();
 * exampleArm.setDefaultCommand(RotaryCommands.holdCurrentPosition(exampleArm));
 * }</pre>
 *
 * <p>TEMPLATE INSTRUCTIONS: rename {@code ExampleArm} throughout this file and {@link
 * ExampleArmConstants}, then fill in the values the compiler asks for.
 */
public class ExampleArm extends RotarySubsystem {

  public ExampleArm(RotaryMechanism mechanism) {
    super(mechanism);
  }

  // ============================================================================================
  // Mechanism-specific state — delete if this mechanism has none
  // ============================================================================================
  //
  // Inherited already: isAtPosition(), nearGoal(target, tolerance), getPosition(),
  // getGoalPosition(), isConnected(). Add here only what needs this mechanism's own vocabulary.
  //
  //   /** Clear of the frame perimeter, at a much looser tolerance than "ready to score". */
  //   public boolean isClearOfFrame() {
  //     return getPosition().gt(Constants.Setpoints.ARM_FRAME_CLEARANCE);
  //   }
}
