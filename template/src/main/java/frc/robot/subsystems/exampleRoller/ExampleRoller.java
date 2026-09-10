package frc.robot.subsystems.exampleRoller;

import frc.lib.mechanism.roller.RollerMechanism;
import frc.lib.mechanism.roller.RollerSubsystem;

/**
 * A velocity-controlled mechanism: roller, flywheel, feeder, transport, intake.
 *
 * <p>Anything that spins, where the question is "how fast is it turning" and never "where is it".
 * In 2026 that covered the flywheel, the prestage, both feeders, the transport and the intake
 * roller.
 *
 * <p><b>If the mechanism has to reach a position and hold it, this is the wrong scaffold.</b> Copy
 * {@code exampleArm} for anything that pivots, {@code exampleLift} for anything that travels in a
 * line.
 *
 * <h2>Why this file is almost empty</h2>
 *
 * <p>Reading inputs, logging them, reporting supply current, pushing tuned gains to the device,
 * tracking the goal, deciding what counts as "at speed", detecting jams — all of that lives in
 * {@link RollerMechanism}, because none of it varies between one roller and the next. Through 2026
 * every mechanism carried its own copy, and the differences between those copies were accidents
 * more often than decisions.
 *
 * <p>What belongs <i>here</i> is whatever is true of <b>this</b> mechanism and no other. Usually
 * that is a predicate or two with a name the rest of the code can read:
 *
 * <pre>{@code
 * public boolean isReadyToShoot() {
 *   return isAtVelocity() && !isJammed();
 * }
 * }</pre>
 *
 * <p>Put those here rather than assembling the same expression inside three different command
 * factories. A named predicate is also what {@code Triggers.java} should bind to.
 *
 * <h2>What must NOT go here</h2>
 *
 * <ul>
 *   <li><b>Hardware calls.</b> No {@code TalonFX}, no {@code CANcoder}. The moment one appears, log
 *       replay stops working and you lose the ability to debug a match after the fact.
 *   <li><b>Another subsystem.</b> Read {@code RobotState}, or take a {@code Supplier} in the
 *       constructor. See {@code .claude/rules/01-architecture.md}.
 *   <li><b>Command logic.</b> Anything deciding <i>when</i> belongs in a command factory. {@code
 *       RollerCommands} already has the common verbs; write an {@code ExampleRollerCommands} only
 *       when this mechanism has verbs of its own.
 * </ul>
 *
 * <h2>Wiring</h2>
 *
 * <p>{@code RobotContainer} picks the implementation, exactly as it does for {@code Drive} and
 * {@code Vision}:
 *
 * <pre>{@code
 * switch (Constants.currentMode) {
 *   case REAL -> exampleRoller = new ExampleRoller(
 *       RollerMechanism.real(ExampleRollerConstants.CONFIG, ExampleRollerConstants.SETTINGS));
 *   case SIM -> exampleRoller = new ExampleRoller(
 *       RollerMechanism.sim(
 *           ExampleRollerConstants.CONFIG,
 *           ExampleRollerConstants.SETTINGS,
 *           ExampleRollerConstants.SIM));
 *   case REPLAY -> exampleRoller = new ExampleRoller(
 *       RollerMechanism.replay(ExampleRollerConstants.CONFIG, ExampleRollerConstants.SETTINGS));
 * }
 * exampleRoller.registerFaultMonitors();
 * exampleRoller.setDefaultCommand(RollerCommands.idle(exampleRoller));
 * }</pre>
 *
 * <p>Do not forget {@code registerFaultMonitors()}. It is what puts a motor that has dropped off
 * the bus, rebooted mid-match or overheated in front of the pit crew instead of only in the log.
 *
 * <p>TEMPLATE INSTRUCTIONS: rename {@code ExampleRoller} throughout this file and {@link
 * ExampleRollerConstants}, then fill in the values the compiler asks for.
 */
public class ExampleRoller extends RollerSubsystem {

  public ExampleRoller(RollerMechanism mechanism) {
    super(mechanism);
  }

  // ============================================================================================
  // Mechanism-specific state — delete if this mechanism has none
  // ============================================================================================
  //
  // Everything a generic roller can answer is already inherited: isAtVelocity(), isJammed(),
  // getVelocity(), getGoalVelocity(), isConnected(). Add here only what needs this mechanism's own
  // vocabulary.
  //
  //   /** Spun up and not jammed — what a scoring sequence actually waits on. */
  //   public boolean isReadyToShoot() {
  //     return isAtVelocity() && !isJammed();
  //   }
}
