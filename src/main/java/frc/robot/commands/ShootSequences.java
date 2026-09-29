package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Triggers;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.upperFeeder.UpperFeeder;
import org.littletonrobotics.junction.Logger;

/**
 * The 2026 shooting sequences — the command-composition layer that spans subsystems. Ported from
 * Rebuilt2026 {@code ShootSequences}.
 *
 * <p>These read {@code Constants.Setpoints} directly rather than taking every setpoint as a
 * parameter, as 2026 did. A sequence is the one place that composes a whole shot; threading seven
 * setpoints through each signature would only move the same list into {@code RobotContainer}.
 */
public final class ShootSequences {

  private ShootSequences() {}

  /**
   * The auto "Shoot" named command: aim flywheel and hood at the hub, then feed once spun up and
   * loosely aligned (or after {@link Constants.Waits#SPIN_UP_TIMEOUT_SECONDS}), agitating the
   * intake and compressing the hopper. Runs until interrupted.
   */
  public static Command autoShootToHub(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      IntakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.parallel(
            Commands.runOnce(() -> Logger.recordOutput("RobotState/shooting", true)),
            Commands.parallel(
                FlywheelCommands.setVelocityForHub(flywheel),
                PrestageCommands.setPrestageVelocity(
                    prestage, Constants.Setpoints.PRESTAGE_VELOCITY),
                HoodCommands.setHoodPosForHub(hood)),
            Commands.sequence(
                Commands.waitUntil(
                        Triggers.getInstance()
                            .isFlywheelSpunUp
                            .and(Triggers.getInstance().isAlignedLooser))
                    .withTimeout(Constants.Waits.SPIN_UP_TIMEOUT_SECONDS),
                Commands.parallel(
                    FeederCommands.setLowerFeederVelocity(
                        lowerFeeder, Constants.Setpoints.FEEDER_VELOCITY),
                    FeederCommands.setUpperFeederVelocity(
                        upperFeeder, Constants.Setpoints.FEEDER_VELOCITY),
                    TransportCommands.setTransportVelocity(
                        transport, Constants.Setpoints.TRANSPORT_VELOCITY),
                    IntakeRollerCommands.setRollerVoltage(
                        intakeRoller, Constants.Setpoints.INTAKE_ROLLER_AGITATE_VOLTAGE),
                    IntakePivotCommands.autoPivotCompress(
                        intakePivot, Constants.Setpoints.PIVOT_JOSTLE_UP_POSITION))))
        .withName("ShootToHub");
  }

  /** Unbound in 2026. Stop feeding, drop the intake, then stop the flywheel a beat later. */
  public static Command shootEndBehavior(
      Flywheel flywheel,
      Prestage prestage,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      IntakeRoller intakeRoller,
      IntakePivot intakePivot) {
    return Commands.sequence(
            Commands.parallel(
                PrestageCommands.stop(prestage),
                FeederCommands.stopUpper(upperFeeder),
                FeederCommands.stopLower(lowerFeeder),
                TransportCommands.stop(transport),
                IntakeRollerCommands.stopIntakeRoller(intakeRoller),
                IntakePivotCommands.setPivotPosition(
                    intakePivot, Constants.Setpoints.PIVOT_DOWN_POSITION)),
            Commands.waitSeconds(Constants.Waits.SHOOT_END_FLYWHEEL_DELAY_SECONDS),
            FlywheelCommands.stop(flywheel))
        .withName("ShootEndBehavior");
  }

  /** Stop everything but the drivetrain and the pivot. The auto "stopAll" and teleopInit. */
  public static Command stopAll(
      Flywheel flywheel,
      Prestage prestage,
      Hood hood,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      IntakeRoller intakeRoller) {
    return Commands.parallel(
            FlywheelCommands.stop(flywheel),
            PrestageCommands.stop(prestage),
            HoodCommands.stowHood(hood),
            FeederCommands.stopUpper(upperFeeder),
            FeederCommands.stopLower(lowerFeeder),
            TransportCommands.stop(transport),
            IntakeRollerCommands.stopIntakeRoller(intakeRoller))
        .withName("StopAll");
  }
}
