package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.lowerFeeder.LowerFeeder;
import frc.robot.subsystems.prestage.Prestage;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.upperFeeder.UpperFeeder;

/**
 * The 2026 spit sequences: run the fuel path to clear the robot. Ported from Rebuilt2026 {@code
 * SpitSequences}. <b>None of these was bound to a button or registered as a named command in
 * 2026</b>; they are here because the port brings everything across.
 *
 * <p>Reads {@code Constants.Setpoints} directly, for the reason given on {@link ShootSequences}.
 */
public final class SpitSequences {

  private SpitSequences() {}

  public static Command spitAll(
      Flywheel flywheel,
      Prestage prestage,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      IntakeRoller intakeRoller) {
    return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, Constants.Setpoints.FLYWHEEL_SPIT_VELOCITY),
            PrestageCommands.setPrestageVelocity(
                prestage, Constants.Setpoints.PRESTAGE_SPIT_VELOCITY),
            FeederCommands.setUpperFeederVelocity(
                upperFeeder, Constants.Setpoints.FEEDER_SPIT_VELOCITY),
            FeederCommands.setLowerFeederVelocity(
                lowerFeeder, Constants.Setpoints.FEEDER_SPIT_VELOCITY),
            TransportCommands.setTransportVoltage(
                transport, Constants.Setpoints.TRANSPORT_SPIT_VOLTAGE),
            IntakeRollerCommands.setRollerVoltage(
                intakeRoller, Constants.Setpoints.INTAKE_ROLLER_SPIT_VOLTAGE))
        .finallyDo(
            () -> {
              flywheel.setVelocity(RotationsPerSecond.of(0));
              prestage.setVelocity(RotationsPerSecond.of(0));
              lowerFeeder.setVelocity(RotationsPerSecond.of(0));
              upperFeeder.setVelocity(RotationsPerSecond.of(0));
              transport.setVoltage(Volts.of(0));
              intakeRoller.setVoltage(Volts.of(0));
            })
        .withName("SpitAll");
  }

  public static Command spitHopper(
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      IntakeRoller intakeRoller) {
    return Commands.parallel(
            FeederCommands.setUpperFeederVelocity(
                upperFeeder, Constants.Setpoints.FEEDER_SPIT_VELOCITY),
            FeederCommands.setLowerFeederVelocity(
                lowerFeeder, Constants.Setpoints.FEEDER_SPIT_VELOCITY),
            TransportCommands.setTransportVoltage(
                transport, Constants.Setpoints.TRANSPORT_SPIT_VOLTAGE),
            IntakeRollerCommands.setRollerVoltage(
                intakeRoller, Constants.Setpoints.INTAKE_ROLLER_SPIT_VOLTAGE))
        .finallyDo(
            () -> {
              lowerFeeder.setVelocity(RotationsPerSecond.of(0));
              upperFeeder.setVelocity(RotationsPerSecond.of(0));
              transport.setVoltage(Volts.of(0));
              intakeRoller.setVoltage(Volts.of(0));
            })
        .withName("SpitHopper");
  }

  public static Command clearShooter(
      Flywheel flywheel, Prestage prestage, UpperFeeder upperFeeder, LowerFeeder lowerFeeder) {
    return Commands.parallel(
            FlywheelCommands.setFlywheelVelocity(
                flywheel, Constants.Setpoints.FLYWHEEL_SPIT_VELOCITY),
            PrestageCommands.setPrestageVelocity(
                prestage, Constants.Setpoints.PRESTAGE_SPIT_VELOCITY),
            FeederCommands.setUpperFeederVelocity(
                upperFeeder, Constants.Setpoints.FEEDER_SPIT_VELOCITY),
            FeederCommands.setLowerFeederVelocity(
                lowerFeeder, Constants.Setpoints.FEEDER_SPIT_VELOCITY))
        .finallyDo(
            () -> {
              flywheel.setVelocity(RotationsPerSecond.of(0));
              prestage.setVelocity(RotationsPerSecond.of(0));
              upperFeeder.setVelocity(RotationsPerSecond.of(0));
              lowerFeeder.setVelocity(RotationsPerSecond.of(0));
            })
        .withName("ClearShooter");
  }

  /** Spit for {@link Constants.Waits#SPIT_AFTER_SHOOT_SECONDS}, then stop everything. */
  public static Command spitAfterShoot(
      Flywheel flywheel,
      Prestage prestage,
      UpperFeeder upperFeeder,
      LowerFeeder lowerFeeder,
      Transport transport,
      IntakeRoller intakeRoller) {
    // The wait is the deadline. The motor commands are run(), not runOnce(), so they keep applying
    // their setpoints and do not end the group early.
    return Commands.sequence(
            Commands.waitSeconds(Constants.Waits.SPIT_AFTER_SHOOT_SECONDS)
                .deadlineFor(
                    Commands.run(
                        () -> flywheel.setVelocity(Constants.Setpoints.FLYWHEEL_SPIT_VELOCITY),
                        flywheel),
                    Commands.run(
                        () -> prestage.setVelocity(Constants.Setpoints.PRESTAGE_SPIT_VELOCITY),
                        prestage),
                    Commands.run(
                        () -> upperFeeder.setVelocity(Constants.Setpoints.FEEDER_SPIT_VELOCITY),
                        upperFeeder),
                    Commands.run(
                        () -> lowerFeeder.setVelocity(Constants.Setpoints.FEEDER_SPIT_VELOCITY),
                        lowerFeeder),
                    Commands.run(
                        () -> transport.setVoltage(Constants.Setpoints.TRANSPORT_SPIT_VOLTAGE),
                        transport),
                    Commands.run(
                        () ->
                            intakeRoller.setVoltage(Constants.Setpoints.INTAKE_ROLLER_SPIT_VOLTAGE),
                        intakeRoller)),
            Commands.parallel(
                FlywheelCommands.setFlywheelVelocity(flywheel, RotationsPerSecond.of(0)),
                PrestageCommands.setPrestageVelocity(prestage, RotationsPerSecond.of(0)),
                FeederCommands.setUpperFeederVelocity(upperFeeder, RotationsPerSecond.of(0)),
                FeederCommands.setLowerFeederVelocity(lowerFeeder, RotationsPerSecond.of(0)),
                TransportCommands.setTransportVoltage(transport, Volts.of(0)),
                IntakeRollerCommands.setRollerVoltage(intakeRoller, Volts.of(0))))
        .withName("SpitAfterShoot");
  }
}
