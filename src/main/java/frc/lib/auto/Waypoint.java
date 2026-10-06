package frc.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * One stop in a point-to-point auto: where to go, how close counts as there, and how long to try.
 *
 * <p>The timeout is part of the waypoint on purpose. A drive-to-pose that never arrives — blocked
 * by another robot, or a pose the drivetrain cannot quite reach — would otherwise hang the rest of
 * the auto, and {@code .claude/rules/03-commands.md} forbids an untimed wait for exactly that
 * reason. On timeout the auto moves on to the next waypoint from wherever the robot is.
 *
 * @param pose Field pose, <b>blue-alliance origin</b>. Flipped for red at run time, never here.
 * @param positionTolerance Translation error that counts as arrived
 * @param headingTolerance Heading error that counts as arrived
 * @param timeout Give up on this waypoint after this long and continue
 */
public record Waypoint(
    Pose2d pose, Distance positionTolerance, Angle headingTolerance, Time timeout) {}
