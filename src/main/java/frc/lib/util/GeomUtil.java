package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Conversions between WPILib geometry types.
 *
 * <p>These are the handful of one-liners that get rewritten inline, slightly differently, in five
 * places over a season. Nothing here is clever; the value is that it is written once and named.
 */
public class GeomUtil {

  private GeomUtil() {}

  /** Translation to a transform with no rotation. */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  /** X/Y offset to a transform with no rotation. */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, Rotation2d.kZero);
  }

  /** Rotation to a transform with no translation. */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  /** Pose to the transform that would produce it from the origin. */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /** Transform to the pose it would produce from the origin. */
  public static Pose2d toPose2d(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /** Translation to a pose with no rotation. */
  public static Pose2d toPose2d(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.kZero);
  }

  /** Rotation to a pose at the origin. */
  public static Pose2d toPose2d(Rotation2d rotation) {
    return new Pose2d(Translation2d.kZero, rotation);
  }

  /** Pose to a 3D transform at z = 0. */
  public static Transform3d toTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /** 3D transform to the pose it would produce from the origin. */
  public static Pose3d toPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Inverts a pose — the transform that undoes it.
   *
   * <p>Handy for "where is the field, from the robot's point of view".
   */
  public static Pose2d inverse(Pose2d pose) {
    Rotation2d rotationInverse = pose.getRotation().unaryMinus();
    return new Pose2d(
        pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
  }

  /** Scales a twist. Useful for integrating a partial timestep. */
  public static Twist2d multiply(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  /**
   * Chassis speeds as a twist, i.e. the displacement over one second.
   *
   * <p>Multiply by the timestep with {@link #multiply} to integrate a single loop.
   */
  public static Twist2d toTwist2d(ChassisSpeeds speeds) {
    return new Twist2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /** Same pose, different translation. */
  public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
    return new Pose2d(translation, pose.getRotation());
  }

  /** Same pose, different rotation. */
  public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), rotation);
  }
}
