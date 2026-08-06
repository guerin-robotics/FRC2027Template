// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class VisionConstants {
  // AprilTag layout — single source of truth lives in FieldConstants.
  public static AprilTagFieldLayout aprilTagLayout = FieldConstants.aprilTagLayout;

  // ---- Camera setup ----
  //
  // TODO(2027): rename these to match the names configured on each coprocessor, and
  // delete any cameras the 2027 robot does not have. The count here must match the
  // number of VisionIO instances constructed in RobotContainer for ALL THREE modes
  // (REAL, SIM, REPLAY) — replay needs one VisionIO per camera so every camera's
  // logged inputs are replayed.
  public static final String camera0Name = "camera0";
  public static final String camera1Name = "camera1";
  public static final String camera2Name = "camera2";
  public static final String camera3Name = "camera3";

  // ---- Camera transforms ----
  //
  // Robot-to-camera transform for each camera: translation from robot center (X forward,
  // Y left, Z up) plus rotation (roll, pitch, yaw). Note that in WPILib a camera tilted
  // UPWARD has a NEGATIVE pitch.
  //
  // TODO(2027): MEASURE THESE ON THE REAL ROBOT. They are identity placeholders. Vision
  // pose estimates are garbage until these are correct — a 1 inch error in camera
  // position is a 1 inch error in every pose estimate that camera produces, and a 1°
  // yaw error grows with tag distance.
  public static final Transform3d robotToCamera0 = new Transform3d();
  public static final Transform3d robotToCamera1 = new Transform3d();
  public static final Transform3d robotToCamera2 = new Transform3d();
  public static final Transform3d robotToCamera3 = new Transform3d();

  // ---- Filtering thresholds ----
  //
  // These were tuned against real 2026 match logs and are the main thing this file
  // carries forward. They are geometry-independent, so they are good starting values
  // for 2027. Re-validate them against 2027 logs before trusting them at competition.

  // Single-tag ambiguity above this is rejected (multi-tag is always trusted).
  // Log analysis of 2026 State q30/e3 showed accepted single-tag solves with
  // ambiguity 0.33–0.37 that were 3.3–3.4 m wrong (wrong PnP solution).
  public static double maxAmbiguity = 0.35;

  // Estimated pose Z (height) must be below this to be realistic
  public static double maxZError = 2;

  // Not below the floor
  public static double floorError = inchesToMeters(6);

  // Tags farther than this are unreliable — reject the observation entirely.
  // At long range, small pixel errors become large pose errors.
  public static double maxDistanceMeters = 8.0;

  // Stricter distance limit for SINGLE-tag observations only. Every
  // catastrophic accepted pose in the 2026 State/Worlds logs (2.5–9.6 m wrong,
  // including some with ambiguity ≈ 0) was a single-tag solve at ≥ 3.8 m.
  // Multi-tag solves at 4–6 m were never catastrophically wrong, so they
  // keep the looser maxDistanceMeters limit above.
  public static double maxSingleTagDistanceMeters = 4.0;

  // If the robot is spinning faster than this (rad/s), vision is unreliable
  // because motion blur and timestamp misalignment degrade the estimate.
  public static double maxAngularVelocityRadPerSec = 6.0;

  // Maximum pitch or roll (radians) allowed in an estimated pose.
  // A real robot on flat carpet should never be tilted more than ~10°.
  // Large pitch/roll in the estimate means the solve is wrong.
  public static double maxPitchRollRadians = Math.toRadians(25.0);

  // ---- Standard deviation baselines ----
  // For 1 meter distance and 1 tag. Automatically scaled by distance² / tagCount.
  public static double linearStdDevBaseline = 0.01; // Meters
  public static double angularStdDevBaseline = 0.03; // Radians

  // Extra multiplier applied to single-tag observations (tagCount == 1).
  // Single-tag PnP is inherently less constrained than multi-tag, so we
  // trust it less. This prevents a single ambiguous solve from yanking the
  // pose estimator.
  public static double singleTagStdDevMultiplier = 2.0;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Logging
  static {
    Logger.recordOutput("Vision/Camera0/name", VisionConstants.camera0Name);
    Logger.recordOutput("Vision/Camera0/robot_position", VisionConstants.robotToCamera0);
    Logger.recordOutput("Vision/Camera1/name", VisionConstants.camera1Name);
    Logger.recordOutput("Vision/Camera1/robot_position", VisionConstants.robotToCamera1);
    Logger.recordOutput("Vision/Camera2/name", VisionConstants.camera2Name);
    Logger.recordOutput("Vision/Camera2/robot_position", VisionConstants.robotToCamera2);
    Logger.recordOutput("Vision/Camera3/name", VisionConstants.camera3Name);
    Logger.recordOutput("Vision/Camera3/robot_position", VisionConstants.robotToCamera3);
  }
}
