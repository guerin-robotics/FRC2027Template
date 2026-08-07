// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.io.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

/**
 * Exercises every branch of the pose-rejection ladder in {@code Vision.periodic()}.
 *
 * <p>Vision is the one subsystem whose failures are invisible from the driver station. A filter
 * that stops rejecting bad poses does not error — it feeds garbage into the pose estimator, and the
 * robot teleports mid-auto. A filter that starts rejecting good ones does not error either; the
 * robot just drifts as though vision were unplugged. {@code .claude/rules/00-safety.md} lists both
 * under "modified vision filter threshold — bad poses accepted; robot teleports during auto".
 *
 * <p>The thresholds themselves are deliberately untouched here. CLAUDE.md records that they were
 * tuned against real 2026 match logs and are geometry-independent, so they carry into 2027 — this
 * test reads them from {@link VisionConstants} rather than restating them, and so proves the
 * <i>logic</i> that consumes them without pinning the numbers. Retuning a threshold does not break
 * these tests; deleting or reordering a filter does.
 *
 * <p>Each case feeds one synthetic observation through a fake {@link VisionIO} and asserts whether
 * the {@code VisionConsumer} — the pose estimator on a real robot — was handed it. No hardware, no
 * PhotonVision, no camera.
 *
 * <p><b>Cleanup matters.</b> {@code Vision} extends {@code SubsystemBase}, whose constructor
 * registers into the JVM-wide {@code CommandScheduler}. Each instance built here is unregistered
 * immediately — {@link #periodicWith} drives {@code periodic()} directly — so that a dozen stray
 * Vision objects are not still running during {@code DriveToPoseSimTest}.
 */
class VisionFilterTest {

  /** Comfortably inside the field, on the floor, level. The baseline every case perturbs. */
  private static final Pose3d GOOD_POSE = new Pose3d(4.0, 3.0, 0.0, Rotation3d.kZero);

  private static final double GOOD_TIMESTAMP = 12.34;

  @BeforeAll
  static void initializeHal() {
    // See GainSweepTest's javadoc for why this must not be inside a Java assert.
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize; sim-backed tests cannot run");

    // Vision.periodic() calls Logger.processInputs and Logger.recordOutput unconditionally.
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();

    // Report disabled before the first periodic(). AdvantageKit builds the struct schema for a
    // record type the first time one is logged, and warns when that happens while enabled
    // because the work can blow the loop budget. VisionIOInputs carries PoseObservation and
    // TargetObservation records, so this test is where they are first seen — and the
    // CommandScheduler is a JVM-wide singleton, so whichever test class ran first may have left
    // the DriverStation enabled. On a real robot the schema is built during disabled, before
    // the match starts; saying so here makes the test match the robot rather than silencing a
    // warning that would be real.
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void restoreRobotState() {
    // RobotState is a singleton shared with every other test class. Leaving the robot "spinning"
    // would make the angular-velocity pre-filter reject everything in whatever runs next.
    setYawRate(0.0);
    RobotState.getInstance().setPoseSupplier(Pose2d::new);
  }

  // ============================================================================================
  // ACCEPTED
  // ============================================================================================

  @Test
  void acceptsAGoodMultiTagObservation() {
    assertAccepted(observation(GOOD_POSE, 0.0, 2, 2.0));
  }

  @Test
  void acceptsAGoodSingleTagObservation() {
    assertAccepted(observation(GOOD_POSE, 0.0, 1, 2.0));
  }

  @Test
  void acceptsAnAmbiguousSolveWhenMultipleTagsAreVisible() {
    // The ambiguity filter is guarded on tagCount == 1 on purpose: ambiguity is the PnP solver
    // reporting it could not choose between two solutions, which only happens with one tag. A
    // multi-tag solve reporting high ambiguity is not the same failure and must not be dropped.
    assertAccepted(observation(GOOD_POSE, VisionConstants.maxAmbiguity + 0.2, 2, 2.0));
  }

  @Test
  void acceptsAFarMultiTagObservationThatASingleTagSolveWouldFail() {
    // Between the single-tag limit and the overall limit: rejected with one tag, kept with two.
    double distance =
        midpoint(VisionConstants.maxSingleTagDistanceMeters, VisionConstants.maxDistanceMeters);
    assertAccepted(observation(GOOD_POSE, 0.0, 2, distance));
  }

  // ============================================================================================
  // REJECTED
  // ============================================================================================

  @Test
  void rejectsObservationsWithNoTags() {
    assertRejected(observation(GOOD_POSE, 0.0, 0, 2.0));
  }

  @Test
  void rejectsNonPositiveTimestamps() {
    assertRejected(
        new PoseObservation(0.0, GOOD_POSE, 0.0, 2, 2.0, PoseObservationType.PHOTONVISION));
  }

  @Test
  void rejectsAmbiguousSingleTagSolves() {
    assertRejected(observation(GOOD_POSE, VisionConstants.maxAmbiguity + 0.01, 1, 2.0));
  }

  @Test
  void rejectsPosesBelowTheFloor() {
    Pose3d underground = translated(GOOD_POSE, 0.0, 0.0, -VisionConstants.floorError - 0.1);
    assertRejected(observation(underground, 0.0, 2, 2.0));
  }

  @Test
  void rejectsPosesFloatingAboveTheField() {
    Pose3d airborne = translated(GOOD_POSE, 0.0, 0.0, VisionConstants.maxZError + 0.1);
    assertRejected(observation(airborne, 0.0, 2, 2.0));
  }

  @Test
  void rejectsTagsBeyondTheMaximumDistance() {
    assertRejected(observation(GOOD_POSE, 0.0, 2, VisionConstants.maxDistanceMeters + 0.1));
  }

  @Test
  void rejectsFarSingleTagSolves() {
    // Inside the overall distance limit but outside the tighter single-tag one. This filter was
    // added because match-log analysis found single-tag solves flipping their PnP solution at
    // ~3.8 m with near-zero reported ambiguity — the ambiguity filter alone does not catch it.
    double distance =
        midpoint(VisionConstants.maxSingleTagDistanceMeters, VisionConstants.maxDistanceMeters);
    assertRejected(observation(GOOD_POSE, 0.0, 1, distance));
  }

  @Test
  void rejectsImplausiblePitch() {
    Pose3d pitched =
        new Pose3d(
            GOOD_POSE.getTranslation(),
            new Rotation3d(0.0, VisionConstants.maxPitchRollRadians + 0.05, 0.0));
    assertRejected(observation(pitched, 0.0, 2, 2.0));
  }

  @Test
  void rejectsImplausibleRoll() {
    Pose3d rolled =
        new Pose3d(
            GOOD_POSE.getTranslation(),
            new Rotation3d(VisionConstants.maxPitchRollRadians + 0.05, 0.0, 0.0));
    assertRejected(observation(rolled, 0.0, 2, 2.0));
  }

  @Test
  void rejectsPosesBeyondTheFarEndOfTheField() {
    Pose3d offField =
        translated(GOOD_POSE, VisionConstants.aprilTagLayout.getFieldLength(), 0.0, 0.0);
    assertRejected(observation(offField, 0.0, 2, 2.0));
  }

  @Test
  void rejectsPosesWithNegativeCoordinates() {
    Pose3d offField = translated(GOOD_POSE, -GOOD_POSE.getX() - 1.0, 0.0, 0.0);
    assertRejected(observation(offField, 0.0, 2, 2.0));
  }

  @Test
  void rejectsEverythingWhileTheRobotIsSpinningFast() {
    // A pre-filter, not a per-observation one: while the robot spins past the threshold, motion
    // blur and timestamp misalignment make every camera unreliable, so an otherwise perfect
    // observation is still dropped.
    setYawRate(VisionConstants.maxAngularVelocityRadPerSec + 1.0);
    assertRejected(observation(GOOD_POSE, 0.0, 2, 2.0));
  }

  @Test
  void keepsAcceptingWhileSpinningBelowTheThreshold() {
    setYawRate(VisionConstants.maxAngularVelocityRadPerSec - 1.0);
    assertAccepted(observation(GOOD_POSE, 0.0, 2, 2.0));
  }

  // ============================================================================================
  // HARNESS
  // ============================================================================================

  private static void assertAccepted(PoseObservation observation) {
    List<Pose2d> accepted = periodicWith(observation);
    assertEquals(
        1,
        accepted.size(),
        "Expected this observation to reach the pose estimator, but Vision rejected it. Check the"
            + " rejection ladder in Vision.periodic() — the RejectionReason it logs names the"
            + " filter that fired.");
    assertEquals(observation.pose().toPose2d(), accepted.get(0));
  }

  private static void assertRejected(PoseObservation observation) {
    List<Pose2d> accepted = periodicWith(observation);
    assertTrue(
        accepted.isEmpty(),
        () ->
            "Expected this observation to be rejected, but it reached the pose estimator: "
                + observation
                + "\n\nA filter that stopped firing does not raise an error — it feeds a bad pose"
                + " into the estimator, which shows up as the robot teleporting mid-match.");
  }

  /**
   * Runs one {@code Vision.periodic()} with a single scripted observation and returns the poses
   * handed to the consumer.
   */
  private static List<Pose2d> periodicWith(PoseObservation observation) {
    List<Pose2d> accepted = new ArrayList<>();
    Vision vision =
        new Vision(
            (pose, timestamp, stdDevs) -> accepted.add(pose),
            new FakeVisionIO(new PoseObservation[] {observation}));

    // periodic() is driven directly, so this instance has no business in the scheduler.
    CommandScheduler.getInstance().unregisterSubsystem(vision);

    vision.periodic();
    return accepted;
  }

  /** An observation at the given pose, with everything else within limits unless overridden. */
  private static PoseObservation observation(
      Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {
    return new PoseObservation(
        GOOD_TIMESTAMP,
        pose,
        ambiguity,
        tagCount,
        averageTagDistance,
        PoseObservationType.PHOTONVISION);
  }

  private static Pose3d translated(Pose3d pose, double dx, double dy, double dz) {
    return new Pose3d(pose.getX() + dx, pose.getY() + dy, pose.getZ() + dz, pose.getRotation());
  }

  private static double midpoint(double low, double high) {
    return (low + high) / 2.0;
  }

  /**
   * Sets the yaw rate {@code Vision} reads through {@link RobotState}.
   *
   * <p>{@code RobotState} derives velocity from module states rather than exposing a setter, so the
   * rate is converted to module states with the same kinematics it uses to convert them back.
   */
  private static void setYawRate(double radiansPerSecond) {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
    RobotState.getInstance()
        .updateModuleStates(
            kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, radiansPerSecond)));
  }

  /** Replays a fixed set of observations, standing in for a camera. */
  private record FakeVisionIO(PoseObservation[] observations) implements VisionIO {
    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected = true;
      inputs.hasCalibration = true;
      inputs.poseObservations = observations;
    }
  }
}
