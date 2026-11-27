"""Empirical robotToCamera from one stationary robot facing one AprilTag.

Adapted from neia-robotics/2026-Robot-Code (calculateRobotToCameraTransform.py).
Needs: pip install robotpy-wpimath

Setup: robot square to a mounted tag, robot center on the tag center (so --robot-y is 0).
Measure --robot-x (robot center to tag face) and --robot-z (tag center to floor, NEGATIVE:
the robot is below the tag). In AdvantageScope, take the 30 s+ MEDIAN of PhotonVision's
targetPose translation (x y z) and quaternion (w x y z) for the camera being calibrated.

Example:
  python robot_to_camera.py --robot-x 1.07 --robot-z -1.124 --yaw -90 \
      --cam-xyz 0.922 0.423 0.422 --cam-quat 0.375 -0.149 0.083 -0.911

Prints a Transform3d (meters, radians) as a PROPOSAL. Do not paste it into VisionConstants
without the confirmation /camera-calibrate requires. Repeat per camera.
"""

import argparse

from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Transform3d, Translation3d

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--robot-x", type=float, required=True, help="m, robot center to tag face")
p.add_argument("--robot-y", type=float, default=0.0, help="m, robot center to tag center (0 if aligned)")
p.add_argument("--robot-z", type=float, required=True, help="m, tag center to floor (negative)")
p.add_argument("--yaw", type=float, required=True, help="deg, robot facing tag = 180, rear to tag = 0, sides = +/-90")
p.add_argument("--cam-xyz", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"), help="median targetPose translation")
p.add_argument("--cam-quat", type=float, nargs=4, required=True, metavar=("W", "X", "Y", "Z"), help="median targetPose quaternion")
a = p.parse_args()

target_to_robot = Transform3d(
    Translation3d(a.robot_x, a.robot_y, a.robot_z), Rotation3d.fromDegrees(0, 0, a.yaw)
)
target_to_camera = Transform3d(Translation3d(*a.cam_xyz), Rotation3d(Quaternion(*a.cam_quat)))

robot_to_camera = Pose3d().transformBy(target_to_camera.inverse()) - Pose3d().transformBy(target_to_robot)
print(str(robot_to_camera).replace("x=", "").replace("y=", "").replace("z=", ""))
