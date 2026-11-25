---
name: camera-calibrate
description: Measure and verify the robotToCameraN transforms in VisionConstants, using logged tag observations. Use when a camera is added, moved or remounted, when vision poses are offset or swim with distance, or at season start when the transforms are still identity placeholders. Proposes values from evidence; the user confirms before anything is written.
---

# Camera Calibration

Vision pose estimates are only as good as `robotToCameraN` in `subsystems/vision/VisionConstants.java`.
A 1 inch error in position is a 1 inch error in every pose that camera produces, and a 1 degree yaw
error grows with tag distance. The template ships **identity placeholders** — until this skill has
been run on the real robot, vision output is wrong.

**Rules for this skill**

- Changing a transform is a **High risk** change (it moves vision-fused pose, which drives auto and
  alignment). Propose; the user confirms. Do not touch the filter thresholds in `VisionConstants` —
  they are geometry-independent and were tuned on real 2026 logs.
- Never edit the AprilTag layout here. `FieldConstants.aprilTagLayout` is the single source.
- Camera count must match the `VisionIO` instances in `RobotContainer` for all three modes.

## Step 1 — Establish what is being calibrated

Ask for, or read:

1. Which cameras (`camera0Name` ... `camera3Name`) and whether their names match the coprocessor.
2. Current `robotToCameraN` values (placeholder identity = nothing has been measured yet).
3. A log or live session where the robot sat **still** with tags in view. Drive-by data is not
   usable for this: motion adds latency and odometry error that look like a transform error.

## Step 2 — Physical measurement first (the starting guess)

The transform is robot center to camera lens: X forward, Y left, Z up, then roll/pitch/yaw.
In WPILib a camera tilted **upward has NEGATIVE pitch**.

Have a human measure from the robot origin (center of the frame, floor level) to the lens with a
tape measure, and the mounting angles with a level or angle finder. Record each in inches, then
convert in code with `Units.inchesToMeters` / `Units.degreesToRadians`. Write the starting guess
down before refining it so the refinement can be judged against it.

## Step 3 — Refine from logged observations

With the robot stationary at a **known pose** (taped on the floor, a measured distance and angle
from a tag), read these per-camera channels (`Vision/CameraN/...`):

| Channel | Use |
|---|---|
| `TagPoses`, `RobotPoses` | Where the camera thinks the tag and robot are |
| `TagIds`, `TagCount`, `IsMultiTag` | Prefer multi-tag frames — they constrain the transform best |
| `AverageTagDistance` | Error that grows with distance is rotation; constant error is translation |
| `Ambiguity` | Discard high-ambiguity single-tag frames |
| `RejectionReason` | A camera with a bad transform is rejected by the filters — see which |
| `HasCalibration` | False means lens intrinsics are missing; fix that before the extrinsics |

Diagnose by pattern, then adjust **one axis at a time**:

| Symptom in the log | Likely cause |
|---|---|
| Robot pose offset by a constant amount in the robot frame | X/Y/Z translation error |
| Offset that grows as the robot moves back from the tag | Yaw or pitch error |
| Two cameras seeing the same tag disagree on the robot pose | One camera's transform is wrong — compare each to a multi-tag solution |
| Pose jumps when the tag set changes | A single camera is wrong while the multi-tag solve is right |
| Heading wrong but position right | Yaw (or Pigeon mount calibration — check that first) |

Check the Pigeon 2 mount calibration before blaming a camera — a tilted gyro mimics a yaw error.
See `docs/characterization-and-tuning.md`.

## Step 4 — Verify in sim and replay

- Sim: confirm the camera count and the transforms are consumed in all three modes.
- Replay: run the log against the new transform (`.agent/skills/replay-testing.md`) and compare
  `Vision/Summary/RobotPoses` against the known pose. Report the error before and after, per camera,
  at two or more distances.

## Step 5 — Propose, do not write

Output a table: camera, old transform, proposed transform, evidence (error before/after, distances,
frame counts). State the behavioral consequence ("vision-fused pose shifts by about X cm at Y m")
and the failure mode if wrong ("pose snaps during auto, robot drives to the wrong place").
Write the constants only after explicit confirmation.

## What the human must still do

Sim cannot validate a transform — sim cameras are placed by the same constants, so it agrees with
itself. The real check is a stationary robot at a measured pose with a real camera. Tell the user
that, and that a calibration done on blocks or with a loose camera mount is wrong the moment the
mount flexes. Re-check after any hit that could have bent a mount.

## Related

- `docs/VISION_DEBUG_CHECKLIST.md` — camera names (§2) and transforms (§5)
- `docs/skills.md` — how to run this skill in sim and on the robot
