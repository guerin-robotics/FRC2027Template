---
name: debug-match-log
description: Analyze an AdvantageKit match log (.wpilog/.hoot) to root-cause a field problem. Use when something went wrong on the robot during a match or practice and a log file exists. Correlates signals around the anomaly, tests hypotheses against the log, and sets up deterministic replay.
---

# Debug a Match Log

You are analyzing an AdvantageKit log to find the root cause of an on-field
problem. This is a **read-only** investigation — do not modify robot code
unless the user explicitly asks for a fix afterward.

## Step 1 — Gather the facts

If the user hasn't provided them, ask for:

1. **Log file path** (`.wpilog` or `.hoot` — usually from the USB stick or `/U/logs`)
2. **What went wrong** — what the robot did vs. what it should have done, as specifically as possible
3. **When it happened** — match time, or an event ("3rd auto shot", "first teleop shoot press")
4. **What the human saw** — physical behavior: directions, oscillation frequency, sounds, anything

Remember: the log has the data; the human has the physical context. You need both.

## Step 2 — Form hypotheses, then check signals

For each plausible cause, state **what the log would show if this hypothesis
were true**, then check. Work through hypotheses one at a time and report
which signals confirm or eliminate each.

### Key signal map for common problems

**A mechanism didn't act when the button was pressed**
```
Triggers/[gatingTrigger]           — was the composite trigger true?
[Mechanism]/closedLoopReference    — was it targeting a setpoint?
[Mechanism]/isReady                — did the readiness condition pass?
Commands/Active                    — was the command actually scheduled?
```

**Robot aligned wrong direction**
```
AutoAim/TargetAngle                — what heading was being targeted?
AutoAim/CurrentAngle               — what heading was measured?
AutoAim/AngleErrorRad              — did the controller converge?
RobotState/EstimatedPose           — was the pose correct?
Odometry/Robot                     — field visualization
Vision/Summary/RobotPosesAccepted  — were vision updates being applied?
```

**Auto ran wrong path / started from wrong position**
```
Odometry/Trajectory                — what path was being followed?
Odometry/TrajectorySetpoint        — where PathPlanner wanted the robot
Odometry/Robot                     — where the robot actually was
```

**Vision caused pose jump**
```
Vision/Camera[N]/RobotPosesAccepted — which camera sent a pose?
Vision/Camera[N]/RobotPosesRejected — what was rejected?
Vision/Camera[N]/RejectionReason    — why was it rejected?
Vision/Camera[N]/Ambiguity          — single-tag PnP ambiguity
Vision/Camera[N]/AverageTagDistance — how far were the tags?
Vision/Camera[N]/TagCount           — single-tag or multi-tag solve?
RobotState/FieldRelativeVelocity    — was the robot spinning at the jump?
```

Single-tag solves at long range were the cause of every catastrophic pose error
found in the 2026 logs. Check `TagCount` and `AverageTagDistance` together
before blaming ambiguity.

**Brownout during match**
```
PowerDistribution/Voltage          — when did voltage drop below 7 V?
BatteryLogger/[Subsystem]/Current  — which subsystem drew the most current?
Drive/Module[0-3]-Drive            — per-module drive current
Drive/Module[0-3]-Turn             — per-module steer current
[Subsystem]/StatorAmps             — stall detection per subsystem
```

**Loop overrun**
```
LoggedRobot/FullCycleMS            — total loop time vs the 20 ms budget
Drive/...                          — Drive and Vision dominated in 2026
```

Also always check `Commands/Active` around the event time — a missing or
unexpectedly-interrupted command explains most "it just didn't do it" reports.

## Step 3 — Report

Deliver:
1. **Root cause** (or the shortlist, ranked, with the evidence for each)
2. **The evidence** — which signals showed what, at what timestamps
3. **What was ruled out** and why
4. **Suggested fix** as a description only — do not implement it in this
   session unless asked. If the fix touches gains, thresholds, timeouts, or
   vision filters, name the risk tier and the failure mode per the rules.
5. **Regression-test note** — once a fix lands, a test must lock the bug in
   (see `.claude/prompts/write-test.md`).

## Replay: adding instrumentation

If the existing signals aren't enough to decide between hypotheses, use
deterministic replay to add new outputs:

1. Set `Constants.getMode()` to `REPLAY`
2. Point `WPILOGReader` at the log file
3. Add `Logger.recordOutput()` calls for the values you need
4. Run `./gradlew simulateJava`
5. Open AdvantageScope and connect to the replay output

Same inputs, same code path, same bug, every time — with new eyes on it.
