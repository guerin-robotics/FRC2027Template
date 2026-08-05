# Prompt: Update Tuning Constants

Use this prompt when you have measured new values and want to update constants safely.
Fill in all bracketed values. Do not use this prompt to guess at values.

---

```
Update tuning constants for [SUBSYSTEM NAME].

I have measured/determined the following new values:

[Constant name]: [old value] → [new value]
Reason: [why this value was determined — test data, calculation, etc.]

[Repeat for each constant]

Rules:
- Change only the constants listed above
- Do not change any other constants in the same file
- Do not refactor the constants file
- Run ./gradlew compileJava after the change
- Report the exact lines changed
```

---

## Safety Notes for Tuning Changes

**Units come first.** Check the mechanism's `ClosedLoopOutputType`. Under
`TorqueCurrentFOC` (both drive and steer here) gains are in **amps**; under `Voltage`
they are volts. A value that looks wrong is usually a value being read in the wrong
unit. See [characterization-and-tuning.md](../../docs/characterization-and-tuning.md).

**PID gains (Kp, Ki, Kd):** State the expected behavior change.
A higher Kp increases stiffness but risks oscillation. Test at low speed first.

**Feedforward (Ks, Kv, Ka):** Ks is static friction (A), Kv is velocity constant (A/rps).
Wrong Ks causes the motor to not break static friction. Wrong Kv causes steady-state error.

**Current limits:** Raising supply or stator limits risks brownout or motor damage.
Always state why the old limit was insufficient before raising it.

**Vision thresholds:**
- Lowering `maxAmbiguity` rejects more tags (fewer false poses, more pose gaps)
- Raising `maxAngularVelocity` accepts poses while spinning (may degrade accuracy)
- Changing std dev scaling directly affects auto accuracy

**Timeout constants:**
- Lowering `alignmentTimeoutSeconds` means the robot fires sooner without alignment
- Raising it means the robot waits longer before giving up
- Both have match-outcome consequences

---

## Common Constant Locations

| Constant type | File |
|---|---|
| Swerve CAN IDs, gains, geometry, current limits | `generated/TunerConstants` (generated — see hardware rules) |
| Other CAN IDs | `Constants.CanIds` |
| Vision thresholds | `subsystems/vision/VisionConstants` |
| Heading-hold gains | `commands/DriveCommands` (ANGLE_KP / ANGLE_KD) |
| Path-following gains | `subsystems/drive/Drive` (`AutoBuilder.configure`) |
| Joystick shaping, limited speed | `subsystems/drive/DriveConstants` |
| Field dimensions, AprilTag layout | `frc/lib/FieldConstants` |
| PID/FF gains (mechanisms) | `[Subsystem]Constants` or IO implementation |
| Setpoints, timeouts, tolerances | `Constants.Setpoints` / `.Waits` / `.Thresholds` — never inline |
