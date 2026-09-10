# Prompt: Add a New Subsystem

Use this prompt when scaffolding a new mechanism subsystem from scratch.
Copy and fill in the bracketed values before sending.

> **There is also a `/add-subsystem` skill**, which runs the same process without the
> copy-paste and asks for anything you leave out. Use this form when you would rather
> state everything up front, or when you want to review the request before sending it.

---

```
Add a new subsystem for [MECHANISM NAME].

Hardware:
- Motor model: [Kraken X60 / Kraken X44], count: [N]
- Motor: [TalonFX / other] CAN ID [XX] on ["rio" / "Canivore"] bus
- [Additional motor if follower: TalonFX CAN ID XX, opposes leader: yes/no]
- Gear ratio: [XX]:1 (motor rotations per mechanism rotation)
- Mechanism type: [rotating / linear]
- [Linear only: drum or sprocket PITCH diameter XX in, rigging stage count N]
- [Encoder if position-controlled: CANcoder CAN ID XX on [bus]]
- Encoder location: [motor encoder only / CANcoder on the mechanism shaft /
  CANcoder on an intermediate shaft — if so, give the gearing above AND below it]
- Control mode: [VoltageOut / VelocityTorqueCurrentFOC / MotionMagicTorqueCurrentFOC]

What it needs to do:
- [Action 1, e.g.: run at a set voltage]
- [Action 2, e.g.: run at a target velocity]
- [Action 3 if applicable]

Logged signals needed:
- Voltage, supply current, stator current, velocity, temperature (standard set)
- Torque current (REQUIRED if the control mode above is any *TorqueCurrentFOC —
  it is the control signal, and stator current is not a substitute)
- [Any additional: position, closed-loop reference/error]

State queries needed (for Triggers or commands):
- [e.g.: isAtVelocity() — true when within 200 RPM of target]

Setpoints (what it gets commanded to) — velocities in RPM, rotating positions in
degrees, linear positions in inches:
- [e.g.: intake velocity 1800 RPM, unjam velocity -1200 RPM]
- [e.g.: stow height 0 in, score height 24 in]
- [e.g.: stow angle 0 deg, deployed angle 95 deg]
- Put these in Constants.Setpoints, not in the subsystem constants file and not
  in RobotContainer. Guess reasonable starting values and mark them as unmeasured.

Build it on frc.lib.mechanism — do not write a new IO layer. Create:
1. subsystems/[name]/[Name].java — extends RollerSubsystem, RotarySubsystem or
   LinearSubsystem depending on the kind
2. subsystems/[name]/[Name]Constants.java — NAME, the MotorConfig, the settings,
   the sim model

Add a commands/[Name]Commands.java ONLY if this mechanism has verbs that
RollerCommands / RotaryCommands / LinearCommands do not already cover.

Wire the real and sim implementations in RobotContainer under the existing
real/sim/replay switch — all three branches — and call registerFaultMonitors(). Add the CAN IDs to Constants.CanIds,
with a // CANivore or // RIO CAN comment on each line, and the setpoints above to
Constants.Setpoints. RobotContainer should end up with no bare numbers in it.
Run ./gradlew build before reporting complete, and ./gradlew simulateJava if you
claim the mechanism works end to end.
```

---

## What Claude Will Do

1. Create the two files from the matching scaffold in `template/src/` — `exampleRoller`
   if it spins, `exampleArm` if it pivots to an angle, `exampleLift` if it travels in a
   line. The IO layer, simulation, visualizer and common commands come from
   `frc/lib/mechanism/`; there is nothing to copy for those
2. Add the CAN IDs to `Constants.CanIds`, with the bus in a comment on each line
3. Add the setpoints to `Constants.Setpoints` — never inline in `RobotContainer`,
   never as a private field there, never in the subsystem's own constants file
4. Add real/sim/replay wiring to `RobotContainer`, and call `registerFaultMonitors()`
5. Provide a compile check

## What Claude Will Not Do Without Asking

- Change existing subsystem code
- Change button bindings
- Add this subsystem to auto sequences
- Change CAN IDs of existing devices
