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
- Motor: [TalonFX / other] CAN ID [XX] on ["rio" / "Canivore"] bus
- [Additional motor if follower: TalonFX CAN ID XX, opposes leader: yes/no]
- [Encoder if position-controlled: CANcoder CAN ID XX on [bus]]
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

Setpoints (what it gets commanded to):
- [e.g.: intake velocity 30 rot/s, unjam velocity -20 rot/s]
- [e.g.: stow height 0 in, score height 24 in]
- Put these in Constants.Setpoints, not in the subsystem constants file and not
  in RobotContainer. Guess reasonable starting values and mark them as unmeasured.

Follow the AdvantageKit IO pattern used by the vision subsystem
(subsystems/vision/Vision.java + io/VisionIO.java + io/VisionIOPhotonVision.java).
Create:
1. subsystems/[name]/io/[Name]IO.java
2. subsystems/[name]/io/[Name]IOReal.java
3. subsystems/[name]/io/[Name]IOSim.java
4. subsystems/[name]/[Name].java
5. subsystems/[name]/[Name]Constants.java
6. commands/[Name]Commands.java

Wire the real and sim implementations in RobotContainer under the existing
real/sim/replay switch — all three branches. Add the CAN IDs to Constants.CanIds,
with a // CANivore or // RIO CAN comment on each line, and the setpoints above to
Constants.Setpoints. RobotContainer should end up with no bare numbers in it.
Run ./gradlew compileJava before reporting complete.
```

---

## What Claude Will Do

1. Create the six files using the IO pattern from the vision subsystem
2. Add the CAN IDs to `Constants.CanIds`, with the bus in a comment on each line
3. Add the setpoints to `Constants.Setpoints` — never inline in `RobotContainer`,
   never as a private field there, never in the subsystem's own constants file
4. Add real/sim/replay wiring to `RobotContainer`
5. Provide a compile check

## What Claude Will Not Do Without Asking

- Change existing subsystem code
- Change button bindings
- Add this subsystem to auto sequences
- Change CAN IDs of existing devices
