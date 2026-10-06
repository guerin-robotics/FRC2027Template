# The Library — `frc/lib`

Everything reusable lives in `frc/lib`. A season robot repo is generated from this template and
adds only what is specific to that robot: subsystems built on these pieces, commands, autos, and
constants.

The shape is modelled on FRC 3467's [W8-Library](https://github.com/WHS-FRC-3467/W8-Library):
a library of hardware-abstracted building blocks with a working robot on top. Theirs is
GPL-3.0, so nothing was copied. The ideas were rebuilt to fit this repo's rules (static command
factories, controllers in `Triggers.java`, one pose estimator) and its safety layer (the
`MotorConfig` builder, rate-split status signals, fault monitoring). See
[third-party-code.md](third-party-code.md).

---

## What is in it

| Package | What it gives you | Read |
|---|---|---|
| `frc/lib/mechanism/` | Motor-driven mechanisms: `roller/` (velocity), `rotary/` (angle), `linear/` (height). Shared `MotorIO`, the `MotorConfig` safety builder, Phoenix-backed sim, visualizers, command factories | [template/GUIDE.md](../template/GUIDE.md), [new-mechanism-bringup.md](new-mechanism-bringup.md) |
| `frc/lib/device/` | Sensors and outputs with no motor: `absoluteencoder/` (CANcoder), `distance/` (CANrange, LaserCAN), `beambreak/` (DIO, LaserCAN), `lights/` (CANdle). Plus `CanIdRegistry`, the runtime duplicate-ID check | [devices.md](devices.md) |
| `frc/lib/auto/` | Point-to-point autos as data: `Waypoint`, `WaypointAuto` | [autos.md](autos.md) |
| `frc/lib/command/` | Command infrastructure: `SteppableCommandGroup` for walking an auto through in the pit | [autos.md](autos.md) |
| `frc/lib/util/` | Logging and monitoring (`BatteryLogger`, `FaultMonitor`, `LoopTimeMonitor`, `CANBusMonitor`, `CommandLogger`, `PhoenixSignalLogger`), tunables, `AllianceFlipUtil`, geometry helpers | [subsystem-ownership.md](subsystem-ownership.md), [tunables.md](tunables.md) |

Drive and vision stay in `frc/robot/subsystems/`. They are the reference implementations of the
IO pattern, but each robot only ever has one of each, so neither is a library type.

---

## The one pattern everything follows

Every piece of hardware sits behind an IO interface, with three implementations:

| Mode | Implementation | Example |
|---|---|---|
| `REAL` | Talks to the device | `DistanceSensorIOCANrange` |
| `SIM` | Simulated or supplied values | `DistanceSensorIOSim` |
| `REPLAY` | Does nothing; AdvantageKit fills the inputs from the log | `new DistanceSensorIO() {}` |

A **wrapper** (`DistanceSensor`, `RollerMechanism`, …) owns the IO and its `@AutoLog` inputs.
Its `periodic()` calls `io.updateInputs` and `Logger.processInputs`. Wrappers are **not**
subsystems: a subsystem owns one or more of them and calls each wrapper's `periodic()` from its
own. One intake can then own a roller, a pivot and a beam break as a single subsystem.

Every real IO for a CAN device:

- claims its ID with `CanIdRegistry.claim(...)` before constructing the device;
- applies its whole configuration once, with retry;
- caches its status signals, rate-splits them per `.claude/rules/02-hardware.md`, and calls
  `optimizeBusUtilization()` last;
- derives `connected` from its fast signals alone, with a 0.5 s falling debounce.

Every wrapper has `registerFaultMonitors()`. Call it once from `RobotContainer`, and the
device's health rolls into the pit's single "Robot OK" boolean.

---

## Adding to the library

Add a kind here only when it will be used by more than one subsystem or more than one season.
A one-off belongs in the robot. When you do add one:

1. Copy the shape of the nearest existing kind: wrapper, IO, real IO, sim IO.
2. Claim the CAN ID; follow the signal-rate rules.
3. Add a row to the table above and a section to [devices.md](devices.md).
4. Record any borrowed code in [third-party-code.md](third-party-code.md) **before** it is
   committed.
