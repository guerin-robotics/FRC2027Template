# Devices — `frc/lib/device`

Sensors and outputs that are not motors. Each has the same shape (see
[library.md](library.md)): a wrapper the subsystem owns, an IO interface, a real IO and a sim IO.

| Kind | Wrapper | Real IOs | Sim IO |
|---|---|---|---|
| Absolute encoder | `AbsoluteEncoder` | `AbsoluteEncoderIOCANcoder` | `AbsoluteEncoderIOSim` (angle supplier) |
| Distance sensor | `DistanceSensor` | `DistanceSensorIOCANrange`, `DistanceSensorIOLaserCAN` | `DistanceSensorIOSim` (optional-distance supplier) |
| Beam break | `BeamBreak` | `BeamBreakIODigitalInput`, `BeamBreakIOLaserCAN` | `BeamBreakIOSim` (boolean supplier) |
| Lights | `Lights` | `LightsIOCANdle` | `LightsIOSim` |

---

## Wiring a device into a subsystem

The subsystem takes the wrapper (or its IO) in its constructor and calls `periodic()`:

```java
public class Indexer extends SubsystemBase {
  private final BeamBreak staged;

  public Indexer(BeamBreakIO beamBreakIO) {
    staged = new BeamBreak("Indexer/Staged", beamBreakIO);
  }

  @Override
  public void periodic() {
    staged.periodic(); // NEVER skip — it is what logs the inputs for replay
  }

  public boolean hasPiece() {
    return staged.isBroken();
  }
}
```

`RobotContainer` picks the IO for each mode, the same three-way switch drive and vision use:

```java
// REAL
indexer = new Indexer(new BeamBreakIODigitalInput(Constants.DioPorts.INDEXER_BEAM_BREAK, false));
// SIM
indexer = new Indexer(new BeamBreakIOSim(() -> RobotState.getInstance().hasPieceInIndexer()));
// REPLAY
indexer = new Indexer(new BeamBreakIO() {});
```

Then, once:

```java
indexer.registerFaultMonitors(); // forwards to staged.registerFaultMonitors()
```

Put the trigger in `Triggers.java`, named for what it means:

```java
public Trigger pieceStaged() { return new Trigger(indexer::hasPiece); }
```

(Triggers that read a subsystem need the subsystem handed to `Triggers` once at wiring time —
the same way any state trigger there is built.)

Every ID goes in `Constants.CanIds` or `Constants.DioPorts` with a bus comment, and in
[hardware-layout.md](hardware-layout.md). The scaffolds and `/add-subsystem` follow the same
rule.

---

## Per kind

### Absolute encoder (CANcoder)

For an encoder that is **not** a motor's feedback sensor: a passive hinge, a turret's second gear,
anything without a motor of its own. If the encoder belongs to a motor, use
`MotorConfig.Builder.encoder(...)` instead. Fused into the TalonFX, the motor closes its loop on it.

Pass a complete `CANcoderConfiguration` (magnet offset, direction, discontinuity point). It is
applied once. A red magnet (`magnetHealth == 1`) raises a fault: the readings look plausible and
are wrong.

### Distance sensor (CANrange, LaserCAN)

`getDistance()` returns `Optional<Distance>`, which is empty whenever the reading is not valid.
There are no sentinel values like −1. `connected` and `valid` are separate:

| `connected` | `valid` | Meaning |
|---|---|---|
| false | false | Off the bus, or no measurement yet |
| true | false | Working, but nothing in range or too much ambient light |
| true | true | Trust `distance` |

- **CANrange:** `valid` is the sensor's own detection flag, so the threshold and hysteresis live
  in `config.ProximityParams`.
- **LaserCAN:** ranging mode, region of interest and timing budget are set once at construction,
  with retry. Use `SHORT` mode under ~1.3 m; it is far less sensitive to ambient light.
  LaserCAN has no bus argument and always sits on `"rio"`, where its ID can collide with a
  TalonFX. `CanIdRegistry` checks that.

### Beam break (DIO, LaserCAN)

`isBroken()` and `broken()` (a `Trigger`).

- **DIO:** set `brokenWhenHigh` from a bench test with the sensor's LED, not from the datasheet.
  A DIO input cannot tell "unplugged" from "clear" and always reports connected.
- **LaserCAN:** broken when a *valid* reading is closer than the threshold. An invalid reading
  reads as clear, never broken. Measure the threshold with a game piece in place and without one,
  and set it between the two with margin.

### Lights (CANdle)

`off()`, `setSolid(Color)`, `setStrobe(Color)`, `setRainbow()`. Requests are forwarded only when
they change, so a command can set the same colour every loop. They are re-sent when the CANdle
reconnects. What was requested is logged as `<name>/Mode` and `<name>/Color`.

`ledCount` includes the CANdle's 8 onboard LEDs, so a 30-LED strip is `38`.

---

## Not built yet

W8-Library also has servo, object-detection and custom-coprocessor vision devices. They were
left out until a robot needs them. Add them with the same shape, per
[library.md](library.md#adding-to-the-library).
