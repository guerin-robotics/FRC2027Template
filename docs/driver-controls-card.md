# Driver Controls — Quick Refresh

> **Branch `feature/rebuilt2026-port`: the 2026 robot, as this port binds it.** Carried from
> the Rebuilt2026 card (27 July 2026) and corrected against the Rebuilt2026 `main` code,
> which changed after the card was written: flightstick 8 became the tower odometry reset
> (bump align is no longer bound), pass moved to 9, and D-pad down was added.

Two ways to drive the robot. Someone picks the mode on the dashboard before you enable;
find your mode below and ignore the other one.

```
PLUG-IN CHECK        Port 1  Xbox controller
                     Port 2  Flightstick

If your controller does nothing at all, this is almost always why.
Check the Driver Station USB tab before anything else.

PICKING THE MODE     Dashboard -> "Drive controller" -> Thrustmaster  or  Xbox

It only takes effect at the next ENABLE. Change it mid-match and nothing
happens. After a code redeploy it goes back to Thrustmaster on its own.
```

---

## MODE 1 — FLIGHTSTICK DRIVES  (the usual)

You hold the flightstick. Someone else holds the Xbox as a backup controller.

```
THRUSTMASTER T.16000M
────────────────────────────────────────────────────────────────────────────────

                        STICK HEAD  (looking down)

                          ╭─────────────╮
                         ╱     ╭───╮     ╲
                        │      │ ✛ │      │      the HAT does nothing
         INTAKE IN ─────┤ (3)  ╰───╯  (4) ├───── INTAKE OUT
           pivot up     │                 │        pivot down
                         ╲     (2)       ╱
                          ╰──────┬───────╯        (2) = TRENCH ALIGN
                                 │
                            ┌────┴────┐
                            │   (1)   │
                            └────┬────┘
                                 └───── SHOOT ★   trigger, front of grip

            push / pull / left / right = DRIVE        twist = TURN


                              BASE  (looking down)

        LEFT SIDE                                       RIGHT SIDE
   ┌────────┬────────┬────────┐                  ┌────────┬────────┬────────┐
   │   5    │   6    │   7    │                  │   11   │   12   │   13   │
   │ INTAKE │COMPRESS│ demo   │                  │        │ CANCEL │        │
   │ ROLLER │        │ shot   │                  │        │ AUTO-X │        │
   ├────────┼────────┼────────┤       ⊙ stick    ├────────┼────────┼────────┤
   │   8    │   9    │   10   │                  │   14   │   15   │   16   │
   │ ODOM   │  PASS  │ TOWER  │                  │        │        │        │
   │ RESET !│        │  SHOT  │                  │        │        │        │
   └────────┴────────┴────────┘                  └────────┴────────┴────────┘

   Blank buttons do nothing. Button 7 only works in practice/demo mode.
```

| Control | What it does |
|---|---|
| Push / pull / left / right | Drive |
| Twist | Turn |
| **Trigger (1)** | **Shoot** — aims, spins up and fires. See *What the trigger does* below |
| 2 | Trench align — locks your heading for the trench |
| 3 | Intake in (pivot up) |
| 4 | Intake out (pivot down) |
| 5 | Intake roller — hold to pull fuel in |
| 6 | Compress by hand |
| 7 | Demo shot — practice and demos only, dead in a match |
| 8 | Odometry reset — tells the robot it is centred in front of our tower. **See warning below** |
| 9 | Pass — fixed pass, no aiming, fires on a timer |
| 10 | Tower shot — fixed shot, no aiming, no waiting |
| 12 | Cancel auto-X (stop the wheels locking up) |
| Hat, 11, 13–16 | Nothing |

**The Xbox does three things in this mode**

| Xbox | What it does |
|---|---|
| A | Flip which alliance the hub is favouring |
| B | Switch compress to double |
| Y | Turn the hub timer off completely |

Tap them — holding does nothing extra. Every other Xbox button is dead.

---

## MODE 2 — XBOX DRIVES

You hold the Xbox. The flightstick sits on the desk as the backup controller.

```
XBOX ELITE SERIES 2
────────────────────────────────────────────────────────────────────────────────

    INTAKE ROLLER ── LT                                RT ── SHOOT ★
   INTAKE IN, up ── LB                                 RB ── INTAKE OUT, down
   ╔═════════════════════════════════════════════════════════════════════════╗
   ║                                                                         ║
   ║      ╭─────╮                                              ( Y )         ║
   ║      │  L  │       (  )   (  )   (  )                                   ║
   ║      ╰─────╯                                        ( X )     ( B )     ║
   ║       DRIVE                                                             ║
   ║                                                           ( A )         ║
   ║          ↑                          ╭─────╮                            ║
   ║        ← ✛ →                        │  R  │             Y  TOWER SHOT  ║
   ║          ↓                          ╰─────╯             X  TRENCH      ║
   ║        D-PAD                          TURN              A  nothing     ║
   ║  ↑ PASS  ↓ ODOM RESET !          left / right           B  nothing     ║
   ║                                                                         ║
   ╚═════════════════════════════════════════════════════════════════════════╝
      Menu, View, the small centre buttons, clicking the sticks and the
      back paddles all do nothing.
```

| Control | What it does |
|---|---|
| Left stick | Drive |
| Right stick, left / right | Turn |
| **Right trigger** | **Shoot** — aims, spins up and fires. See below. Squeeze it properly, a light pull won't register |
| Left trigger | Intake roller — hold to pull fuel in |
| Left bumper | Intake in (pivot up) |
| Right bumper | Intake out (pivot down) |
| X | Trench align |
| Y | Tower shot |
| D-pad Up | Pass |
| D-pad Down | Odometry reset. **See warning below** |
| A, B | Nothing in this mode |
| Everything else | Nothing |

> **First time driving in Xbox mode:** push the left stick gently forward in open space
> before you trust it in a match. If the robot goes backwards, stop and tell a
> programmer — it's a one-line fix, but not one to discover mid-match.

**The flightstick does five things in this mode**

| Flightstick | What it does |
|---|---|
| 2 | Turn the hub timer off completely |
| 3 or 4 | Flip which alliance the hub is favouring (either button, same result) |
| 6 | Compress by hand |
| 7 | Demo shot — practice and demos only |
| 12 | Cancel auto-X |

Everything else on the flightstick is dead while the Xbox is driving — including the
trigger. **Pulling the flightstick trigger in this mode does nothing.**

---

## What the trigger does

One button. The robot decides what it means from where you are on the field.

```
  IN OUR ZONE, hub is ours          ──►   FULL SHOT
                                          Turns to the hub, spins up, sets the
                                          hood, then fires when it's ready and
                                          lined up. Compress happens on its own.

  IN OUR ZONE, hub is NOT ours     ──►   AIMS BUT WON'T FIRE
                                          It will still turn to face the hub and
                                          then just sit there. This is normal.
                                          It is not jammed. Wait for the hub.

  OUTSIDE OUR ZONE                 ──►   PASS
                                          Turns to your alliance partner's spot
                                          and passes instead of shooting.
```

**Hold it.** All three cases need the trigger held down — it aims, waits, then fires.
Let go early and nothing comes out.

---

## Good to know

- **The robot slows to half speed while it's aiming.** That is on purpose. It is not a
  dying battery and it is not a brownout.

- **Touching the intake cancels auto-compress.** If you press intake in, intake out, or
  compress, the robot stops compressing by itself for the rest of that trigger pull. Let
  go of the trigger and it comes back.

- **Tower shot, the pass button and demo shot don't aim.** They fire a fixed shot on a
  timer. Line the robot up yourself first. (Passing with the *trigger* outside our zone
  does aim.)

- **The trigger and trench align steer for you.** You keep translation control, the robot
  takes the heading. Don't fight it with the twist axis or the right stick.

- **Odometry reset takes the drivetrain and does not give it back.** After you press it the
  sticks stop driving the robot. Pull the trigger (or trench align) once to get driving
  back. This is how the 2026 code behaved; it is flagged, not fixed, on this branch.

- **At the start of teleop the intake deploys and the roller runs on its own.** The roller
  keeps running until you use the intake roller or the trigger.

- **After any redeploy, re-check the mode dropdown.** It resets to Thrustmaster. A
  brownout does *not* reset it.

---

*Button positions confirmed with the drive team in 2026. Functions read from this branch's
`Triggers.java` and `RobotContainer.configureRealBindings()` — if a button does something
different from this card, the card is wrong, so tell a programmer.*
