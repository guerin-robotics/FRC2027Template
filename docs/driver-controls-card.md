# Driver Controls — Quick Refresh

> **THIS IS A BLANK TEMPLATE. It describes no robot.**
>
> The 2026 version of this card was a two-mode button map (flightstick and Xbox) for that
> season's mechanisms. None of it applies to a 2027 robot, so it was not carried over.
>
> Fill this in once bindings exist, print it, and keep a copy in the pit. The 2026 drive
> team used it between matches and it was worth the effort to maintain.

---

## How to use this card

Keep it to **one printed page per mode.** A driver refreshing between matches has about
thirty seconds. Structure that worked in 2026:

1. A diagram or list of every button, in physical layout order
2. What each button does, in the driver's language — not the code's
3. What happens on *release*, which is where drivers get surprised
4. Any automatic behavior the driver doesn't press a button for
5. A short "good to know" section for the non-obvious interactions

Write it for someone who has never read the code.

---

## Controller assignment

| Port | Device | Who |
|---|---|---|
| 0 | TODO | TODO |
| 1 | TODO | TODO |

The template ships with a single `CommandXboxController` on port 0, created directly in
`RobotContainer`. Once there are two operators and real state triggers, move all trigger
objects into `Triggers.java` — see `.claude/rules/01-architecture.md`.

---

## Driver controls

| Control | Action | On release |
|---|---|---|
| Left stick | Drive (field-relative translation) | Robot coasts to stop |
| Right stick X | Rotate | — |
| A (hold) | Hold heading at 0° while still driving | Returns to normal drive |
| X (hold) | X the wheels — resists being pushed | Wheels return to normal on next move |
| B | Reset gyro heading to 0° | — |
| | | |

*Rows above are the template's default bindings. Replace as the 2027 controls are built.*

---

## Operator controls

| Control | Action | On release |
|---|---|---|
| | | |

---

## Automatic behavior (no button)

List anything the robot does on its own, so the drive team is never surprised by robot
motion they did not command. In 2026 this section covered the auto-align, auto-X, and
auto-compress behaviors that triggered off a single button press.

| Behavior | When it happens | How to override |
|---|---|---|
| | | |

---

## Good to know

- Field-relative drive is relative to **your alliance wall**. `AllianceFlipUtil` handles
  the red/blue flip; if the robot drives backward from what the driver expects, check
  the alliance color in the driver station before touching code.
- Resetting the gyro (B) only changes heading, not position. Vision corrects position.
- If a mechanism stops responding mid-match, check whether a command with a timeout has
  already expired — the robot is designed to give up rather than hang.

---

## Related

- `docs/drive-controller-mode.md` — swapping between controller types between matches
- `.claude/rules/03-commands.md` — how bindings are structured in code
