# Driver Controls — Quick Refresh

> **MOSTLY BLANK. The drive controls below are real; the operator controls are not.**
>
> The 2026 version of this card was a two-mode button map for that season's mechanisms.
> None of it applies to a 2027 robot, so it was not carried over.
>
> Fill in the operator section as mechanisms land, print it, and keep a copy in the pit.
> The 2026 drive team used it between matches and it was worth the effort to maintain.

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
| 0 | Flight stick | Driver — translation and rotation |
| 1 | Xbox controller | Operator — mechanisms and overrides |

Ports are set in `Constants.Controllers`. **Verify them in the DS USB tab before the first
match.** The driver station assigns ports in plug order, not by device type, and a controller
that gets unplugged and replugged can come back on a different one. The symptom is "my
controller does nothing" with no error anywhere.

Both controller objects live in `Triggers.java` and are private. Nothing else in the codebase
touches them — see `.claude/rules/01-architecture.md`.

---

## Driver controls

Flight stick, port 0.

| Control | Action | On release |
|---|---|---|
| Stick forward/back | Drive forward/back (field-relative) | Robot coasts to stop |
| Stick left/right | Drive left/right (field-relative) | Robot coasts to stop |
| Twist | Rotate | — |
| Button 2 | Reset gyro heading to 0° | — |
| Button 3 (hold) | Hold heading at 0° while still driving | Returns to normal drive |
| Button 4 (hold) | X the wheels — resists being pushed | Wheels return to normal on next move |

*These are the template's defaults and are wired up now. Button numbers are whatever your
stick reports — check them in the DS before printing this card.*

---

## Operator controls

Xbox controller, port 1. Nothing is bound yet — add a row per function as mechanisms land.

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
- Resetting the gyro only changes heading, not position. Vision corrects position.
- If a mechanism stops responding mid-match, check whether a command with a timeout has
  already expired — the robot is designed to give up rather than hang.

---

## Related

- `docs/drive-controller-mode.md` — swapping between controller types between matches
- `.claude/rules/03-commands.md` — how bindings are structured in code
