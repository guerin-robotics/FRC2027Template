# Third-Party Code and Licences

Where the borrowed code in this repo came from, and what licence it carries. Check this before
copying anything new in, and add a row when you do.

Most FRC code is permissively licensed and mixes freely. **GPL is the exception** — it is
copyleft, so a project that distributes GPL-licensed files can be obliged to license the whole
combined work under the GPL. That is a decision to make deliberately, not one to inherit by
copy-paste.

---

## What we carry

| File | Origin | Licence | Notes |
|---|---|---|---|
| `frc/lib/LoggedTrigger.java` | FRC 3467 (Windham Windup) | **GPLv3** | Header intact. Load-bearing — required by `.claude/rules/01-architecture.md` |
| `frc/lib/ThrowingRunnable.java` | FRC 3467 (Windham Windup) | **GPLv3** | Header intact |
| `frc/lib/LoggedTunableProfiledPID.java` | FRC 3467 | WPILib BSD | Their file carries the WPILib BSD header, not their GPL one |
| `frc/lib/LoggedTunableNumber.java` | FRC 6328 (Mechanical Advantage) | MIT-style | The `hasChanged(int id)` design is 6328's. 3467 ships a copy under GPL; ours descends from the MIT original |
| `frc/lib/LoggedTunableBoolean.java` | FRC 6328 pattern | MIT-style | Same lineage as above |
| `frc/lib/BatteryLogger.java` | FRC 6328 | MIT-style | Attributed in the file |
| `frc/lib/AllianceFlipUtil.java` | FRC 6328 | MIT-style | Attributed in the file |
| `frc/lib/FieldConstants.java` | FRC 6328 | MIT-style | Attributed in the file |
| `frc/lib/GeomUtil.java` | FRC 6328 | MIT-style | 3467 ships a copy too; common ancestor is 6328 |
| `frc/lib/LocalADStarAK.java` | FRC 6328 / PathPlanner | MIT-style | Required for PathPlanner replay |
| `frc/lib/ContinuousConditionalCommand.java` | FRC 6328 | MIT-style | Attributed in the file |
| `frc/lib/PhoenixUtil.java` | FRC 6328 | MIT-style | Attributed in the file |
| `frc/lib/Elastic.java` | Gold87 / Elastic dashboard | MIT | Vendor-published helper |
| `frc/lib/PointInPolygon.java` | Ours | — | Ray-casting crossing-number. 3467 has a GPL file of the same name using `java.awt.geom.Path2D`; unrelated implementation |
| `frc/lib/MotorSpecs.java` | Ours | — | Reads free speeds from WPILib `DCMotor` |
| `subsystems/drive/`, `Robot`, `RobotContainer` | AdvantageKit swerve template | BSD | Littleton Robotics |

---

## The open question

**Two files are GPLv3 and this repo has no `LICENSE` file.**

`LoggedTrigger` and `ThrowingRunnable` came from 3467 by way of the 2026 robot, headers intact.
While the repo stays private among the team this is inert. If it is ever published, GPLv3's
copyleft applies to the distributed work.

Three ways to resolve it, in rough order of effort:

1. **Adopt GPLv3 for the project.** Add a `LICENSE` file, done. The whole repo becomes copyleft:
   anyone who receives it can redistribute and modify, and derivative works must stay GPLv3.
   Fine for a team codebase nobody is trying to commercialise.
2. **Replace the two files.** `LoggedTrigger` wraps a `Trigger` with logging on poll;
   `ThrowingRunnable` is a functional interface. Both are small enough to reimplement, which
   frees the project to be MIT or BSD like the rest of its dependencies.
3. **Keep the repo private.** No distribution, no obligation. This is the status quo and it is a
   decision only until someone pushes it public.

Doing nothing is option 3 by default. That is fine, as long as it is known rather than assumed.

---

## Adding new borrowed code

1. Keep the original header. Stripping it is what creates the problem later.
2. Add a row above.
3. If it is GPL, say so out loud before merging — it changes the project's obligations, not just
   that file's.
4. Prefer the upstream original over someone else's copy. 3467's `LoggedTunableNumber` is 6328's
   MIT code shipped under GPL; taking it from 6328 avoids inheriting a licence the author never
   applied.
