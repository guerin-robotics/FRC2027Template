# tools/

Developer tooling that sits alongside the robot code. None of it is compiled or
deployed by the robot build — `settings.gradle` does not include any of it, so nothing
here can break `./gradlew build`.

Everything in this directory is **season- and robot-agnostic**. It operates on WPILib
logs, NetworkTables, and simulation, none of which change between games.

---

## ClaudeScope

A CLI + daemon (Go) for querying `.wpilog` files and live NetworkTables, built so an
agent can investigate a log without loading the whole thing into context. Ships two
Claude Code skills:

- `/scope` — analyze a `.wpilog` or query live NT
- `/simulate` — launch sim, attach ClaudeScope, run a goal-driven investigation headlessly

Upstream: [rylero/TheFRCSuite](https://github.com/rylero/TheFRCSuite). Binaries are not
committed (see `tools/.gitignore`) — download them from the upstream releases page for
your platform and put one on your PATH as `ClaudeScope` / `ClaudeScope.exe`.

Source is vendored here so the tool is versioned with the robot code. Pull upstream
changes rather than editing it in place.

---

## log-sync

Pulls new `.wpilog` files off the roboRIO over SFTP and pushes them to Google Drive via
`rclone`, so the USB stick never has to be moved to a laptop. Includes a macOS launchd
plist for running it on a schedule.

```bash
./sync-logs.sh            # sync once
./sync-logs.sh --watch    # sync every $INTERVAL seconds
```

**Needs configuring per season:** the `roborio` rclone remote points at a team-number
IP (`10.TE.AM.2`). Everything else — `/U/logs` as the source path, `*.wpilog` as the
filter — carries over unchanged. See `log-sync/README.md` for the one-time rclone setup.

Note the practical limit documented there: at competition the roboRIO is firewalled off
the internet on the field, so uploads happen from the pit between matches, not live.

---

## wpilib-agent-tools

A Python CLI for agent-driven WPILib workflows: running simulation, recording NT4,
reading and graphing `.wpilog` data, sandboxed edit-and-validate loops, and harness
support for Claude Code, Codex, and Cursor.

```bash
./scripts/install_all.sh --workspace /path/to/robot-repo --harnesses all
```

Upstream: [edanliahovetsky/wpilib-agent-tools](https://github.com/edanliahovetsky/wpilib-agent-tools).

The installer creates `wpilib-agent-tools/.venv/`, which is gitignored — do not commit it.

`agent/src/wpilib_agent_tools/integrations/codex/skill_bundle/references/profiles/2026-robot-code.md`
is **upstream's own example profile**, not this team's 2026 robot. It is left as-is
because it documents how profile-driven validation works. If you write a profile for the
2027 robot, add it beside that file rather than editing it.

---

## Not carried over

`log-analyze` existed in the 2026 repo but only ever had compiled `.class` files
committed under `build/` — no tracked source — so there was nothing to bring forward.
