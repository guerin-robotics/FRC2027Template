# log-sync — auto-upload Systemcore logs to Google Drive

Pulls new `.wpilog` files off the Systemcore **over the network** and uploads them to
Google Drive. Works on **macOS, Windows and Linux**. The robot's USB stick never has to be
plugged into a computer.

```
Systemcore /U/logs  --(SFTP)-->  this laptop  --(HTTPS)-->  Google Drive
                                 (streamed, nothing saved to local disk)
```

`rclone` only transfers files Drive doesn't already have, so the newest logs are
always uploaded and re-running is cheap.

**Requirement:** the laptop running this must reach *both* the robot and the
internet at once. In the shop that's constant. At a competition the robot is
firewalled off the internet on the field, so uploads happen from the pit network
between matches — not live during a match.

> **Alpha caveats (Systemcore, checked against `wpilibsuite/SystemCoreTesting`; nothing here
> has been run on hardware):**
> - USB mounting on Systemcore is documented as unreliable on alpha/beta units. Avoid USB 3
>   drives and format with an MBR partition map. If `/U/logs` is empty or missing, set
>   `SRC_PATH=/home/systemcore/logs` (internal storage) — but then the robot code must be
>   logging there too (`Robot.java` / `PhoenixSignalLogger` currently use `/U/logs`).
> - The default SSH login is user `systemcore`, password `systemcore`. Whether SFTP is
>   enabled and the exact address scheme for team 10021 are **not confirmed**; the
>   other addresses below are the ones the alpha docs list.

---

## One-time setup

### 1. Install rclone

| OS | Command |
|---|---|
| macOS | `brew install rclone` |
| Windows | `winget install Rclone.Rclone` (then reopen the terminal) |

### 2. Configure the Google Drive remote (interactive, opens a browser)

```bash
rclone config
```

- `n` (new remote) → name it **`gdrive`**
- Storage: **`drive`** (Google Drive)
- Leave client_id / client_secret blank (press Enter)
- Scope: **`1`** (full access) or `3` (drive.file — only files this tool creates)
- Accept defaults, choose **`y`** to auto-authenticate — a browser opens, log in,
  allow access.

### 3. Configure the Systemcore SFTP remote

```bash
rclone config
```

- `n` (new remote) → name it **`systemcore`**
- Storage: **`sftp`**
- **host:** pick whichever reaches your robot:
  - `10.100.21.2` (team 10021's usual `10.TE.AM.2` address — **unconfirmed** on Systemcore, try it first)
  - `robot.local` (any OS, when on the robot's network)
  - USB cable: `172.26.0.1` (Windows) or `172.27.0.1` (macOS / Linux)
  - Systemcore Wi-Fi AP: `172.30.0.1`
- **user:** `systemcore`
- **port:** blank (22)
- **pass:** the Systemcore password (`systemcore` by default). rclone stores it obscured;
  an SSH key (`ssh-keygen -t ed25519`, append the `.pub` to `~/.ssh/authorized_keys` on the robot
  - `ssh-copy-id` on Mac/Linux, manually on Windows - then set the remote's **key_file**) works too and is preferred once you change the default password.

### 4. Test it

```bash
rclone lsf systemcore:/U/logs        # should list the .wpilog files on the robot
```

Then run one sync and check the `FRC-Logs` folder in Drive:

| OS | Command |
|---|---|
| macOS / Linux | `./tools/log-sync/sync-logs.sh` |
| Windows (PowerShell) | `.\tools\log-sync\sync-logs.ps1` |

(On Windows, `sync-logs.sh` also runs from Git Bash, but the `.ps1` needs nothing extra.)

---

## Running it

Keep syncing every 2 minutes while you work:

```bash
./tools/log-sync/sync-logs.sh --watch          # macOS / Linux
```
```powershell
.\tools\log-sync\sync-logs.ps1 -Watch          # Windows
```

### Run automatically in the background

**macOS (launchd).** Edit the script path in `com.guerin.logsync.plist` to where the repo
lives (it ships with a placeholder), then:

```bash
cp tools/log-sync/com.guerin.logsync.plist ~/Library/LaunchAgents/
launchctl load ~/Library/LaunchAgents/com.guerin.logsync.plist
```

It starts on login and runs `sync-logs.sh --watch`. Logs go to
`/tmp/guerin-log-sync.out`. To stop:

```bash
launchctl unload ~/Library/LaunchAgents/com.guerin.logsync.plist
```

**Windows (Task Scheduler).** Run once in PowerShell, from the repo root:

```powershell
$script = (Resolve-Path tools\log-sync\sync-logs.ps1).Path
$action = New-ScheduledTaskAction -Execute pwsh.exe -Argument "-NoProfile -WindowStyle Hidden -File `"$script`" -Watch"
# Zero time limit and battery-OK, or Windows kills the watcher after 72 hours / on battery.
$settings = New-ScheduledTaskSettingsSet -ExecutionTimeLimit ([TimeSpan]::Zero) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
Register-ScheduledTask -TaskName GuerinLogSync -Action $action -Trigger (New-ScheduledTaskTrigger -AtLogOn) -Settings $settings
```

Use `powershell.exe` instead of `pwsh.exe` if PowerShell 7 is not installed. Stop and remove
it with `Unregister-ScheduledTask -TaskName GuerinLogSync -Confirm:$false`.

---

## Options (environment variables)

Same names for both scripts.

| Var          | Default      | Meaning                                        |
|--------------|--------------|------------------------------------------------|
| `SRC_REMOTE` | `systemcore` | rclone remote for the robot                    |
| `SRC_PATH`   | `/U/logs`    | Log directory on the Systemcore (`/home/systemcore/logs` for internal storage) |
| `DST_PATH`   | `FRC-Logs`   | Google Drive destination folder                |
| `INCLUDE`    | `*.wpilog`   | File filter — set to `{*.wpilog,*.hoot}` for CTRE Hoot logs too (large) |
| `INTERVAL`   | `120`        | Seconds between syncs in watch mode            |

Example: also grab Hoot logs, check every 5 min:

```bash
INCLUDE='{*.wpilog,*.hoot}' INTERVAL=300 ./tools/log-sync/sync-logs.sh --watch
```
```powershell
$env:INCLUDE='{*.wpilog,*.hoot}'; $env:INTERVAL=300; .\tools\log-sync\sync-logs.ps1 -Watch
```

---

## Notes

- `.hoot` (CTRE signal) logs are large; they're excluded by default. Include them
  deliberately if you have the bandwidth.
- This is laptop-side tooling only — it touches nothing under `src/` and does not
  affect robot behavior.
