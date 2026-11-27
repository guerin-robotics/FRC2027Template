# sync-logs.ps1 - Windows version of sync-logs.sh (same rclone remotes, same defaults).
# Pulls new logs off the Systemcore over SFTP and uploads them to Google Drive.
#
#   .\sync-logs.ps1           # sync once
#   .\sync-logs.ps1 -Watch    # sync forever, every $Interval seconds
#
# Env vars override defaults, same names as the bash script: SRC_REMOTE, SRC_PATH,
# DST_REMOTE, DST_PATH, INCLUDE, INTERVAL.
param([switch]$Watch)

function Get-Setting($name, $default) {
  $v = [Environment]::GetEnvironmentVariable($name)
  if ($v) { $v } else { $default }
}
$SrcRemote = Get-Setting 'SRC_REMOTE' 'systemcore'
$SrcPath   = Get-Setting 'SRC_PATH'   '/U/logs'
$DstRemote = Get-Setting 'DST_REMOTE' 'gdrive'
$DstPath   = Get-Setting 'DST_PATH'   'FRC-Logs'
$Include   = Get-Setting 'INCLUDE'    '*.wpilog'
$Interval  = [int](Get-Setting 'INTERVAL' '120')

if (-not (Get-Command rclone -ErrorAction SilentlyContinue)) {
  Write-Error 'rclone is not installed. See tools/log-sync/README.md'
  exit 1
}

# Single-instance lock, held per sync (like the bash script) so a one-off run still works
# while a -Watch loop sleeps. A named mutex is released automatically if the process dies.
$mutex = New-Object System.Threading.Mutex($false, 'Global\guerin-log-sync')

function Invoke-Sync {
  if (-not $mutex.WaitOne(0)) {
    Write-Host 'another sync is already running; skipping'
    return
  }
  try {
    rclone copy "${SrcRemote}:${SrcPath}" "${DstRemote}:${DstPath}" `
      --include $Include --update --low-level-retries 3 --stats-one-line --log-level INFO
    $ok = ($LASTEXITCODE -eq 0)
  } finally { $mutex.ReleaseMutex() }
  $stamp = Get-Date -Format 'HH:mm:ss'
  if ($ok) { Write-Host "[$stamp] sync ok" }
  else { Write-Warning "[$stamp] sync failed (robot or internet unreachable?)" }
}

if ($Watch) {
  Write-Host "watching: ${SrcRemote}:${SrcPath} -> ${DstRemote}:${DstPath} every ${Interval}s"
  while ($true) { Invoke-Sync; Start-Sleep -Seconds $Interval }
} else { Invoke-Sync }
