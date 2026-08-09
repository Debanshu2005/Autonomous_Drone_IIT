param(
    [string]$PiHost = "pi",
    [string]$PiUser = "pi",
    [string]$RemotePath = "/home/pi/Autonomous_Drone_IIT"
)

$ErrorActionPreference = "Stop"

$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$target = "${PiUser}@${PiHost}:${RemotePath}/"
$files = @(
    "README.md",
    "requirements.txt",
    "src/autonomous_mission.py",
    "missions/example_mission.json",
    "docs/failsafe_checklist.md",
    "scripts/setup_pi.sh"
)

ssh "${PiUser}@${PiHost}" "mkdir -p '$RemotePath/src' '$RemotePath/missions' '$RemotePath/docs' '$RemotePath/scripts'"

foreach ($file in $files) {
    $local = Join-Path $root $file
    $remoteFile = "$RemotePath/$file"
    $remoteDir = Split-Path $remoteFile -Parent
    ssh "${PiUser}@${PiHost}" "mkdir -p '$remoteDir'"
    scp $local "${PiUser}@${PiHost}:$remoteFile"
}

Write-Host "Deployed project files to $target"
