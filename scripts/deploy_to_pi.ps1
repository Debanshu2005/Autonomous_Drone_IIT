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
    "src/main.py",
    "src/mavlink_io.py",
    "src/sensor_check.py",
    "src/trajectory_engine.py",
    "src/flight_controller.py",
    "legacy/autonomous_mission.py",
    "missions/example_mission.json",
    "docs/failsafe_checklist.md",
    "scripts/setup_pi.sh"
)

ssh "${PiUser}@${PiHost}" "mkdir -p '$RemotePath/src' '$RemotePath/legacy' '$RemotePath/missions' '$RemotePath/docs' '$RemotePath/scripts'"

foreach ($file in $files) {
    $local = Join-Path $root $file
    $remoteFile = "$RemotePath/$file"
    $remoteDir = Split-Path $remoteFile -Parent
    ssh "${PiUser}@${PiHost}" "mkdir -p '$remoteDir'"
    scp $local "${PiUser}@${PiHost}:$remoteFile"
}

Write-Host "Deployed project files to $target"
