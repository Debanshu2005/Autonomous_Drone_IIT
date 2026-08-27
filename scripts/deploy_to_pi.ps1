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
    "onboard_edge/requirements.txt",
    "onboard_edge/pi_edge_brain.py",
    "onboard_edge/mavlink_io.py",
    "onboard_edge/sensor_check.py",
    "onboard_edge/trajectory_engine.py",
    "onboard_edge/flight_controller.py",
    "onboard_edge/safety.py",
    "ground_station/requirements.txt",
    "ground_station/laptop_client.py",
    "ground_station/terminal_ui.py",
    "legacy/autonomous_mission.py",
    "missions/example_mission.json",
    "docs/failsafe_checklist.md",
    "scripts/setup_pi.sh"
)

ssh "${PiUser}@${PiHost}" "mkdir -p '$RemotePath/onboard_edge' '$RemotePath/ground_station' '$RemotePath/legacy' '$RemotePath/missions' '$RemotePath/docs' '$RemotePath/scripts'"

foreach ($file in $files) {
    $local = Join-Path $root $file
    $remoteFile = "$RemotePath/$file"
    $remoteDir = Split-Path $remoteFile -Parent
    ssh "${PiUser}@${PiHost}" "mkdir -p '$remoteDir'"
    scp $local "${PiUser}@${PiHost}:$remoteFile"
}

Write-Host "Deployed project files to $target"
