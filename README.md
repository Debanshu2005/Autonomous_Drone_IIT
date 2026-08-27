# Autonomous Drone IIT

Raspberry Pi companion-computer mission controller for a Pixhawk V6X, adaptive
GPS/optical-flow navigation, and terminal-driven natural-language flight tasks.

The Pi sends high-level MAVLink setpoints and commands through MAVLink. The
Pixhawk remains responsible for stabilization, motor output, RC override,
EKF/GPS checks, battery failsafe, and landing execution.

## Interactive Dashboard

<img width="1268" height="1020" alt="image" src="https://github.com/user-attachments/assets/da5a3c2d-08e0-4bb4-a7c5-4757bfebd6bd" />


Built using the `Textual` framework, the interactive terminal UI features a persistent 4Hz live telemetry header, a 500-line scrollable system log buffer with semantic color markup, and a native non-blocking command prompt with full arrow-key cursor support.

## What This Does

- Waits for Pixhawk connection, estimator health, and battery telemetry.
- Detects GPS-enabled, GPS-denied optical-flow/vision, or degraded navigation modes.
- Parses robust natural-language and compact commands using a regex lexer.
  - Supports unit normalization (cm, m, km) for all distances.
  - Commands include takeoff, hover, land, GOTO offsets, circles, squares, grids, spirals, and figure-8s.
- Arms, takes off, streams global or local NED setpoints, and verifies waypoint
  acceptance radius.
- Monitors software failsafes during flight via `safety.py` watchdog daemon:
  - MAVLink link loss (2.0s timeout triggers emergency LAND)
  - Battery critical voltage (hardcoded 10.5V triggers emergency LAND)
  - GPS/local-position degradation (pauses spatial formations with HOLD)
  - State-aware Ctrl+C interrupt (graceful soft-land or immediate exit if grounded)
  - Double-tap Ctrl+C hard abort
- Refuses to arm by default until fresh Pixhawk battery telemetry is available.
- Commands a soft landing with `LAND` on mission errors.

No obstacle avoidance is included. Test only in an open legal flight area with a
human pilot ready to switch to manual/Loiter/Stabilize and land.

## Install On Raspberry Pi

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Or run the helper after installing the system packages:

```bash
bash scripts/setup_pi.sh
source .venv/bin/activate
```

Connect the Pixhawk to the Raspberry Pi with a USB cable. The mission
`connection_url` intelligently auto-detects the connected Pixhawk by sweeping
Linux serial ports (`ttyAMA*`, `ttyUSB*`, `ttyACM*`) and a multi-baud fallback 
matrix (921600, 115200, 57600), automatically caching the fastest route to `config.json`!

```text
serial://auto:115200
```

Common URLs:

```text
serial://auto:115200
serial:///dev/ttyACM0:115200
serial:///dev/ttyUSB0:115200
serial:///dev/serial0:115200
serial:///dev/ttyAMA0:57600
serial:///dev/ttyUSB0:57600
udpin://0.0.0.0:14550
udpin://0.0.0.0:14540
```

## Run The Terminal Controller

Run the adaptive natural-language controller on the Raspberry Pi with:

```bash
python3 src/main.py --connect serial://auto:115200 --default-altitude 3
```

Common REPL commands:

```text
status
[CMD] Take off to 300cm, hover for 2 seconds, and land
[CMD] Takeoff to 5m, circle with 3m radius, then return to launch
[CMD] Hover for 10s
switch mode to guided
mode alt_hold
fly in a 5m radius circle at 3m altitude
square search pattern 10m h=3
spiral size=10 h=5 duration=20
go 10 meters north and 5 meters east at 3 meters altitude
goto x=10 y=5 h=3
hold
land
rtl
quit
```

If a command cannot be parsed, the REPL prints an in-terminal command guide with
natural-language examples and the compact parameter dictionary.

The terminal accepts commands with or without the `[CMD]` prefix. Compound
sentences are broken into a sequential task queue. For example:

```text
[CMD] Takeoff to 5m, circle with 3m radius, then return to launch
```

queues:

```text
TAKEOFF(alt=5m) -> CIRCLE(radius=3m, alt=5m) -> RTL
```

The background telemetry thread prints at 0.5Hz to 5Hz, default 2Hz, using:

```text
[TELEM] MODE=<mode> | ARMED=<bool> | BAT=<pct>% (<volt>V) | ALT=<alt>m | NAV=<GPS/FLOW/NONE> | POS=(<x>,<y>,<z>)
```

Execution feedback uses `[STATUS]` for parser/state transitions and `[EXEC]`
for live task progress. Typing `[CMD] HOLD`, `[CMD] LAND`, or `[CMD] RTL` during
a sequence flushes the pending queue and interrupts the active autonomous task.

Before each navigation task, the controller probes Pixhawk telemetry and selects:

- `GPS-Enabled`: fresh `GPS_RAW_INT`, healthy EKF absolute horizontal position, and `LOCAL_POSITION_NED`; trajectories are converted to global WGS84 targets.
- `GPS-Denied / Optical Flow`: fresh optical-flow/vision/odometry aiding plus healthy local EKF; trajectories stay in local NED/body frames.
- `Degraded`: insufficient position estimate; navigation commands are refused and the terminal prints the reason.

Use `--help` for tunable acceptance radius, waypoint timeout, satellite threshold, battery thresholds, altitude ceiling, and final action.

## Legacy JSON Runner

The previous GPS-only MAVSDK mission runner has been moved to
`legacy/autonomous_mission.py`. Keep it as a reference or fallback while the new
terminal controller is tested.

Run the legacy path with:

```bash
python3 legacy/autonomous_mission.py --config missions/example_mission.json
```

The telemetry-only output includes battery voltage, optional current if your
firmware exposes it, remaining percentage, and the age of the latest battery
sample. It also prints `battery_state`; values such as `missing`,
`invalid_voltage`, or `missing_percent` mean the Pixhawk is not publishing usable
battery data yet.

The monitor also listens to raw MAVLink `SYS_STATUS`, `BATTERY_STATUS`,
`GLOBAL_POSITION_INT`, and `VFR_HUD` through MAVSDK direct messages. This is
important because Mission Planner may display battery voltage or altitude from
those raw messages even when MAVSDK's higher-level telemetry plugin still shows
`n/a`.

To see the exact raw battery and altitude messages reaching the Pi over USB:

```bash
python3 legacy/autonomous_mission.py --config missions/example_mission.json --raw-mavlink-probe
```

The active controller modules live in `src/`.

## Battery Telemetry Settings

`missions/example_mission.json` enables required battery telemetry before arm:

- `require_battery_before_arm`: wait for valid battery voltage and remaining percentage.
- `battery_telemetry_timeout_s`: fail safe if in-flight battery samples go stale.
- `min_prearm_battery_percent`: minimum charge required before arming.
- `min_prearm_battery_voltage_v`: optional voltage floor; keep `0.0` to disable this check.
- `low_battery_percent` and `critical_battery_percent`: in-flight companion failsafe thresholds.

## Important Setup

Before flying, configure Pixhawk-side failsafes in QGroundControl or Mission Planner. The script is a second layer, not a replacement for flight-controller failsafes.

- Calibrate accelerometer, compass, RC, ESC/motors, and battery monitor.
- Configure the Pixhawk battery monitor so MAVLink reports voltage and remaining percentage.
- Verify NEO 3 GPS lock and home position outdoors.
- Configure RadioLink receiver failsafe so throttle and mode channel produce the desired flight-controller failsafe action.
- Put a safe assisted mode on one AT95 Pro switch, such as Loiter/Position, and a land/RTL mode on another switch.
- Set conservative fence radius and altitude limits on the Pixhawk as well as in `missions/example_mission.json`.
- Start with props off, then SITL, then tethered/low-altitude tests, then a very small mission.

See `docs/failsafe_checklist.md` for a practical preflight list.

## TEMPORARY Hotspot Containment - REMOVE LATER

This project currently includes a temporary mobile-hotspot containment layer because both drones will be connected to the same hotspot during early tests.

Important: a mobile hotspot cannot prove a clean physical boundary. Signal strength changes with phone placement, antenna angle, battery state, people nearby, and interference. The script therefore uses two conservative checks instead:

- GPS radius from launch: `hotspot_containment.max_radius_m`
- Wi-Fi peer heartbeat: each drone listens for the other drone on UDP

The example config is for `drone-1` expecting `drone-2`. On the second drone, copy `missions/example_mission.json` and swap:

```json
{
  "drone_id": "drone-2",
  "expected_peer_ids": ["drone-1"]
}
```

If the hotspot blocks broadcast traffic, assign fixed IPs to both Pis and put the other Pi address in `peer_unicast_ips`.

## References

- MAVSDK Python quickstart: https://mavsdk.mavlink.io/main/en/python/quickstart.html
- PX4 safety/failsafe configuration: https://docs.px4.io/main/en/config/safety
- ArduPilot Copter failsafes: https://ardupilot.org/copter/docs/failsafe-landing-page.html
