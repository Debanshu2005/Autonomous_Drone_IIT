# Autonomous Drone IIT

Raspberry Pi companion-computer script for a GPS-only autonomous mission using a Pixhawk V6X, NEO 3 GPS, and RadioLink AT95 Pro transmitter.

The Pi sends high-level MAVLink commands through MAVSDK. The Pixhawk remains responsible for stabilization, motor output, RC override, EKF/GPS checks, battery failsafe, and landing execution.

## What This Does

- Waits for Pixhawk connection, GPS health, and home position.
- Arms, performs a slow staged takeoff to 5 m, hovers, and then flies local north/east waypoints from the launch point.
- Monitors software failsafes during flight:
  - GPS/global-position degradation
  - telemetry timeout
  - battery telemetry timeout
  - software geofence radius
  - temporary hotspot containment radius
  - optional peer heartbeat loss over Wi-Fi
  - altitude limit
  - waypoint timeout
  - mission runtime timeout
  - low and critical battery percentage
- Refuses to arm by default until fresh Pixhawk battery telemetry is available.
- Commands a soft landing with `LAND` on mission errors.
- Can return to launch on normal completion or low battery.

No obstacle avoidance is possible with only GPS. Test only in an open legal flight area with a human pilot ready to switch to manual/Loiter/Stabilize and land.

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
`connection_url` auto-detects the single connected Pixhawk USB serial device at
`115200` baud by default:

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

## Run

Edit `missions/example_mission.json` for your field and limits, then:

```bash
python3 src/autonomous_mission.py --config missions/example_mission.json
```

For SITL or bench testing:

```bash
python3 src/autonomous_mission.py --config missions/example_mission.json --log-level DEBUG
```

To view live Pixhawk telemetry without arming or flying:

```bash
python3 src/autonomous_mission.py --config missions/example_mission.json --telemetry-only
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
python3 src/autonomous_mission.py --config missions/example_mission.json --raw-mavlink-probe
```

## Adaptive Terminal Mission Controller

The modular pymavlink controller lives in:

- `src/mavlink_io.py`: non-blocking MAVLink reader/cache and command helpers.
- `src/sensor_check.py`: GPS, EKF, local-position, optical-flow/vision, and battery discovery.
- `src/trajectory_engine.py`: natural-language command parsing and NED/global waypoint generation.
- `src/flight_controller.py`: `IDLE -> HARDWARE_CHECK -> ARMING -> TAKEOFF -> TRAJECTORY_FOLLOW -> RTL_OR_HOLD` FSM.
- `src/main.py`: interactive terminal REPL.

Run it on the Raspberry Pi with:

```bash
python3 src/main.py --connect serial://auto:115200 --default-altitude 3
```

Common REPL commands:

```text
status
take off to 3 meters, hover for two seconds, and land
circle r=5 h=3 n=36
fly in a 5m radius circle at 3m altitude
square search pattern 10m h=3
go 10 meters north and 5 meters east at 3 meters altitude
goto x=10 y=5 h=3
hold
land
rtl
quit
```

If a command cannot be parsed, the REPL prints an in-terminal command guide with
natural-language examples and the compact parameter dictionary.

Before each navigation task, the controller probes Pixhawk telemetry and selects:

- `GPS-Enabled`: fresh `GPS_RAW_INT`, healthy EKF absolute horizontal position, and `LOCAL_POSITION_NED`; trajectories are converted to global WGS84 targets.
- `GPS-Denied / Optical Flow`: fresh optical-flow/vision/odometry aiding plus healthy local EKF; trajectories stay in local NED/body frames.
- `Degraded`: insufficient position estimate; navigation commands are refused and the terminal prints the reason.

Use `--help` for tunable acceptance radius, waypoint timeout, satellite threshold, battery thresholds, altitude ceiling, and final action.

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
