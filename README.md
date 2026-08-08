# Autonomous Drone IIT

Raspberry Pi companion-computer script for a GPS-only autonomous mission using a Pixhawk V6X, NEO 3 GPS, and RadioLink AT95 Pro transmitter.

The Pi sends high-level MAVLink commands through MAVSDK. The Pixhawk remains responsible for stabilization, motor output, RC override, EKF/GPS checks, battery failsafe, and landing execution.

## What This Does

- Waits for Pixhawk connection, GPS health, and home position.
- Arms, performs a slow staged takeoff to 5 m, hovers, and then flies local north/east waypoints from the launch point.
- Monitors software failsafes during flight:
  - GPS/global-position degradation
  - telemetry timeout
  - software geofence radius
  - temporary hotspot containment radius
  - optional peer heartbeat loss over Wi-Fi
  - altitude limit
  - waypoint timeout
  - mission runtime timeout
  - low and critical battery percentage
- Commands a soft landing with `LAND` on mission errors.
- Can return to launch on normal completion or low battery.

No obstacle avoidance is possible with only GPS. Test only in an open legal flight area with a human pilot ready to switch to manual/Loiter/Stabilize and land.

## Install On Raspberry Pi

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
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

## Important Setup

Before flying, configure Pixhawk-side failsafes in QGroundControl or Mission Planner. The script is a second layer, not a replacement for flight-controller failsafes.

- Calibrate accelerometer, compass, RC, ESC/motors, and battery monitor.
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
