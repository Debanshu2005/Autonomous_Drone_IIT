# Failsafe Checklist

This aircraft has no obstacle sensors, so "safe" means bounded GPS flight, fast pilot takeover, and predictable LAND/RTL behavior.

## Pixhawk Configuration

Set these in your ground-control software for the exact firmware on the Pixhawk V6X.

### Required

- RC loss failsafe: enabled, with action set to LAND or RTL for your test area.
- Battery failsafe: enabled for low and critical thresholds.
- Battery monitor: enabled and calibrated so MAVLink reports voltage and remaining percentage.
- GPS/EKF failsafe: enabled, with LAND as the safest default for GPS-only flight.
- Geofence: enabled with maximum radius and altitude matching or tighter than the JSON mission.
- Datalink/GCS failsafe: enabled if the Pi or ground station link is part of your operation.
- Disarm after landing: enabled with a short, tested delay.

### RadioLink AT95 Pro

- Calibrate all channels in the ground station.
- Verify channel direction and endpoints.
- Program receiver failsafe with transmitter off and confirm the Pixhawk detects RC loss.
- Assign one switch to an assisted hold mode such as Loiter/Position.
- Assign one switch to LAND or RTL.
- Practice switching out of autonomous flight before the first real mission.

## Companion Script Failsafes

The script commands `LAND` when it detects:

- no usable global position/GPS for more than `gps_loss_grace_s`
- stale position or GPS telemetry for more than `telemetry_timeout_s`
- stale battery telemetry for more than `battery_telemetry_timeout_s`
- software geofence breach
- temporary hotspot containment radius breach
- temporary hotspot peer heartbeat loss
- altitude above `max_altitude_agl_m`
- waypoint timeout
- mission runtime timeout
- critical battery
- missing remaining-percent battery telemetry after arming
- uncaught software exception
- Ctrl+C or service stop

For low battery, `low_battery_action` can be:

- `return`: command RTL first
- `land`: command LAND immediately

## Slow Takeoff And 5 m Hover

The default mission performs a cautious staged takeoff:

- Pixhawk initial takeoff target: `initial_takeoff_altitude_m`, currently 1.2 m
- Pi-commanded climb steps: `slow_takeoff_step_m`, currently 0.5 m
- pause at each step: `slow_takeoff_step_hold_s`, currently 2.0 s
- final hover height: `takeoff_altitude_m`, currently 5.0 m
- hover before mission: `hover_before_mission_s`, currently 20.0 s

For an even softer climb, reduce the Pixhawk takeoff/climb-speed parameter in the flight-controller firmware as well. The script can stage the climb, but the Pixhawk still controls motor output and vertical speed.

## TEMPORARY Hotspot Containment - REMOVE LATER

Keep this section while both drones are using the same mobile hotspot. Remove it when you replace the hotspot setup with the final communications design.

This is not a true range guarantee. A mobile hotspot does not provide a dependable physical boundary, and Wi-Fi signal can fail late, early, or unevenly. The temporary safety approach is:

- Set `hotspot_containment.enabled` to `true`.
- Set `hotspot_containment.max_radius_m` to a radius smaller than the tested reliable hotspot range.
- Keep the normal Pixhawk geofence equal to or tighter than the same radius.
- Enable `network_watchdog_enabled` so the drone lands if the expected peer heartbeat disappears.
- Use `require_peers_before_arm` so both drones must see each other before takeoff.

For two drones on the same hotspot:

- Drone 1 config: `drone_id` = `drone-1`, `expected_peer_ids` = `["drone-2"]`
- Drone 2 config: `drone_id` = `drone-2`, `expected_peer_ids` = `["drone-1"]`

Both drones must use the same `udp_port`. If the hotspot blocks broadcast packets, assign fixed IP addresses to the Raspberry Pis and put the other drone IP in `peer_unicast_ips`.

## First Flight Progression

1. Bench test with propellers removed.
2. Confirm the Pi connects and reads GPS, battery, armed state, and position.
3. Confirm both Pis join the same hotspot and exchange peer heartbeats.
4. Test RC mode switches with the script connected but not flying.
5. Run SITL if available for your firmware.
6. Fly manually and confirm LAND/RTL behavior.
7. Run a 5 m hover test with a 10-20 m radius.
8. Increase radius only after logs show stable GPS, Wi-Fi, and battery behavior.

## Abort Criteria

Abort before arming if:

- GPS has fewer satellites than the mission minimum.
- Home position is not set.
- Compass/EKF health is not green.
- Battery voltage/current readings are missing or wrong.
- The remote cannot take mode control from the companion script.
- Wind, people, vehicles, buildings, wires, or trees are inside the planned area.

## Official References

- MAVSDK Python quickstart: https://mavsdk.mavlink.io/main/en/python/quickstart.html
- PX4 safety/failsafe configuration: https://docs.px4.io/main/en/config/safety
- ArduPilot Copter failsafes: https://ardupilot.org/copter/docs/failsafe-landing-page.html
