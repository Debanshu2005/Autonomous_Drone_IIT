#!/usr/bin/env python3
"""
GPS-only autonomous mission runner for a Raspberry Pi companion computer.

Hardware target:
- Pixhawk V6X flight controller running PX4 or ArduPilot
- Raspberry Pi 8 GB connected to a Pixhawk TELEM port
- NEO 3 GPS
- RadioLink AT95 Pro transmitter/receiver for manual override

The script intentionally delegates stabilization, motor mixing, RC failsafe, and
low-level landing behavior to the Pixhawk. It commands only high-level actions
and switches to LAND/RTL on software-detected problems.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import signal
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, AsyncIterator, Optional

from mavsdk import System


LOGGER = logging.getLogger("autonomous_mission")
EARTH_RADIUS_M = 6378137.0
DEFAULT_USB_SERIAL_BAUD = 115200
USB_SERIAL_PATTERNS = ("ttyACM*", "ttyUSB*")
REQUESTED_MAVLINK_STREAM_RATE_HZ = 2
REQUESTED_BATTERY_MESSAGE_RATE_HZ = 2


class MissionAbort(RuntimeError):
    """Raised when the mission should stop and the aircraft should land."""


class MissionReturnToLaunch(RuntimeError):
    """Raised when the mission should stop and the aircraft should RTL."""


@dataclass(frozen=True)
class HotspotContainmentConfig:
    enabled: bool
    max_radius_m: float
    network_watchdog_enabled: bool
    drone_id: str
    expected_peer_ids: list[str]
    udp_port: int
    broadcast_ip: str
    peer_unicast_ips: list[str]
    heartbeat_interval_s: float
    peer_timeout_s: float
    require_peers_before_arm: bool


@dataclass(frozen=True)
class Waypoint:
    name: str
    north_m: float
    east_m: float
    relative_altitude_m: float
    hold_s: float = 0.0
    yaw_deg: float = math.nan


@dataclass(frozen=True)
class MissionConfig:
    connection_url: str
    takeoff_altitude_m: float
    initial_takeoff_altitude_m: float
    slow_takeoff_step_m: float
    slow_takeoff_step_hold_s: float
    hover_before_mission_s: float
    return_to_launch_on_complete: bool
    land_at_final_waypoint: bool
    waypoint_acceptance_radius_m: float
    waypoint_timeout_s: float
    max_mission_time_s: float
    geofence_radius_m: float
    max_altitude_agl_m: float
    gps_min_satellites: int
    gps_loss_grace_s: float
    telemetry_timeout_s: float
    battery_telemetry_timeout_s: float
    require_battery_before_arm: bool
    min_prearm_battery_percent: float
    min_prearm_battery_voltage_v: float
    low_battery_percent: float
    critical_battery_percent: float
    low_battery_action: str
    prearm_health_timeout_s: float
    hotspot_containment: HotspotContainmentConfig
    mission: list[Waypoint]


class TelemetryCache:
    def __init__(self, drone: System) -> None:
        self.drone = drone
        self.position: Any = None
        self.gps_info: Any = None
        self.health: Any = None
        self.battery: Any = None
        self.raw_battery: dict[str, Any] = {}
        self.raw_altitude: dict[str, Any] = {}
        self.flight_mode: Any = None
        self.in_air: Optional[bool] = None
        self.armed: Optional[bool] = None
        self.last_position_s = 0.0
        self.last_gps_s = 0.0
        self.last_health_s = 0.0
        self.last_battery_s = 0.0
        self.last_raw_battery_s = 0.0
        self.last_raw_altitude_s = 0.0
        self._tasks: list[asyncio.Task[None]] = []
        self._stopping = False

    def start(self) -> None:
        self._stopping = False
        self._tasks = [
            asyncio.create_task(self._watch("position", self.drone.telemetry.position())),
            asyncio.create_task(self._watch("gps_info", self.drone.telemetry.gps_info())),
            asyncio.create_task(self._watch("health", self.drone.telemetry.health())),
            asyncio.create_task(self._watch("battery", self.drone.telemetry.battery())),
            asyncio.create_task(self._watch("flight_mode", self.drone.telemetry.flight_mode())),
            asyncio.create_task(self._watch("in_air", self.drone.telemetry.in_air())),
            asyncio.create_task(self._watch("armed", self.drone.telemetry.armed())),
            asyncio.create_task(self._watch_raw_mavlink("SYS_STATUS")),
            asyncio.create_task(self._watch_raw_mavlink("BATTERY_STATUS")),
            asyncio.create_task(self._watch_raw_mavlink("GLOBAL_POSITION_INT")),
            asyncio.create_task(self._watch_raw_mavlink("VFR_HUD")),
        ]

    async def stop(self) -> None:
        self._stopping = True
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)

    async def _watch(self, name: str, stream: AsyncIterator[Any]) -> None:
        try:
            async for value in stream:
                now = time.monotonic()
                setattr(self, name, value)
                if name == "position":
                    self.last_position_s = now
                elif name == "gps_info":
                    self.last_gps_s = now
                elif name == "health":
                    self.last_health_s = now
                elif name == "battery":
                    self.last_battery_s = now
        except asyncio.CancelledError:
            raise
        except Exception:
            if self._stopping:
                LOGGER.debug("Telemetry stream stopped during shutdown: %s", name)
                return
            LOGGER.exception("Telemetry stream failed: %s", name)

    async def _watch_raw_mavlink(self, message_name: str) -> None:
        try:
            async for message in self.drone.mavlink_direct.message(message_name):
                self._apply_raw_mavlink_message(message_name, message.fields_json)
        except asyncio.CancelledError:
            raise
        except Exception:
            if self._stopping:
                LOGGER.debug("Raw MAVLink stream stopped during shutdown: %s", message_name)
                return
            LOGGER.exception("Raw MAVLink stream failed: %s", message_name)

    def _apply_raw_mavlink_message(self, message_name: str, fields_json: str) -> None:
        try:
            fields = json.loads(fields_json)
        except json.JSONDecodeError:
            LOGGER.debug("Could not parse raw MAVLink %s: %s", message_name, fields_json)
            return

        now = time.monotonic()
        if message_name == "SYS_STATUS":
            voltage_mv = get_finite_float(
                first_existing_field(fields, "voltage_battery", "voltageBattery")
            )
            current_ca = get_finite_float(
                first_existing_field(fields, "current_battery", "currentBattery")
            )
            remaining_percent = get_finite_float(
                first_existing_field(fields, "battery_remaining", "batteryRemaining")
            )
            raw_battery: dict[str, Any] = {"source": "SYS_STATUS"}
            if voltage_mv is not None and voltage_mv > 0:
                raw_battery["voltage_v"] = voltage_mv / 1000.0
            if current_ca is not None and current_ca >= 0:
                raw_battery["current_battery_a"] = current_ca / 100.0
            if remaining_percent is not None and remaining_percent >= 0:
                raw_battery["remaining_percent"] = remaining_percent / 100.0
            self.raw_battery.update(raw_battery)
            self.last_raw_battery_s = now
            return

        if message_name == "BATTERY_STATUS":
            raw_battery = {"source": "BATTERY_STATUS"}
            voltage_v = battery_status_voltage_v(
                first_existing_field(fields, "voltages", "voltagesExt")
            )
            current_ca = get_finite_float(
                first_existing_field(fields, "current_battery", "currentBattery")
            )
            remaining_percent = get_finite_float(
                first_existing_field(fields, "battery_remaining", "batteryRemaining")
            )
            if voltage_v is not None:
                raw_battery["voltage_v"] = voltage_v
            if current_ca is not None and current_ca >= 0:
                raw_battery["current_battery_a"] = current_ca / 100.0
            if remaining_percent is not None and remaining_percent >= 0:
                raw_battery["remaining_percent"] = remaining_percent / 100.0
            self.raw_battery.update(raw_battery)
            self.last_raw_battery_s = now
            return

        if message_name == "GLOBAL_POSITION_INT":
            relative_alt_mm = get_finite_float(
                first_existing_field(fields, "relative_alt", "relativeAlt")
            )
            absolute_alt_mm = get_finite_float(fields.get("alt"))
            if relative_alt_mm is not None:
                self.raw_altitude["relative_altitude_m"] = relative_alt_mm / 1000.0
            if absolute_alt_mm is not None:
                self.raw_altitude["absolute_altitude_m"] = absolute_alt_mm / 1000.0
            self.raw_altitude["source"] = "GLOBAL_POSITION_INT"
            self.last_raw_altitude_s = now
            return

        if message_name == "VFR_HUD":
            alt_m = get_finite_float(fields.get("alt"))
            if alt_m is not None:
                self.raw_altitude["vfr_altitude_m"] = alt_m
            self.raw_altitude["source"] = "VFR_HUD"
            self.last_raw_altitude_s = now


class PeerHeartbeatProtocol(asyncio.DatagramProtocol):
    def __init__(self, drone_id: str) -> None:
        self.drone_id = drone_id
        self.last_seen_s: dict[str, float] = {}

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            message = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return

        if message.get("type") != "drone_heartbeat":
            return

        peer_id = str(message.get("id", ""))
        if not peer_id or peer_id == self.drone_id:
            return

        self.last_seen_s[peer_id] = time.monotonic()
        LOGGER.debug("Peer heartbeat from %s at %s", peer_id, addr[0])


class PeerLink:
    def __init__(self, config: HotspotContainmentConfig) -> None:
        self.config = config
        self.protocol = PeerHeartbeatProtocol(config.drone_id)
        self.transport: Optional[asyncio.DatagramTransport] = None
        self._task: Optional[asyncio.Task[None]] = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: self.protocol,
            local_addr=("0.0.0.0", self.config.udp_port),
            allow_broadcast=True,
        )
        self.transport = transport
        self._task = asyncio.create_task(self._send_heartbeats())
        LOGGER.info(
            "Hotspot peer heartbeat active as %s on UDP %d",
            self.config.drone_id,
            self.config.udp_port,
        )

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        if self.transport:
            self.transport.close()

    def missing_peers(self) -> list[str]:
        now = time.monotonic()
        missing = []
        for peer_id in self.config.expected_peer_ids:
            last_seen = self.protocol.last_seen_s.get(peer_id, 0.0)
            if now - last_seen > self.config.peer_timeout_s:
                missing.append(peer_id)
        return missing

    async def _send_heartbeats(self) -> None:
        assert self.transport is not None
        while True:
            payload = json.dumps(
                {
                    "type": "drone_heartbeat",
                    "id": self.config.drone_id,
                    "sent_at": time.time(),
                },
                separators=(",", ":"),
            ).encode("utf-8")

            targets = [(self.config.broadcast_ip, self.config.udp_port)]
            targets.extend((ip, self.config.udp_port) for ip in self.config.peer_unicast_ips)
            for target in targets:
                try:
                    self.transport.sendto(payload, target)
                except (OSError, socket.error):
                    LOGGER.exception("Failed to send heartbeat to %s:%d", *target)
            await asyncio.sleep(self.config.heartbeat_interval_s)


class AutonomousMission:
    def __init__(self, config: MissionConfig) -> None:
        self.config = config
        self.drone = System()
        self.telemetry = TelemetryCache(self.drone)
        self.peer_link: Optional[PeerLink] = None
        self.home_position: Any = None
        self.mission_started_s = 0.0
        self._failsafe_started = asyncio.Event()
        self._low_battery_handled = False
        self._arm_command_sent = False

    async def run(self) -> None:
        await self.drone.connect(system_address=self.config.connection_url)
        LOGGER.info("Connecting to vehicle at %s", self.config.connection_url)
        await self._wait_for_connection()

        self.telemetry.start()
        if (
            self.config.hotspot_containment.enabled
            and self.config.hotspot_containment.network_watchdog_enabled
        ):
            self.peer_link = PeerLink(self.config.hotspot_containment)
            await self.peer_link.start()

        monitor: Optional[asyncio.Task[None]] = None
        flight: Optional[asyncio.Task[None]] = None
        try:
            await self._wait_for_prearm_health()
            self.home_position = self.telemetry.position
            monitor = asyncio.create_task(self._monitor_failsafes())
            flight = asyncio.create_task(self._fly_mission())
            done, pending = await asyncio.wait(
                {monitor, flight},
                return_when=asyncio.FIRST_COMPLETED,
            )
            for task in done:
                exc = task.exception()
                if exc:
                    for pending_task in pending:
                        pending_task.cancel()
                    await asyncio.gather(*pending, return_exceptions=True)
                    raise exc
            for pending_task in pending:
                pending_task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
        except MissionAbort as exc:
            LOGGER.error("Mission abort: %s", exc)
            await self._soft_land(str(exc))
        except MissionReturnToLaunch as exc:
            LOGGER.error("Mission return-to-launch: %s", exc)
            await self._return_to_launch(str(exc))
        except asyncio.CancelledError:
            LOGGER.warning("Mission cancelled")
            if self._vehicle_may_need_failsafe():
                await self._soft_land("mission cancelled")
            else:
                LOGGER.warning("Mission cancelled before arming; no landing command sent")
            raise
        except Exception:
            LOGGER.exception("Unexpected mission error")
            await self._soft_land("unexpected mission error")
            raise
        finally:
            for task in (monitor, flight):
                if task and not task.done():
                    task.cancel()
            await asyncio.gather(
                *(task for task in (monitor, flight) if task),
                return_exceptions=True,
            )
            if self.peer_link:
                await self.peer_link.stop()
            await self.telemetry.stop()

    async def _wait_for_connection(self) -> None:
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                LOGGER.info("Vehicle discovered")
                return

    async def _wait_for_prearm_health(self) -> None:
        LOGGER.info("Waiting for global position, home position, GPS lock, and battery telemetry")
        deadline = time.monotonic() + self.config.prearm_health_timeout_s
        while time.monotonic() < deadline:
            health = self.telemetry.health
            gps_info = self.telemetry.gps_info
            battery_ready = (
                not self.config.require_battery_before_arm
                or self._battery_is_usable_for_arm()
            )
            if health and gps_info and self._gps_is_usable(gps_info, health):
                if getattr(health, "is_home_position_ok", False) and battery_ready:
                    LOGGER.info("Prearm health OK")
                    await self._wait_for_required_peers()
                    return
            await asyncio.sleep(0.5)
        raise MissionAbort("prearm health timeout: GPS/home position/battery telemetry not ready")

    async def _wait_for_required_peers(self) -> None:
        hotspot = self.config.hotspot_containment
        if (
            not hotspot.enabled
            or not hotspot.network_watchdog_enabled
            or not hotspot.require_peers_before_arm
            or not hotspot.expected_peer_ids
        ):
            return

        LOGGER.info("Waiting for hotspot peers before arming: %s", hotspot.expected_peer_ids)
        deadline = time.monotonic() + self.config.prearm_health_timeout_s
        while time.monotonic() < deadline:
            missing = self.peer_link.missing_peers() if self.peer_link else hotspot.expected_peer_ids
            if not missing:
                LOGGER.info("Required hotspot peers are visible")
                return
            await asyncio.sleep(0.5)
        raise MissionAbort(f"required hotspot peers missing before arm: {missing}")

    async def _fly_mission(self) -> None:
        self.mission_started_s = time.monotonic()
        await self.drone.action.set_takeoff_altitude(self.config.initial_takeoff_altitude_m)
        LOGGER.info("Arming")
        self._arm_command_sent = True
        await self.drone.action.arm()
        LOGGER.info(
            "Taking off slowly: initial %.1f m, final hover %.1f m",
            self.config.initial_takeoff_altitude_m,
            self.config.takeoff_altitude_m,
        )
        await self.drone.action.takeoff()
        await self._wait_until_altitude(self.config.initial_takeoff_altitude_m * 0.8, 45.0)
        await self._slow_climb_to_hover_altitude()
        if self.config.hover_before_mission_s > 0:
            LOGGER.info("Holding hover at %.1f m", self.config.takeoff_altitude_m)
            await asyncio.sleep(self.config.hover_before_mission_s)

        for waypoint in self.config.mission:
            await self._goto_waypoint(waypoint)

        if self.config.land_at_final_waypoint:
            await self._soft_land("mission complete")
        elif self.config.return_to_launch_on_complete:
            LOGGER.info("Mission complete; returning to launch")
            await self._return_to_launch("mission complete")
        else:
            LOGGER.info("Mission complete; holding position")

    async def _slow_climb_to_hover_altitude(self) -> None:
        if not self.home_position:
            raise MissionAbort("home position unavailable during takeoff")

        current_target_m = max(
            self.config.initial_takeoff_altitude_m,
            (self.telemetry.position.relative_altitude_m if self.telemetry.position else 0.0),
        )
        while current_target_m < self.config.takeoff_altitude_m:
            current_target_m = min(
                current_target_m + self.config.slow_takeoff_step_m,
                self.config.takeoff_altitude_m,
            )
            absolute_altitude_m = self.home_position.absolute_altitude_m + current_target_m
            LOGGER.info("Slow climb step: %.1f m", current_target_m)
            await self.drone.action.goto_location(
                self.home_position.latitude_deg,
                self.home_position.longitude_deg,
                absolute_altitude_m,
                0.0,
            )
            await self._wait_until_altitude(current_target_m - 0.2, 20.0)
            await asyncio.sleep(self.config.slow_takeoff_step_hold_s)

    async def _goto_waypoint(self, waypoint: Waypoint) -> None:
        if not self.home_position:
            raise MissionAbort("home position unavailable")

        lat, lon = offset_lat_lon(
            self.home_position.latitude_deg,
            self.home_position.longitude_deg,
            waypoint.north_m,
            waypoint.east_m,
        )
        absolute_altitude_m = (
            self.home_position.absolute_altitude_m + waypoint.relative_altitude_m
        )
        LOGGER.info(
            "Going to %s: north=%.1f east=%.1f rel_alt=%.1f",
            waypoint.name,
            waypoint.north_m,
            waypoint.east_m,
            waypoint.relative_altitude_m,
        )
        await self.drone.action.goto_location(
            lat,
            lon,
            absolute_altitude_m,
            waypoint.yaw_deg,
        )

        deadline = time.monotonic() + self.config.waypoint_timeout_s
        while time.monotonic() < deadline:
            position = self.telemetry.position
            if position:
                horizontal_m = distance_m(
                    position.latitude_deg,
                    position.longitude_deg,
                    lat,
                    lon,
                )
                vertical_m = abs(
                    position.relative_altitude_m - waypoint.relative_altitude_m
                )
                if (
                    horizontal_m <= self.config.waypoint_acceptance_radius_m
                    and vertical_m <= max(2.0, self.config.waypoint_acceptance_radius_m)
                ):
                    LOGGER.info("Reached %s", waypoint.name)
                    if waypoint.hold_s > 0:
                        await asyncio.sleep(waypoint.hold_s)
                    return
            await asyncio.sleep(0.5)
        raise MissionAbort(f"waypoint timeout: {waypoint.name}")

    async def _wait_until_altitude(self, relative_altitude_m: float, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            position = self.telemetry.position
            if position and position.relative_altitude_m >= relative_altitude_m:
                return
            await asyncio.sleep(0.5)
        raise MissionAbort("takeoff altitude timeout")

    async def _monitor_failsafes(self) -> None:
        await asyncio.sleep(1.0)
        last_gps_ok_s = time.monotonic()
        while True:
            await asyncio.sleep(0.5)
            if self._failsafe_started.is_set():
                continue

            now = time.monotonic()
            if self.mission_started_s:
                elapsed_s = now - self.mission_started_s
                if elapsed_s > self.config.max_mission_time_s:
                    raise MissionAbort("maximum mission time exceeded")

            if self.telemetry.in_air:
                if now - self.telemetry.last_position_s > self.config.telemetry_timeout_s:
                    raise MissionAbort("position telemetry timeout")
                if now - self.telemetry.last_gps_s > self.config.telemetry_timeout_s:
                    raise MissionAbort("GPS telemetry timeout")
                battery_age_s = latest_battery_age_s(self.telemetry)
                if (
                    battery_age_s is None
                    or battery_age_s > self.config.battery_telemetry_timeout_s
                ):
                    raise MissionAbort("battery telemetry timeout")

            if self.telemetry.gps_info and self.telemetry.health:
                if self._gps_is_usable(self.telemetry.gps_info, self.telemetry.health):
                    last_gps_ok_s = now
                elif self.telemetry.in_air and now - last_gps_ok_s > self.config.gps_loss_grace_s:
                    raise MissionAbort("GPS/global position degraded")

            if self.telemetry.position and self.home_position:
                position = self.telemetry.position
                radius_m = distance_m(
                    self.home_position.latitude_deg,
                    self.home_position.longitude_deg,
                    position.latitude_deg,
                    position.longitude_deg,
                )
                if radius_m > self.config.geofence_radius_m:
                    raise MissionAbort(f"software geofence exceeded: {radius_m:.1f} m")
                hotspot = self.config.hotspot_containment
                if hotspot.enabled and radius_m > hotspot.max_radius_m:
                    raise MissionAbort(
                        f"temporary hotspot containment radius exceeded: {radius_m:.1f} m"
                    )
                if position.relative_altitude_m > self.config.max_altitude_agl_m:
                    raise MissionAbort(
                        f"software altitude limit exceeded: {position.relative_altitude_m:.1f} m"
                    )

            hotspot = self.config.hotspot_containment
            if (
                self.telemetry.in_air
                and hotspot.enabled
                and hotspot.network_watchdog_enabled
                and hotspot.expected_peer_ids
                and self.peer_link
            ):
                missing = self.peer_link.missing_peers()
                if missing:
                    raise MissionAbort(f"hotspot peer heartbeat lost: {missing}")

            if latest_battery_age_s(self.telemetry) is not None:
                battery_state = describe_battery_telemetry(self.telemetry)
                if battery_state != "ok":
                    raise MissionAbort(f"battery telemetry invalid: {battery_state}")

                remaining = telemetry_battery_remaining_percent(self.telemetry)
                if remaining is None and self.config.require_battery_before_arm:
                    raise MissionAbort("battery remaining-percent telemetry unavailable")
                if remaining is None:
                    raise MissionAbort("battery remaining-percent telemetry unavailable")
                if remaining <= self.config.critical_battery_percent:
                    raise MissionAbort(f"critical battery: {remaining:.0%}")
                if (
                    remaining <= self.config.low_battery_percent
                    and not self._low_battery_handled
                ):
                    self._low_battery_handled = True
                    if self.config.low_battery_action == "land":
                        raise MissionAbort(f"low battery: {remaining:.0%}")
                    raise MissionReturnToLaunch(f"low battery: {remaining:.0%}")

    async def _soft_land(self, reason: str) -> None:
        if self._failsafe_started.is_set():
            return
        self._failsafe_started.set()
        if not self._vehicle_may_need_failsafe():
            LOGGER.warning("Skipping LAND because the vehicle was not armed or in air: %s", reason)
            return
        LOGGER.warning("Starting soft landing: %s", reason)
        try:
            await self.drone.action.land()
            await self._wait_until_landed(120.0)
        except Exception:
            LOGGER.exception("LAND command failed; trying RTL as fallback")
            try:
                await self.drone.action.return_to_launch()
            except Exception:
                LOGGER.exception("RTL fallback failed; use RC/manual mode immediately")

    async def _return_to_launch(self, reason: str) -> None:
        if self._failsafe_started.is_set():
            return
        self._failsafe_started.set()
        if not self._vehicle_may_need_failsafe():
            LOGGER.warning("Skipping RTL because the vehicle was not armed or in air: %s", reason)
            return
        LOGGER.warning("Starting RTL: %s", reason)
        try:
            await self.drone.action.return_to_launch()
            await self._wait_until_landed(240.0)
        except Exception:
            LOGGER.exception("RTL command failed; trying LAND as fallback")
            try:
                await self.drone.action.land()
                await self._wait_until_landed(120.0)
            except Exception:
                LOGGER.exception("LAND fallback failed; use RC/manual mode immediately")

    async def _wait_until_landed(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.telemetry.in_air is False:
                LOGGER.info("Vehicle reports landed")
                return
            await asyncio.sleep(1.0)
        LOGGER.warning("Landing wait timed out; verify vehicle state manually")

    def _vehicle_may_need_failsafe(self) -> bool:
        return bool(self._arm_command_sent or self.telemetry.armed or self.telemetry.in_air)

    def _gps_is_usable(self, gps_info: Any, health: Any) -> bool:
        satellites = getattr(gps_info, "num_satellites", 0)
        fix_text = str(getattr(gps_info, "fix_type", "")).upper()
        has_3d_fix = "3D" in fix_text or "RTK" in fix_text
        global_position_ok = getattr(health, "is_global_position_ok", False)
        return (
            satellites >= self.config.gps_min_satellites
            and has_3d_fix
            and global_position_ok
        )

    def _battery_is_usable_for_arm(self) -> bool:
        battery_age_s = latest_battery_age_s(self.telemetry)
        if battery_age_s is None:
            return False
        if battery_age_s > self.config.battery_telemetry_timeout_s:
            return False

        voltage = telemetry_battery_voltage_v(self.telemetry)
        remaining = telemetry_battery_remaining_percent(self.telemetry)
        if voltage is None or voltage <= 0:
            return False
        if remaining is None:
            return False
        if remaining < self.config.min_prearm_battery_percent:
            return False
        if (
            self.config.min_prearm_battery_voltage_v > 0
            and voltage < self.config.min_prearm_battery_voltage_v
        ):
            return False
        return True


def load_config(path: Path) -> MissionConfig:
    raw = json.loads(path.read_text(encoding="utf-8"))
    hotspot_raw = raw.get("hotspot_containment", {})
    hotspot_containment = HotspotContainmentConfig(
        enabled=bool(hotspot_raw.get("enabled", False)),
        max_radius_m=float(hotspot_raw.get("max_radius_m", 25.0)),
        network_watchdog_enabled=bool(hotspot_raw.get("network_watchdog_enabled", False)),
        drone_id=str(hotspot_raw.get("drone_id", "drone-1")),
        expected_peer_ids=[
            str(peer_id) for peer_id in hotspot_raw.get("expected_peer_ids", [])
        ],
        udp_port=int(hotspot_raw.get("udp_port", 50555)),
        broadcast_ip=str(hotspot_raw.get("broadcast_ip", "255.255.255.255")),
        peer_unicast_ips=[
            str(peer_ip) for peer_ip in hotspot_raw.get("peer_unicast_ips", [])
        ],
        heartbeat_interval_s=float(hotspot_raw.get("heartbeat_interval_s", 1.0)),
        peer_timeout_s=float(hotspot_raw.get("peer_timeout_s", 5.0)),
        require_peers_before_arm=bool(hotspot_raw.get("require_peers_before_arm", True)),
    )
    mission = [
        Waypoint(
            name=str(item.get("name", f"wp-{index + 1}")),
            north_m=float(item["north_m"]),
            east_m=float(item["east_m"]),
            relative_altitude_m=float(item["relative_altitude_m"]),
            hold_s=float(item.get("hold_s", 0.0)),
            yaw_deg=float(item.get("yaw_deg", 0.0)),
        )
        for index, item in enumerate(raw["mission"])
    ]
    if not mission:
        raise ValueError("mission must contain at least one waypoint")

    low_battery_action = str(raw.get("low_battery_action", "return")).lower()
    if low_battery_action not in {"return", "land"}:
        raise ValueError("low_battery_action must be 'return' or 'land'")

    config = MissionConfig(
        connection_url=normalize_connection_url(
            str(raw.get("connection_url", f"serial://auto:{DEFAULT_USB_SERIAL_BAUD}"))
        ),
        takeoff_altitude_m=float(raw.get("takeoff_altitude_m", 5.0)),
        initial_takeoff_altitude_m=float(raw.get("initial_takeoff_altitude_m", 1.2)),
        slow_takeoff_step_m=float(raw.get("slow_takeoff_step_m", 0.5)),
        slow_takeoff_step_hold_s=float(raw.get("slow_takeoff_step_hold_s", 2.0)),
        hover_before_mission_s=float(raw.get("hover_before_mission_s", 20.0)),
        return_to_launch_on_complete=bool(raw.get("return_to_launch_on_complete", True)),
        land_at_final_waypoint=bool(raw.get("land_at_final_waypoint", False)),
        waypoint_acceptance_radius_m=float(raw.get("waypoint_acceptance_radius_m", 3.0)),
        waypoint_timeout_s=float(raw.get("waypoint_timeout_s", 90.0)),
        max_mission_time_s=float(raw.get("max_mission_time_s", 600.0)),
        geofence_radius_m=float(raw.get("geofence_radius_m", 60.0)),
        max_altitude_agl_m=float(raw.get("max_altitude_agl_m", 25.0)),
        gps_min_satellites=int(raw.get("gps_min_satellites", 8)),
        gps_loss_grace_s=float(raw.get("gps_loss_grace_s", 5.0)),
        telemetry_timeout_s=float(raw.get("telemetry_timeout_s", 3.0)),
        battery_telemetry_timeout_s=float(
            raw.get("battery_telemetry_timeout_s", raw.get("telemetry_timeout_s", 3.0))
        ),
        require_battery_before_arm=bool(raw.get("require_battery_before_arm", True)),
        min_prearm_battery_percent=float(
            raw.get("min_prearm_battery_percent", raw.get("low_battery_percent", 0.3))
        ),
        min_prearm_battery_voltage_v=float(raw.get("min_prearm_battery_voltage_v", 0.0)),
        low_battery_percent=float(raw.get("low_battery_percent", 0.3)),
        critical_battery_percent=float(raw.get("critical_battery_percent", 0.2)),
        low_battery_action=low_battery_action,
        prearm_health_timeout_s=float(raw.get("prearm_health_timeout_s", 120.0)),
        hotspot_containment=hotspot_containment,
        mission=mission,
    )
    validate_config(config)
    return config


def normalize_connection_url(connection_url: str) -> str:
    connection_url = connection_url.strip()
    if connection_url == "serial://auto":
        return autodetect_usb_serial_url(DEFAULT_USB_SERIAL_BAUD)
    if connection_url.startswith("serial://auto:"):
        baud_text = connection_url.removeprefix("serial://auto:")
        try:
            baud = int(baud_text)
        except ValueError as exc:
            raise ValueError(f"invalid auto serial baud: {baud_text}") from exc
        return autodetect_usb_serial_url(baud)
    if connection_url.startswith("udp://:"):
        return connection_url.replace("udp://:", "udpin://0.0.0.0:", 1)
    if connection_url.startswith("udpin://:"):
        return connection_url.replace("udpin://:", "udpin://0.0.0.0:", 1)
    return connection_url


def autodetect_usb_serial_url(baud: int) -> str:
    devices = find_usb_serial_devices()
    if not devices:
        raise ValueError(
            "no USB serial device found; connect Pixhawk USB and check /dev/ttyACM* or /dev/ttyUSB*"
        )
    if len(devices) > 1:
        tty_acm_devices = [device for device in devices if Path(device).name.startswith("ttyACM")]
        if tty_acm_devices:
            LOGGER.warning(
                "Multiple USB serial devices found (%s); using %s",
                ", ".join(devices),
                tty_acm_devices[0],
            )
            devices = tty_acm_devices
        else:
            joined_devices = ", ".join(devices)
            raise ValueError(
                f"multiple USB serial devices found ({joined_devices}); set connection_url explicitly"
            )
    connection_url = f"serial://{devices[0]}:{baud}"
    LOGGER.info("Auto-detected USB serial connection: %s", connection_url)
    return connection_url


def find_usb_serial_devices() -> list[str]:
    devices: list[str] = []
    dev_dir = Path("/dev")
    for pattern in USB_SERIAL_PATTERNS:
        devices.extend(str(path) for path in dev_dir.glob(pattern) if path.exists())
    return sorted(devices)


def validate_config(config: MissionConfig) -> None:
    if config.takeoff_altitude_m <= 0:
        raise ValueError("takeoff_altitude_m must be greater than 0")
    if config.initial_takeoff_altitude_m <= 0:
        raise ValueError("initial_takeoff_altitude_m must be greater than 0")
    if config.initial_takeoff_altitude_m > config.takeoff_altitude_m:
        raise ValueError("initial_takeoff_altitude_m cannot exceed takeoff_altitude_m")
    if config.slow_takeoff_step_m <= 0:
        raise ValueError("slow_takeoff_step_m must be greater than 0")
    if config.slow_takeoff_step_hold_s < 0:
        raise ValueError("slow_takeoff_step_hold_s cannot be negative")
    if config.max_altitude_agl_m < config.takeoff_altitude_m:
        raise ValueError("max_altitude_agl_m cannot be lower than takeoff_altitude_m")
    if config.hotspot_containment.drone_id in config.hotspot_containment.expected_peer_ids:
        raise ValueError("hotspot_containment.drone_id cannot be in expected_peer_ids")
    if config.telemetry_timeout_s <= 0:
        raise ValueError("telemetry_timeout_s must be greater than 0")
    if config.battery_telemetry_timeout_s <= 0:
        raise ValueError("battery_telemetry_timeout_s must be greater than 0")
    for field_name in (
        "low_battery_percent",
        "critical_battery_percent",
        "min_prearm_battery_percent",
    ):
        value = getattr(config, field_name)
        if not 0 <= value <= 1:
            raise ValueError(f"{field_name} must be between 0.0 and 1.0")
    if config.critical_battery_percent > config.low_battery_percent:
        raise ValueError("critical_battery_percent cannot exceed low_battery_percent")
    if config.min_prearm_battery_percent < config.low_battery_percent:
        raise ValueError("min_prearm_battery_percent should be at least low_battery_percent")
    if config.min_prearm_battery_voltage_v < 0:
        raise ValueError("min_prearm_battery_voltage_v cannot be negative")


def offset_lat_lon(
    origin_lat_deg: float,
    origin_lon_deg: float,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    lat_rad = math.radians(origin_lat_deg)
    d_lat = north_m / EARTH_RADIUS_M
    d_lon = east_m / (EARTH_RADIUS_M * math.cos(lat_rad))
    return (
        origin_lat_deg + math.degrees(d_lat),
        origin_lon_deg + math.degrees(d_lon),
    )


def distance_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1 = math.radians(lat1_deg)
    lat2 = math.radians(lat2_deg)
    d_lat = lat2 - lat1
    d_lon = math.radians(lon2_deg - lon1_deg)
    a = (
        math.sin(d_lat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2.0) ** 2
    )
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


async def run_telemetry_monitor(config: MissionConfig) -> int:
    request_mavlink_data_streams(config.connection_url)

    drone = System()
    await drone.connect(system_address=config.connection_url)
    LOGGER.info("Connecting to vehicle at %s", config.connection_url)
    async for state in drone.core.connection_state():
        if state.is_connected:
            LOGGER.info("Vehicle discovered")
            break

    telemetry = TelemetryCache(drone)
    telemetry.start()
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    LOGGER.info("Streaming Pixhawk telemetry. Press Ctrl+C to stop.")
    try:
        while not stop_event.is_set():
            LOGGER.info(format_telemetry_snapshot(telemetry))
            try:
                await asyncio.wait_for(stop_event.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                pass
    finally:
        await telemetry.stop()
    return 0


async def run_raw_mavlink_probe(config: MissionConfig, duration_s: float) -> int:
    request_mavlink_data_streams(config.connection_url)

    drone = System()
    await drone.connect(system_address=config.connection_url)
    LOGGER.info("Connecting to vehicle at %s", config.connection_url)
    async for state in drone.core.connection_state():
        if state.is_connected:
            LOGGER.info("Vehicle discovered")
            break

    message_names = [
        "SYS_STATUS",
        "BATTERY_STATUS",
        "GLOBAL_POSITION_INT",
        "VFR_HUD",
    ]
    deadline_s = time.monotonic() + duration_s
    seen: set[str] = set()
    tasks = [
        asyncio.create_task(_probe_raw_mavlink_message(drone, message_name, seen, deadline_s))
        for message_name in message_names
    ]
    try:
        await asyncio.wait_for(
            asyncio.gather(*tasks, return_exceptions=True),
            timeout=duration_s + 2.0,
        )
    except asyncio.TimeoutError:
        pass
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    missing = [message_name for message_name in message_names if message_name not in seen]
    if missing:
        LOGGER.warning("No raw MAVLink messages seen during probe: %s", ", ".join(missing))
    return 0


async def _probe_raw_mavlink_message(
    drone: System,
    message_name: str,
    seen: set[str],
    deadline_s: float,
) -> None:
    async for message in drone.mavlink_direct.message(message_name):
        if time.monotonic() > deadline_s:
            return
        if message_name in seen:
            continue
        seen.add(message_name)
        LOGGER.info("Raw %s fields: %s", message_name, message.fields_json)
        if len(seen) >= 4:
            return


def format_telemetry_snapshot(telemetry: TelemetryCache) -> str:
    position = telemetry.position
    gps_info = telemetry.gps_info
    health = telemetry.health
    battery = telemetry.battery

    lat = format_number(getattr(position, "latitude_deg", None), precision=7)
    lon = format_number(getattr(position, "longitude_deg", None), precision=7)
    rel_alt = format_number(
        first_existing_value(
            getattr(position, "relative_altitude_m", None),
            telemetry.raw_altitude.get("relative_altitude_m"),
        ),
        precision=1,
    )
    abs_alt = format_number(
        first_existing_value(
            getattr(position, "absolute_altitude_m", None),
            telemetry.raw_altitude.get("absolute_altitude_m"),
            telemetry.raw_altitude.get("vfr_altitude_m"),
        ),
        precision=1,
    )
    raw_altitude_source = format_value(telemetry.raw_altitude.get("source"))
    sats = format_value(getattr(gps_info, "num_satellites", None))
    fix = format_value(getattr(gps_info, "fix_type", None))
    battery_voltage = format_number(telemetry_battery_voltage_v(telemetry), precision=2)
    battery_state = describe_battery_telemetry(telemetry)
    battery_percent_value = telemetry_battery_remaining_percent(telemetry)
    battery_percent = (
        format_percent(battery_percent_value)
        if battery_state == "ok"
        else "unknown"
    )
    battery_current = format_number(telemetry_battery_current_a(telemetry), precision=2)
    battery_age_value = latest_battery_age_s(telemetry)
    battery_age = "n/a" if battery_age_value is None else format_number(battery_age_value, 1)
    battery_source = battery_telemetry_source(telemetry)
    global_ok = format_bool(getattr(health, "is_global_position_ok", None))
    home_ok = format_bool(getattr(health, "is_home_position_ok", None))

    return (
        f"armed={format_value(telemetry.armed)} "
        f"in_air={format_value(telemetry.in_air)} "
        f"mode={format_value(telemetry.flight_mode)} "
        f"battery_state={battery_state} "
        f"battery_source={battery_source} "
        f"battery_v={battery_voltage} battery_current_a={battery_current} "
        f"battery={battery_percent} battery_age_s={battery_age} "
        f"gps_fix={fix} sats={sats} "
        f"global_ok={global_ok} home_ok={home_ok} "
        f"lat={lat} lon={lon} rel_alt_m={rel_alt} abs_alt_m={abs_alt} "
        f"raw_alt_source={raw_altitude_source}"
    )


def format_value(value: Any) -> str:
    if value is None:
        return "n/a"
    return str(value)


def format_bool(value: Any) -> str:
    if value is None:
        return "n/a"
    return "yes" if bool(value) else "no"


def describe_battery_telemetry(telemetry: TelemetryCache) -> str:
    battery_age_s = latest_battery_age_s(telemetry)
    if battery_age_s is None:
        return "missing"

    if battery_age_s > 5.0:
        return "stale"

    voltage = telemetry_battery_voltage_v(telemetry)
    remaining = telemetry_battery_remaining_percent(telemetry)
    if voltage is None or voltage <= 0:
        return "invalid_voltage"
    if remaining is None:
        return "missing_percent"
    if not 0 <= remaining <= 1:
        return "invalid_percent"
    return "ok"


def latest_battery_age_s(telemetry: TelemetryCache) -> Optional[float]:
    latest_s = max(telemetry.last_battery_s, telemetry.last_raw_battery_s)
    if latest_s <= 0:
        return None
    return time.monotonic() - latest_s


def battery_telemetry_source(telemetry: TelemetryCache) -> str:
    if telemetry.last_battery_s <= 0 and telemetry.last_raw_battery_s <= 0:
        return "n/a"
    if telemetry.last_battery_s >= telemetry.last_raw_battery_s:
        return "mavsdk"
    return str(telemetry.raw_battery.get("source", "raw_mavlink"))


def telemetry_battery_voltage_v(telemetry: TelemetryCache) -> Optional[float]:
    return first_positive_number(
        getattr(telemetry.battery, "voltage_v", None),
        telemetry.raw_battery.get("voltage_v"),
    )


def telemetry_battery_current_a(telemetry: TelemetryCache) -> Optional[float]:
    return first_existing_number(
        first_existing_attr(
            telemetry.battery,
            "current_battery_a",
            "current_a",
        ),
        telemetry.raw_battery.get("current_battery_a"),
    )


def telemetry_battery_remaining_percent(telemetry: TelemetryCache) -> Optional[float]:
    return first_battery_fraction(
        getattr(telemetry.battery, "remaining_percent", None),
        telemetry.raw_battery.get("remaining_percent"),
    )


def battery_status_voltage_v(voltages: Any) -> Optional[float]:
    if not isinstance(voltages, list):
        return None

    valid_cell_mv = []
    for voltage in voltages:
        voltage_mv = get_finite_float(voltage)
        if voltage_mv is None:
            continue
        if voltage_mv <= 0 or voltage_mv >= 65535:
            continue
        valid_cell_mv.append(voltage_mv)

    if not valid_cell_mv:
        return None
    return sum(valid_cell_mv) / 1000.0


def format_number(value: Any, precision: int) -> str:
    value = get_finite_float(value)
    if value is None:
        return "n/a"
    return f"{value:.{precision}f}"


def format_percent(value: Any) -> str:
    value = get_finite_float(value)
    if value is None:
        return "n/a"
    return f"{value:.0%}"


def get_finite_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def first_existing_attr(source: Any, *names: str) -> Any:
    if source is None:
        return None
    for name in names:
        value = getattr(source, name, None)
        if value is not None:
            return value
    return None


def first_existing_value(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def first_existing_field(fields: dict[str, Any], *names: str) -> Any:
    for name in names:
        if name in fields:
            return fields[name]
    lower_fields = {key.lower(): value for key, value in fields.items()}
    for name in names:
        value = lower_fields.get(name.lower())
        if value is not None:
            return value
    return None


def first_existing_number(*values: Any) -> Optional[float]:
    for value in values:
        number = get_finite_float(value)
        if number is not None:
            return number
    return None


def first_positive_number(*values: Any) -> Optional[float]:
    for value in values:
        number = get_finite_float(value)
        if number is not None and number > 0:
            return number
    return None


def first_battery_fraction(*values: Any) -> Optional[float]:
    for value in values:
        number = get_finite_float(value)
        if number is None or number < 0:
            continue
        if number <= 1:
            return number
        if number <= 100:
            return number / 100.0
    return None


def request_mavlink_data_streams(connection_url: str) -> None:
    serial_connection = parse_serial_connection_url(connection_url)
    if not serial_connection:
        LOGGER.debug("Skipping MAVLink data stream request for non-serial URL: %s", connection_url)
        return

    device, baud = serial_connection
    try:
        from pymavlink import mavutil
    except ImportError:
        LOGGER.warning("pymavlink is not installed; skipping MAVLink data stream request")
        return

    master: Any = None
    try:
        master = mavutil.mavlink_connection(device, baud=baud)
        heartbeat = master.wait_heartbeat(timeout=5)
        if heartbeat is None:
            LOGGER.warning("No MAVLink heartbeat while requesting data streams on %s", device)
            return

        requested_streams = [
            ("POSITION", mavutil.mavlink.MAV_DATA_STREAM_POSITION),
            ("EXTRA1", mavutil.mavlink.MAV_DATA_STREAM_EXTRA1),
            ("EXTRA2", mavutil.mavlink.MAV_DATA_STREAM_EXTRA2),
            ("EXTENDED_STATUS", mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS),
        ]
        for stream_name, stream_id in requested_streams:
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                stream_id,
                REQUESTED_MAVLINK_STREAM_RATE_HZ,
                1,
            )
        requested_messages = [
            ("BATTERY_STATUS", getattr(mavutil.mavlink, "MAVLINK_MSG_ID_BATTERY_STATUS", None)),
            ("SYS_STATUS", getattr(mavutil.mavlink, "MAVLINK_MSG_ID_SYS_STATUS", None)),
            (
                "GLOBAL_POSITION_INT",
                getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", None),
            ),
            ("VFR_HUD", getattr(mavutil.mavlink, "MAVLINK_MSG_ID_VFR_HUD", None)),
        ]
        interval_us = int(1_000_000 / REQUESTED_BATTERY_MESSAGE_RATE_HZ)
        for message_name, message_id in requested_messages:
            if message_id is None:
                LOGGER.debug("MAVLink message id unavailable: %s", message_name)
                continue
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        LOGGER.info(
            "Requested MAVLink telemetry streams and battery messages at %d Hz on %s",
            REQUESTED_MAVLINK_STREAM_RATE_HZ,
            device,
        )
    except ModuleNotFoundError as exc:
        if exc.name == "serial":
            LOGGER.warning("pyserial is not installed; skipping MAVLink data stream request")
            return
        raise
    except Exception:
        LOGGER.exception("Failed to request MAVLink data streams on %s", device)
    finally:
        if master:
            master.close()


def parse_serial_connection_url(connection_url: str) -> Optional[tuple[str, int]]:
    if not connection_url.startswith("serial://"):
        return None

    serial_target = connection_url.removeprefix("serial://")
    if ":" not in serial_target:
        return None

    device, baud_text = serial_target.rsplit(":", 1)
    if not device:
        return None

    try:
        baud = int(baud_text)
    except ValueError:
        return None

    return device, baud


async def amain() -> int:
    parser = argparse.ArgumentParser(description="Run a GPS-only autonomous drone mission")
    parser.add_argument(
        "--config",
        default="missions/example_mission.json",
        type=Path,
        help="Path to the mission JSON file",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Console logging verbosity",
    )
    parser.add_argument(
        "--telemetry-only",
        action="store_true",
        help="Connect and print live Pixhawk telemetry without arming or flying",
    )
    parser.add_argument(
        "--raw-mavlink-probe",
        action="store_true",
        help="Print one raw battery/altitude MAVLink message from the Pixhawk and exit",
    )
    parser.add_argument(
        "--raw-mavlink-probe-seconds",
        default=10.0,
        type=float,
        help="How long to wait for raw MAVLink probe messages",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config = load_config(args.config)
    if args.raw_mavlink_probe:
        return await run_raw_mavlink_probe(config, args.raw_mavlink_probe_seconds)
    if args.telemetry_only:
        return await run_telemetry_monitor(config)

    request_mavlink_data_streams(config.connection_url)
    mission = AutonomousMission(config)

    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    mission_task = asyncio.create_task(mission.run())
    stop_task = asyncio.create_task(stop_event.wait())
    done, pending = await asyncio.wait(
        {mission_task, stop_task},
        return_when=asyncio.FIRST_COMPLETED,
    )
    if stop_task in done and not mission_task.done():
        LOGGER.warning("Shutdown requested")
        mission_task.cancel()
    for task in pending:
        task.cancel()
    await asyncio.gather(*pending, return_exceptions=True)
    result = await asyncio.gather(mission_task, return_exceptions=True)
    if result and isinstance(result[0], asyncio.CancelledError):
        return 130
    if result and isinstance(result[0], BaseException):
        raise result[0]
    return 0


def main() -> None:
    try:
        raise SystemExit(asyncio.run(amain()))
    except KeyboardInterrupt:
        LOGGER.warning("Interrupted by user")
        raise SystemExit(130)


if __name__ == "__main__":
    main()
