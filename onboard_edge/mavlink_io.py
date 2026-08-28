"""
Small asynchronous wrapper around pymavlink for companion-computer control.

The reader loop never blocks the asyncio event loop for telemetry: it polls
MAVLink with ``blocking=False`` and keeps a timestamped cache of recent messages.
Command helpers remain conservative and wait for COMMAND_ACK where MAVLink
provides one.
"""

from __future__ import annotations

import asyncio
import json
import glob
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from pymavlink import mavutil


LOGGER = logging.getLogger(__name__)
DEFAULT_BAUD = 115200
USB_SERIAL_PATTERNS = ("ttyACM*", "ttyUSB*")


class MavlinkError(RuntimeError):
    """Raised when a MAVLink command or connection action fails."""


@dataclass(frozen=True)
class ConnectionSpec:
    device: str
    baud: int = DEFAULT_BAUD
    is_serial: bool = True


@dataclass(frozen=True)
class CachedMessage:
    message: Any
    received_at_s: float


def parse_connection_url(url: str) -> ConnectionSpec:
    """Parse MAVSDK-style URLs into pymavlink connection settings."""

    if url.startswith("serial://"):
        target = url.removeprefix("serial://")
        device, baud = _split_device_and_baud(target)
        # Pass 'auto' through to connect() for the new handshake logic
        return ConnectionSpec(device=device, baud=baud, is_serial=True)

    if url.startswith("udpin://"):
        return ConnectionSpec(device="udpin:" + url.removeprefix("udpin://"), is_serial=False)
    if url.startswith("udpout://"):
        return ConnectionSpec(device="udpout:" + url.removeprefix("udpout://"), is_serial=False)
    if url.startswith("tcp://"):
        return ConnectionSpec(device="tcp:" + url.removeprefix("tcp://"), is_serial=False)

    if url.startswith(("udpin:", "udpout:", "tcp:", "udp:")):
        return ConnectionSpec(device=url, is_serial=False)

    device, baud = _split_device_and_baud(url)
    return ConnectionSpec(device=device, baud=baud, is_serial=True)


def _split_device_and_baud(target: str) -> tuple[str, int]:
    if ":" not in target:
        return target, DEFAULT_BAUD
    device, baud_text = target.rsplit(":", 1)
    try:
        baud = int(baud_text)
    except ValueError:
        return target, DEFAULT_BAUD
    return device, baud


def autodetect_pixhawk_connection(
    source_system: int = 255, 
    source_component: int = 191
) -> Any:
    config_file = Path("config.json")
    
    # 1. Local Config Caching
    if config_file.exists():
        try:
            with open(config_file, "r") as f:
                config = json.load(f)
                cached_port = config.get("pixhawk_port")
                cached_baud = config.get("pixhawk_baud", 921600)
            
            if cached_port:
                LOGGER.info(f"Probing cached Pixhawk port: {cached_port} at {cached_baud} baud")
                master = mavutil.mavlink_connection(
                    cached_port, 
                    baud=cached_baud, 
                    source_system=source_system, 
                    source_component=source_component,
                    autoreconnect=True
                )
                # 2. Fast Cache Validation
                msg = master.wait_heartbeat(blocking=True, timeout=2.0)  # type: ignore
                if msg and msg.get_srcSystem() > 0:
                    LOGGER.info(f"Cache validation successful for {cached_port} at {cached_baud}")
                    return master
                else:
                    LOGGER.warning("Cache validation failed, closing cached port.")
                    master.close()  # type: ignore
        except Exception as e:
            LOGGER.warning(f"Failed to read or validate cached config: {e}")

    # 3. Comprehensive Port Scanning
    LOGGER.info("Scanning for Pixhawk connection...")
    patterns = ["/dev/ttyAMA*", "/dev/ttyUSB*", "/dev/ttyACM*"]
    ports = []
    for p in patterns:
        ports.extend(glob.glob(p))
        
    baud_rates = [921600, 115200, 57600]
        
    # 4. Handshake Verification (Nested Matrix Scan)
    for port in ports:
        for baud in baud_rates:
            LOGGER.info(f"Probing {port} at {baud} baud...")
            try:
                master = mavutil.mavlink_connection(
                    port, 
                    baud=baud, 
                    source_system=source_system, 
                    source_component=source_component,
                    autoreconnect=True
                )
                msg = master.wait_heartbeat(blocking=True, timeout=1.0)  # type: ignore
                if msg and msg.get_srcSystem() > 0:
                    # 5. Auto-Save & Return
                    LOGGER.info(f"Valid Pixhawk found at {port} with {baud} baud")
                    try:
                        with open(config_file, "w") as f:
                            json.dump({"pixhawk_port": port, "pixhawk_baud": baud}, f)
                    except Exception as e:
                        LOGGER.warning(f"Could not save to config.json: {e}")
                    return master
                master.close()  # type: ignore
            except Exception as e:
                LOGGER.debug(f"Failed to probe {port} at {baud}: {e}")
            
    raise ConnectionError("No responding Pixhawk flight controller found on any scanned serial ports or baud rates.")


class MavlinkConnection:
    """Owns a pymavlink connection and a non-blocking telemetry cache."""

    def __init__(
        self,
        connection_url: str,
        *,
        source_system: int = 255,
        source_component: int = 191,
    ) -> None:
        self.connection_url = connection_url
        self.source_system = source_system
        self.source_component = source_component
        self.master: Any = None
        self.target_system = 0
        self.target_component = 0
        self._cache: dict[str, CachedMessage] = {}
        self._reader_task: Optional[asyncio.Task[None]] = None
        self._stopping = False
        self._lock = asyncio.Lock()

    async def connect(self, heartbeat_timeout_s: float = 30.0) -> None:
        spec = parse_connection_url(self.connection_url)
        LOGGER.info("Connecting to Pixhawk at %s", self.connection_url)
        
        if spec.device == "auto":
            self.master = await asyncio.to_thread(
                autodetect_pixhawk_connection,
                self.source_system,
                self.source_component
            )
            heartbeat = await asyncio.to_thread(
                self.master.wait_heartbeat,
                timeout=2.0,
            )
        else:
            self.master = mavutil.mavlink_connection(
                spec.device,
                baud=spec.baud,
                source_system=self.source_system,
                source_component=self.source_component,
                autoreconnect=True,
            )
            heartbeat = await asyncio.to_thread(
                self.master.wait_heartbeat,
                timeout=heartbeat_timeout_s,
            )

        if heartbeat is None:
            raise MavlinkError(f"no heartbeat from Pixhawk within {heartbeat_timeout_s:.0f}s")

        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self._cache_message(heartbeat)
        LOGGER.info(
            "MAVLink heartbeat received from system=%s component=%s",
            self.target_system,
            self.target_component,
        )

    async def start(self, requested_rates_hz: Optional[dict[str, float]] = None) -> None:
        if self.master is None:
            raise MavlinkError("cannot start reader before connect")
        self._stopping = False
        self._reader_task = asyncio.create_task(self._reader_loop())
        await asyncio.sleep(0)
        await self.request_data_streams()
        for message_name, rate_hz in (requested_rates_hz or {}).items():
            message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
            if message_id is not None:
                await self.set_message_interval(int(message_id), rate_hz)

    async def close(self) -> None:
        self._stopping = True
        if self._reader_task:
            self._reader_task.cancel()
            await asyncio.gather(self._reader_task, return_exceptions=True)
        if self.master is not None:
            self.master.close()

    async def _reader_loop(self) -> None:
        assert self.master is not None
        while not self._stopping:
            try:
                msg = self.master.recv_match(blocking=False)
            except Exception:
                LOGGER.exception("MAVLink receive failed")
                await asyncio.sleep(0.25)
                continue
            if msg is None:
                await asyncio.sleep(0.02)
                continue
            if msg.get_type() == "BAD_DATA":
                continue
            if not msg or msg.get_srcSystem() != self.target_system:
                continue
            self._cache_message(msg)

    def _cache_message(self, msg: Any) -> None:
        self._cache[msg.get_type()] = CachedMessage(msg, time.monotonic())

    def latest(self, message_type: str, max_age_s: Optional[float] = None) -> Optional[CachedMessage]:
        cached = self._cache.get(message_type)
        if cached is None:
            return None
        if max_age_s is not None and time.monotonic() - cached.received_at_s > max_age_s:
            return None
        return cached

    def latest_message(self, message_type: str, max_age_s: Optional[float] = None) -> Any:
        cached = self.latest(message_type, max_age_s)
        return cached.message if cached else None

    async def wait_for_message(
        self,
        message_type: str,
        *,
        timeout_s: float = 5.0,
        newer_than_s: Optional[float] = None,
        predicate: Optional[Callable[[Any], bool]] = None,
    ) -> Any:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            cached = self.latest(message_type)
            if cached is not None:
                is_new = newer_than_s is None or cached.received_at_s >= newer_than_s
                if is_new and (predicate is None or predicate(cached.message)):
                    return cached.message
            await asyncio.sleep(0.05)
        raise MavlinkError(f"timed out waiting for {message_type}")

    async def request_data_streams(self, rate_hz: int = 4) -> None:
        for stream_id in (
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        ):
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                stream_id,
                rate_hz,
                1,
            )

    async def set_message_interval(self, message_id: int, rate_hz: float) -> None:
        interval_us = -1 if rate_hz <= 0 else int(1_000_000 / rate_hz)
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
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

    async def command_long(
        self,
        command: int,
        params: tuple[float, float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0, 0),
        *,
        timeout_s: float = 5.0,
        require_ack: bool = True,
    ) -> Any:
        sent_at = time.monotonic()
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            command,
            0,
            *params,
        )
        if not require_ack:
            return None

        ack = await self.wait_for_message(
            "COMMAND_ACK",
            timeout_s=timeout_s,
            newer_than_s=sent_at,
            predicate=lambda msg: int(getattr(msg, "command", -1)) == int(command),
        )
        result = int(getattr(ack, "result", mavutil.mavlink.MAV_RESULT_FAILED))
        if result not in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            raise MavlinkError(
                f"command {command} rejected with {mavlink_result_name(result)} ({result})"
            )
        return ack

    async def set_mode(self, mode_name: str, timeout_s: float = 8.0) -> None:
        mode_mapping = self.master.mode_mapping() or {}
        mode_id = mode_mapping.get(mode_name)
        if mode_id is None:
            available = ", ".join(sorted(mode_mapping)) or "none"
            raise MavlinkError(f"mode {mode_name!r} is unavailable; modes reported: {available}")

        sent_at = time.monotonic()
        self.master.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        await self.wait_for_message(
            "HEARTBEAT",
            timeout_s=timeout_s,
            newer_than_s=sent_at,
            predicate=lambda msg: self.mode_name_from_heartbeat(msg) == mode_name,
        )

    def mode_name_from_heartbeat(self, heartbeat: Any) -> str:
        if heartbeat is None:
            return "UNKNOWN"
        try:
            mapping = self.master.mode_mapping() or {}
            reverse = {mode_id: name for name, mode_id in mapping.items()}
            return reverse.get(int(heartbeat.custom_mode), f"CUSTOM_{heartbeat.custom_mode}")
        except Exception:
            return "UNKNOWN"

    async def arm(self) -> None:
        await self.command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1, 0, 0, 0, 0, 0, 0),
            timeout_s=8.0,
        )

    async def disarm(self) -> None:
        await self.command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (0, 0, 0, 0, 0, 0, 0),
            timeout_s=8.0,
        )

    async def takeoff(
        self,
        altitude_m: float,
        *,
        lat_deg: float = 0.0,
        lon_deg: float = 0.0,
    ) -> None:
        await self.command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            (0, 0, 0, float("nan"), lat_deg, lon_deg, altitude_m),
            timeout_s=10.0,
        )

    async def land(self) -> None:
        await self.command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            (0, 0, 0, float("nan"), 0, 0, 0),
            timeout_s=5.0,
            require_ack=False,
        )

    async def rtl(self) -> None:
        await self.command_long(
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            timeout_s=5.0,
            require_ack=False,
        )

    def send_global_position_target(
        self,
        lat_deg: float,
        lon_deg: float,
        relative_alt_m: float,
        *,
        yaw_rad: Optional[float] = None,
    ) -> None:
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        yaw_value = 0.0
        if yaw_rad is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        else:
            yaw_value = yaw_rad
        self.master.mav.set_position_target_global_int_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat_deg * 1e7),
            int(lon_deg * 1e7),
            relative_alt_m,
            0,
            0,
            0,
            0,
            0,
            0,
            yaw_value,
            0,
        )

    def send_local_position_target(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
        *,
        yaw_rad: Optional[float] = None,
    ) -> None:
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        yaw_value = 0.0
        if yaw_rad is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        else:
            yaw_value = yaw_rad
        self.master.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north_m,
            east_m,
            down_m,
            0,
            0,
            0,
            0,
            0,
            0,
            yaw_value,
            0,
        )

    def send_local_velocity_target(
        self,
        vx_m_s: float,
        vy_m_s: float,
        vz_m_s: float,
        *,
        yaw_rate_rad_s: float = 0.0,
    ) -> None:
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self.master.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0,
            0,
            0,
            vx_m_s,
            vy_m_s,
            vz_m_s,
            0,
            0,
            0,
            0,
            yaw_rate_rad_s,
        )

    def send_body_velocity_target(
        self,
        vx_m_s: float,
        vy_m_s: float,
        vz_m_s: float,
        *,
        yaw_rate_rad_s: float = 0.0,
    ) -> None:
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        self.master.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0,
            0,
            0,
            vx_m_s,
            vy_m_s,
            vz_m_s,
            0,
            0,
            0,
            0,
            yaw_rate_rad_s,
        )


class SwarmManager:
    """Manages multiple MAVLink connections for a swarm."""
    
    def __init__(self, urls: list[str], source_system: int = 255, source_component: int = 191):
        self.urls = urls
        self.source_system = source_system
        self.source_component = source_component
        self.connections: dict[int, MavlinkConnection] = {}
        
    async def connect_all(self, heartbeat_timeout_s: float = 30.0) -> None:
        async def _connect_one(url):
            import asyncio
            conn = MavlinkConnection(url, source_system=self.source_system, source_component=self.source_component)
            try:
                await conn.connect(heartbeat_timeout_s)
                if conn.target_system in self.connections:
                    LOGGER.error(
                        "SwarmManager: Duplicate SYSID %s from %s; configure each vehicle with a unique MAVLink system ID.",
                        conn.target_system,
                        url,
                    )
                    await conn.close()
                    return
                self.connections[conn.target_system] = conn
                LOGGER.info(f"SwarmManager: Registered drone SYSID {conn.target_system} via {url}")
            except Exception as e:
                LOGGER.error(f"SwarmManager: Failed to connect to {url}: {e}")
                
        import asyncio
        await asyncio.gather(*[_connect_one(url) for url in self.urls])
        
    async def start_all(self, requested_rates_hz=None) -> None:
        import asyncio
        await asyncio.gather(*[conn.start(requested_rates_hz) for conn in self.connections.values()])
        
    async def close_all(self) -> None:
        import asyncio
        await asyncio.gather(*[conn.close() for conn in self.connections.values()])


def mavlink_result_name(result: int) -> str:
    for key, enum_entry in mavutil.mavlink.enums.get("MAV_RESULT", {}).items():
        if int(key) == int(result):
            return enum_entry.name
    return "MAV_RESULT_UNKNOWN"
