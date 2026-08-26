"""
Sensor and estimator discovery for adaptive autonomous flight.

The module classifies the active navigation frame before every task:

- MODE_A_GPS: global WGS84 plus local NED are usable.
- MODE_B_LOCAL: GPS-denied operation with local NED/body-frame setpoints only.
- MODE_C_DEGRADED: no trustworthy position estimate; navigation is refused.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

from pymavlink import mavutil

from mavlink_io import MavlinkConnection


class NavigationMode(str, Enum):
    MODE_A_GPS = "GPS-Enabled"
    MODE_B_LOCAL = "GPS-Denied / Optical Flow"
    MODE_C_DEGRADED = "Degraded"


@dataclass(frozen=True)
class GpsState:
    fix_type: int
    satellites_visible: int
    fresh: bool
    healthy: bool


@dataclass(frozen=True)
class EkfState:
    flags: int
    velocity_horiz: bool
    pos_horiz_abs: bool
    pos_horiz_rel: bool
    pred_pos_horiz_abs: bool
    pred_pos_horiz_rel: bool
    fresh: bool
    healthy_for_global: bool
    healthy_for_local: bool


@dataclass(frozen=True)
class LocalPositionState:
    north_m: float
    east_m: float
    down_m: float
    fresh: bool
    valid: bool


@dataclass(frozen=True)
class FlowVisionState:
    source: str
    quality: Optional[int]
    fresh: bool
    valid: bool


@dataclass(frozen=True)
class BatteryState:
    voltage_v: Optional[float]
    remaining_percent: Optional[float]
    fresh: bool
    healthy: bool


@dataclass(frozen=True)
class SensorReport:
    mode: NavigationMode
    gps: GpsState
    ekf: EkfState
    local_position: LocalPositionState
    flow_or_vision: FlowVisionState
    battery: BatteryState
    autopilot_mode: str
    armed: bool
    reasons: list[str]

    @property
    def can_navigate(self) -> bool:
        return self.mode != NavigationMode.MODE_C_DEGRADED


@dataclass(frozen=True)
class SensorThresholds:
    gps_min_fix_type: int = 3
    gps_min_satellites: int = 8
    message_max_age_s: float = 3.0
    local_position_max_age_s: float = 1.5
    optical_flow_min_quality: int = 100
    battery_min_voltage_v: float = 0.0
    battery_min_remaining_percent: float = 0.20


class SensorDiscovery:
    def __init__(
        self,
        mavlink: MavlinkConnection,
        thresholds: SensorThresholds = SensorThresholds(),
    ) -> None:
        self.mavlink = mavlink
        self.thresholds = thresholds

    async def request_required_messages(self) -> None:
        requested_rates = {
            "SYS_STATUS": 2.0,
            "BATTERY_STATUS": 1.0,
            "GPS_RAW_INT": 2.0,
            "EKF_STATUS_REPORT": 2.0,
            "LOCAL_POSITION_NED": 10.0,
            "OPTICAL_FLOW": 5.0,
            "OPTICAL_FLOW_RAD": 5.0,
            "VISION_POSITION_ESTIMATE": 10.0,
            "ODOMETRY": 10.0,
            "RC_CHANNELS": 2.0,
        }
        for name, rate_hz in requested_rates.items():
            message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
            if message_id is not None:
                await self.mavlink.set_message_interval(int(message_id), rate_hz)

    async def probe(self, wait_s: float = 2.0) -> SensorReport:
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if self.mavlink.latest("SYS_STATUS") and (
                self.mavlink.latest("GPS_RAW_INT") or self.mavlink.latest("LOCAL_POSITION_NED")
            ):
                break
            await self.request_required_messages()
            await _sleep_short()
        return self.snapshot()

    def snapshot(self) -> SensorReport:
        gps = self._gps_state()
        ekf = self._ekf_state()
        local_position = self._local_position_state()
        flow_or_vision = self._flow_or_vision_state()
        battery = self._battery_state()
        heartbeat = self.mavlink.latest_message("HEARTBEAT", self.thresholds.message_max_age_s)
        autopilot_mode = self.mavlink.mode_name_from_heartbeat(heartbeat) if heartbeat else "UNKNOWN"
        armed = bool(
            heartbeat
            and int(getattr(heartbeat, "base_mode", 0))
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        reasons: list[str] = []
        if gps.healthy and ekf.healthy_for_global and local_position.valid:
            mode = NavigationMode.MODE_A_GPS
            reasons.append("GPS fix, satellite count, EKF absolute horizontal position, and local NED are healthy")
        elif flow_or_vision.valid and ekf.healthy_for_local and local_position.valid:
            mode = NavigationMode.MODE_B_LOCAL
            reasons.append(f"{flow_or_vision.source} aiding and local NED estimate are healthy")
        else:
            mode = NavigationMode.MODE_C_DEGRADED
            reasons.extend(self._degraded_reasons(gps, ekf, local_position, flow_or_vision))

        return SensorReport(
            mode=mode,
            gps=gps,
            ekf=ekf,
            local_position=local_position,
            flow_or_vision=flow_or_vision,
            battery=battery,
            autopilot_mode=autopilot_mode,
            armed=armed,
            reasons=reasons,
        )

    def _gps_state(self) -> GpsState:
        cached = self.mavlink.latest("GPS_RAW_INT")
        msg = cached.message if cached else None
        age_ok = _fresh(cached, self.thresholds.message_max_age_s)
        fix_type = int(getattr(msg, "fix_type", 0) or 0)
        satellites = int(getattr(msg, "satellites_visible", 0) or 0)
        healthy = (
            age_ok
            and fix_type >= self.thresholds.gps_min_fix_type
            and satellites >= self.thresholds.gps_min_satellites
        )
        return GpsState(
            fix_type=fix_type,
            satellites_visible=satellites,
            fresh=age_ok,
            healthy=healthy,
        )

    def _ekf_state(self) -> EkfState:
        cached = self.mavlink.latest("EKF_STATUS_REPORT")
        msg = cached.message if cached else None
        age_ok = _fresh(cached, self.thresholds.message_max_age_s)
        flags = int(getattr(msg, "flags", 0) or 0)

        velocity_horiz = _flag(flags, "EKF_VELOCITY_HORIZ", 1 << 0)
        pos_horiz_abs = _flag(flags, "EKF_POS_HORIZ_ABS", 1 << 3)
        pos_horiz_rel = _flag(flags, "EKF_POS_HORIZ_REL", 1 << 2)
        pred_pos_horiz_abs = _flag(flags, "EKF_PRED_POS_HORIZ_ABS", 1 << 9)
        pred_pos_horiz_rel = _flag(flags, "EKF_PRED_POS_HORIZ_REL", 1 << 8)

        healthy_for_global = age_ok and velocity_horiz and (pos_horiz_abs or pred_pos_horiz_abs)
        healthy_for_local = age_ok and velocity_horiz and (
            pos_horiz_rel or pred_pos_horiz_rel or pos_horiz_abs or pred_pos_horiz_abs
        )
        return EkfState(
            flags=flags,
            velocity_horiz=velocity_horiz,
            pos_horiz_abs=pos_horiz_abs,
            pos_horiz_rel=pos_horiz_rel,
            pred_pos_horiz_abs=pred_pos_horiz_abs,
            pred_pos_horiz_rel=pred_pos_horiz_rel,
            fresh=age_ok,
            healthy_for_global=healthy_for_global,
            healthy_for_local=healthy_for_local,
        )

    def _local_position_state(self) -> LocalPositionState:
        cached = self.mavlink.latest("LOCAL_POSITION_NED")
        msg = cached.message if cached else None
        fresh = _fresh(cached, self.thresholds.local_position_max_age_s)
        north_m = _finite(getattr(msg, "x", math.nan))
        east_m = _finite(getattr(msg, "y", math.nan))
        down_m = _finite(getattr(msg, "z", math.nan))
        valid = fresh and all(math.isfinite(value) for value in (north_m, east_m, down_m))
        return LocalPositionState(
            north_m=north_m,
            east_m=east_m,
            down_m=down_m,
            fresh=fresh,
            valid=valid,
        )

    def _flow_or_vision_state(self) -> FlowVisionState:
        for message_name in ("OPTICAL_FLOW_RAD", "OPTICAL_FLOW"):
            cached = self.mavlink.latest(message_name)
            msg = cached.message if cached else None
            if not _fresh(cached, self.thresholds.message_max_age_s):
                continue
            quality = getattr(msg, "quality", None)
            if quality is None:
                quality = getattr(msg, "integration_time_us", 0)
                valid = float(quality) > 0
                return FlowVisionState(message_name, None, True, valid)
            quality_int = int(quality)
            return FlowVisionState(
                message_name,
                quality_int,
                True,
                quality_int >= self.thresholds.optical_flow_min_quality,
            )

        for message_name in ("VISION_POSITION_ESTIMATE", "ODOMETRY"):
            cached = self.mavlink.latest(message_name)
            if _fresh(cached, self.thresholds.message_max_age_s):
                return FlowVisionState(message_name, None, True, True)

        return FlowVisionState("none", None, False, False)

    def _battery_state(self) -> BatteryState:
        sys_status = self.mavlink.latest_message("SYS_STATUS", self.thresholds.message_max_age_s)
        battery_status = self.mavlink.latest_message("BATTERY_STATUS", self.thresholds.message_max_age_s)
        voltage_v = _sys_status_voltage_v(sys_status)
        remaining = _percent_fraction(getattr(sys_status, "battery_remaining", None))
        fresh = sys_status is not None

        if voltage_v is None:
            voltage_v = _battery_status_voltage_v(battery_status)
            fresh = fresh or battery_status is not None
        if remaining is None:
            remaining = _percent_fraction(getattr(battery_status, "battery_remaining", None))

        voltage_ok = self.thresholds.battery_min_voltage_v <= 0 or (
            voltage_v is not None and voltage_v >= self.thresholds.battery_min_voltage_v
        )
        percent_ok = remaining is not None and remaining >= self.thresholds.battery_min_remaining_percent
        return BatteryState(
            voltage_v=voltage_v,
            remaining_percent=remaining,
            fresh=fresh,
            healthy=fresh and voltage_ok and percent_ok,
        )

    def _degraded_reasons(
        self,
        gps: GpsState,
        ekf: EkfState,
        local_position: LocalPositionState,
        flow_or_vision: FlowVisionState,
    ) -> list[str]:
        reasons: list[str] = []
        if not gps.healthy:
            reasons.append(
                f"GPS unhealthy: fix={gps.fix_type}, sats={gps.satellites_visible}, fresh={gps.fresh}"
            )
        if not ekf.healthy_for_local:
            reasons.append(f"EKF local position flags unhealthy: flags=0x{ekf.flags:x}, fresh={ekf.fresh}")
        if not local_position.valid:
            reasons.append("LOCAL_POSITION_NED is missing, stale, or non-finite")
        if not flow_or_vision.valid:
            reasons.append("no fresh optical-flow, vision, or odometry aiding source detected")
        return reasons


def format_sensor_report(report: SensorReport) -> str:
    battery = "n/a"
    if report.battery.remaining_percent is not None:
        battery = f"{report.battery.remaining_percent:.0%}"
    voltage = "n/a" if report.battery.voltage_v is None else f"{report.battery.voltage_v:.2f}V"
    return (
        f"mode={report.mode.value} autopilot={report.autopilot_mode} armed={report.armed} "
        f"gps_fix={report.gps.fix_type} sats={report.gps.satellites_visible} "
        f"local=({report.local_position.north_m:.1f}N,"
        f"{report.local_position.east_m:.1f}E,{report.local_position.down_m:.1f}D) "
        f"battery={battery} {voltage}"
    )


def _fresh(cached: Any, max_age_s: float) -> bool:
    return cached is not None and time.monotonic() - cached.received_at_s <= max_age_s


def _flag(flags: int, name: str, fallback: int) -> bool:
    return bool(flags & int(getattr(mavutil.mavlink, name, fallback)))


def _finite(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def _percent_fraction(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number < 0:
        return None
    if number <= 1.0:
        return number
    if number <= 100.0:
        return number / 100.0
    return None


def _sys_status_voltage_v(msg: Any) -> Optional[float]:
    voltage_mv = _finite(getattr(msg, "voltage_battery", math.nan))
    if not math.isfinite(voltage_mv) or voltage_mv <= 0:
        return None
    return voltage_mv / 1000.0


def _battery_status_voltage_v(msg: Any) -> Optional[float]:
    voltages = getattr(msg, "voltages", None)
    if not isinstance(voltages, list):
        return None
    valid_cell_mv: list[float] = []
    for voltage in voltages:
        try:
            voltage_mv = float(voltage)
        except (TypeError, ValueError):
            continue
        if 0 < voltage_mv < 65535:
            valid_cell_mv.append(voltage_mv)
    if not valid_cell_mv:
        return None
    return sum(valid_cell_mv) / 1000.0


async def _sleep_short() -> None:
    await asyncio.sleep(0.1)
