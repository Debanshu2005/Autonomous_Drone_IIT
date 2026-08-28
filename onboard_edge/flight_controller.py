"""
Finite-state flight controller for adaptive GPS/local autonomous tasks.

The controller is deliberately conservative. It refuses navigation in degraded
sensor mode, keeps setpoints streaming while moving, and makes LAND the default
software failsafe action.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

from pymavlink import mavutil

from mavlink_io import MavlinkConnection, MavlinkError
from sensor_check import NavigationMode, SensorDiscovery, SensorReport
from trajectory_engine import (
    GlobalTarget,
    LocalTarget,
    ParsedTask,
    TargetFrame,
    TaskAction,
    TrajectoryPlan,
    global_distance_m,
    local_distance_m,
)
from hotspot import HotspotContainmentConfig, PeerLink


LOGGER = logging.getLogger(__name__)


class FlightState(str, Enum):
    IDLE = "IDLE"
    HARDWARE_CHECK = "HARDWARE_CHECK"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    TRAJECTORY_FOLLOW = "TRAJECTORY_FOLLOW"
    RTL_OR_HOLD = "RTL_OR_HOLD"


class FlightAbort(RuntimeError):
    """Raised when the controller must stop autonomous navigation."""


@dataclass(frozen=True)
class FlightControllerConfig:
    takeoff_altitude_m: float = 3.0
    waypoint_acceptance_radius_m: float = 0.5
    waypoint_timeout_s: float = 60.0
    setpoint_rate_hz: float = 5.0
    heartbeat_timeout_s: float = 3.0
    battery_warning_percent: float = 0.20
    critical_battery_percent: float = 0.12
    critical_battery_voltage_v: float = 0.0
    max_altitude_m: float = 15.0
    rc_abort_channel: int = 7
    rc_abort_pwm: int = 1800
    post_takeoff_settle_s: float = 2.0
    takeoff_start_timeout_s: float = 8.0
    takeoff_min_climb_m: float = 0.3
    takeoff_velocity_fallback_m_s: float = 0.6
    final_action: str = "hold"
    battery_debounce_s: float = 1.5
    hotspot: HotspotContainmentConfig = HotspotContainmentConfig()


@dataclass(frozen=True)
class AltitudeSample:
    relative_alt_m: float
    source: str


class FlightController:
    def __init__(
        self,
        mavlink: MavlinkConnection,
        sensors: SensorDiscovery,
        config: FlightControllerConfig = FlightControllerConfig(),
        status_sink: Optional[Callable[[str], None]] = None,
        peer_link: Optional[PeerLink] = None,
    ) -> None:
        self.mavlink = mavlink
        self.sensors = sensors
        self.config = config
        self.status_sink = status_sink
        self.peer_link = peer_link
        self.state = FlightState.IDLE
        self._watchdog_task: Optional[asyncio.Task[None]] = None
        self._failsafe_reason: Optional[str] = None
        self._pause_reason: Optional[str] = None
        self._battery_critical_start_s: Optional[float] = None
        self._startup_global_position: Optional[tuple[float, float, float]] = None
        self._startup_local_position: Optional[tuple[float, float, float]] = None

    async def execute_plan(self, task: ParsedTask, plan: TrajectoryPlan) -> None:
        if task.action == TaskAction.HOLD:
            await self.hold()
            return
        if task.action == TaskAction.LAND:
            await self.land("operator command")
            return
        if task.action == TaskAction.RTL:
            await self.rtl("operator command")
            return

        self._failsafe_reason = None
        self._watchdog_task = asyncio.create_task(self._watchdog_loop())
        try:
            report = await self._hardware_check()
            if report.mode == NavigationMode.MODE_C_DEGRADED:
                raise FlightAbort("; ".join(report.reasons))

            await self._enter_guided_mode(report.mode)
            await self._arm_and_takeoff(plan.target_altitude_m)

            if task.action in {TaskAction.TAKEOFF, TaskAction.TAKEOFF_LAND}:
                await self._hold_for(task.params.get("hover_s", 0.0))
                self.state = FlightState.RTL_OR_HOLD
                if task.action == TaskAction.TAKEOFF_LAND:
                    await self.land("takeoff-hover-land sequence complete")
                elif self.config.final_action.lower() == "rtl":
                    await self.rtl("takeoff sequence complete")
                elif self.config.final_action.lower() == "land":
                    await self.land("takeoff sequence complete")
                else:
                    await self.hold()
                return

            self.state = FlightState.TRAJECTORY_FOLLOW
            if plan.frame == TargetFrame.GLOBAL_RELATIVE_ALT:
                for target in plan.global_targets:
                    await self._fly_global_target(target)
            else:
                for target in plan.local_targets:
                    await self._fly_local_target(target)

            self.state = FlightState.RTL_OR_HOLD
            if self.config.final_action.lower() == "rtl":
                await self.rtl("mission complete")
            elif self.config.final_action.lower() == "land":
                await self.land("mission complete")
            else:
                await self.hold()
        except asyncio.CancelledError:
            raise
        except Exception:
            if self._vehicle_armed():
                await self.land("autonomous task aborted")
            raise
        finally:
            if self._watchdog_task:
                self._watchdog_task.cancel()
                await asyncio.gather(self._watchdog_task, return_exceptions=True)
            self.state = FlightState.IDLE

    async def execute_task_queue(
        self,
        tasks: list[ParsedTask],
        build_plan: Callable[[ParsedTask, SensorReport], TrajectoryPlan],
    ) -> None:
        if not tasks:
            return

        if len(tasks) == 1 and tasks[0].action in {
            TaskAction.SET_MODE,
            TaskAction.HOLD,
            TaskAction.LAND,
            TaskAction.RTL,
        }:
            await self._execute_immediate_command(tasks[0])
            return

        self._failsafe_reason = None
        self._watchdog_task = asyncio.create_task(self._watchdog_loop())
        try:
            report = await self._hardware_check()
            self._emit("STATUS", f"Sensor check passed: Active navigation frame is {_nav_label(report)}.")
            await self._enter_guided_mode(report.mode)

            for index, task in enumerate(tasks, start=1):
                await self._raise_if_failsafe()
                report = self.sensors.snapshot()
                plan = build_plan(task, report)
                self._emit(
                    "STATUS",
                    f"Task {index}/{len(tasks)}: {task.action.name} queued for execution.",
                )
                await self._execute_queue_task(task, plan)

            self._emit("STATUS", "Sequence complete.")
        except asyncio.CancelledError:
            self._emit("STATUS", "Sequence interrupted.")
            raise
        except Exception:
            if self._vehicle_armed():
                await self.land("autonomous sequence aborted")
            raise
        finally:
            if self._watchdog_task:
                self._watchdog_task.cancel()
                await asyncio.gather(self._watchdog_task, return_exceptions=True)
            self.state = FlightState.IDLE

    async def _execute_immediate_command(self, task: ParsedTask) -> None:
        if task.action == TaskAction.SET_MODE:
            mode_name = task.notes[0].removeprefix("mode=") if task.notes else ""
            await self.change_mode(mode_name)
        elif task.action == TaskAction.HOLD:
            await self.hold()
        elif task.action == TaskAction.LAND:
            await self.land("operator interrupt")
            await self._wait_until_landed_or_disarmed()
        elif task.action == TaskAction.RTL:
            await self.rtl("operator interrupt")

    async def change_mode(self, mode_name: str) -> None:
        if not mode_name:
            raise FlightAbort("mode command did not include a target mode")
        self._emit("STATUS", f"Changing autopilot mode to {mode_name}.")
        await self.mavlink.set_mode(mode_name)
        self._emit("STATUS", f"Mode changed to {mode_name}.")

    async def _execute_queue_task(self, task: ParsedTask, plan: TrajectoryPlan) -> None:
        if task.action == TaskAction.SET_MODE:
            mode_name = task.notes[0].removeprefix("mode=") if task.notes else ""
            await self.change_mode(mode_name)
            return
        if task.action == TaskAction.TAKEOFF:
            await self._arm_and_takeoff(plan.target_altitude_m)
            return
        if task.action == TaskAction.HOVER:
            await self._hold_for(task.params.get("hover_s", 0.0))
            return
        if task.action == TaskAction.HOLD:
            await self.hold()
            return
        if task.action == TaskAction.LAND:
            await self.land("operator sequence")
            await self._wait_until_landed_or_disarmed()
            return
        if task.action == TaskAction.RTL:
            await self.rtl("operator sequence")
            return

        if not self._vehicle_armed():
            await self._arm_and_takeoff(plan.target_altitude_m)
        elif (self._current_relative_altitude() or 0.0) < max(0.5, plan.target_altitude_m - 0.75):
            await self._wait_until_altitude(plan.target_altitude_m)

        self.state = FlightState.TRAJECTORY_FOLLOW
        if plan.frame == TargetFrame.GLOBAL_RELATIVE_ALT:
            for target in plan.global_targets:
                await self._fly_global_target(target)
        else:
            for target in plan.local_targets:
                await self._fly_local_target(target)

    async def hold(self) -> None:
        self.state = FlightState.RTL_OR_HOLD
        local = self._current_local_position()
        if local:
            self.mavlink.send_local_position_target(*local)
        else:
            self.mavlink.send_body_velocity_target(0.0, 0.0, 0.0)
        LOGGER.info("Holding current position")

    async def land(self, reason: str) -> None:
        self.state = FlightState.RTL_OR_HOLD
        self._emit("STATUS", f"LAND requested: {reason}")
        LOGGER.warning("LAND requested: %s", reason)
        await self.mavlink.land()

    async def rtl(self, reason: str) -> None:
        self.state = FlightState.RTL_OR_HOLD
        self._emit("STATUS", f"RTL requested: {reason}")
        LOGGER.warning("RTL requested: %s", reason)
        await self.mavlink.rtl()

    async def _hardware_check(self) -> SensorReport:
        self.state = FlightState.HARDWARE_CHECK
        report = await self.sensors.probe(wait_s=2.0)
        LOGGER.info("Sensor mode selected: %s", report.mode.value)
        if not report.can_navigate:
            raise FlightAbort("; ".join(report.reasons))
        if not report.battery.healthy:
            raise FlightAbort("battery telemetry is missing or below configured threshold")
            
        hotspot = self.config.hotspot
        if hotspot.enabled and hotspot.network_watchdog_enabled and hotspot.require_peers_before_arm and hotspot.expected_peer_ids:
            if not self.peer_link:
                raise FlightAbort("hotspot containment is required but peer link is not active")
            missing = self.peer_link.missing_peers()
            if missing:
                raise FlightAbort(f"required hotspot peers missing: {missing}")
                
        self._startup_global_position = self._current_global_position(allow_gps_fallback=True)
        self._startup_local_position = self._current_local_position()
        return report

    async def _enter_guided_mode(self, mode: NavigationMode) -> None:
        mode_candidates = ["GUIDED"]
        if mode == NavigationMode.MODE_B_LOCAL:
            mode_candidates.extend(["GUIDED_NOGPS", "OFFBOARD"])

        last_error: Optional[Exception] = None
        for mode_name in mode_candidates:
            try:
                self._emit("STATUS", f"Changing autopilot mode to {mode_name}.")
                LOGGER.info("Changing autopilot mode to %s", mode_name)
                if mode_name == "OFFBOARD":
                    await self._prestream_offboard_setpoints()
                await self.mavlink.set_mode(mode_name)
                self._emit("STATUS", f"Autopilot mode confirmed: {mode_name}.")
                return
            except Exception as exc:
                last_error = exc
                LOGGER.debug("Mode %s failed: %s", mode_name, exc)
        raise FlightAbort(f"could not enter guided/offboard mode: {last_error}")

    async def _prestream_offboard_setpoints(self) -> None:
        local = self._current_local_position()
        if local is None:
            local = (0.0, 0.0, -self.config.takeoff_altitude_m)
        for _ in range(15):
            self.mavlink.send_local_position_target(*local)
            await asyncio.sleep(0.05)

    async def _arm_and_takeoff(self, target_altitude_m: float) -> None:
        self.state = FlightState.ARMING
        if not self._vehicle_armed():
            self._emit("STATUS", "Arming vehicle.")
            LOGGER.info("Arming vehicle")
            try:
                await self.mavlink.arm()
            except MavlinkError as exc:
                raise FlightAbort(
                    f"arm command failed: {exc}; check Mission Planner Messages for pre-arm failures"
                ) from exc
            await self._wait_until_armed()

        self.state = FlightState.TAKEOFF
        self._emit("STATUS", f"Taking off to {target_altitude_m:.1f}m.")
        LOGGER.info("Taking off to %.1fm AGL", target_altitude_m)
        mode_name = self._current_mode_name()
        if mode_name in {"GUIDED_NOGPS", "OFFBOARD"}:
            raise FlightAbort(
                f"autonomous takeoff is not supported in {mode_name}; switch to GUIDED with a healthy EKF/GPS "
                "or take off manually before sending local movement commands"
            )
        global_position = self._current_global_position(allow_gps_fallback=False)
        lat_deg = global_position[0] if global_position else 0.0
        lon_deg = global_position[1] if global_position else 0.0
        accepted = False
        try:
            await self.mavlink.takeoff(target_altitude_m, lat_deg=lat_deg, lon_deg=lon_deg)
            accepted = True
            self._emit("STATUS", "NAV_TAKEOFF accepted by autopilot.")
        except MavlinkError as exc:
            message = str(exc)
            if "rejected" in message:
                raise FlightAbort(
                    f"NAV_TAKEOFF rejected by autopilot: {message}; check mode, arming state, and Mission Planner Messages"
                ) from exc
            LOGGER.warning("NAV_TAKEOFF was not acknowledged; probing guided climb: %s", exc)
            self._emit("STATUS", f"NAV_TAKEOFF ACK timeout; probing guided climb ({exc}).")
        if accepted:
            try:
                await self._wait_for_takeoff_start(target_altitude_m)
            except FlightAbort as exc:
                LOGGER.warning("NAV_TAKEOFF did not start a climb: %s", exc)
                self._emit("STATUS", "NAV_TAKEOFF did not start climb; trying guided vertical velocity.")
                await self._probe_guided_velocity_climb(target_altitude_m)
        else:
            await self._probe_guided_velocity_climb(target_altitude_m)
        await self._wait_until_altitude(target_altitude_m)
        await asyncio.sleep(self.config.post_takeoff_settle_s)

    async def _wait_until_armed(self, timeout_s: float = 8.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._vehicle_armed():
                self._emit("STATUS", "Vehicle armed confirmed.")
                return
            await asyncio.sleep(0.1)
        raise FlightAbort(
            "arm command was acknowledged but the vehicle never reported ARMED; "
            "check Mission Planner Messages for pre-arm failures"
        )

    async def _wait_for_takeoff_start(self, target_altitude_m: float) -> None:
        baseline = self._current_altitude_sample()
        baseline_alt_m = baseline.relative_alt_m if baseline else 0.0
        climb_required_m = min(
            self.config.takeoff_min_climb_m,
            max(0.1, target_altitude_m * 0.5),
        )
        deadline = time.monotonic() + self.config.takeoff_start_timeout_s
        while time.monotonic() < deadline:
            await self._raise_if_failsafe()
            if not self._vehicle_armed():
                raise FlightAbort(
                    "vehicle disarmed during takeoff; check Mission Planner Messages for failsafe/pre-arm details"
                )
            sample = self._current_altitude_sample()
            if sample is not None:
                climbed_m = sample.relative_alt_m - baseline_alt_m
                self._emit(
                    "EXEC",
                    f"Starting TAKEOFF | Current Alt: {sample.relative_alt_m:.1f}m "
                    f"({sample.source}) | Climb: {climbed_m:.1f}m",
                )
                if sample.relative_alt_m >= target_altitude_m - 0.2 or climbed_m >= climb_required_m:
                    return
            await asyncio.sleep(0.2)
        raise FlightAbort(
            f"takeoff command accepted but altitude did not increase within "
            f"{self.config.takeoff_start_timeout_s:.1f}s"
        )

    async def _probe_guided_velocity_climb(self, target_altitude_m: float) -> None:
        baseline = self._current_altitude_sample()
        baseline_alt_m = baseline.relative_alt_m if baseline else 0.0
        climb_required_m = min(
            self.config.takeoff_min_climb_m,
            max(0.1, target_altitude_m * 0.5),
        )
        deadline = time.monotonic() + self.config.takeoff_start_timeout_s
        interval_s = 1.0 / max(1.0, self.config.setpoint_rate_hz)
        while time.monotonic() < deadline:
            await self._raise_if_failsafe()
            if not self._vehicle_armed():
                raise FlightAbort(
                    "vehicle disarmed during guided climb probe; check Mission Planner Messages"
                )
            self.mavlink.send_local_velocity_target(
                0.0,
                0.0,
                -abs(self.config.takeoff_velocity_fallback_m_s),
            )
            sample = self._current_altitude_sample()
            if sample is not None:
                climbed_m = sample.relative_alt_m - baseline_alt_m
                self._emit(
                    "EXEC",
                    f"Probing TAKEOFF climb | Current Alt: {sample.relative_alt_m:.1f}m "
                    f"({sample.source}) | Climb: {climbed_m:.1f}m",
                )
                if sample.relative_alt_m >= target_altitude_m - 0.2 or climbed_m >= climb_required_m:
                    return
            await asyncio.sleep(interval_s)
        raise FlightAbort(
            "takeoff did not start climbing; confirm props/motors, safety switch, GUIDED mode, "
            "EKF/home position, and Mission Planner pre-arm/messages"
        )

    async def _wait_until_altitude(self, target_altitude_m: float) -> None:
        interval_s = 1.0 / max(1.0, self.config.setpoint_rate_hz)
        deadline = time.monotonic() + max(30.0, self.config.waypoint_timeout_s)
        while time.monotonic() < deadline:
            await self._raise_if_failsafe()
            
            if self._pause_reason:
                self.mavlink.send_body_velocity_target(0.0, 0.0, 0.0)
                deadline += interval_s
                await asyncio.sleep(interval_s)
                continue
                
            if not self._vehicle_armed():
                raise FlightAbort(
                    "vehicle disarmed during takeoff; check Mission Planner Messages for failsafe/pre-arm details"
                )
            sample = self._current_altitude_sample()
            current_alt_m = sample.relative_alt_m if sample is not None else None
            local = self._current_local_position()
            global_position = self._current_global_position(allow_gps_fallback=False)
            if global_position is not None:
                lat_deg, lon_deg, _relative_alt_m = global_position
                self.mavlink.send_global_position_target(lat_deg, lon_deg, target_altitude_m)
            elif local is not None:
                north_m, east_m, _down_m = local
                self.mavlink.send_local_position_target(north_m, east_m, -target_altitude_m)
            if current_alt_m is not None:
                error_m = abs(target_altitude_m - current_alt_m)
                source = sample.source if sample is not None else "unknown"
                self._emit(
                    "EXEC",
                    f"Executing TAKEOFF to {target_altitude_m:.1f}m | "
                    f"Current Alt: {current_alt_m:.1f}m ({source}) | Error: {error_m:.1f}m",
                )
                if error_m <= 0.2:
                    self._emit("STATUS", "Takeoff altitude reached.")
                    return
            await asyncio.sleep(interval_s)
        raise FlightAbort(f"takeoff altitude {target_altitude_m:.1f}m was not reached")

    async def _hold_for(self, duration_s: float) -> None:
        duration_s = max(0.0, float(duration_s))
        if duration_s <= 0:
            return
        LOGGER.info("Hovering for %.1fs", duration_s)

        interval_s = 1.0 / max(1.0, self.config.setpoint_rate_hz)
        hold_position = self._current_local_position()
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            await self._raise_if_failsafe()
            
            if self._pause_reason:
                self.mavlink.send_body_velocity_target(0.0, 0.0, 0.0)
                deadline += interval_s
                await asyncio.sleep(interval_s)
                continue
                
            if hold_position is not None:
                self.mavlink.send_local_position_target(*hold_position)
            else:
                self.mavlink.send_body_velocity_target(0.0, 0.0, 0.0)
            remaining_s = max(0.0, deadline - time.monotonic())
            self._emit("EXEC", f"Executing HOVER | Time remaining: {remaining_s:.1f}s")
            await asyncio.sleep(interval_s)

    async def _fly_local_target(self, target: LocalTarget) -> None:
        LOGGER.info(
            "Local target %s: N %.2f E %.2f D %.2f",
            target.name,
            target.north_m,
            target.east_m,
            target.down_m,
        )

        interval_s = 1.0 / max(1.0, self.config.setpoint_rate_hz)
        deadline = time.monotonic() + self.config.waypoint_timeout_s
        while time.monotonic() < deadline:
            await self._raise_if_failsafe()
            
            if self._pause_reason:
                self.mavlink.send_body_velocity_target(0.0, 0.0, 0.0)
                deadline += interval_s
                await asyncio.sleep(interval_s)
                continue
                
            self.mavlink.send_local_position_target(
                target.north_m,
                target.east_m,
                target.down_m,
                yaw_rad=_yaw_rad(target.yaw_deg),
            )
            local = self._current_local_position()
            if local is not None:
                distance = local_distance_m(target, *local)
                self._emit(
                    "EXEC",
                    f"Executing {target.name} | Current POS: "
                    f"({local[0]:.1f},{local[1]:.1f},{local[2]:.1f}) | Error: {distance:.1f}m",
                )
                if distance <= max(0.2, self.config.waypoint_acceptance_radius_m):
                    if target.hold_s > 0:
                        await self._hold_for(target.hold_s)
                    return
            await asyncio.sleep(interval_s)
        raise FlightAbort(f"local waypoint {target.name} timed out")

    async def _fly_global_target(self, target: GlobalTarget) -> None:
        LOGGER.info(
            "Global target %s: lat %.7f lon %.7f alt %.1f",
            target.name,
            target.lat_deg,
            target.lon_deg,
            target.relative_alt_m,
        )

        interval_s = 1.0 / max(1.0, self.config.setpoint_rate_hz)
        deadline = time.monotonic() + self.config.waypoint_timeout_s
        while time.monotonic() < deadline:
            await self._raise_if_failsafe()
            
            if self._pause_reason:
                self.mavlink.send_body_velocity_target(0.0, 0.0, 0.0)
                deadline += interval_s
                await asyncio.sleep(interval_s)
                continue
                
            self.mavlink.send_global_position_target(
                target.lat_deg,
                target.lon_deg,
                target.relative_alt_m,
                yaw_rad=_yaw_rad(target.yaw_deg),
            )
            global_position = self._current_global_position()
            if global_position is not None:
                lat, lon, relative_alt = global_position
                horizontal_error = global_distance_m(lat, lon, target.lat_deg, target.lon_deg)
                vertical_error = abs(relative_alt - target.relative_alt_m)
                self._emit(
                    "EXEC",
                    f"Executing {target.name} | Current Alt: {relative_alt:.1f}m | "
                    f"Horizontal Error: {horizontal_error:.1f}m | Vertical Error: {vertical_error:.1f}m",
                )
                if max(horizontal_error, vertical_error) <= max(0.2, self.config.waypoint_acceptance_radius_m):
                    if target.hold_s > 0:
                        await self._hold_for(target.hold_s)
                    return
            await asyncio.sleep(interval_s)
        raise FlightAbort(f"global waypoint {target.name} timed out")

    async def _watchdog_loop(self) -> None:
        while True:
            reason = self._failsafe_status()
            if reason:
                self._failsafe_reason = reason
                LOGGER.error("Failsafe triggered: %s", reason)
                return
            await asyncio.sleep(0.25)

    async def _raise_if_failsafe(self) -> None:
        if self._failsafe_reason:
            raise FlightAbort(self._failsafe_reason)
        if self._watchdog_task and self._watchdog_task.done():
            raise FlightAbort(self._failsafe_reason or "failsafe watchdog stopped")

    def _failsafe_status(self) -> Optional[str]:
        heartbeat = self.mavlink.latest("HEARTBEAT")
        if heartbeat is None or time.monotonic() - heartbeat.received_at_s > self.config.heartbeat_timeout_s:
            return "MAVLink heartbeat timeout"

        report = self.sensors.snapshot()
        battery = report.battery
        battery_critical = False
        reason = None
        
        if battery.remaining_percent is not None and battery.remaining_percent <= self.config.critical_battery_percent:
            battery_critical = True
            reason = f"critical battery {battery.remaining_percent:.0%}"
        if (
            self.config.critical_battery_voltage_v > 0
            and battery.voltage_v is not None
            and battery.voltage_v <= self.config.critical_battery_voltage_v
        ):
            battery_critical = True
            reason = f"critical battery voltage {battery.voltage_v:.2f}V"

        if battery_critical:
            if self._battery_critical_start_s is None:
                self._battery_critical_start_s = time.monotonic()
            elif time.monotonic() - self._battery_critical_start_s >= self.config.battery_debounce_s:
                return reason
        else:
            self._battery_critical_start_s = None
            
        if self.state == FlightState.TRAJECTORY_FOLLOW and report.mode == NavigationMode.MODE_A_GPS:
            if getattr(report.gps, "fix_type", 3) < 3:
                self._pause_reason = "GPS degraded"
            else:
                self._pause_reason = None
        else:
            self._pause_reason = None

        rc = self.mavlink.latest_message("RC_CHANNELS", 1.0)
        if rc is not None:
            channel_name = f"chan{self.config.rc_abort_channel}_raw"
            pwm = int(getattr(rc, channel_name, 0) or 0)
            if pwm >= self.config.rc_abort_pwm:
                return f"RC abort channel {self.config.rc_abort_channel} high ({pwm})"

        local = self._current_local_position()
        if local is not None:
            altitude_m = -local[2]
            if altitude_m > self.config.max_altitude_m:
                return f"altitude limit exceeded ({altitude_m:.1f}m)"

        hotspot = self.config.hotspot
        if hotspot.enabled:
            # Geofence check
            if self._startup_local_position and local:
                dist_m = local_distance_m(
                    LocalTarget("", self._startup_local_position[0], self._startup_local_position[1], 0),
                    local[0], local[1], 0
                )
                if dist_m > hotspot.max_radius_m:
                    return f"hotspot geofence exceeded ({dist_m:.1f}m > {hotspot.max_radius_m}m)"
            elif self._startup_global_position:
                global_pos = self._current_global_position(allow_gps_fallback=True)
                if global_pos:
                    dist_m = global_distance_m(
                        self._startup_global_position[0], self._startup_global_position[1],
                        global_pos[0], global_pos[1]
                    )
                    if dist_m > hotspot.max_radius_m:
                        return f"hotspot geofence exceeded ({dist_m:.1f}m > {hotspot.max_radius_m}m)"

            # Network watchdog check
            if hotspot.network_watchdog_enabled and hotspot.expected_peer_ids and self.peer_link:
                missing = self.peer_link.missing_peers()
                if missing:
                    return f"hotspot peer heartbeat lost: {missing}"

        return None

    def _emit(self, prefix: str, message: str) -> None:
        if self.status_sink:
            self.status_sink(f"[{prefix}] {message}")

    async def _wait_until_landed_or_disarmed(self, timeout_s: float = 60.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            local = self._current_local_position()
            altitude_m = -local[2] if local is not None else None
            armed = self._vehicle_armed()
            if altitude_m is not None:
                self._emit(
                    "EXEC",
                    f"Executing LAND | Current Alt: {altitude_m:.1f}m | Armed: {armed}",
                )
            if not armed or (altitude_m is not None and altitude_m <= 0.2):
                self._emit("STATUS", "Sequence complete. Drone landed or disarmed.")
                return
            await asyncio.sleep(0.1)
        self._emit("STATUS", "LAND command sent; landed/disarmed confirmation timed out.")


    def _vehicle_armed(self) -> bool:
        heartbeat = self.mavlink.latest_message("HEARTBEAT")
        return bool(
            heartbeat
            and int(getattr(heartbeat, "base_mode", 0))
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

    def _current_mode_name(self) -> str:
        heartbeat = self.mavlink.latest_message("HEARTBEAT")
        return self.mavlink.mode_name_from_heartbeat(heartbeat) if heartbeat else "UNKNOWN"

    def _current_local_position(self) -> Optional[tuple[float, float, float]]:
        msg = self.mavlink.latest_message("LOCAL_POSITION_NED", 1.5)
        if msg is None:
            return None
        x = getattr(msg, "x", None)
        y = getattr(msg, "y", None)
        z = getattr(msg, "z", None)
        if x is None or y is None or z is None:
            return None
        north_m, east_m, down_m = float(x), float(y), float(z)
        if not all(math.isfinite(value) for value in (north_m, east_m, down_m)):
            return None
        return north_m, east_m, down_m

    def _current_global_position(self, *, allow_gps_fallback: bool = True) -> Optional[tuple[float, float, float]]:
        msg = self.mavlink.latest_message("GLOBAL_POSITION_INT", 2.0)
        if msg is None:
            if not allow_gps_fallback:
                return None
            gps = self.mavlink.latest_message("GPS_RAW_INT", 2.0)
            if gps is None:
                return None
            lat = float(getattr(gps, "lat", 0)) / 1e7
            lon = float(getattr(gps, "lon", 0)) / 1e7
            relative_alt = self._current_relative_altitude()
            if relative_alt is None:
                return None
            return lat, lon, relative_alt

        lat = float(getattr(msg, "lat", 0)) / 1e7
        lon = float(getattr(msg, "lon", 0)) / 1e7
        relative_alt = float(getattr(msg, "relative_alt", 0)) / 1000.0
        if not all(math.isfinite(value) for value in (lat, lon, relative_alt)):
            return None
        return lat, lon, relative_alt

    def _current_relative_altitude(self) -> Optional[float]:
        sample = self._current_altitude_sample()
        return sample.relative_alt_m if sample else None

    def _current_altitude_sample(self) -> Optional[AltitudeSample]:
        global_position = self.mavlink.latest_message("GLOBAL_POSITION_INT", 2.0)
        if global_position is not None:
            relative_alt_m = float(getattr(global_position, "relative_alt", 0)) / 1000.0
            if math.isfinite(relative_alt_m):
                return AltitudeSample(relative_alt_m, "GLOBAL_POSITION_INT")

        local = self._current_local_position()
        if local is not None:
            relative_alt_m = -local[2]
            if math.isfinite(relative_alt_m):
                return AltitudeSample(relative_alt_m, "LOCAL_POSITION_NED")
        return None


def _yaw_rad(yaw_deg: Optional[float]) -> Optional[float]:
    if yaw_deg is None or not math.isfinite(yaw_deg):
        return None
    return math.radians(yaw_deg)


def _nav_label(report: SensorReport) -> str:
    if report.mode == NavigationMode.MODE_A_GPS:
        return "GLOBAL_GPS"
    if report.mode == NavigationMode.MODE_B_LOCAL:
        return "LOCAL_OPTICAL_FLOW"
    return "NONE"
