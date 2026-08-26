#!/usr/bin/env python3
"""
Interactive terminal mission controller for a Pixhawk companion computer.

Example commands:
    circle r=5 h=3
    fly in a 5m radius circle at 3m altitude
    square search pattern 10m
    goto x=10 y=5 h=3
    hold
    land
    rtl
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
from dataclasses import dataclass
from typing import Optional

from flight_controller import FlightAbort, FlightController, FlightControllerConfig
from mavlink_io import MavlinkConnection, MavlinkError
from sensor_check import (
    NavigationMode,
    SensorDiscovery,
    SensorReport,
    SensorThresholds,
    format_sensor_report,
)
from trajectory_engine import (
    ParsedTask,
    TaskAction,
    TrajectoryPlan,
    VehicleOrigin,
    build_trajectory,
    command_guide,
    parse_task,
)


LOGGER = logging.getLogger("mission_controller")


@dataclass(frozen=True)
class CliConfig:
    connection_url: str
    default_altitude_m: float
    gps_min_satellites: int
    acceptance_radius_m: float
    waypoint_timeout_s: float
    max_altitude_m: float
    battery_min_percent: float
    battery_min_voltage_v: float
    critical_battery_percent: float
    critical_battery_voltage_v: float
    final_action: str
    status_interval_s: float


class MissionRepl:
    def __init__(
        self,
        mavlink: MavlinkConnection,
        sensors: SensorDiscovery,
        controller: FlightController,
        config: CliConfig,
    ) -> None:
        self.mavlink = mavlink
        self.sensors = sensors
        self.controller = controller
        self.config = config
        self._active_task: Optional[asyncio.Task[None]] = None
        self._status_task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()

    async def run(self) -> None:
        self._status_task = asyncio.create_task(self._status_loop())
        print("Autonomous terminal mission controller ready. Type 'help' for commands.")
        try:
            while not self._stop.is_set():
                line = await asyncio.to_thread(input, "mission> ")
                await self._handle_line(line)
        finally:
            if self._active_task and not self._active_task.done():
                self._active_task.cancel()
                await asyncio.gather(self._active_task, return_exceptions=True)
            if self._status_task:
                self._status_task.cancel()
                await asyncio.gather(self._status_task, return_exceptions=True)

    async def stop(self) -> None:
        self._stop.set()

    async def _handle_line(self, line: str) -> None:
        command = line.strip()
        if not command:
            return
        if command.lower() in {"quit", "exit", "q"}:
            await self.stop()
            return
        if command.lower() in {"help", "?"}:
            self._print_help()
            return
        if command.lower() == "status":
            report = await self.sensors.probe(wait_s=1.0)
            print(format_sensor_report(report))
            for reason in report.reasons:
                print(f"  - {reason}")
            return

        try:
            task = parse_task(command, default_altitude_m=self.config.default_altitude_m)
        except ValueError as exc:
            print(f"Command parse error: {exc}")
            print(command_guide())
            return

        if self._active_task and not self._active_task.done():
            if task.action in {TaskAction.LAND, TaskAction.RTL, TaskAction.HOLD}:
                print("Stopping current autonomous task.")
                self._active_task.cancel()
                await asyncio.gather(self._active_task, return_exceptions=True)
            else:
                print("A trajectory is already running. Use 'hold', 'land', or 'rtl' first.")
                return

        report = await self.sensors.probe(wait_s=2.0)
        if task.action not in {TaskAction.LAND, TaskAction.RTL, TaskAction.HOLD} and not report.can_navigate:
            print("Navigation aborted: position estimate is degraded.")
            for reason in report.reasons:
                print(f"  - {reason}")
            return

        try:
            plan = build_trajectory(task, report, self._origin_from_report(report))
        except ValueError as exc:
            print(f"Trajectory error: {exc}")
            return

        if task.action in {TaskAction.LAND, TaskAction.RTL, TaskAction.HOLD}:
            await self._execute(task, plan)
            return

        self._print_plan_summary(report, plan)
        if not await _confirm("Execute this autonomous task? [y/N] "):
            print("Task cancelled.")
            return

        self._active_task = asyncio.create_task(self._execute(task, plan))

    async def _execute(self, task: ParsedTask, plan: TrajectoryPlan) -> None:
        try:
            await self.controller.execute_plan(task, plan)
            print(f"Task complete: {plan.description}")
        except asyncio.CancelledError:
            print("Task cancelled.")
        except (FlightAbort, MavlinkError) as exc:
            print(f"Task aborted: {exc}")
        except Exception as exc:
            LOGGER.exception("Unexpected task failure")
            print(f"Unexpected task failure: {exc}")

    async def _status_loop(self) -> None:
        while True:
            try:
                report = self.sensors.snapshot()
                print(f"\n[health] {format_sensor_report(report)} state={self.controller.state.value}")
            except Exception as exc:
                LOGGER.debug("Could not print health status: %s", exc)
            await asyncio.sleep(self.config.status_interval_s)

    def _origin_from_report(self, report: SensorReport) -> VehicleOrigin:
        lat_deg: Optional[float] = None
        lon_deg: Optional[float] = None
        relative_alt_m: Optional[float] = None

        global_position = self.mavlink.latest_message("GLOBAL_POSITION_INT", 2.0)
        if global_position is not None:
            lat_deg = float(getattr(global_position, "lat", 0)) / 1e7
            lon_deg = float(getattr(global_position, "lon", 0)) / 1e7
            relative_alt_m = float(getattr(global_position, "relative_alt", 0)) / 1000.0
        elif report.mode == NavigationMode.MODE_A_GPS:
            gps = self.mavlink.latest_message("GPS_RAW_INT", 2.0)
            if gps is not None:
                lat_deg = float(getattr(gps, "lat", 0)) / 1e7
                lon_deg = float(getattr(gps, "lon", 0)) / 1e7

        if relative_alt_m is None and report.local_position.valid:
            relative_alt_m = -report.local_position.down_m

        return VehicleOrigin(
            local_north_m=report.local_position.north_m,
            local_east_m=report.local_position.east_m,
            local_down_m=report.local_position.down_m,
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            relative_alt_m=relative_alt_m,
        )

    def _print_plan_summary(self, report: SensorReport, plan: TrajectoryPlan) -> None:
        print(f"Sensor mode: {report.mode.value}")
        print(f"Plan: {plan.description}")
        print(f"Frame: {plan.frame.value}")
        print(f"Targets: {plan.count}")
        print(f"Acceptance radius: {self.config.acceptance_radius_m:.2f}m")
        if plan.local_targets:
            first = plan.local_targets[0]
            last = plan.local_targets[-1]
            print(
                "Local NED preview: "
                f"first=({first.north_m:.1f},{first.east_m:.1f},{first.down_m:.1f}) "
                f"last=({last.north_m:.1f},{last.east_m:.1f},{last.down_m:.1f})"
            )
        if plan.global_targets:
            first_global = plan.global_targets[0]
            print(
                "Global preview: "
                f"lat={first_global.lat_deg:.7f} lon={first_global.lon_deg:.7f} "
                f"alt={first_global.relative_alt_m:.1f}m"
            )

    def _print_help(self) -> None:
        print(command_guide())


async def _confirm(prompt: str) -> bool:
    answer = await asyncio.to_thread(input, prompt)
    return answer.strip().lower() in {"y", "yes"}


async def amain() -> int:
    parser = argparse.ArgumentParser(description="Adaptive MAVLink terminal mission controller")
    parser.add_argument("--connect", default="serial://auto:115200", help="MAVLink connection URL")
    parser.add_argument("--default-altitude", type=float, default=3.0, help="Default task altitude in meters")
    parser.add_argument("--gps-min-satellites", type=int, default=8, help="Minimum satellites for GPS mode")
    parser.add_argument("--acceptance-radius", type=float, default=0.5, help="Waypoint acceptance radius in meters")
    parser.add_argument("--waypoint-timeout", type=float, default=60.0, help="Per-waypoint timeout in seconds")
    parser.add_argument("--max-altitude", type=float, default=15.0, help="Software altitude ceiling in meters")
    parser.add_argument("--battery-min-percent", type=float, default=0.20, help="Minimum pre-task battery fraction")
    parser.add_argument("--battery-min-voltage", type=float, default=0.0, help="Optional minimum pre-task voltage")
    parser.add_argument("--critical-battery-percent", type=float, default=0.12, help="LAND below this battery fraction")
    parser.add_argument("--critical-battery-voltage", type=float, default=0.0, help="LAND below this voltage if set")
    parser.add_argument("--final-action", choices=["hold", "land", "rtl"], default="hold")
    parser.add_argument("--status-interval", type=float, default=5.0, help="Health print interval in seconds")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    config = CliConfig(
        connection_url=args.connect,
        default_altitude_m=args.default_altitude,
        gps_min_satellites=args.gps_min_satellites,
        acceptance_radius_m=args.acceptance_radius,
        waypoint_timeout_s=args.waypoint_timeout,
        max_altitude_m=args.max_altitude,
        battery_min_percent=args.battery_min_percent,
        battery_min_voltage_v=args.battery_min_voltage,
        critical_battery_percent=args.critical_battery_percent,
        critical_battery_voltage_v=args.critical_battery_voltage,
        final_action=args.final_action,
        status_interval_s=args.status_interval,
    )

    mavlink = MavlinkConnection(config.connection_url)
    await mavlink.connect()
    await mavlink.start()

    thresholds = SensorThresholds(
        gps_min_satellites=config.gps_min_satellites,
        battery_min_voltage_v=config.battery_min_voltage_v,
        battery_min_remaining_percent=config.battery_min_percent,
    )
    sensors = SensorDiscovery(mavlink, thresholds)
    await sensors.request_required_messages()

    controller = FlightController(
        mavlink,
        sensors,
        FlightControllerConfig(
            takeoff_altitude_m=config.default_altitude_m,
            waypoint_acceptance_radius_m=config.acceptance_radius_m,
            waypoint_timeout_s=config.waypoint_timeout_s,
            battery_warning_percent=config.battery_min_percent,
            critical_battery_percent=config.critical_battery_percent,
            critical_battery_voltage_v=config.critical_battery_voltage_v,
            max_altitude_m=config.max_altitude_m,
            final_action=config.final_action,
        ),
    )
    repl = MissionRepl(mavlink, sensors, controller, config)

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, lambda: asyncio.create_task(repl.stop()))
        except NotImplementedError:
            pass

    try:
        await repl.run()
    finally:
        await mavlink.close()
    return 0


def main() -> None:
    try:
        raise SystemExit(asyncio.run(amain()))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except MavlinkError as exc:
        LOGGER.error("%s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
