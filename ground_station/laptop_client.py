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
import concurrent.futures
import logging
import queue
import signal
import threading
from dataclasses import dataclass
from typing import Optional

from flight_controller import FlightAbort, FlightController, FlightControllerConfig
from mavlink_io import SwarmManager, MavlinkConnection, MavlinkError
from sensor_check import (
    NavigationMode,
    SensorDiscovery,
    SensorReport,
    SensorThresholds,
    format_sensor_report,
)
from trajectory_engine import (
    TaskAction,
    TaskSequence,
    VehicleOrigin,
    build_trajectory,
    command_guide,
    parse_task_sequence,
    parse_swarm_target,
)
from terminal_ui import DroneDashboardApp


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
    telemetry_rate_hz: float


class MissionRepl:
    """Read-eval-print loop for parsing and executing commands across a swarm."""

    def __init__(
        self,
        swarm: SwarmManager,
        sensors_map: dict[int, SensorDiscovery],
        controllers: dict[int, FlightController],
        config: CliConfig,
        dashboard: DroneDashboardApp,
    ) -> None:
        self.swarm = swarm
        self.sensors_map = sensors_map
        self.controllers = controllers
        self.config = config
        self.dashboard = dashboard
        
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._queues: dict[int, asyncio.Queue] = {}
        self._active_futures: dict[int, asyncio.Task] = {}
        self._stop = asyncio.Event()

    async def run(self) -> None:
        self._loop = asyncio.get_running_loop()
        
        # Create an async worker queue for each drone
        for sysid in self.controllers:
            self._queues[sysid] = asyncio.Queue()
            asyncio.create_task(self._worker_loop(sysid))
            
        self.dashboard.system_log("[STATUS] Autonomous swarm controller ready. Type 'help' for commands.")
        try:
            while not self._stop.is_set():
                line = await self.dashboard.get_command_async()
                await self._handle_line(line)
        finally:
            self._flush_queue()
            for task in self._active_futures.values():
                if not task.done():
                    task.cancel()
                    
    async def stop(self) -> None:
        self._stop.set()
        self.dashboard._cmd_queue.put_nowait("")

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
        if command.lower().removeprefix("[cmd]").strip() == "status":
            for sysid, sensors in self.sensors_map.items():
                report = await sensors.probe(wait_s=1.0)
                self.dashboard.system_log(f"[SYSID:{sysid}] [STATUS] {format_sensor_report(report)}")
                for reason in report.reasons:
                    self.dashboard.system_log(f"[SYSID:{sysid}] [STATUS] {reason}")
            return

        try:
            target_sysids, command_body = parse_swarm_target(command, list(self.controllers.keys()))
            if not command_body:
                return
            if not target_sysids:
                self.dashboard.system_log(f"[STATUS] No valid drones targeted. Active sysids: {list(self.controllers.keys())}")
                return
                
            sequence = parse_task_sequence(command_body, default_altitude_m=self.config.default_altitude_m)
        except ValueError as exc:
            self.dashboard.system_log(f"[STATUS] Command parse error: {exc}")
            self.dashboard.system_log(command_guide())
            return

        names = ", ".join(sequence.action_names)
        self.dashboard.system_log(
            f"[STATUS] Queued {len(sequence.tasks)} tasks ({names}) for SYSIDs: {target_sysids}"
        )
        for note in sequence.notes:
            self.dashboard.system_log(f"[STATUS] Parser note: {note}")

        if self._is_interrupt(sequence):
            self.dashboard.system_log(f"[STATUS] High-priority interrupt received. Flushing task queues for {target_sysids}.")
            for sysid in target_sysids:
                self._flush_queue_sysid(sysid)
                task = self._active_futures.get(sysid)
                if task and not task.done():
                    task.cancel()

        for sysid in target_sysids:
            self._queues[sysid].put_nowait(sequence)

    async def _worker_loop(self, sysid: int) -> None:
        queue = self._queues[sysid]
        while not self._stop.is_set():
            sequence = await queue.get()
            if sequence is None:
                continue
                
            task = asyncio.create_task(self._execute_sequence(sysid, sequence))
            self._active_futures[sysid] = task
            try:
                await task
            except asyncio.CancelledError:
                self.dashboard.system_log(f"[SYSID:{sysid}] [STATUS] Active sequence cancelled.")
            except Exception as exc:
                self.dashboard.system_log(f"[SYSID:{sysid}] [STATUS] Sequence failed: {exc}")
            finally:
                if self._active_futures.get(sysid) is task:
                    del self._active_futures[sysid]
            queue.task_done()

    async def _execute_sequence(self, sysid: int, sequence: TaskSequence) -> None:
        controller = self.controllers[sysid]
        sensors = self.sensors_map[sysid]
        
        def origin_from_report(report):
            return VehicleOrigin(
                local_north_m=report.local_position.north_m,
                local_east_m=report.local_position.east_m,
                local_down_m=report.local_position.down_m,
                lat_deg=report.global_position.lat_deg,
                lon_deg=report.global_position.lon_deg,
                relative_alt_m=report.global_position.relative_alt_m,
            )
            
        try:
            await controller.execute_task_queue(
                sequence.tasks,
                lambda task, report: build_trajectory(task, report, origin_from_report(report)),
            )
        except (FlightAbort, MavlinkError) as exc:
            self.dashboard.system_log(f"[SYSID:{sysid}] [STATUS] Task aborted: {exc}")
        except asyncio.CancelledError:
            self.dashboard.system_log(f"[SYSID:{sysid}] [STATUS] Sequence cancellation acknowledged.")
            raise
        except Exception as exc:
            LOGGER.exception("Unexpected task failure")
            self.dashboard.system_log(f"[SYSID:{sysid}] [STATUS] Unexpected task failure: {exc}")

    def _flush_queue(self) -> None:
        for sysid in self._queues:
            self._flush_queue_sysid(sysid)
            
    def _flush_queue_sysid(self, sysid: int) -> None:
        if sysid not in self._queues: return
        q = self._queues[sysid]
        while not q.empty():
            q.get_nowait()
            q.task_done()

    def _is_interrupt(self, sequence: TaskSequence) -> bool:
        return (
            len(sequence.tasks) == 1
            and sequence.tasks[0].action in {
                TaskAction.SET_MODE,
                TaskAction.HOLD,
                TaskAction.LAND,
                TaskAction.RTL,
            }
        )

    def _print_help(self) -> None:
        self.dashboard.system_log("Swarm commands: all: <cmd>, drone2: <cmd>")
        self.dashboard.system_log(command_guide())

async def amain() -> int:
    parser = argparse.ArgumentParser(description="Adaptive MAVLink terminal mission controller")
    parser.add_argument("--connect", default="serial://auto:115200", help="MAVLink connection URL (or base URL)")
    parser.add_argument("--swarm-count", type=int, default=1, help="Number of swarm instances to connect to, increments port by 10")
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
    parser.add_argument("--telemetry-rate", type=float, default=2.0, help="Background telemetry print rate, 0.5Hz to 5Hz")
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
        telemetry_rate_hz=args.telemetry_rate,
    )

    urls = [u.strip() for u in config.connection_url.split(",")]
    if args.swarm_count > 1 and len(urls) == 1:
        import re
        m = re.match(r"(.*?):(\d+)$", urls[0])
        if m:
            base_prefix = m.group(1)
            base_port = int(m.group(2))
            urls = [f"{base_prefix}:{base_port + i * 10}" for i in range(args.swarm_count)]
        else:
            LOGGER.warning("Could not parse port from --connect for --swarm-count. Using as single connection.")
    swarm = SwarmManager(urls)
    await swarm.connect_all()
    
    if not swarm.connections:
        LOGGER.error("No drones connected! Exiting.")
        return 1

    thresholds = SensorThresholds(
        gps_min_satellites=config.gps_min_satellites,
        battery_min_voltage_v=config.battery_min_voltage_v,
        battery_min_remaining_percent=config.battery_min_percent,
    )

    sensors_map = {}
    for sysid, conn in swarm.connections.items():
        sensors = SensorDiscovery(conn, thresholds)
        await sensors.request_required_messages()
        sensors_map[sysid] = sensors

    dashboard = DroneDashboardApp(sensors_map, prompt="[CMD] > ", rate_hz=config.telemetry_rate_hz)

    controllers = {}
    controller_states = {}
    for sysid, conn in swarm.connections.items():
        sensors = sensors_map[sysid]
        controller = FlightController(
            conn,
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
            status_sink=dashboard.system_log,
        )
        controllers[sysid] = controller
        # Need closure to capture sysid
        def get_state(c=controller):
            return c.state.value
        controller_states[sysid] = get_state
        
    dashboard.controller_state_map = controller_states
    repl = MissionRepl(swarm, sensors_map, controllers, config, dashboard)

    from safety import SafetyMonitor
    safety = SafetyMonitor(repl, config.critical_battery_voltage_v)
    safety.start()
    
    await swarm.start_all()

    repl_task = asyncio.create_task(repl.run())
    try:
        await dashboard.run_async()
    finally:
        await repl.stop()
        await repl_task
        await swarm.close_all()
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
