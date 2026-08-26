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
from mavlink_io import MavlinkConnection, MavlinkError
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
)
from terminal_ui import TelemetryThread, TerminalPrinter


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
    def __init__(
        self,
        mavlink: MavlinkConnection,
        sensors: SensorDiscovery,
        controller: FlightController,
        config: CliConfig,
        printer: TerminalPrinter,
    ) -> None:
        self.mavlink = mavlink
        self.sensors = sensors
        self.controller = controller
        self.config = config
        self.printer = printer
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._queue: queue.Queue[Optional[TaskSequence]] = queue.Queue()
        self._active_future: Optional[concurrent.futures.Future[None]] = None
        self._worker_stop = threading.Event()
        self._worker_thread = threading.Thread(target=self._worker_loop, name="mission-worker", daemon=True)
        self._telemetry_thread = TelemetryThread(
            sensors,
            printer,
            controller_state=lambda: self.controller.state.value,
            rate_hz=config.telemetry_rate_hz,
        )
        self._status_task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()

    async def run(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._worker_thread.start()
        self._telemetry_thread.start()
        self.printer.write_line("[STATUS] Autonomous terminal mission controller ready. Type 'help' for commands.")
        try:
            while not self._stop.is_set():
                line = await asyncio.to_thread(self.printer.input)
                await self._handle_line(line)
        finally:
            self._telemetry_thread.stop()
            self._worker_stop.set()
            self._flush_queue()
            if self._active_future and not self._active_future.done():
                self._active_future.cancel()
            self._queue.put(None)
            self._worker_thread.join(timeout=3.0)
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
        if command.lower().removeprefix("[cmd]").strip() == "status":
            report = await self.sensors.probe(wait_s=1.0)
            self.printer.write_line(f"[STATUS] {format_sensor_report(report)}")
            for reason in report.reasons:
                self.printer.write_line(f"[STATUS] {reason}")
            return

        try:
            sequence = parse_task_sequence(command, default_altitude_m=self.config.default_altitude_m)
        except ValueError as exc:
            self.printer.write_line(f"[STATUS] Command parse error: {exc}")
            self.printer.write_line(command_guide())
            return

        names = ", ".join(sequence.action_names)
        self.printer.write_line(
            f"[STATUS] Parsing command: Queued {len(sequence.tasks)} tasks ({names})."
        )
        for note in sequence.notes:
            self.printer.write_line(f"[STATUS] Parser note: {note}")

        if self._is_interrupt(sequence):
            self.printer.write_line("[STATUS] High-priority interrupt received. Flushing task queue.")
            self._flush_queue()
            if self._active_future and not self._active_future.done():
                self._active_future.cancel()

        self._queue.put(sequence)

    def _worker_loop(self) -> None:
        while not self._worker_stop.is_set():
            sequence = self._queue.get()
            if sequence is None:
                return
            if self._loop is None:
                self.printer.write_line("[STATUS] Worker loop is not ready.")
                continue
            future = asyncio.run_coroutine_threadsafe(self._execute_sequence(sequence), self._loop)
            self._active_future = future
            try:
                future.result()
            except concurrent.futures.CancelledError:
                self.printer.write_line("[STATUS] Active sequence cancelled.")
            except Exception as exc:
                self.printer.write_line(f"[STATUS] Sequence failed: {exc}")
            finally:
                if self._active_future is future:
                    self._active_future = None

    async def _execute_sequence(self, sequence: TaskSequence) -> None:
        try:
            await self.controller.execute_task_queue(
                sequence.tasks,
                lambda task, report: build_trajectory(task, report, self._origin_from_report(report)),
            )
        except (FlightAbort, MavlinkError) as exc:
            self.printer.write_line(f"[STATUS] Task aborted: {exc}")
        except asyncio.CancelledError:
            self.printer.write_line("[STATUS] Sequence cancellation acknowledged.")
            raise
        except Exception as exc:
            LOGGER.exception("Unexpected task failure")
            self.printer.write_line(f"[STATUS] Unexpected task failure: {exc}")

    def _flush_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

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

    def _print_plan_summary(self, report: SensorReport, sequence: TaskSequence) -> None:
        for task in sequence.tasks:
            if task.action in {TaskAction.HOVER, TaskAction.HOLD, TaskAction.LAND, TaskAction.RTL}:
                continue
            plan = build_trajectory(task, report, self._origin_from_report(report))
            self.printer.write_line(f"[STATUS] Sensor mode: {report.mode.value}")
            self.printer.write_line(f"[STATUS] Plan: {plan.description}")
            self.printer.write_line(f"[STATUS] Frame: {plan.frame.value}")
            self.printer.write_line(f"[STATUS] Targets: {plan.count}")
            if plan.local_targets:
                first = plan.local_targets[0]
                last = plan.local_targets[-1]
                self.printer.write_line(
                    "[STATUS] Local NED preview: "
                    f"first=({first.north_m:.1f},{first.east_m:.1f},{first.down_m:.1f}) "
                    f"last=({last.north_m:.1f},{last.east_m:.1f},{last.down_m:.1f})"
                )
            if plan.global_targets:
                first_global = plan.global_targets[0]
                self.printer.write_line(
                    "[STATUS] Global preview: "
                    f"lat={first_global.lat_deg:.7f} lon={first_global.lon_deg:.7f} "
                    f"alt={first_global.relative_alt_m:.1f}m"
                )
            return

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

    def _print_help(self) -> None:
        self.printer.write_line(command_guide())


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

    printer = TerminalPrinter(prompt="[CMD] ")
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
        status_sink=printer.write_line,
    )
    repl = MissionRepl(mavlink, sensors, controller, config, printer)

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
