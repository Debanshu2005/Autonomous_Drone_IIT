"""
Thread-safe terminal output helpers for the mission controller.

The printer clears the current line, writes telemetry/status output, and redraws
the prompt. On terminals with readline support it preserves the active input
buffer too.
"""

from __future__ import annotations

import sys
import threading
from typing import Callable, Optional

from sensor_check import NavigationMode, SensorDiscovery, SensorReport


try:
    import readline
except ImportError:  # pragma: no cover - readline is not available on Windows by default.
    readline = None  # type: ignore[assignment]


class TerminalPrinter:
    def __init__(self, prompt: str = "[CMD] ") -> None:
        self.prompt = prompt
        self._lock = threading.RLock()

    def write_line(self, line: str) -> None:
        with self._lock:
            buffer = self._input_buffer()
            sys.stdout.write("\r\033[K")
            sys.stdout.write(line.rstrip() + "\n")
            sys.stdout.write(self.prompt + buffer)
            sys.stdout.flush()

    def write_status_line(self, line: str) -> None:
        with self._lock:
            buffer = self._input_buffer()
            sys.stdout.write("\r\033[K")
            sys.stdout.write(line.rstrip())
            sys.stdout.write("\n" + self.prompt + buffer)
            sys.stdout.flush()

    def input(self) -> str:
        with self._lock:
            sys.stdout.write(self.prompt)
            sys.stdout.flush()
        return input()

    def _input_buffer(self) -> str:
        if readline is None:
            return ""
        try:
            return readline.get_line_buffer()
        except Exception:
            return ""


class TelemetryThread:
    def __init__(
        self,
        sensors: SensorDiscovery,
        printer: TerminalPrinter,
        *,
        controller_state: Optional[Callable[[], str]] = None,
        rate_hz: float = 2.0,
    ) -> None:
        self.sensors = sensors
        self.printer = printer
        self.controller_state = controller_state
        self.rate_hz = max(0.5, min(5.0, rate_hz))
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="telemetry-printer", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        interval_s = 1.0 / self.rate_hz
        while not self._stop.is_set():
            try:
                report = self.sensors.snapshot()
                self.printer.write_status_line(format_telem_line(report, self.controller_state))
            except Exception as exc:
                self.printer.write_status_line(f"[TELEM] unavailable: {exc}")
            self._stop.wait(interval_s)


def format_telem_line(
    report: SensorReport,
    controller_state: Optional[Callable[[], str]] = None,
) -> str:
    battery_pct = "NA"
    if report.battery.remaining_percent is not None:
        battery_pct = f"{report.battery.remaining_percent * 100:.0f}"
    voltage = "NA" if report.battery.voltage_v is None else f"{report.battery.voltage_v:.2f}"
    altitude = -report.local_position.down_m if report.local_position.valid else 0.0
    nav = _nav_short(report.mode)
    mode = report.autopilot_mode
    if controller_state is not None:
        state = controller_state()
        if state and state != "IDLE":
            mode = f"{mode}/{state}"
    return (
        f"[TELEM] MODE={mode} | ARMED={report.armed} | BAT={battery_pct}% ({voltage}V) | "
        f"ALT={altitude:.1f}m | NAV={nav} | "
        f"POS=({report.local_position.north_m:.1f},"
        f"{report.local_position.east_m:.1f},"
        f"{report.local_position.down_m:.1f})"
    )


def _nav_short(mode: NavigationMode) -> str:
    if mode == NavigationMode.MODE_A_GPS:
        return "GPS"
    if mode == NavigationMode.MODE_B_LOCAL:
        return "FLOW"
    return "NONE"
