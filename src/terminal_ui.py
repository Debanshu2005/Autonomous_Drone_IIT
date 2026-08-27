"""
Thread-safe terminal output helpers for the mission controller using Rich.
"""
from __future__ import annotations

import asyncio
import sys
import threading
from typing import Callable, Optional

from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.text import Text

from sensor_check import NavigationMode, SensorDiscovery, SensorReport

try:
    import msvcrt
except ImportError:
    msvcrt = None


class RichDashboard:
    def __init__(
        self,
        sensors: SensorDiscovery,
        prompt: str = "[CMD] > ",
        controller_state: Optional[Callable[[], str]] = None,
        rate_hz: float = 4.0,
    ) -> None:
        self.sensors = sensors
        self.prompt = prompt
        self.controller_state = controller_state
        self.rate_hz = rate_hz
        
        self.console = Console()
        self.layout = Layout()
        self._init_layout()
        
        self._logs: list[Text] = []
        self._max_logs = 50
        
        self._input_buffer = ""
        self._cmd_queue: asyncio.Queue[str] = asyncio.Queue()
        
        self._stop_event = threading.Event()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        
        self._live = Live(self.layout, console=self.console, refresh_per_second=self.rate_hz, screen=True)
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, name="keyboard-listener", daemon=True)
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, name="telemetry-updater", daemon=True)

    def _init_layout(self) -> None:
        self.layout.split(
            Layout(name="header", size=3),
            Layout(name="body"),
            Layout(name="footer", size=3),
        )
        self._update_footer()
        self._update_body()

    def start(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._live.start()
        self._keyboard_thread.start()
        self._telemetry_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._live.is_started:
            self._live.stop()
        if self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=1.0)
        if self._keyboard_thread.is_alive():
            self._keyboard_thread.join(timeout=1.0)

    async def get_command_async(self) -> str:
        return await self._cmd_queue.get()

    def log(self, line: str) -> None:
        """Parse log lines to add rich semantic colors."""
        line = line.strip()
        text = Text(line)
        
        if line.startswith("[SUCCESS]"):
            text.stylize("bold green")
        elif line.startswith("[TELEM]"):
            text.stylize("green")
        elif line.startswith("[STATUS]"):
            text.stylize("yellow")
        elif line.startswith("[CRITICAL]") or line.startswith("[ERROR]"):
            text.stylize("bold red")
        elif line.startswith("[EXEC]"):
            text.stylize("bold cyan")
            
        self._logs.append(text)
        if len(self._logs) > self._max_logs:
            self._logs.pop(0)
            
        self._update_body()

    def write_line(self, line: str) -> None:
        # Compatibility layer for existing write_line calls
        self.log(line)
        
    def write_status_line(self, line: str) -> None:
        # Status line is used for telem which is now in header. We can ignore or log it.
        pass

    def _update_header(self) -> None:
        try:
            report = self.sensors.snapshot()
            
            # Formatting
            battery_pct = "NA"
            if report.battery.remaining_percent is not None:
                battery_pct = f"{report.battery.remaining_percent * 100:.0f}"
            voltage = "NA" if report.battery.voltage_v is None else f"{report.battery.voltage_v:.2f}"
            
            bat_color = "red" if (report.battery.voltage_v and report.battery.voltage_v < 10.5) else "green"
            bat_text = f"[{bat_color}]BAT: {battery_pct}% ({voltage}V)[/{bat_color}]"
            
            armed_marker = "🟢 ARMED" if report.armed else "🔴 DISARMED"
            
            altitude = -report.local_position.down_m if report.local_position.valid else 0.0
            nav = "GPS" if report.mode == NavigationMode.MODE_A_GPS else ("FLOW" if report.mode == NavigationMode.MODE_B_LOCAL else "NONE")
            mode = report.autopilot_mode
            if self.controller_state is not None:
                state = self.controller_state()
                if state and state != "IDLE":
                    mode = f"{mode}/{state}"
                    
            header_text = Text.from_markup(
                f"[bold cyan]MODE:[/bold cyan] {mode} | [bold]{armed_marker}[/bold] | {bat_text} | "
                f"[bold cyan]ALT:[/bold cyan] {altitude:.1f}m | [bold cyan]NAV:[/bold cyan] {nav} | "
                f"[bold cyan]POS:[/bold cyan] ({report.local_position.north_m:.1f}, {report.local_position.east_m:.1f}, {report.local_position.down_m:.1f})"
            )
            self.layout["header"].update(Panel(header_text, title="Flight Telemetry", style="bold blue"))
        except Exception as exc:
            self.layout["header"].update(Panel(f"Telemetry unavailable: {exc}", title="Flight Telemetry", style="bold red"))

    def _update_body(self) -> None:
        body_text = Text("\\n").join(self._logs)
        self.layout["body"].update(Panel(body_text, title="System Logs"))

    def _update_footer(self) -> None:
        footer_text = Text(self.prompt + self._input_buffer + "█")
        self.layout["footer"].update(Panel(footer_text, title="Input"))

    def _telemetry_loop(self) -> None:
        interval_s = 1.0 / self.rate_hz
        while not self._stop_event.is_set():
            self._update_header()
            self._stop_event.wait(interval_s)

    def _keyboard_loop(self) -> None:
        if msvcrt is None:
            self.log("[CRITICAL] msvcrt not available. Interactive input disabled.")
            return
        
        while not self._stop_event.is_set():
            if msvcrt.kbhit():
                try:
                    char = msvcrt.getch()
                    
                    if char in (b'\\x03', b'\\x1a'):  # Ctrl+C or Ctrl+Z
                        pass # Handled by signal handler in safety.py
                    elif char in (b'\\r', b'\\n'):
                        # Enter pressed
                        cmd = self._input_buffer.strip()
                        self._input_buffer = ""
                        self._update_footer()
                        if cmd and self._loop:
                            asyncio.run_coroutine_threadsafe(self._cmd_queue.put(cmd), self._loop)
                    elif char == b'\\x08':
                        # Backspace
                        self._input_buffer = self._input_buffer[:-1]
                        self._update_footer()
                    elif char in (b'\\x00', b'\\xe0'):
                        # Arrow keys (ignore)
                        msvcrt.getch()
                    else:
                        # Normal char
                        decoded = char.decode('utf-8', errors='ignore')
                        if decoded.isprintable():
                            self._input_buffer += decoded
                            self._update_footer()
                except Exception:
                    pass
            else:
                self._stop_event.wait(0.05)
