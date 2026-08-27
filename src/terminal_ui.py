"""
Thread-safe terminal output helpers for the mission controller using Textual.
"""
from __future__ import annotations

import asyncio
from typing import Callable, Optional

from rich.text import Text
from textual.app import App, ComposeResult
from textual.widgets import Input, RichLog, Static

from sensor_check import NavigationMode, SensorDiscovery, SensorReport


class DroneDashboardApp(App[None]):
    CSS = """
    Screen {
        layout: vertical;
    }
    #header {
        dock: top;
        height: 3;
        background: $boost;
        content-align: center middle;
        border-bottom: solid $primary;
    }
    #logs {
        height: 1fr;
    }
    #cmd-input {
        dock: bottom;
    }
    """

    def __init__(
        self,
        sensors: SensorDiscovery,
        prompt: str = "[CMD] > ",
        controller_state: Optional[Callable[[], str]] = None,
        rate_hz: float = 4.0,
    ) -> None:
        super().__init__()
        self.sensors = sensors
        self.cmd_prompt = prompt
        self.controller_state = controller_state
        self.rate_hz = rate_hz
        
        self._cmd_queue: asyncio.Queue[str] = asyncio.Queue()
        self.log_widget = RichLog(id="logs", max_lines=500, markup=True, wrap=True)
        self.header_widget = Static(id="header")
        self.input_widget = Input(placeholder=self.cmd_prompt, id="cmd-input")

    def compose(self) -> ComposeResult:
        yield self.header_widget
        yield self.log_widget
        yield self.input_widget

    def on_mount(self) -> None:
        self.set_interval(1.0 / self.rate_hz, self.update_telemetry)
        self.input_widget.focus()
        
    def _get_telemetry_markup(self) -> str:
        report = self.sensors.snapshot()
        
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
                
        return (
            f"[bold cyan]MODE:[/bold cyan] {mode} | [bold]{armed_marker}[/bold] | {bat_text} | "
            f"[bold cyan]ALT:[/bold cyan] {altitude:.1f}m | [bold cyan]NAV:[/bold cyan] {nav} | "
            f"[bold cyan]POS:[/bold cyan] ({report.local_position.north_m:.1f}, {report.local_position.east_m:.1f}, {report.local_position.down_m:.1f})"
        )

    def update_telemetry(self) -> None:
        try:
            self.header_widget.update(self._get_telemetry_markup())
        except Exception as exc:
            self.header_widget.update(f"[bold red]Telemetry unavailable: {exc}[/bold red]")

    def on_input_submitted(self, event: Input.Submitted) -> None:
        cmd = event.value.strip()
        self.input_widget.value = ""
        if cmd:
            self._cmd_queue.put_nowait(cmd)

    async def get_command_async(self) -> str:
        return await self._cmd_queue.get()

    def system_log(self, line: str) -> None:
        """Call this from ANY thread/task. Uses call_from_thread internally."""
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

        try:
            self.call_from_thread(self.log_widget.write, text)
        except Exception:
            pass # Fails if app not fully started/stopped, safely ignore
