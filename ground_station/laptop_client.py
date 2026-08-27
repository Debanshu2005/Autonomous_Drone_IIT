from __future__ import annotations

import argparse
import re
import socket
import threading
from dataclasses import dataclass
from typing import Optional

from rich.text import Text
from textual.app import App, ComposeResult
from textual.widgets import Footer, Input, RichLog, Static


FIELD_PATTERNS = {
    "sysid": re.compile(r"\bSYSID[:=]\s*(\d+)", re.IGNORECASE),
    "mode": re.compile(r"\bMODE[:=]\s*([^|]+?)(?=\s*\||$)", re.IGNORECASE),
    "armed": re.compile(r"\bARMED[:=]\s*(true|false|yes|no|1|0)", re.IGNORECASE),
    "battery": re.compile(
        r"\bBAT[:=]\s*(-?\d+(?:\.\d+)?)%?\s*(?:\((-?\d+(?:\.\d+)?)V\))?",
        re.IGNORECASE,
    ),
    "altitude": re.compile(r"\bALT[:=]\s*(-?\d+(?:\.\d+)?)m?", re.IGNORECASE),
    "nav": re.compile(r"\bNAV[:=]\s*([^|]+?)(?=\s*\||$)", re.IGNORECASE),
    "position": re.compile(r"\bPOS[:=]\s*\(([^)]+)\)", re.IGNORECASE),
    "gps": re.compile(r"\bGPS[:=]\s*\(([^)]+)\)", re.IGNORECASE),
}


@dataclass
class TelemetryState:
    sysid: int = 1
    mode: str = "--"
    armed: Optional[bool] = None
    battery_percent: Optional[float] = None
    voltage_v: Optional[float] = None
    altitude_m: Optional[float] = None
    nav: str = "--"
    position: str = "--"
    gps: str = "--"


class GroundStationApp(App[None]):
    """Interactive TCP client for the Raspberry Pi Edge Brain."""

    BINDINGS = [
        ("ctrl+c", "quit", "Quit"),
        ("ctrl+q", "quit", "Quit"),
    ]

    CSS = """
    Screen {
        layout: vertical;
        background: #111318;
        color: #d7dde8;
    }

    #telemetry-header {
        height: auto;
        min-height: 3;
        padding: 1 2;
        background: #141922;
        border-bottom: tall #1c7ed6;
        content-align: center middle;
    }

    #system-log {
        height: 1fr;
        background: #0f1117;
        padding: 1 2;
    }

    #command-input {
        height: 3;
        padding: 0 2;
        background: #171a21;
        border-top: tall #202633;
    }
    """

    def __init__(self, host: str, port: int) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.telemetry: dict[int, TelemetryState] = {}
        self.connected = False
        self._sock: Optional[socket.socket] = None
        self._sock_lock = threading.Lock()
        self._stop_event = threading.Event()

        self.header = Static(id="telemetry-header")
        self.log_widget = RichLog(
            id="system-log",
            max_lines=500,
            markup=True,
            wrap=True,
            highlight=False,
            auto_scroll=True,
        )
        self.command_input = Input(
            placeholder="[CMD] >",
            id="command-input",
            select_on_focus=False,
        )

    def compose(self) -> ComposeResult:
        yield self.header
        yield self.log_widget
        yield self.command_input
        yield Footer()

    def on_mount(self) -> None:
        self.title = "Autonomous Drone IIT"
        self.sub_title = f"Edge Brain {self.host}:{self.port}"
        self.set_interval(0.25, self.refresh_header)
        self.command_input.focus()
        self.system_log(f"[STATUS] Connecting to Edge Brain at {self.host}:{self.port}...")
        threading.Thread(target=self._socket_worker, daemon=True).start()

    def on_unmount(self) -> None:
        self._stop_event.set()
        with self._sock_lock:
            if self._sock is not None:
                try:
                    self._sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self._sock.close()
                self._sock = None

    def refresh_header(self) -> None:
        if not self.telemetry:
            status = "[bold green]ONLINE[/bold green]" if self.connected else "[bold red]OFFLINE[/bold red]"
            self.header.update(f"{status} | [bold cyan]MODE:[/bold cyan] -- | [bold cyan]ALT:[/bold cyan] -- | [bold cyan]NAV:[/bold cyan] -- | [bold cyan]POS:[/bold cyan] --")
            return

        rows = []
        for sysid in sorted(self.telemetry):
            state = self.telemetry[sysid]
            armed = self._armed_markup(state.armed)
            battery = self._battery_markup(state)
            altitude = "--" if state.altitude_m is None else f"{state.altitude_m:.1f}m"
            pos = state.position if state.position != "--" else state.gps
            rows.append(
                f"[bold yellow]SYSID:{state.sysid}[/bold yellow] | "
                f"[bold cyan]MODE:[/bold cyan] {self._escape(state.mode)} | "
                f"{armed} | {battery} | "
                f"[bold cyan]ALT:[/bold cyan] {altitude} | "
                f"[bold cyan]NAV:[/bold cyan] {self._escape(state.nav)} | "
                f"[bold cyan]POS:[/bold cyan] {self._escape(pos)}"
            )
        self.header.update("\n".join(rows))

    def on_input_submitted(self, event: Input.Submitted) -> None:
        command = event.value.strip()
        self.command_input.value = ""
        if not command:
            return

        if command.lower() in {"quit", "exit"}:
            self.exit()
            return

        self.system_log(f"[CMD] {command}")
        try:
            with self._sock_lock:
                if self._sock is None:
                    raise ConnectionError("not connected")
                self._sock.sendall(f"{command}\n".encode("utf-8"))
        except Exception as exc:
            self.system_log(f"[ERROR] Failed to send command: {exc}")

    def _socket_worker(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(8.0)
            sock.connect((self.host, self.port))
            sock.settimeout(1.0)
            with self._sock_lock:
                self._sock = sock
            self.call_from_thread(self._set_connected, True)
            self.call_from_thread(self.system_log, "[SUCCESS] Connected to Edge Brain.")

            buffer = ""
            while not self._stop_event.is_set():
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    self.call_from_thread(self._handle_server_line, line.strip())
            if buffer.strip():
                self.call_from_thread(self._handle_server_line, buffer.strip())
        except Exception as exc:
            self.call_from_thread(self.system_log, f"[ERROR] Connection failed: {exc}")
        finally:
            with self._sock_lock:
                if self._sock is not None:
                    self._sock.close()
                    self._sock = None
            self.call_from_thread(self._set_connected, False)
            self.call_from_thread(self.system_log, "[STATUS] Disconnected.")

    def _handle_server_line(self, line: str) -> None:
        if not line:
            return
        if line.startswith("[TELEM]"):
            self._update_telemetry(line)
        self.system_log(self._normalize_line(line))

    def _update_telemetry(self, line: str) -> None:
        sysid_match = FIELD_PATTERNS["sysid"].search(line)
        sysid = int(sysid_match.group(1)) if sysid_match else 1
        state = self.telemetry.get(sysid, TelemetryState(sysid=sysid))

        mode_match = FIELD_PATTERNS["mode"].search(line)
        if mode_match:
            state.mode = mode_match.group(1).strip()

        armed_match = FIELD_PATTERNS["armed"].search(line)
        if armed_match:
            state.armed = armed_match.group(1).lower() in {"true", "yes", "1"}

        battery_match = FIELD_PATTERNS["battery"].search(line)
        if battery_match:
            state.battery_percent = float(battery_match.group(1))
            if battery_match.group(2) is not None:
                state.voltage_v = float(battery_match.group(2))

        altitude_match = FIELD_PATTERNS["altitude"].search(line)
        if altitude_match:
            state.altitude_m = float(altitude_match.group(1))

        nav_match = FIELD_PATTERNS["nav"].search(line)
        if nav_match:
            state.nav = nav_match.group(1).strip()

        position_match = FIELD_PATTERNS["position"].search(line)
        if position_match:
            state.position = f"({position_match.group(1).strip()})"

        gps_match = FIELD_PATTERNS["gps"].search(line)
        if gps_match:
            state.gps = f"({gps_match.group(1).strip()})"
            if state.nav == "--":
                state.nav = "GPS"

        self.telemetry[sysid] = state

    def _set_connected(self, value: bool) -> None:
        self.connected = value

    def system_log(self, line: str) -> None:
        text = Text.from_markup(line)
        plain = text.plain
        if plain.startswith("[SUCCESS]"):
            text.stylize("bold green")
        elif plain.startswith("[TELEM]"):
            text.stylize("green")
        elif plain.startswith("[STATUS]"):
            text.stylize("yellow")
        elif plain.startswith("[CRITICAL]") or plain.startswith("[ERROR]"):
            text.stylize("bold red")
        elif plain.startswith("[EXEC]"):
            text.stylize("bold cyan")
        elif plain.startswith("[CMD]"):
            text.stylize("bold white")
        elif plain.startswith("ACK:"):
            text.stylize("dim green")
        self.log_widget.write(text)

    @staticmethod
    def _normalize_line(line: str) -> str:
        if line.startswith("ACK:"):
            return f"[SUCCESS] {line}"
        return line

    @staticmethod
    def _armed_markup(armed: Optional[bool]) -> str:
        if armed is None:
            return "[dim]ARMED: --[/dim]"
        if armed:
            return "[bold green]ARMED[/bold green]"
        return "[bold red]DISARMED[/bold red]"

    @staticmethod
    def _battery_markup(state: TelemetryState) -> str:
        if state.battery_percent is None:
            return "[dim]BAT: --[/dim]"
        color = "green"
        if state.battery_percent <= 20:
            color = "red"
        elif state.battery_percent <= 35:
            color = "yellow"
        voltage = "" if state.voltage_v is None else f" ({state.voltage_v:.2f}V)"
        return f"[{color}]BAT: {state.battery_percent:.0f}%{voltage}[/{color}]"

    @staticmethod
    def _escape(value: str) -> str:
        return value.replace("[", "\\[").replace("]", "\\]")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Textual laptop client for the Raspberry Pi Edge Brain.")
    parser.add_argument("host", nargs="?", default="192.168.4.1", help="Edge Brain host/IP address.")
    parser.add_argument("--port", type=int, default=5000, help="Edge Brain TCP command port.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    GroundStationApp(args.host, args.port).run()


if __name__ == "__main__":
    main()
