from __future__ import annotations

import argparse
import re
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional

from rich.text import Text
from textual.app import App, ComposeResult
from textual.widgets import Footer, Input, RichLog, Static

SWARM_FLEET = {
    "drone1": ("127.0.0.1", 5001),
    "drone2": ("127.0.0.1", 5002),
    "drone3": ("127.0.0.1", 5003),
}

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
    node_id: str = ""
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
    """Interactive TCP Swarm Client using Textual."""

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

    def __init__(self) -> None:
        super().__init__()
        self.telemetry: dict[str, TelemetryState] = {}
        self.connected_nodes: dict[str, bool] = {node: False for node in SWARM_FLEET}
        self.sockets: dict[str, socket.socket] = {}
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
            placeholder="[CMD] (e.g. 'all: takeoff 5' or 'drone2: rtl') >",
            id="command-input",
            select_on_focus=False,
        )

    def compose(self) -> ComposeResult:
        yield self.header
        yield self.log_widget
        yield self.command_input
        yield Footer()

    def on_mount(self) -> None:
        self.title = "Autonomous Drone Swarm IIT"
        self.sub_title = "Swarm Command Center"
        self.set_interval(0.25, self.refresh_header)
        self.command_input.focus()
        
        for node, (ip, port) in SWARM_FLEET.items():
            self.telemetry[node] = TelemetryState(node_id=node)
            self.system_log(f"[STATUS] Connecting to {node} at {ip}:{port}...")
            threading.Thread(target=self._socket_worker, args=(node, ip, port), daemon=True).start()
            
        threading.Thread(target=self._heartbeat_worker, daemon=True).start()

    def on_unmount(self) -> None:
        self._stop_event.set()
        with self._sock_lock:
            for node, sock in list(self.sockets.items()):
                try:
                    # Automatically command RTL to the drone if the ground station is closed
                    sock.sendall(b"rtl\n")
                except Exception:
                    pass
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                sock.close()
            self.sockets.clear()

    def refresh_header(self) -> None:
        rows = []
        for node in sorted(SWARM_FLEET.keys()):
            state = self.telemetry.get(node, TelemetryState(node_id=node))
            is_connected = self.connected_nodes.get(node, False)
            status_color = "bold green" if is_connected else "bold red"
            
            armed = self._armed_markup(state.armed)
            battery = self._battery_markup(state)
            altitude = "--" if state.altitude_m is None else f"{state.altitude_m:.1f}m"
            pos = state.position if state.position != "--" else state.gps
            
            rows.append(
                f"[{status_color}]{node.upper()}[/{status_color}] | "
                f"[bold cyan]MODE:[/bold cyan] {self._escape(state.mode)} | "
                f"{armed} | {battery} | "
                f"[bold cyan]ALT:[/bold cyan] {altitude} | "
                f"[bold cyan]NAV:[/bold cyan] {self._escape(state.nav)} | "
                f"[bold cyan]POS:[/bold cyan] {self._escape(pos)}"
            )
        self.header.update("\n".join(rows))

    def on_input_submitted(self, event: Input.Submitted) -> None:
        user_input = event.value.strip()
        self.command_input.value = ""
        if not user_input:
            return

        if user_input.lower() in {"quit", "exit"}:
            self.exit()
            return

        if ":" not in user_input:
            self.system_log("[ERROR] Invalid syntax. Use <target>: <command>")
            return
            
        target, command = user_input.split(":", 1)
        target = target.strip()
        command = command.strip()
        
        if not command:
            self.system_log("[ERROR] Command cannot be empty.")
            return

        self.system_log(f"[CMD] {user_input}")
        
        targets = []
        if target == "all":
            targets = list(SWARM_FLEET.keys())
        elif target in SWARM_FLEET:
            targets = [target]
        else:
            self.system_log(f"[ERROR] Unknown target '{target}'.")
            return
            
        for node in targets:
            try:
                with self._sock_lock:
                    sock = self.sockets.get(node)
                    if sock is None:
                        raise ConnectionError("not connected")
                    sock.sendall(f"{command}\n".encode("utf-8"))
            except Exception as exc:
                self.system_log(f"[ERROR] Failed to send command to {node}: {exc}")

    def _socket_worker(self, node: str, ip: str, port: int) -> None:
        while not self._stop_event.is_set():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(8.0)
                sock.connect((ip, port))
                sock.settimeout(1.0)
                with self._sock_lock:
                    self.sockets[node] = sock
                self.call_from_thread(self._set_connected, node, True)
                self.call_from_thread(self.system_log, f"[SUCCESS] {node} connected.")

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
                        self.call_from_thread(self._handle_server_line, node, line.strip())
                if buffer.strip():
                    self.call_from_thread(self._handle_server_line, node, buffer.strip())
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.call_from_thread(self.system_log, f"[ERROR] {node} connection failed: {exc}")
            finally:
                with self._sock_lock:
                    if node in self.sockets:
                        self.sockets[node].close()
                        del self.sockets[node]
                self.call_from_thread(self._set_connected, node, False)
                if not self._stop_event.is_set():
                    self.call_from_thread(self.system_log, f"[STATUS] {node} disconnected. Retrying in 2s...")
                    time.sleep(2.0)

    def _heartbeat_worker(self) -> None:
        while not self._stop_event.is_set():
            time.sleep(1.0)
            with self._sock_lock:
                for node, sock in list(self.sockets.items()):
                    try:
                        sock.sendall(b"__heartbeat__\n")
                    except Exception:
                        pass

    def _handle_server_line(self, node: str, line: str) -> None:
        if not line:
            return
        if line.startswith("[TELEM]"):
            self._update_telemetry(node, line)
        else:
            self.system_log(self._normalize_line(f"[{node}] {line}"))

    def _update_telemetry(self, node: str, line: str) -> None:
        state = self.telemetry.get(node, TelemetryState(node_id=node))

        sysid_match = FIELD_PATTERNS["sysid"].search(line)
        if sysid_match:
            state.sysid = int(sysid_match.group(1))

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

        self.telemetry[node] = state

    def _set_connected(self, node: str, value: bool) -> None:
        self.connected_nodes[node] = value

    def system_log(self, line: str) -> None:
        text = Text.from_markup(line)
        plain = text.plain
        if "[SUCCESS]" in plain:
            text.stylize("bold green")
        elif "[TELEM]" in plain:
            text.stylize("green")
        elif "[STATUS]" in plain:
            text.stylize("yellow")
        elif "[CRITICAL]" in plain or "[ERROR]" in plain:
            text.stylize("bold red")
        elif "[EXEC]" in plain:
            text.stylize("bold cyan")
        elif "[CMD]" in plain:
            text.stylize("bold white")
        elif "ACK:" in plain:
            text.stylize("dim green")
        self.log_widget.write(text)

    @staticmethod
    def _normalize_line(line: str) -> str:
        if "ACK:" in line and "[SUCCESS]" not in line:
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


def main() -> None:
    GroundStationApp().run()


if __name__ == "__main__":
    main()
