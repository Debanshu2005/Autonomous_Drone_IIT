import asyncio
import logging
import socket
import time
import threading
from typing import Optional, Sequence

# AI Pipeline Hooks (Placeholders)
try:
    import vosk
    import onnxruntime
except ImportError:
    logging.warning("Vosk/ONNX modules missing, AI pipelines will run in stub mode.")

from flight_controller import FlightAbort, FlightController, FlightControllerConfig
from mavlink_io import SwarmManager, MavlinkError
from sensor_check import SensorDiscovery, SensorThresholds, format_sensor_report
from trajectory_engine import (
    VehicleOrigin,
    build_trajectory,
    command_guide,
    parse_task_sequence,
)

LOGGER = logging.getLogger("pi_edge_brain")
DEFAULT_CONNECTION_URL = "serial:///dev/ttyAMA0:921600"


def expand_connection_urls(values: Optional[Sequence[str]]) -> list[str]:
    if not values:
        return [DEFAULT_CONNECTION_URL]
    urls: list[str] = []
    for value in values:
        urls.extend(part.strip() for part in value.split(",") if part.strip())
    return urls or [DEFAULT_CONNECTION_URL]


def parse_targeted_command(cmd: str) -> tuple[Optional[int], str]:
    text = cmd.strip()
    lower = text.lower()
    for prefix in ("sysid:", "sysid=", "drone:", "drone="):
        if lower.startswith(prefix):
            rest = text[len(prefix):].strip()
            sysid_text, _, command = rest.partition(" ")
            try:
                return int(sysid_text), command.strip()
            except ValueError:
                return None, text
    if text.startswith("@"):
        sysid_text, _, command = text[1:].partition(" ")
        try:
            return int(sysid_text), command.strip()
        except ValueError:
            return None, text
    return None, text

class EdgeBrain:
    def __init__(
        self,
        serial_url: str | Sequence[str] = DEFAULT_CONNECTION_URL,
        host="0.0.0.0",
        port=5000,
    ):
        if isinstance(serial_url, str):
            self.connection_urls = expand_connection_urls([serial_url])
        else:
            self.connection_urls = expand_connection_urls(serial_url)
        self.serial_url = ", ".join(self.connection_urls)
        self.host = host
        self.port = port
        self.swarm = SwarmManager(self.connection_urls)
        self.sensors_map = {}
        self.controllers = {}
        
        self.last_heartbeat_time = time.time()
        self.client_connected = False
        
        self.server_sock = None
        self.client_sock = None
        self._client_lock = threading.Lock()
        
        self._stop_event = asyncio.Event()

    def run_ai_pipelines(self):
        # Stub for Vosk/Whisper speech-to-text
        # Stub for ONNX face recognition
        pass

    async def watchdog_task(self):
        """Failsafe network watchdog: RTL if laptop socket connection drops > 5s."""
        while not self._stop_event.is_set():
            await asyncio.sleep(1.0)
            if self.client_connected and time.time() - self.last_heartbeat_time > 5.0:
                LOGGER.error("CRITICAL: Network watchdog timeout! Laptop connection lost > 5s.")
                self.client_connected = False # Prevent multiple RTL triggers for same drop
                # Trigger RTL on all connected controllers
                for sysid, controller in self.controllers.items():
                    LOGGER.info(f"Triggering MAVLink RTL for SYSID {sysid}")
                    # Using parse_task_sequence to parse RTL
                    seq = parse_task_sequence("rtl")
                    await self._execute_sequence(sysid, seq)

    async def _execute_sequence(self, sysid: int, sequence) -> None:
        if sysid not in self.controllers:
            return
        controller = self.controllers[sysid]
        
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
        except Exception as exc:
            LOGGER.error(f"[SYSID:{sysid}] Sequence failed: {exc}")

    def _socket_listener(self, loop):
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(1)
        LOGGER.info(f"Listening for ground station on {self.host}:{self.port}...")
        
        while not self._stop_event.is_set():
            try:
                self.server_sock.settimeout(1.0)
                client, addr = self.server_sock.accept()
                LOGGER.info(f"Ground station connected from {addr}")
                self.client_sock = client
                self.client_connected = True
                self.last_heartbeat_time = time.time()
                
                while not self._stop_event.is_set():
                    data = client.recv(1024)
                    if not data:
                        break
                    self.last_heartbeat_time = time.time()
                    cmd = data.decode('utf-8').strip()
                    if cmd:
                        LOGGER.info(f"Received CMD: {cmd}")
                        asyncio.run_coroutine_threadsafe(self.process_command(cmd), loop)
                        
                        # Send ack back
                        client.sendall(f"ACK: {cmd}\n".encode('utf-8'))
                        
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop_event.is_set():
                    LOGGER.error(f"Socket error: {e}")
            finally:
                if self.client_sock:
                    self.client_sock.close()
                    self.client_sock = None

    async def process_command(self, cmd: str):
        # Broadcast unless the laptop command starts with sysid:<id> or @<id>.
        try:
            target_sysid, command_text = parse_targeted_command(cmd)
            if not command_text:
                raise ValueError("empty command")
            sequence = parse_task_sequence(command_text)
            target_sysids = [target_sysid] if target_sysid is not None else list(self.controllers)
            missing = [sysid for sysid in target_sysids if sysid not in self.controllers]
            if missing:
                raise ValueError(f"unknown SYSID(s): {', '.join(str(sysid) for sysid in missing)}")
            target_label = (
                f"SYSID:{target_sysid}"
                if target_sysid is not None
                else f"all {len(target_sysids)} drone(s)"
            )
            self._send_client_line(
                f"[STATUS] Parsed command for {target_label}: queued {len(sequence.tasks)} task(s)."
            )
            for sysid in target_sysids:
                asyncio.create_task(self._execute_sequence(sysid, sequence))
        except Exception as e:
            msg = f"Command parse error: {e}"
            LOGGER.error(msg)
            self._send_client_line(f"[ERROR] {msg}")

    def _send_client_line(self, line: str) -> None:
        if not self.client_sock or not self.client_connected:
            return
        payload = line if line.endswith("\n") else f"{line}\n"
        try:
            with self._client_lock:
                self.client_sock.sendall(payload.encode("utf-8"))
        except Exception as exc:
            LOGGER.warning("Failed to send laptop client message: %s", exc)
            self.client_connected = False

    @staticmethod
    def _line_with_sysid(sysid: int, line: str) -> str:
        if line.startswith("[") and "] " in line:
            return line.replace("] ", f"] SYSID:{sysid} ", 1)
        return f"[SYSID:{sysid}] {line}"

    @staticmethod
    def _nav_label(report) -> str:
        mode = str(report.mode.value).lower()
        if "gps-enabled" in mode:
            return "GPS"
        if "gps-denied" in mode or "optical" in mode:
            return "FLOW"
        return "NONE"

    @staticmethod
    def _telemetry_line(sysid: int, report) -> str:
        battery = "BAT: --"
        if report.battery.remaining_percent is not None:
            battery = f"BAT: {report.battery.remaining_percent * 100:.0f}%"
            if report.battery.voltage_v is not None:
                battery += f" ({report.battery.voltage_v:.2f}V)"
        elif report.battery.voltage_v is not None:
            battery = f"BAT: -- ({report.battery.voltage_v:.2f}V)"

        local = report.local_position
        global_position = report.global_position
        altitude_m = (
            global_position.relative_alt_m
            if global_position.valid
            else (-local.down_m if local.valid else 0.0)
        )

        return (
            f"[TELEM] SYSID:{sysid} MODE:{report.autopilot_mode} | "
            f"ARMED:{str(report.armed).lower()} | {battery} | "
            f"ALT:{altitude_m:.1f}m | NAV:{EdgeBrain._nav_label(report)} | "
            f"POS:({local.north_m:.1f}, {local.east_m:.1f}, {local.down_m:.1f}) | "
            f"GPS:({global_position.lat_deg:.7f}, {global_position.lon_deg:.7f})"
        )

    async def start(self):
        LOGGER.info(f"Connecting to Pixhawk v6x at {self.serial_url}...")
        await self.swarm.connect_all()
        
        if not self.swarm.connections:
            LOGGER.error("No MAVLink connection established.")
            return

        thresholds = SensorThresholds(
            gps_min_satellites=8,
            battery_min_voltage_v=0.0,
            battery_min_remaining_percent=0.20,
        )

        for sysid, conn in self.swarm.connections.items():
            sensors = SensorDiscovery(conn, thresholds)
            await sensors.request_required_messages()
            self.sensors_map[sysid] = sensors
            
            def status_sink(message: str, sysid=sysid) -> None:
                line = self._line_with_sysid(sysid, message)
                LOGGER.info(line)
                self._send_client_line(line)

            controller = FlightController(
                conn,
                sensors,
                FlightControllerConfig(
                    takeoff_altitude_m=3.0,
                    waypoint_acceptance_radius_m=0.5,
                    waypoint_timeout_s=60.0,
                    battery_warning_percent=0.20,
                    critical_battery_percent=0.12,
                    critical_battery_voltage_v=0.0,
                    max_altitude_m=15.0,
                    final_action="hold",
                ),
                status_sink=status_sink,
            )
            self.controllers[sysid] = controller

        await self.swarm.start_all()
        
        loop = asyncio.get_running_loop()
        threading.Thread(target=self._socket_listener, args=(loop,), daemon=True).start()
        
        asyncio.create_task(self.watchdog_task())
        
        # Telemetry loop to client
        while not self._stop_event.is_set():
            await asyncio.sleep(0.5)
            self.run_ai_pipelines()
            
            if self.client_sock and self.client_connected:
                for sysid, sensors in self.sensors_map.items():
                    report = sensors.snapshot()
                    self._send_client_line(self._telemetry_line(sysid, report))

    async def stop(self):
        self._stop_event.set()
        if self.server_sock:
            self.server_sock.close()
        await self.swarm.close_all()


async def main():
    import argparse
    parser = argparse.ArgumentParser(description="Raspberry Pi Edge Brain for Pixhawk v6x")
    parser.add_argument(
        "--connect",
        action="append",
        help=(
            "MAVLink connection URL. Repeat for a swarm or pass comma-separated URLs "
            "(e.g. --connect tcp:127.0.0.1:5762 --connect tcp:127.0.0.1:5772)."
        ),
    )
    parser.add_argument("--host", default="0.0.0.0", help="Host IP for socket listener")
    parser.add_argument("--port", type=int, default=5000, help="Port for socket listener")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    
    brain = EdgeBrain(serial_url=expand_connection_urls(args.connect), host=args.host, port=args.port)
    try:
        await brain.start()
    except KeyboardInterrupt:
        await brain.stop()

if __name__ == "__main__":
    asyncio.run(main())
