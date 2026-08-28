import asyncio
import logging
import os
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

from flight_controller import FlightController, FlightControllerConfig
from mavlink_io import SwarmManager
from sensor_check import SensorDiscovery, SensorThresholds
from trajectory_engine import (
    VehicleOrigin,
    build_trajectory,
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
        drone_id: str = "drone1",
        serial_url: str | Sequence[str] = DEFAULT_CONNECTION_URL,
        host="0.0.0.0",
        port=5000,
        expected_peers: Optional[list[str]] = None,
    ):
        if isinstance(serial_url, str):
            self.connection_urls = expand_connection_urls([serial_url])
        else:
            self.connection_urls = expand_connection_urls(serial_url)
        self.drone_id = drone_id
        self.serial_url = ", ".join(self.connection_urls)
        self.host = host
        self.port = port
        self.swarm = SwarmManager(self.connection_urls)
        self.sensors_map = {}
        self.controllers = {}
        
        self._queues = {}
        self._active_tasks = {}
        
        from hotspot import HotspotContainmentConfig, PeerLink
        import os
        hotspot_enabled = os.environ.get("EDGE_BRAIN_HOTSPOT_ENABLED", "0") == "1"
        self.hotspot_config = HotspotContainmentConfig(
            drone_id=self.drone_id,
            enabled=hotspot_enabled,
            network_watchdog_enabled=hotspot_enabled,
            expected_peer_ids=expected_peers if expected_peers is not None else [],
        )
        self.peer_link = PeerLink(self.hotspot_config) if self.hotspot_config.network_watchdog_enabled else None
        
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

    async def _command_runner_loop(self, sysid: int) -> None:
        while not self._stop_event.is_set():
            try:
                queue = self._queues.get(sysid)
                if not queue:
                    await asyncio.sleep(1.0)
                    continue
                    
                sequence = await queue.get()
                
                # Run the sequence and track it
                task = asyncio.create_task(self._execute_sequence(sysid, sequence))
                self._active_tasks[sysid] = task
                
                try:
                    await task
                except asyncio.CancelledError:
                    LOGGER.info(f"[SYSID:{sysid}] Active task cancelled.")
                except Exception as exc:
                    LOGGER.error(f"[SYSID:{sysid}] Task error: {exc}")
                finally:
                    queue.task_done()
            except asyncio.CancelledError:
                break
            except Exception as e:
                LOGGER.error(f"Error in command runner loop for sysid {sysid}: {e}")
                await asyncio.sleep(1.0)

    def _socket_listener(self, loop):
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(1)
        LOGGER.info(f"[{self.drone_id}] Listening for ground station on {self.host}:{self.port}...")
        
        import os
        auth_token = os.environ.get("EDGE_BRAIN_AUTH_TOKEN")
        if not auth_token:
            LOGGER.warning("EDGE_BRAIN_AUTH_TOKEN is not set. The command socket will be unauthenticated.")
        
        while not self._stop_event.is_set():
            try:
                self.server_sock.settimeout(1.0)
                client, addr = self.server_sock.accept()
                LOGGER.info(f"[{self.drone_id}] Ground station connected from {addr}")
                buffer = ""
                authenticated = False if auth_token else True
                
                if authenticated:
                    with self._client_lock:
                        self.client_sock = client
                        self.client_connected = True
                        self.last_heartbeat_time = time.time()
                
                while not self._stop_event.is_set():
                    data = client.recv(1024)
                    if not data:
                        break
                    buffer += data.decode("utf-8", errors="replace")
                    
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        cmd = line.strip()
                        if not cmd:
                            continue
                            
                        if not authenticated:
                            if cmd == f"AUTH:{auth_token}":
                                authenticated = True
                                LOGGER.info(f"[{self.drone_id}] Client authenticated successfully.")
                                with self._client_lock:
                                    self.client_sock = client
                                    self.client_connected = True
                                    self.last_heartbeat_time = time.time()
                                continue
                            else:
                                LOGGER.warning(f"[{self.drone_id}] Authentication failed from {addr}. Rejecting.")
                                client.close()
                                break
                                    
                        self.last_heartbeat_time = time.time()
                        if cmd == "__heartbeat__":
                            continue
                        LOGGER.info(f"[{self.drone_id}] Received CMD: {cmd}")
                        asyncio.run_coroutine_threadsafe(self.process_command(cmd), loop)
                        self._send_client_line(f"ACK: {cmd}")
                        
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop_event.is_set():
                    LOGGER.error(f"Socket error: {e}")
            finally:
                with self._client_lock:
                    if self.client_sock:
                        self.client_sock.close()
                        self.client_sock = None
                    self.client_connected = False

    async def process_command(self, cmd: str):
        # Broadcast unless the laptop command starts with sysid:<id> or @<id>.
        try:
            target_sysid, command_text = parse_targeted_command(cmd)
            if not command_text:
                raise ValueError("empty command")
            sequence = parse_task_sequence(command_text)
            target_sysids = [target_sysid] if target_sysid is not None else list(self.controllers)
            if not target_sysids:
                raise ValueError("no connected drones")
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
                if not sequence.tasks:
                    continue
                
                from trajectory_engine import TaskAction
                task_action = sequence.tasks[0].action
                
                if task_action in {TaskAction.HOLD, TaskAction.LAND, TaskAction.RTL}:
                    # Fast interrupt: clear queue and cancel active task
                    if sysid in self._queues:
                        while not self._queues[sysid].empty():
                            try:
                                self._queues[sysid].get_nowait()
                                self._queues[sysid].task_done()
                            except asyncio.QueueEmpty:
                                break
                    active = self._active_tasks.get(sysid)
                    if active and not active.done():
                        active.cancel()
                    
                    if sysid in self._queues:
                        self._queues[sysid].put_nowait(sequence)
                else:
                    # Normal queueing
                    if sysid in self._queues:
                        self._queues[sysid].put_nowait(sequence)
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
                if self.client_sock is None:
                    return
                self.client_sock.sendall(payload.encode("utf-8"))
        except Exception as exc:
            LOGGER.warning("Failed to send laptop client message: %s", exc)
            with self._client_lock:
                if self.client_sock:
                    self.client_sock.close()
                    self.client_sock = None
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
        LOGGER.info(f"[{self.drone_id}] Connecting to Pixhawk v6x at {self.serial_url}...")
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
            
            self._queues[sysid] = asyncio.Queue()
            asyncio.create_task(self._command_runner_loop(sysid))
            
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
                    hotspot=self.hotspot_config,
                ),
                status_sink=status_sink,
                peer_link=self.peer_link,
            )
            self.controllers[sysid] = controller

        if self.peer_link:
            await self.peer_link.start()

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
        self.client_connected = False
        with self._client_lock:
            if self.client_sock:
                self.client_sock.close()
                self.client_sock = None
        if self.server_sock:
            self.server_sock.close()
        
        # Issue RTL to all connected drones before shutting down
        LOGGER.info("Issuing RTL to all drones before shutting down...")
        import asyncio
        self._stop_event.set()
        
        peer_link = getattr(self, "peer_link", None)
        if peer_link is not None:
            await peer_link.stop()
            
        rtl_tasks = []
        for sysid, conn in self.swarm.connections.items():
            try:
                rtl_tasks.append(asyncio.create_task(conn.rtl()))
            except Exception as e:
                LOGGER.error(f"Failed to issue RTL to {sysid}: {e}")
        if rtl_tasks:
            # Wait up to 3 seconds for RTL commands to be sent
            done, pending = await asyncio.wait(rtl_tasks, timeout=3.0)
            if pending:
                LOGGER.warning("Some RTL commands did not complete in time.")

        await self.swarm.close_all()


async def main():
    import argparse
    parser = argparse.ArgumentParser(description="Raspberry Pi Edge Brain for Pixhawk v6x")
    parser.add_argument("--drone-id", type=str, default="drone1", help="Drone ID (e.g. drone1)")
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
    parser.add_argument("--peers", type=str, default="", help="Comma-separated list of expected peer drone IDs")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    
    expected_peers = []
    if args.peers:
        expected_peers = [p.strip() for p in args.peers.split(",") if p.strip()]
    elif os.environ.get("EDGE_BRAIN_EXPECTED_PEERS"):
        expected_peers = [p.strip() for p in os.environ["EDGE_BRAIN_EXPECTED_PEERS"].split(",") if p.strip()]

    brain = EdgeBrain(
        drone_id=args.drone_id, 
        serial_url=expand_connection_urls(args.connect), 
        host=args.host, 
        port=args.port,
        expected_peers=expected_peers
    )
    import signal
    import sys
    shutdown_count = 0
    def handle_sigint(signum, frame):
        nonlocal shutdown_count
        shutdown_count += 1
        if shutdown_count > 1:
            LOGGER.error("CRITICAL: Hard abort initiated via double-tap Ctrl+C.")
            sys.exit(1)
        LOGGER.warning("SIGINT received. Initiating soft shutdown and RTL...")
        raise KeyboardInterrupt()

    try:
        signal.signal(signal.SIGINT, handle_sigint)
    except Exception:
        pass

    try:
        await brain.start()
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        await brain.stop()

if __name__ == "__main__":
    asyncio.run(main())
