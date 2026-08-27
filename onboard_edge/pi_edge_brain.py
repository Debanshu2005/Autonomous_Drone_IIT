import asyncio
import logging
import socket
import time
import threading
from typing import Optional

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

class EdgeBrain:
    def __init__(self, serial_url="serial:///dev/ttyAMA0:921600", host="0.0.0.0", port=5000):
        self.serial_url = serial_url
        self.host = host
        self.port = port
        self.swarm = SwarmManager([self.serial_url])
        self.sensors_map = {}
        self.controllers = {}
        
        self.last_heartbeat_time = time.time()
        self.client_connected = False
        
        self.server_sock = None
        self.client_sock = None
        
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
        # Broadcast to all for now if no specific sysid targeted
        try:
            sequence = parse_task_sequence(cmd)
            for sysid in self.controllers:
                asyncio.create_task(self._execute_sequence(sysid, sequence))
        except Exception as e:
            msg = f"Command parse error: {e}"
            LOGGER.error(msg)
            if self.client_sock:
                try:
                    self.client_sock.sendall(f"{msg}\n".encode('utf-8'))
                except: pass

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
                status_sink=lambda msg: LOGGER.info(f"[SYSID:{sysid}] {msg}"),
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
                    lat = report.global_position.lat_deg
                    lon = report.global_position.lon_deg
                    alt = report.global_position.relative_alt_m
                    telem = f"[TELEM] SYSID:{sysid} GPS:({lat}, {lon}) ALT:{alt}m MODE:{report.autopilot_mode}\n"
                    try:
                        self.client_sock.sendall(telem.encode('utf-8'))
                    except:
                        pass

    async def stop(self):
        self._stop_event.set()
        if self.server_sock:
            self.server_sock.close()
        await self.swarm.close_all()


async def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    brain = EdgeBrain()
    try:
        await brain.start()
    except KeyboardInterrupt:
        await brain.stop()

if __name__ == "__main__":
    asyncio.run(main())
