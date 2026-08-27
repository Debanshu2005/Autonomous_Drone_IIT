import asyncio
import logging
import signal
import threading
import time

from flight_controller import FlightState
from trajectory_engine import TaskAction, ParsedTask, TaskSequence

LOGGER = logging.getLogger(__name__)


class SafetyMonitor:
    def __init__(self, repl, min_battery_voltage_v: float = 10.5) -> None:
        self.repl = repl
        self.mavlink = repl.mavlink
        self.sensors = repl.sensors
        self.controller = repl.controller
        self.min_battery_voltage_v = min_battery_voltage_v
        
        self._stop_watchdog = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="safety-watchdog", daemon=True
        )

    def start(self) -> None:
        self._watchdog_thread.start()
        
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, lambda: asyncio.create_task(self._emergency_shutdown()))
            except NotImplementedError:
                pass

    async def _emergency_shutdown(self) -> None:
        LOGGER.error("CRITICAL: SIGINT received. Executing emergency shutdown!")
        
        # 1. Abort active tasks
        self.repl._flush_queue()
        if self.repl._active_future and not self.repl._active_future.done():
            self.repl._active_future.cancel()
            
        # 2. Command LAND mode
        try:
            await self.mavlink.set_mode("LAND", timeout_s=3.0)
            LOGGER.info("Emergency LAND mode activated.")
        except Exception as exc:
            LOGGER.error(f"Failed to set LAND mode during emergency: {exc}")
            
        # 3. Monitor descent
        LOGGER.info("Monitoring descent...")
        while True:
            report = self.sensors.snapshot()
            if not report.armed:
                LOGGER.info("Drone is disarmed. Emergency landing complete.")
                break
                
            await asyncio.sleep(0.5)
            
        # 4. Clean exit
        await self.repl.stop()

    def _watchdog_loop(self) -> None:
        while not self._stop_watchdog.is_set():
            try:
                self._check_vitals()
            except Exception as exc:
                LOGGER.error(f"Safety watchdog error: {exc}")
                
            time.sleep(0.5)
            
    def _check_vitals(self) -> None:
        # Link Loss Check
        heartbeat = self.mavlink.latest("HEARTBEAT")
        if heartbeat is None or (time.monotonic() - heartbeat.received_at_s) > 2.0:
            LOGGER.error("CRITICAL: MAVLink connection lost for > 2.0s!")
            self._trigger_emergency_land()
            return
            
        report = self.sensors.snapshot()
        
        # Battery Voltage Check
        if report.battery.voltage_v is not None and report.battery.voltage_v < self.min_battery_voltage_v:
            LOGGER.error(f"CRITICAL: Battery voltage ({report.battery.voltage_v:.2f}V) below threshold ({self.min_battery_voltage_v}V)!")
            self._trigger_emergency_land()
            return
            
        # GPS Lock Check during spatial formation
        if report.gps.fix_type < 3 and self.controller.state == FlightState.TRAJECTORY_FOLLOW:
            LOGGER.warning("GPS fix degraded during spatial formation. Pausing execution.")
            self._trigger_pause()

    def _trigger_emergency_land(self) -> None:
        self.repl._flush_queue()
        if self.repl._active_future and not self.repl._active_future.done():
            self.repl._active_future.cancel()
            
        sequence = TaskSequence([ParsedTask(TaskAction.LAND, {}, "emergency land")], "emergency land")
        self.repl._queue.put(sequence)
        
    def _trigger_pause(self) -> None:
        self.repl._flush_queue()
        if self.repl._active_future and not self.repl._active_future.done():
            self.repl._active_future.cancel()
            
        sequence = TaskSequence([ParsedTask(TaskAction.HOLD, {}, "safety pause")], "safety pause")
        self.repl._queue.put(sequence)

    def stop(self) -> None:
        self._stop_watchdog.set()
        if self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=1.0)
