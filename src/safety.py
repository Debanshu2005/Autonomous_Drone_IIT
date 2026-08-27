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
        self.swarm = repl.swarm
        self.sensors_map = repl.sensors_map
        self.controllers = repl.controllers
        self.min_battery_voltage_v = min_battery_voltage_v
        
        self._stop_watchdog = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="safety-watchdog", daemon=True
        )

    def start(self) -> None:
        self._watchdog_thread.start()
        
        loop = asyncio.get_running_loop()
        self._shutdown_count = 0
        
        def _signal_handler(signum, frame):
            self._shutdown_count += 1
            if self._shutdown_count > 1:
                LOGGER.error("CRITICAL: Hard abort initiated via double-tap Ctrl+C.")
                import sys
                sys.exit(1)
                
            LOGGER.warning(f"Signal {signum} caught by safety monitor.")
            asyncio.run_coroutine_threadsafe(self._emergency_shutdown(), loop)

        try:
            signal.signal(signal.SIGINT, _signal_handler)
            signal.signal(signal.SIGTERM, _signal_handler)
        except Exception as exc:
            LOGGER.error(f"Failed to set signal handler: {exc}")

    async def _emergency_shutdown(self) -> None:
        import sys
        LOGGER.error("CRITICAL: SIGINT received. Evaluating swarm state...")
        
        for sysid, sensors in self.sensors_map.items():
            report = sensors.snapshot()
            altitude_m = -report.local_position.down_m if report.local_position.valid else 0.0
            
            if not report.armed or altitude_m <= 0.2:
                LOGGER.info(f"[SYSID:{sysid}] Grounded/disarmed.")
                continue
                
            LOGGER.info(f"[SYSID:{sysid}] Airborne (Alt: {altitude_m:.1f}m). Emergency soft-land...")
            
            self.repl._flush_queue_sysid(sysid)
            task = self.repl._active_futures.get(sysid)
            if task and not task.done():
                task.cancel()
                
            conn = self.swarm.connections.get(sysid)
            if conn:
                try:
                    await conn.set_mode("LAND", timeout_s=3.0)
                except Exception as exc:
                    LOGGER.error(f"[SYSID:{sysid}] Failed to set LAND mode: {exc}")

        # In a real swarm we might monitor all descents, for now we exit after a delay
        await asyncio.sleep(5)
        await self.repl.stop()
        sys.exit(0)

    def _watchdog_loop(self) -> None:
        while not self._stop_watchdog.is_set():
            try:
                self._check_vitals()
            except Exception as exc:
                LOGGER.error(f"Safety watchdog error: {exc}")
                
            time.sleep(0.5)
            
    def _check_vitals(self) -> None:
        for sysid, sensors in self.sensors_map.items():
            conn = self.swarm.connections.get(sysid)
            if not conn:
                continue
                
            heartbeat = conn.latest("HEARTBEAT")
            if heartbeat is None or (time.monotonic() - heartbeat.received_at_s) > 2.0:
                LOGGER.error(f"[SYSID:{sysid}] CRITICAL: MAVLink connection lost for > 2.0s!")
                self._trigger_emergency_land(sysid)
                continue
                
            report = sensors.snapshot()
            
            if report.battery.voltage_v is not None and report.battery.voltage_v < self.min_battery_voltage_v:
                LOGGER.error(f"[SYSID:{sysid}] CRITICAL: Battery voltage ({report.battery.voltage_v:.2f}V) below threshold!")
                self._trigger_emergency_land(sysid)
                continue
                
            controller = self.controllers.get(sysid)
            if controller and report.gps.fix_type < 3 and controller.state == FlightState.TRAJECTORY_FOLLOW:
                LOGGER.warning(f"[SYSID:{sysid}] GPS fix degraded. Pausing execution.")
                self._trigger_pause(sysid)

    def _trigger_emergency_land(self, sysid: int) -> None:
        self.repl._flush_queue_sysid(sysid)
        task = self.repl._active_futures.get(sysid)
        if task and not task.done():
            task.cancel()
            
        sequence = TaskSequence([ParsedTask(TaskAction.LAND, {}, "emergency land")], "emergency land")
        if sysid in self.repl._queues:
            self.repl._queues[sysid].put_nowait(sequence)
        
    def _trigger_pause(self, sysid: int) -> None:
        self.repl._flush_queue_sysid(sysid)
        task = self.repl._active_futures.get(sysid)
        if task and not task.done():
            task.cancel()
            
        sequence = TaskSequence([ParsedTask(TaskAction.HOLD, {}, "safety pause")], "safety pause")
        if sysid in self.repl._queues:
            self.repl._queues[sysid].put_nowait(sequence)

    def stop(self) -> None:
        self._stop_watchdog.set()
        if self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=1.0)
