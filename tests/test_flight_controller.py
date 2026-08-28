import asyncio
import pytest
from unittest.mock import AsyncMock, MagicMock
import time

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../onboard_edge')))

from flight_controller import FlightController, FlightControllerConfig, FlightState
from sensor_check import SensorReport, NavigationMode, GpsState, EkfState, GlobalPositionState, LocalPositionState, FlowVisionState, BatteryState
from hotspot import HotspotContainmentConfig

@pytest.fixture
def mock_mavlink():
    mav = MagicMock()
    mav.arm = AsyncMock()
    mav.takeoff = AsyncMock()
    mav.set_mode = AsyncMock()
    mav.land = AsyncMock()
    mav.rtl = AsyncMock()
    mav.send_local_position_target = MagicMock()
    mav.send_global_position_target = MagicMock()
    mav.send_body_velocity_target = MagicMock()
    
    # Mock latest_message for HEARTBEAT and LOCAL_POSITION_NED
    heartbeat_msg = MagicMock()
    heartbeat_msg.base_mode = 128 # ARMED flag
    heartbeat_msg.received_at_s = time.monotonic()
    
    local_pos_msg = MagicMock()
    local_pos_msg.x = 0.0
    local_pos_msg.y = 0.0
    local_pos_msg.z = -10.0 # 10m altitude
    
    def latest_message(msg_type, timeout=None):
        if msg_type == "HEARTBEAT":
            return heartbeat_msg
        if msg_type == "LOCAL_POSITION_NED":
            return local_pos_msg
        return None
        
    def latest(msg_type):
        if msg_type == "HEARTBEAT":
            class CachedMsg:
                message = heartbeat_msg
                received_at_s = heartbeat_msg.received_at_s
            return CachedMsg()
        return None
    
    def mode_name_from_heartbeat(msg):
        return "GUIDED"
        
    mav.latest_message = latest_message
    mav.latest = latest
    mav.mode_name_from_heartbeat = mode_name_from_heartbeat
    return mav


@pytest.fixture
def mock_sensors():
    sensors = MagicMock()
    sensors.probe = AsyncMock()
    
    def make_report(fix_type=3, voltage=11.5, remaining=0.5, mode=NavigationMode.MODE_A_GPS):
        gps = GpsState(fix_type=fix_type, satellites_visible=10, fresh=True, healthy=True)
        ekf = EkfState(flags=0, velocity_horiz=True, pos_horiz_abs=True, pos_horiz_rel=True, pred_pos_horiz_abs=True, pred_pos_horiz_rel=True, fresh=True, healthy_for_global=True, healthy_for_local=True)
        global_pos = GlobalPositionState(lat_deg=0.0, lon_deg=0.0, relative_alt_m=10.0, fresh=True, valid=True)
        local_pos = LocalPositionState(north_m=0.0, east_m=0.0, down_m=-10.0, fresh=True, valid=True)
        flow = FlowVisionState(source="none", quality=None, fresh=False, valid=False)
        battery = BatteryState(voltage_v=voltage, remaining_percent=remaining, fresh=True, healthy=True)
        
        return SensorReport(
            mode=mode,
            gps=gps,
            ekf=ekf,
            global_position=global_pos,
            local_position=local_pos,
            flow_or_vision=flow,
            battery=battery,
            autopilot_mode="GUIDED",
            armed=True,
            reasons=[]
        )
    
    sensors.snapshot = MagicMock(return_value=make_report())
    sensors.probe.return_value = make_report()
    return sensors


@pytest.mark.asyncio
async def test_failsafe_gps_degradation_pause(mock_mavlink, mock_sensors):
    config = FlightControllerConfig()
    fc = FlightController(mock_mavlink, mock_sensors, config)
    fc.state = FlightState.TRAJECTORY_FOLLOW
    
    # Degrade GPS
    degraded_report = mock_sensors.snapshot.return_value
    mock_sensors.snapshot.return_value = SensorReport(
        mode=NavigationMode.MODE_A_GPS,
        gps=GpsState(fix_type=2, satellites_visible=5, fresh=True, healthy=False),
        ekf=degraded_report.ekf,
        global_position=degraded_report.global_position,
        local_position=degraded_report.local_position,
        flow_or_vision=degraded_report.flow_or_vision,
        battery=degraded_report.battery,
        autopilot_mode="GUIDED",
        armed=True,
        reasons=[]
    )
    
    reason = fc._failsafe_status()
    assert reason is None  # Should not trigger an abort
    assert fc._pause_reason == "GPS degraded"  # But should set the pause reason
    
    # Restore GPS
    mock_sensors.snapshot.return_value = degraded_report
    reason = fc._failsafe_status()
    assert reason is None
    assert fc._pause_reason is None


@pytest.mark.asyncio
async def test_failsafe_battery_debounce(mock_mavlink, mock_sensors):
    config = FlightControllerConfig(battery_debounce_s=0.5, critical_battery_percent=0.15)
    fc = FlightController(mock_mavlink, mock_sensors, config)
    
    # Normal battery
    reason = fc._failsafe_status()
    assert reason is None
    
    # Critical battery but not debounced yet
    critical_report = mock_sensors.snapshot.return_value
    mock_sensors.snapshot.return_value = SensorReport(
        mode=critical_report.mode,
        gps=critical_report.gps,
        ekf=critical_report.ekf,
        global_position=critical_report.global_position,
        local_position=critical_report.local_position,
        flow_or_vision=critical_report.flow_or_vision,
        battery=BatteryState(voltage_v=10.0, remaining_percent=0.10, fresh=True, healthy=True),
        autopilot_mode="GUIDED",
        armed=True,
        reasons=[]
    )
    
    reason = fc._failsafe_status()
    assert reason is None
    assert fc._battery_critical_start_s is not None
    
    # Wait for debounce
    time.sleep(0.6)
    reason = fc._failsafe_status()
    assert reason == "critical battery 10%"
    
    # Transient sag - recover quickly
    fc._battery_critical_start_s = None
    reason = fc._failsafe_status()
    assert reason is None
    
    # Recover battery
    mock_sensors.snapshot.return_value = critical_report
    reason = fc._failsafe_status()
    assert reason is None
    assert fc._battery_critical_start_s is None


@pytest.mark.asyncio
async def test_failsafe_hotspot_geofence(mock_mavlink, mock_sensors):
    config = FlightControllerConfig(
        hotspot=HotspotContainmentConfig(enabled=True, max_radius_m=10.0, expected_peer_ids=[])
    )
    fc = FlightController(mock_mavlink, mock_sensors, config)
    
    # Startup position
    fc._startup_local_position = (0.0, 0.0, 0.0)
    
    # Still within radius
    fc._current_local_position = MagicMock(return_value=(5.0, 5.0, -10.0))
    reason = fc._failsafe_status()
    assert reason is None
    
    # Breach radius
    fc._current_local_position = MagicMock(return_value=(15.0, 15.0, -10.0))
    reason = fc._failsafe_status()
    assert reason is not None
    assert "hotspot geofence exceeded" in reason


@pytest.mark.asyncio
async def test_hardware_check_missing_peers(mock_mavlink, mock_sensors):
    config = FlightControllerConfig(
        hotspot=HotspotContainmentConfig(
            enabled=True, 
            network_watchdog_enabled=True, 
            require_peers_before_arm=True,
            expected_peer_ids=["drone-2"]
        )
    )
    
    peer_link = MagicMock()
    peer_link.missing_peers.return_value = ["drone-2"]
    
    fc = FlightController(mock_mavlink, mock_sensors, config, peer_link=peer_link)
    
    # Missing peers should raise FlightAbort during hardware check
    from flight_controller import FlightAbort
    with pytest.raises(FlightAbort, match="required hotspot peers missing"):
        await fc._hardware_check()
    
    # Peers present should pass
    peer_link.missing_peers.return_value = []
    report = await fc._hardware_check()
    assert report is not None
