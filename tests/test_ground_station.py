from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
ONBOARD_EDGE = ROOT / "onboard_edge"
if str(ONBOARD_EDGE) not in sys.path:
    sys.path.insert(0, str(ONBOARD_EDGE))

from ground_station.laptop_client import GroundStationApp, TelemetryState
from pi_edge_brain import expand_connection_urls, parse_targeted_command


class GroundStationTests(unittest.TestCase):
    def test_laptop_client_parses_rich_telemetry(self) -> None:
        app = GroundStationApp()
        app.telemetry["drone2"] = TelemetryState(node_id="drone2")
        app._update_telemetry(
            "drone2",
            "[TELEM] SYSID:2 MODE:GUIDED | ARMED:true | BAT: 72% (12.60V) | "
            "ALT:3.1m | NAV:GPS | POS:(1.0, 2.0, -3.1) | GPS:(12.0, 77.0)"
        )
        state = app.telemetry["drone2"]
        self.assertEqual(state.mode, "GUIDED")
        self.assertTrue(state.armed)
        self.assertEqual(state.battery_percent, 72.0)
        self.assertEqual(state.voltage_v, 12.6)
        self.assertEqual(state.altitude_m, 3.1)

    def test_expand_connection_urls(self) -> None:
        self.assertEqual(
            expand_connection_urls(["tcp:127.0.0.1:5762,tcp:127.0.0.1:5772"]),
            ["tcp:127.0.0.1:5762", "tcp:127.0.0.1:5772"],
        )

    def test_parse_targeted_command(self) -> None:
        self.assertEqual(parse_targeted_command("sysid:2 rtl"), (2, "rtl"))
        self.assertEqual(parse_targeted_command("@3 takeoff 3m"), (3, "takeoff 3m"))
        self.assertEqual(parse_targeted_command("land"), (None, "land"))

    @unittest.mock.patch('sys.argv', ['laptop_client.py', '--drone', 'd1=10.0.0.1:9000'])
    @unittest.mock.patch('ground_station.laptop_client.GroundStationApp.run')
    def test_laptop_client_custom_fleet(self, mock_run) -> None:
        from ground_station.laptop_client import main, SWARM_FLEET
        main()
        self.assertIn("d1", SWARM_FLEET)
        self.assertEqual(SWARM_FLEET["d1"], ("10.0.0.1", 9000))


if __name__ == "__main__":
    unittest.main()
